"""Fuse a direct-depth reconstruction with a MapAnything reconstruction.

The two reconstructions have complementary failure modes:

  * the **direct depth** voxels (``reconstruct_depth_voxels.py``) are metric
    ground truth -- the G2 head depth camera measured them -- but they only
    cover surfaces the depth camera actually saw.  Occluded faces, the far side
    of objects and non-returning material are simply absent;
  * the **MapAnything** voxels cover far more of the scene because the model
    completes what it cannot see, but ground-truth comparison showed a residual
    per-pixel depth error (roughly 2 cm in the head view, ~10 cm for surfaces
    only the wrist cameras see).

So depth wins wherever it has evidence, and MapAnything is only allowed to fill
holes.  This module implements that policy on a common 1 cm ``base_link``
lattice:

  1. quantise both clouds onto the same lattice (they are both base_link/1 cm,
     but their grid origins differ, so indices are not directly comparable);
  2. every depth cell is kept and marked ``provenance="depth"``;
  3. a MapAnything cell is admitted only if it is at least ``snap_distance_m``
     away from every depth cell -- a nearer one is treated as the same physical
     surface that depth already measured more accurately, and is dropped
     ("snapped away") rather than laid down as a second shell;
  4. surviving MapAnything cells are marked ``provenance="map"`` so a planner
     can inflate the less-trustworthy fill more aggressively.

Camera extrinsics always come from the depth capture, per the operator's
instruction, and the fused output inherits the depth reconstruction's frame.
"""

from __future__ import annotations

import dataclasses
from pathlib import Path
from typing import Any

import numpy as np
from scipy.spatial import cKDTree

PROVENANCE_DEPTH = 0
PROVENANCE_MAP = 1
DEFAULT_SNAP_DISTANCE_M = 0.03


@dataclasses.dataclass(frozen=True)
class FusionResult:
    points_m: np.ndarray          # (N,3) fused voxel centres, base_link
    colors: np.ndarray            # (N,3) uint8
    provenance: np.ndarray        # (N,) PROVENANCE_DEPTH / PROVENANCE_MAP
    voxel_size_m: float
    report: dict[str, Any]

    @property
    def depth_mask(self) -> np.ndarray:
        return self.provenance == PROVENANCE_DEPTH

    @property
    def map_mask(self) -> np.ndarray:
        return self.provenance == PROVENANCE_MAP


def load_voxel_cloud(path: str | Path) -> tuple[np.ndarray, np.ndarray, float, str]:
    """Read a voxels npz into (world points, colors, voxel size, frame)."""
    data = np.load(Path(path), allow_pickle=True)
    for key in ("indices", "origin", "voxel_size"):
        if key not in data:
            raise ValueError(f"{path} is missing '{key}'")
    voxel_size = float(data["voxel_size"])
    origin = np.asarray(data["origin"], dtype=np.float64)
    points = origin + (data["indices"].astype(np.float64) + 0.5) * voxel_size
    if "colors" in data:
        colors = np.asarray(data["colors"], dtype=np.uint8)
    else:
        colors = np.full((len(points), 3), 200, dtype=np.uint8)
    frame = str(data["world_frame"]) if "world_frame" in data else "unknown"
    return points, colors, voxel_size, frame


def _quantise(points: np.ndarray, voxel_size: float) -> np.ndarray:
    """Snap points onto a shared lattice anchored at the origin."""
    return np.floor(points / voxel_size + 0.5).astype(np.int64)


def _dedupe(keys: np.ndarray, colors: np.ndarray):
    """Collapse duplicate lattice cells, keeping the first colour seen."""
    _, first = np.unique(keys, axis=0, return_index=True)
    first.sort()
    return keys[first], colors[first]


def fuse(depth_points: np.ndarray, depth_colors: np.ndarray,
         map_points: np.ndarray, map_colors: np.ndarray, *,
         voxel_size_m: float, snap_distance_m: float = DEFAULT_SNAP_DISTANCE_M,
         metadata: dict[str, Any] | None = None) -> FusionResult:
    """Depth-first fusion: keep all depth cells, add only non-redundant map cells."""
    if snap_distance_m < 0:
        raise ValueError("snap_distance_m must be non-negative")
    depth_keys, depth_cols = _dedupe(_quantise(depth_points, voxel_size_m),
                                     np.asarray(depth_colors, dtype=np.uint8))
    map_keys, map_cols = _dedupe(_quantise(map_points, voxel_size_m),
                                 np.asarray(map_colors, dtype=np.uint8))
    depth_world = depth_keys * voxel_size_m
    map_world = map_keys * voxel_size_m

    if len(depth_world) and len(map_world):
        distance, _ = cKDTree(depth_world).query(map_world, k=1)
    else:
        distance = np.full(len(map_world), np.inf)
    admit = distance >= snap_distance_m          # only genuine holes get filled

    points = np.vstack([depth_world, map_world[admit]])
    colors = np.vstack([depth_cols, map_cols[admit]])
    provenance = np.concatenate([
        np.full(len(depth_world), PROVENANCE_DEPTH, dtype=np.int8),
        np.full(int(admit.sum()), PROVENANCE_MAP, dtype=np.int8),
    ])
    report = {
        "voxel_size_m": voxel_size_m,
        "snap_distance_m": snap_distance_m,
        "depth_voxels": int(len(depth_world)),
        "map_voxels": int(len(map_world)),
        "map_voxels_admitted_as_fill": int(admit.sum()),
        "map_voxels_snapped_to_depth": int((~admit).sum()),
        "fused_voxels": int(len(points)),
        "map_fill_fraction": round(float(admit.sum()) / max(len(points), 1), 4),
        "policy": "depth_is_geometry_truth_map_fills_holes_only",
        "extrinsics_source": "depth_capture",
    }
    if len(map_world) and np.isfinite(distance).any():
        finite = distance[np.isfinite(distance)]
        report["map_to_depth_distance_m"] = {
            "median": round(float(np.median(finite)), 4),
            "p90": round(float(np.percentile(finite, 90)), 4),
        }
    if metadata:
        report.update(metadata)
    return FusionResult(points, colors, provenance, voxel_size_m, report)


def fuse_files(depth_npz: str | Path, map_npz: str | Path, *,
               snap_distance_m: float = DEFAULT_SNAP_DISTANCE_M,
               keep_mask_depth: np.ndarray | None = None,
               keep_mask_map: np.ndarray | None = None) -> FusionResult:
    """Fuse two voxel npz files, optionally after external filtering.

    ``keep_mask_*`` let the caller drop gripper voxels (see
    :mod:`avoidance.gripper_volume`) before fusion, so the robot's own hand is
    never laid down as an obstacle by either source.
    """
    depth_points, depth_colors, depth_vs, depth_frame = load_voxel_cloud(depth_npz)
    map_points, map_colors, map_vs, map_frame = load_voxel_cloud(map_npz)
    if depth_frame != map_frame:
        raise ValueError(f"frame mismatch: depth={depth_frame} map={map_frame}")
    if not np.isclose(depth_vs, map_vs, rtol=1e-3):
        raise ValueError(f"voxel size mismatch: depth={depth_vs} map={map_vs}")
    if keep_mask_depth is not None:
        depth_points, depth_colors = depth_points[keep_mask_depth], depth_colors[keep_mask_depth]
    if keep_mask_map is not None:
        map_points, map_colors = map_points[keep_mask_map], map_colors[keep_mask_map]
    return fuse(depth_points, depth_colors, map_points, map_colors,
                voxel_size_m=float(depth_vs), snap_distance_m=snap_distance_m,
                metadata={"world_frame": depth_frame,
                          "depth_source": str(Path(depth_npz).resolve()),
                          "map_source": str(Path(map_npz).resolve())})


def save_fused(result: FusionResult, path: str | Path) -> Path:
    """Write the fused cloud in the same npz layout the pipeline consumes."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    voxel_size = result.voxel_size_m
    indices = _quantise(result.points_m, voxel_size)
    origin = indices.min(axis=0) * voxel_size - 0.5 * voxel_size
    rebased = np.rint((result.points_m - origin) / voxel_size - 0.5).astype(np.int32)
    np.savez_compressed(
        path,
        indices=rebased,
        origin=origin.astype(np.float32),
        voxel_size=np.float32(voxel_size),
        dims=(rebased.max(axis=0) + 1).astype(np.int32),
        colors=result.colors,
        counts=np.ones(len(result.points_m), dtype=np.int32),
        conf=np.where(result.depth_mask, 1.0, 0.5).astype(np.float32),
        provenance=result.provenance,
        provenance_names=np.asarray(["depth", "map"]),
        world_frame=np.asarray(result.report.get("world_frame", "base_link")),
        translation_unit=np.asarray("meter"),
        reconstruction_method=np.asarray("depth_first_fusion_map_fills_holes"),
    )
    return path
