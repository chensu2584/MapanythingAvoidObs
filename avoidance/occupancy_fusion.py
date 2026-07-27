"""Confidence-weighted fusion using the depth camera's free-space evidence.

The depth-first fusion in :mod:`avoidance.depth_fusion` only ever compares
*surfaces*: a MapAnything voxel is admitted whenever it is far enough from any
measured surface.  That throws away the strongest evidence a depth image
carries.  Every valid depth pixel does not just say "there is a surface here",
it also says "everything between the camera and that surface is empty" -- the
camera saw straight through it.  Measured on the 7.24Exp frames, 7-10% of the
voxels the surface-only rule admits sit in exactly that observed free space, so
they are provably wrong, and no surface-distance threshold can catch them.

This module classifies each candidate voxel against the depth image instead:

  ``occupied``  within ``surface_tolerance_m`` of the measured surface;
  ``free``      closer to the camera than the measured surface -- the camera
                looked through this cell, so anything claimed here is wrong;
  ``occluded``  behind the measured surface -- unobserved, so model completion
                is legitimate here;
  ``unseen``    outside the depth image, or where the depth returned nothing.

Voxels are then kept with a confidence that reflects how well they are
evidenced, so a planner can inflate weak geometry more aggressively rather than
treating every voxel as equally certain:

  depth surface            1.00   measured directly
  map behind a surface     0.50   plausible completion of an occluded region
  map with no depth cover  0.35   unverifiable, lowest trust
  map in observed free space  --  dropped

Only the head has a depth camera on this robot; wrist depth images, once
captured, slot in as additional views without changing the policy.
"""

from __future__ import annotations

import dataclasses
from typing import Any, Sequence

import numpy as np
from scipy.spatial import cKDTree

STATE_UNSEEN = 0
STATE_FREE = 1
STATE_OCCUPIED = 2
STATE_OCCLUDED = 3
STATE_NAMES = ("unseen", "free", "occupied", "occluded")

CONFIDENCE_DEPTH_SURFACE = 1.0
CONFIDENCE_MAP_OCCLUDED = 0.5
CONFIDENCE_MAP_UNSEEN = 0.35

DEFAULT_SURFACE_TOLERANCE_M = 0.04
DEFAULT_SNAP_DISTANCE_M = 0.03


@dataclasses.dataclass(frozen=True)
class DepthView:
    """One registered depth image plus the pose and intrinsics to project it."""

    depth_m: np.ndarray            # (H,W), metres; <=0 or non-finite = no return
    K: np.ndarray                  # 3x3 pinhole intrinsics of the depth camera
    base_T_camera: np.ndarray      # 4x4, camera -> base_link
    min_depth_m: float = 0.05
    max_depth_m: float = 3.0
    name: str = "head"

    def classify(self, points_m: np.ndarray,
                 surface_tolerance_m: float = DEFAULT_SURFACE_TOLERANCE_M) -> np.ndarray:
        """Label base_link points free / occupied / occluded / unseen for this view."""
        points = np.atleast_2d(np.asarray(points_m, dtype=np.float64))
        state = np.full(len(points), STATE_UNSEEN, dtype=np.int8)
        camera_T_base = np.linalg.inv(np.asarray(self.base_T_camera, dtype=np.float64))
        local = (camera_T_base[:3, :3] @ points.T).T + camera_T_base[:3, 3]
        in_front = local[:, 2] > 1e-6
        if not in_front.any():
            return state
        height, width = self.depth_m.shape
        u = np.full(len(points), -1.0)
        v = np.full(len(points), -1.0)
        u[in_front] = self.K[0, 0] * local[in_front, 0] / local[in_front, 2] + self.K[0, 2]
        v[in_front] = self.K[1, 1] * local[in_front, 1] / local[in_front, 2] + self.K[1, 2]
        ui = np.rint(u).astype(np.int64)
        vi = np.rint(v).astype(np.int64)
        inside = in_front & (ui >= 0) & (ui < width) & (vi >= 0) & (vi < height)
        if not inside.any():
            return state
        measured = np.zeros(len(points))
        measured[inside] = self.depth_m[vi[inside], ui[inside]]
        usable = (inside & np.isfinite(measured)
                  & (measured >= self.min_depth_m) & (measured <= self.max_depth_m))
        if not usable.any():
            return state
        delta = local[:, 2] - measured          # negative = in front of the surface
        state[usable & (np.abs(delta) <= surface_tolerance_m)] = STATE_OCCUPIED
        state[usable & (delta < -surface_tolerance_m)] = STATE_FREE
        state[usable & (delta > surface_tolerance_m)] = STATE_OCCLUDED
        return state


@dataclasses.dataclass(frozen=True)
class OccupancyFusionResult:
    points_m: np.ndarray
    colors: np.ndarray
    provenance: np.ndarray         # 0 = depth, 1 = map (matches depth_fusion)
    confidence: np.ndarray
    state: np.ndarray              # depth-evidence state of each kept voxel
    voxel_size_m: float
    report: dict[str, Any]

    # Same accessors as depth_fusion.FusionResult so the export path is shared.
    @property
    def depth_mask(self) -> np.ndarray:
        return self.provenance == 0

    @property
    def map_mask(self) -> np.ndarray:
        return self.provenance == 1


def classify_points(points_m: np.ndarray, views: Sequence[DepthView],
                    surface_tolerance_m: float = DEFAULT_SURFACE_TOLERANCE_M) -> np.ndarray:
    """Combine per-view labels into one state, strongest evidence winning.

    A cell any camera saw through is free even if another view never covered it,
    and a measured surface outranks an occlusion guess, so the merge order is
    occupied > free > occluded > unseen.
    """
    points = np.atleast_2d(np.asarray(points_m, dtype=np.float64))
    merged = np.full(len(points), STATE_UNSEEN, dtype=np.int8)
    for view in views:
        state = view.classify(points, surface_tolerance_m)
        merged = np.where(state == STATE_OCCUPIED, STATE_OCCUPIED,
                          np.where((state == STATE_FREE) & (merged != STATE_OCCUPIED),
                                   STATE_FREE,
                                   np.where((state == STATE_OCCLUDED)
                                            & (merged == STATE_UNSEEN),
                                            STATE_OCCLUDED, merged)))
    return merged.astype(np.int8)


def _quantise(points: np.ndarray, voxel_size: float) -> np.ndarray:
    return np.floor(points / voxel_size + 0.5).astype(np.int64)


def _dedupe(points: np.ndarray, colors: np.ndarray, voxel_size: float):
    keys = _quantise(points, voxel_size)
    _, first = np.unique(keys, axis=0, return_index=True)
    first.sort()
    return keys[first] * voxel_size, colors[first]


def fuse_with_occupancy(depth_points: np.ndarray, depth_colors: np.ndarray,
                        map_points: np.ndarray, map_colors: np.ndarray, *,
                        views: Sequence[DepthView], voxel_size_m: float,
                        snap_distance_m: float = DEFAULT_SNAP_DISTANCE_M,
                        surface_tolerance_m: float = DEFAULT_SURFACE_TOLERANCE_M,
                        metadata: dict[str, Any] | None = None) -> OccupancyFusionResult:
    """Fuse depth and MapAnything voxels, arbitrating by depth free-space evidence."""
    depth_world, depth_cols = _dedupe(np.asarray(depth_points, float),
                                      np.asarray(depth_colors, np.uint8), voxel_size_m)
    map_world, map_cols = _dedupe(np.asarray(map_points, float),
                                  np.asarray(map_colors, np.uint8), voxel_size_m)

    state = classify_points(map_world, views, surface_tolerance_m) if len(map_world) \
        else np.zeros(0, np.int8)

    if len(depth_world) and len(map_world):
        distance, _ = cKDTree(depth_world).query(map_world, k=1)
    else:
        distance = np.full(len(map_world), np.inf)

    redundant = distance < snap_distance_m          # depth already measured it, better
    contradicted = (state == STATE_FREE) & ~redundant
    admit = ~redundant & ~contradicted

    confidence_map = np.where(state[admit] == STATE_OCCLUDED,
                              CONFIDENCE_MAP_OCCLUDED, CONFIDENCE_MAP_UNSEEN)
    points = np.vstack([depth_world, map_world[admit]])
    colors = np.vstack([depth_cols, map_cols[admit]])
    provenance = np.concatenate([np.zeros(len(depth_world), np.int8),
                                 np.ones(int(admit.sum()), np.int8)])
    confidence = np.concatenate([
        np.full(len(depth_world), CONFIDENCE_DEPTH_SURFACE, np.float32),
        confidence_map.astype(np.float32)])
    kept_state = np.concatenate([
        np.full(len(depth_world), STATE_OCCUPIED, np.int8), state[admit]])

    report = {
        "policy": "confidence_weighted_occupancy_fusion",
        "voxel_size_m": voxel_size_m,
        "snap_distance_m": snap_distance_m,
        "surface_tolerance_m": surface_tolerance_m,
        "depth_views": [view.name for view in views],
        "depth_voxels": int(len(depth_world)),
        "map_voxels": int(len(map_world)),
        "map_snapped_to_depth": int(redundant.sum()),
        "map_rejected_in_observed_free_space": int(contradicted.sum()),
        "map_admitted_occluded": int(((state == STATE_OCCLUDED) & admit).sum()),
        "map_admitted_unseen": int(((state == STATE_UNSEEN) & admit).sum()),
        "map_admitted_at_surface": int(((state == STATE_OCCUPIED) & admit).sum()),
        "fused_voxels": int(len(points)),
        "confidence_levels": {"depth_surface": CONFIDENCE_DEPTH_SURFACE,
                              "map_occluded": CONFIDENCE_MAP_OCCLUDED,
                              "map_unseen": CONFIDENCE_MAP_UNSEEN},
        "extrinsics_source": "depth_capture",
    }
    if metadata:
        report.update(metadata)
    return OccupancyFusionResult(points, colors, provenance, confidence, kept_state,
                                 voxel_size_m, report)
