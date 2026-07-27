"""Shared voxel cleanup: workspace crop, gripper removal, denoise, table crop.

``simplify_g2_snapshots.py`` performs these steps before fitting planning
primitives, and the fused reconstruction must be built from the *same* cleaned
clouds -- fusing raw voxels would drag background walls, floor and speckle noise
into the result and then hand them to the planner.

The steps, in order:

  1. **workspace crop** -- keep only voxels inside the measured table XY
     rectangle, dropping off-scene background;
  2. **gripper removal** -- carve out the operator-measured box anchored to each
     wrist camera (see :mod:`avoidance.gripper_volume`); the URDF omnipicker is
     not the installed gripper and cannot be used for this;
  3. **denoise** -- DBSCAN, dropping unclustered points and clusters too small
     to be a real object;
  4. **table crop** -- locate the support surface by height histogram and drop
     everything more than ``table_thickness_m`` below its top.

The support-surface finder is injected rather than imported so this module stays
free of a hard dependency on the MapAnything pipeline and can be unit-tested on
its own; callers pass ``scene_simplify.find_support_surface``.
"""

from __future__ import annotations

import dataclasses
from typing import Any, Callable

import numpy as np
from sklearn.cluster import DBSCAN

from .gripper_volume import remove_gripper_voxels

# Measured table extent for the 3box scene, base_link metres
# (DEPTH_VOXEL_RECONSTRUCTION.md, "3box 桌面范围").
DEFAULT_TABLE_XY_BOUNDS = (0.239, 1.019, -0.694, 0.706)
DEFAULT_CLUSTER_EPS_M = 0.03
DEFAULT_MIN_CLUSTER = 24
DEFAULT_TABLE_THICKNESS_M = 0.06


@dataclasses.dataclass(frozen=True)
class CleanupResult:
    points_m: np.ndarray
    colors: np.ndarray
    keep_mask: np.ndarray          # against the input cloud, for provenance
    table_top_z_m: float | None
    report: dict[str, Any]


def clean_cloud(points: np.ndarray, colors: np.ndarray, *,
                gripper_boxes=(), table_xy_bounds=DEFAULT_TABLE_XY_BOUNDS,
                cluster_eps_m: float = DEFAULT_CLUSTER_EPS_M,
                min_cluster: int = DEFAULT_MIN_CLUSTER,
                table_thickness_m: float = DEFAULT_TABLE_THICKNESS_M,
                head_z_m: float | None = None,
                support_surface_fn: Callable[..., tuple] | None = None,
                label: str = "cloud") -> CleanupResult:
    """Apply the shared cleanup and report how many voxels each step removed."""
    points = np.asarray(points, dtype=np.float64)
    colors = np.asarray(colors, dtype=np.uint8)
    keep = np.ones(len(points), dtype=bool)
    stages: dict[str, int] = {"input": int(len(points))}

    if table_xy_bounds is not None:
        x_min, x_max, y_min, y_max = table_xy_bounds
        keep &= ((points[:, 0] >= x_min) & (points[:, 0] <= x_max)
                 & (points[:, 1] >= y_min) & (points[:, 1] <= y_max))
        stages["after_workspace_crop"] = int(keep.sum())

    if len(gripper_boxes):
        keep &= remove_gripper_voxels(points, gripper_boxes)
        stages["after_gripper_removal"] = int(keep.sum())

    index = np.where(keep)[0]
    if len(index) and min_cluster > 1:
        labels = DBSCAN(eps=cluster_eps_m, min_samples=2).fit_predict(points[index])
        for value in set(labels):
            member = labels == value
            if value < 0 or member.sum() < min_cluster:
                keep[index[member]] = False
        stages["after_denoise"] = int(keep.sum())

    table_top_z = None
    if support_surface_fn is not None and keep.any():
        retained = points[keep]
        band = (head_z_m - 1.0, head_z_m - 0.4) if head_z_m is not None else None
        table_top_z, _, _ = support_surface_fn(retained, z_band=band)
        below = retained[:, 2] < table_top_z - table_thickness_m
        keep[np.where(keep)[0][below]] = False
        stages["after_table_crop"] = int(keep.sum())

    report = {
        "label": label,
        "stages": stages,
        "removed_total": int(len(points) - keep.sum()),
        "parameters": {
            "table_xy_bounds": list(table_xy_bounds) if table_xy_bounds else None,
            "cluster_eps_m": cluster_eps_m,
            "min_cluster": min_cluster,
            "table_thickness_m": table_thickness_m,
            "gripper_boxes": len(gripper_boxes),
        },
    }
    if table_top_z is not None:
        report["table_top_z_m"] = round(float(table_top_z), 4)
    return CleanupResult(points[keep], colors[keep], keep, table_top_z, report)
