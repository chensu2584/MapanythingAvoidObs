"""Unit tests for the shared voxel cleanup used before fusion."""

from __future__ import annotations

import numpy as np

from avoidance.gripper_volume import gripper_box
from avoidance.voxel_cleanup import DEFAULT_TABLE_XY_BOUNDS, clean_cloud


def grid(n=40, z=0.65, x0=0.4, y0=-0.2, step=0.01):
    """A dense slab of voxels that survives DBSCAN as one cluster."""
    xs = x0 + np.arange(n) * step
    ys = y0 + np.arange(n) * step
    gx, gy = np.meshgrid(xs, ys)
    pts = np.stack([gx.ravel(), gy.ravel(), np.full(gx.size, z)], axis=1)
    return pts, np.full((len(pts), 3), 150, np.uint8)


def test_workspace_crop_drops_outside_points():
    pts, cols = grid()
    outside = np.array([[5.0, 5.0, 0.65]])
    all_pts = np.vstack([pts, outside])
    all_cols = np.vstack([cols, np.full((1, 3), 150, np.uint8)])
    result = clean_cloud(all_pts, all_cols, min_cluster=1)
    assert result.report["stages"]["after_workspace_crop"] == len(pts)
    assert len(result.points_m) == len(pts)


def test_denoise_drops_small_floating_clusters():
    pts, cols = grid()
    speck = np.array([[0.5, 0.0, 1.0], [0.505, 0.0, 1.0]])   # 2 isolated voxels
    all_pts = np.vstack([pts, speck])
    all_cols = np.vstack([cols, np.full((2, 3), 150, np.uint8)])
    result = clean_cloud(all_pts, all_cols, min_cluster=24)
    assert result.report["stages"]["after_denoise"] == len(pts)


def test_gripper_box_carves_its_volume():
    pts, cols = grid()
    pose = np.eye(4)
    pose[:3, 0], pose[:3, 1], pose[:3, 2] = (0, 1, 0), (0, 0, -1), (1, 0, 0)
    pose[:3, 3] = (0.5, 0.0, 0.75)          # above the slab, looking down-forward
    box = gripper_box(pose, "left")
    plain = clean_cloud(pts, cols, min_cluster=1)
    carved = clean_cloud(pts, cols, gripper_boxes=[box], min_cluster=1)
    assert len(carved.points_m) < len(plain.points_m)
    assert carved.report["stages"]["after_gripper_removal"] < plain.report["stages"]["input"]


def test_table_crop_removes_points_far_below_surface():
    pts, cols = grid(z=0.65)
    low, low_cols = grid(z=0.20)             # a floor slab well below the table
    all_pts = np.vstack([pts, low])
    all_cols = np.vstack([cols, low_cols])

    def support(points, z_band=None):
        return 0.65, 0.65, len(points)

    result = clean_cloud(all_pts, all_cols, min_cluster=1,
                         support_surface_fn=support, table_thickness_m=0.06)
    assert result.table_top_z_m == 0.65
    assert result.report["stages"]["after_table_crop"] == len(pts)
    assert result.points_m[:, 2].min() > 0.5


def test_keep_mask_matches_returned_points():
    pts, cols = grid()
    extra = np.array([[5.0, 5.0, 0.65]])
    all_pts = np.vstack([pts, extra])
    all_cols = np.vstack([cols, np.full((1, 3), 150, np.uint8)])
    result = clean_cloud(all_pts, all_cols, min_cluster=1)
    assert result.keep_mask.sum() == len(result.points_m)
    assert np.allclose(all_pts[result.keep_mask], result.points_m)


def test_report_records_parameters_and_defaults():
    pts, cols = grid()
    result = clean_cloud(pts, cols, min_cluster=1, label="depth")
    params = result.report["parameters"]
    assert result.report["label"] == "depth"
    assert params["table_xy_bounds"] == list(DEFAULT_TABLE_XY_BOUNDS)
    assert params["cluster_eps_m"] == 0.03


def test_disabling_workspace_crop_keeps_everything():
    pts, cols = grid()
    outside = np.array([[5.0, 5.0, 0.65]])
    all_pts = np.vstack([pts, outside])
    all_cols = np.vstack([cols, np.full((1, 3), 150, np.uint8)])
    result = clean_cloud(all_pts, all_cols, table_xy_bounds=None, min_cluster=1)
    assert "after_workspace_crop" not in result.report["stages"]
    assert len(result.points_m) == len(all_pts)
