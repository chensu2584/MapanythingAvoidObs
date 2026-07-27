"""Unit tests for depth-first fusion of direct-depth and MapAnything voxels."""

from __future__ import annotations

import numpy as np
import pytest

from avoidance.depth_fusion import (
    PROVENANCE_DEPTH,
    PROVENANCE_MAP,
    fuse,
    load_voxel_cloud,
    save_fused,
)

VS = 0.01


def grey(n: int) -> np.ndarray:
    return np.full((n, 3), 128, dtype=np.uint8)


def test_all_depth_voxels_survive():
    depth = np.array([[0.10, 0.0, 0.0], [0.20, 0.0, 0.0]])
    result = fuse(depth, grey(2), np.empty((0, 3)), grey(0), voxel_size_m=VS)
    assert result.depth_mask.sum() == 2
    assert result.map_mask.sum() == 0
    assert result.report["fused_voxels"] == 2


def test_map_voxel_far_from_depth_is_admitted_as_fill():
    depth = np.array([[0.0, 0.0, 0.0]])
    far = np.array([[0.50, 0.0, 0.0]])          # a genuine hole
    result = fuse(depth, grey(1), far, grey(1), voxel_size_m=VS, snap_distance_m=0.03)
    assert result.map_mask.sum() == 1
    assert result.report["map_voxels_admitted_as_fill"] == 1


def test_map_voxel_near_depth_is_snapped_away():
    depth = np.array([[0.0, 0.0, 0.0]])
    near = np.array([[0.01, 0.0, 0.0]])         # same surface, worse geometry
    result = fuse(depth, grey(1), near, grey(1), voxel_size_m=VS, snap_distance_m=0.03)
    assert result.map_mask.sum() == 0
    assert result.report["map_voxels_snapped_to_depth"] == 1


def test_snap_distance_controls_the_boundary():
    depth = np.array([[0.0, 0.0, 0.0]])
    candidate = np.array([[0.04, 0.0, 0.0]])
    strict = fuse(depth, grey(1), candidate, grey(1), voxel_size_m=VS, snap_distance_m=0.05)
    loose = fuse(depth, grey(1), candidate, grey(1), voxel_size_m=VS, snap_distance_m=0.02)
    assert strict.map_mask.sum() == 0
    assert loose.map_mask.sum() == 1


def test_differing_grid_origins_still_align():
    # the two sources quantise onto the same lattice regardless of their origins
    depth = np.array([[0.123, -0.456, 0.789]])
    same_cell = depth + 0.002                    # sub-voxel jitter
    result = fuse(depth, grey(1), same_cell, grey(1), voxel_size_m=VS, snap_distance_m=0.03)
    assert result.report["fused_voxels"] == 1    # not duplicated


def test_duplicate_cells_within_one_source_collapse():
    depth = np.array([[0.10, 0.0, 0.0], [0.1005, 0.0, 0.0]])
    result = fuse(depth, grey(2), np.empty((0, 3)), grey(0), voxel_size_m=VS)
    assert result.report["depth_voxels"] == 1


def test_provenance_labels_are_distinguishable():
    depth = np.array([[0.0, 0.0, 0.0]])
    far = np.array([[1.0, 0.0, 0.0]])
    result = fuse(depth, grey(1), far, grey(1), voxel_size_m=VS)
    assert set(np.unique(result.provenance)) == {PROVENANCE_DEPTH, PROVENANCE_MAP}


def test_negative_snap_distance_rejected():
    with pytest.raises(ValueError):
        fuse(np.zeros((1, 3)), grey(1), np.zeros((1, 3)), grey(1),
             voxel_size_m=VS, snap_distance_m=-0.01)


def test_save_roundtrip_preserves_geometry(tmp_path):
    depth = np.array([[0.10, -0.20, 0.30], [0.11, -0.20, 0.30]])
    far = np.array([[0.80, 0.40, 0.50]])
    result = fuse(depth, grey(2), far, grey(1), voxel_size_m=VS,
                  metadata={"world_frame": "base_link"})
    path = save_fused(result, tmp_path / "fused.npz")
    points, colors, voxel_size, frame = load_voxel_cloud(path)
    assert frame == "base_link"
    assert np.isclose(voxel_size, VS)
    assert len(points) == result.report["fused_voxels"]
    # every reloaded point matches an original within half a voxel
    for p in points:
        assert np.min(np.linalg.norm(result.points_m - p, axis=1)) <= VS / 2 + 1e-9
