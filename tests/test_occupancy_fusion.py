"""Unit tests for confidence-weighted occupancy fusion (method 3)."""

from __future__ import annotations

import numpy as np

from avoidance.occupancy_fusion import (
    CONFIDENCE_DEPTH_SURFACE,
    CONFIDENCE_MAP_OCCLUDED,
    CONFIDENCE_MAP_UNSEEN,
    STATE_FREE,
    STATE_OCCLUDED,
    STATE_OCCUPIED,
    STATE_UNSEEN,
    DepthView,
    classify_points,
    fuse_with_occupancy,
)

VS = 0.01
K = np.array([[100.0, 0, 50.0], [0, 100.0, 50.0], [0, 0, 1.0]])


def view(distance_m: float = 1.0, shape=(100, 100), name="head") -> DepthView:
    """A camera at the origin looking along +Z at a flat wall `distance_m` away."""
    return DepthView(depth_m=np.full(shape, distance_m, np.float32), K=K,
                     base_T_camera=np.eye(4), name=name)


def test_point_in_front_of_the_wall_is_free():
    state = classify_points(np.array([[0.0, 0.0, 0.5]]), [view()])
    assert state[0] == STATE_FREE


def test_point_on_the_wall_is_occupied():
    state = classify_points(np.array([[0.0, 0.0, 1.0]]), [view()])
    assert state[0] == STATE_OCCUPIED


def test_point_behind_the_wall_is_occluded():
    state = classify_points(np.array([[0.0, 0.0, 1.6]]), [view()])
    assert state[0] == STATE_OCCLUDED


def test_point_outside_the_image_is_unseen():
    state = classify_points(np.array([[5.0, 0.0, 1.0]]), [view()])
    assert state[0] == STATE_UNSEEN


def test_point_behind_the_camera_is_unseen():
    state = classify_points(np.array([[0.0, 0.0, -1.0]]), [view()])
    assert state[0] == STATE_UNSEEN


def test_invalid_depth_returns_unseen():
    empty = DepthView(depth_m=np.zeros((100, 100), np.float32), K=K,
                      base_T_camera=np.eye(4))
    assert classify_points(np.array([[0.0, 0.0, 1.0]]), [empty])[0] == STATE_UNSEEN


def test_free_from_one_view_beats_occluded_from_another():
    near, far = view(distance_m=0.5), view(distance_m=2.0, name="second")
    # 1.0 m is behind the near wall but in front of the far one
    state = classify_points(np.array([[0.0, 0.0, 1.0]]), [near, far])
    assert state[0] == STATE_FREE


def test_map_voxel_in_free_space_is_rejected():
    depth = np.array([[0.0, 0.0, 1.0]])          # the measured wall
    ghost = np.array([[0.0, 0.0, 0.5]])          # model geometry the camera saw through
    result = fuse_with_occupancy(depth, np.full((1, 3), 200, np.uint8),
                                 ghost, np.full((1, 3), 200, np.uint8),
                                 views=[view()], voxel_size_m=VS)
    assert result.report["map_rejected_in_observed_free_space"] == 1
    assert result.report["fused_voxels"] == 1     # only the depth voxel survives
    assert not (result.provenance == 1).any()


def test_map_voxel_behind_a_surface_is_admitted_with_mid_confidence():
    depth = np.array([[0.0, 0.0, 1.0]])
    hidden = np.array([[0.0, 0.0, 1.5]])         # legitimate occlusion fill
    result = fuse_with_occupancy(depth, np.full((1, 3), 200, np.uint8),
                                 hidden, np.full((1, 3), 200, np.uint8),
                                 views=[view()], voxel_size_m=VS)
    assert result.report["map_admitted_occluded"] == 1
    assert result.confidence[result.provenance == 1][0] == CONFIDENCE_MAP_OCCLUDED


def test_map_voxel_outside_coverage_is_admitted_with_low_confidence():
    depth = np.array([[0.0, 0.0, 1.0]])
    elsewhere = np.array([[5.0, 5.0, 1.0]])      # never covered by the depth image
    result = fuse_with_occupancy(depth, np.full((1, 3), 200, np.uint8),
                                 elsewhere, np.full((1, 3), 200, np.uint8),
                                 views=[view()], voxel_size_m=VS)
    assert result.report["map_admitted_unseen"] == 1
    assert result.confidence[result.provenance == 1][0] == CONFIDENCE_MAP_UNSEEN


def test_depth_voxels_always_carry_full_confidence():
    depth = np.array([[0.0, 0.0, 1.0], [0.01, 0.0, 1.0]])
    result = fuse_with_occupancy(depth, np.full((2, 3), 200, np.uint8),
                                 np.empty((0, 3)), np.empty((0, 3), np.uint8),
                                 views=[view()], voxel_size_m=VS)
    assert (result.confidence == CONFIDENCE_DEPTH_SURFACE).all()
    assert (result.state == STATE_OCCUPIED).all()


def test_redundant_map_voxel_is_snapped_not_rejected():
    depth = np.array([[0.0, 0.0, 1.0]])
    nearby = np.array([[0.005, 0.0, 1.0]])       # same surface, measured better already
    result = fuse_with_occupancy(depth, np.full((1, 3), 200, np.uint8),
                                 nearby, np.full((1, 3), 200, np.uint8),
                                 views=[view()], voxel_size_m=VS,
                                 snap_distance_m=0.03)
    assert result.report["map_snapped_to_depth"] == 1
    assert result.report["map_rejected_in_observed_free_space"] == 0


def test_counts_balance():
    depth = np.array([[0.0, 0.0, 1.0]])
    candidates = np.array([[0.0, 0.0, 0.5],      # free -> rejected
                           [0.0, 0.0, 1.5],      # occluded -> admitted
                           [5.0, 5.0, 1.0],      # unseen -> admitted
                           [0.005, 0.0, 1.0]])   # redundant -> snapped
    result = fuse_with_occupancy(depth, np.full((1, 3), 200, np.uint8),
                                 candidates, np.full((4, 3), 200, np.uint8),
                                 views=[view()], voxel_size_m=VS)
    r = result.report
    assert r["map_voxels"] == 4
    assert r["map_snapped_to_depth"] == 1
    assert r["map_rejected_in_observed_free_space"] == 1
    assert r["map_admitted_occluded"] + r["map_admitted_unseen"] == 2
    assert r["fused_voxels"] == 1 + 2
