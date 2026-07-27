"""Unit tests for the operator-defined gripper removal volume."""

from __future__ import annotations

import numpy as np
import pytest

from avoidance.gripper_volume import (
    DEFAULT_CENTRE_DISTANCE_M,
    DEFAULT_HEIGHT_M,
    DEFAULT_LENGTH_M,
    DEFAULT_WIDTH_M,
    gripper_box,
    gripper_boxes,
    gripper_direction,
    remove_gripper_voxels,
)


def camera_pose(position=(0.0, 0.0, 1.0), forward=(1.0, 0.0, 0.0),
                down=(0.0, 0.0, -1.0)) -> np.ndarray:
    """Build a base_T_camera whose optical axes are the given unit vectors."""
    fwd = np.asarray(forward, float); fwd /= np.linalg.norm(fwd)
    dwn = np.asarray(down, float); dwn -= (dwn @ fwd) * fwd; dwn /= np.linalg.norm(dwn)
    right = np.cross(dwn, fwd)
    pose = np.eye(4)
    pose[:3, 0], pose[:3, 1], pose[:3, 2] = right, dwn, fwd
    pose[:3, 3] = position
    return pose


def test_optical_anchor_pitches_45_deg_below_forward():
    pose = camera_pose()                       # forward +X, down -Z
    direction, _ = gripper_direction(pose, anchor="optical")
    expected = np.array([1.0, 0.0, -1.0]) / np.sqrt(2)
    assert np.allclose(direction, expected, atol=1e-9)
    assert np.isclose(np.linalg.norm(direction), 1.0)


def test_world_anchor_tilts_horizontal_forward_toward_gravity():
    pose = camera_pose(forward=(1.0, 0.0, 0.0), down=(0.0, 0.0, -1.0))
    direction, _ = gripper_direction(pose, anchor="world")
    expected = np.array([1.0, 0.0, -1.0]) / np.sqrt(2)
    assert np.allclose(direction, expected, atol=1e-9)


def test_anchors_differ_when_camera_is_rolled():
    # roll the camera about its optical axis: 'down' no longer aligns with -Z
    pose = camera_pose(forward=(1.0, 0.0, 0.0), down=(0.0, -1.0, -1.0))
    optical, _ = gripper_direction(pose, anchor="optical")
    world, _ = gripper_direction(pose, anchor="world")
    assert not np.allclose(optical, world, atol=1e-3)


def test_box_centre_sits_at_centre_distance_along_direction():
    pose = camera_pose(position=(0.5, 0.2, 1.1))
    box = gripper_box(pose, "left")
    offset = box.centre_m - box.camera_centre_m
    # the 7 cm is camera -> box CENTRE (not to the near face)
    assert np.isclose(np.linalg.norm(offset), DEFAULT_CENTRE_DISTANCE_M)
    assert np.allclose(offset / np.linalg.norm(offset), box.direction)


def test_box_dimensions_and_extent_along_axis():
    box = gripper_box(camera_pose(), "left")
    assert np.allclose(box.size_m, [DEFAULT_LENGTH_M, DEFAULT_WIDTH_M, DEFAULT_HEIGHT_M])
    # length is centred on the box centre: reaches 7.5 cm either way
    tip = box.centre_m + box.direction * DEFAULT_LENGTH_M / 2
    heel = box.centre_m - box.direction * DEFAULT_LENGTH_M / 2
    assert box.contains(np.stack([tip, heel]) - np.stack([box.direction, -box.direction]) * 1e-4).all()


def test_axes_are_orthonormal_right_handed():
    box = gripper_box(camera_pose(), "right")
    assert np.allclose(box.axes.T @ box.axes, np.eye(3), atol=1e-9)
    assert np.isclose(np.linalg.det(box.axes), 1.0, atol=1e-9)


def test_contains_matches_box_geometry():
    box = gripper_box(camera_pose(), "left")
    assert box.contains(box.centre_m[None])[0]
    # just outside the width half-extent
    outside = box.centre_m + box.axes[:, 1] * (DEFAULT_WIDTH_M / 2 + 0.005)
    assert not box.contains(outside[None])[0]
    inside = box.centre_m + box.axes[:, 1] * (DEFAULT_WIDTH_M / 2 - 0.005)
    assert box.contains(inside[None])[0]


def test_corners_span_the_declared_size():
    box = gripper_box(camera_pose(), "left")
    local = (box.corners() - box.centre_m) @ box.axes
    assert np.allclose(local.max(0) - local.min(0), box.size_m)


def test_margin_grows_the_box():
    plain = gripper_box(camera_pose(), "left")
    padded = gripper_box(camera_pose(), "left", margin_m=0.02)
    assert np.allclose(padded.size_m, plain.size_m + 0.04)


def test_remove_gripper_voxels_drops_only_inside_points():
    boxes = gripper_boxes({"hand_left_rgb": camera_pose(position=(0.5, 0.3, 1.0)),
                           "hand_right_rgb": camera_pose(position=(0.5, -0.3, 1.0))})
    assert len(boxes) == 2
    inside = np.stack([b.centre_m for b in boxes])
    far = np.array([[3.0, 3.0, 3.0]])
    points = np.vstack([inside, far])
    keep = remove_gripper_voxels(points, boxes)
    assert keep.tolist() == [False, False, True]


def test_unknown_anchor_rejected():
    with pytest.raises(ValueError):
        gripper_direction(camera_pose(), anchor="elbow")


def test_to_dict_is_explicit_about_not_being_a_tcp():
    payload = gripper_box(camera_pose(), "left").to_dict()
    assert "not_a_confirmed_tcp" in payload["semantics"]
    assert payload["side"] == "left"
    assert len(payload["centre_m"]) == 3
