"""Unit tests for the Cartesian goal-point interaction (plan sections 5-6)."""

from __future__ import annotations

import numpy as np
import pytest

from avoidance.cartesian_goal import (
    CartesianGoal,
    anchor_to_goal,
    box_edges,
    build_goal_from_pick,
    compose_goal_pose,
    edge_outward_normal,
    flange_reached,
    goal_rotation,
    move_goal,
    pick_edge,
)
from avoidance.contracts import AvoidanceError

BOUNDS = np.array([[0.6, -0.1, 0.7], [0.8, 0.3, 0.9]])


def ortho_xy(points: np.ndarray) -> np.ndarray:
    """Synthetic top-down projection: world (x,y) -> screen pixels."""
    p = np.atleast_2d(np.asarray(points, float))
    return np.stack([p[:, 0] * 1000.0, -p[:, 1] * 1000.0], axis=1)


def test_box_has_twelve_edges_covering_all_corners():
    edges = box_edges(BOUNDS)
    assert len(edges) == 12
    corners = {tuple(np.round(c, 6)) for e in edges for c in (e["p0"], e["p1"])}
    assert len(corners) == 8  # 12 edges touch exactly the 8 AABB corners
    for e in edges:
        assert np.count_nonzero(~np.isclose(e["p0"], e["p1"])) == 1  # varies on one axis
        assert len(e["face_normals"]) == 2


def test_box_edges_rejects_bad_bounds():
    with pytest.raises(AvoidanceError):
        box_edges(np.array([[1, 0, 0], [0, 1, 1]]))  # max < min on x


def test_pick_selects_nearest_edge_and_recovers_3d_on_edge():
    edges = {0: box_edges(BOUNDS)}
    # aim at the middle of the top (+y) / +z... pick a known edge midpoint
    target_edge = box_edges(BOUNDS)[0]
    mid3d = (target_edge["p0"] + target_edge["p1"]) / 2
    mouse = ortho_xy(mid3d)[0]
    pick = pick_edge(edges, mouse, ortho_xy, pixel_threshold=8.0)
    assert pick is not None
    assert pick["primitive_id"] == 0
    # recovered anchor lies on the box surface (one coordinate at a bound extreme)
    on_face = np.any(np.isclose(pick["position_m"], BOUNDS[0])) or \
        np.any(np.isclose(pick["position_m"], BOUNDS[1]))
    assert on_face
    assert pick["pixel_distance"] < 1e-6


def test_pick_returns_none_outside_threshold():
    edges = {0: box_edges(BOUNDS)}
    assert pick_edge(edges, np.array([5000.0, 5000.0]), ortho_xy, pixel_threshold=10.0) is None


def test_edge_normal_is_unit_and_flippable():
    edges = box_edges(BOUNDS)
    n = edge_outward_normal(edges[0]["face_normals"])
    assert np.isclose(np.linalg.norm(n), 1.0)
    assert np.allclose(edge_outward_normal(edges[0]["face_normals"], flip=True), -n)


def test_edge_normal_faces_camera_when_view_given():
    edges = box_edges(BOUNDS)
    view_dir = np.array([0.0, 0.0, -1.0])  # camera looking down -Z
    n = edge_outward_normal(edges[0]["face_normals"], view_dir=view_dir)
    assert n @ view_dir <= 0  # points back toward the camera


def test_anchor_to_goal_offsets_along_normal():
    anchor = np.array([0.7, 0.3, 0.8])
    normal = np.array([0.0, 1.0, 0.0])
    goal = anchor_to_goal(anchor, normal, 0.08)
    assert np.allclose(goal, [0.7, 0.38, 0.8])
    with pytest.raises(AvoidanceError):
        anchor_to_goal(anchor, normal, -0.01)


def test_hold_start_flange_rotation():
    start = np.eye(4)
    start[:3, :3] = _rz(0.5)
    assert np.allclose(goal_rotation("hold_snapshot_start_flange", start), _rz(0.5))
    with pytest.raises(AvoidanceError):
        goal_rotation("grasp_pose", start)


def test_goal_contract_roundtrip_and_always_unexecutable():
    start = np.eye(4)
    goal = CartesianGoal(
        active_arm="left",
        position_m=np.array([0.70, 0.20, 0.90]),
        base_R_goal=np.eye(3),
        orientation_policy="hold_snapshot_start_flange",
        anchor={"primitive_id": 2, "feature": "box_edge", "edge_id": 7,
                "position_m": np.array([0.70, 0.12, 0.80]),
                "outward_normal": np.array([0.0, 1.0, 0.0]), "approach_offset_m": 0.08},
        provenance={"obstacles_sha256": "abc"},
    )
    d = goal.to_dict()
    assert d["execution_authorized"] is False
    assert d["tracked_frame"] == "arm_l_end_link"
    assert d["schema_version"] == 1
    assert d["position_m"] == [0.7, 0.2, 0.9]
    assert d["anchor"]["outward_normal"] == [0.0, 1.0, 0.0]
    assert d["provenance"]["obstacles_sha256"] == "abc"


def test_goal_rejects_invalid_rotation_and_arm():
    with pytest.raises(AvoidanceError):
        CartesianGoal("left", np.zeros(3), np.full((3, 3), 2.0),
                      "hold_snapshot_start_flange", {})
    with pytest.raises(AvoidanceError):
        CartesianGoal("middle", np.zeros(3), np.eye(3),
                      "hold_snapshot_start_flange", {})


def test_build_goal_from_pick_end_to_end():
    edges = {2: box_edges(BOUNDS)}
    target = box_edges(BOUNDS)[7]
    mouse = ortho_xy((target["p0"] + target["p1"]) / 2)[0]
    pick = pick_edge(edges, mouse, ortho_xy, pixel_threshold=8.0)
    start = np.eye(4)
    start[:3, :3] = _rz(0.3)
    goal = build_goal_from_pick("right", pick, 0.08, start,
                                view_dir=np.array([0, 0, -1.0]))
    assert goal.active_arm == "right"
    assert goal.tracked_frame == "arm_r_end_link"
    assert np.allclose(goal.base_R_goal, _rz(0.3))
    # goal sits one offset off the anchor along the (unit) normal
    off = goal.position_m - np.asarray(goal.anchor["position_m"])
    assert np.isclose(np.linalg.norm(off), 0.08)


def test_move_goal_translates_only_position():
    goal = CartesianGoal("left", np.array([0.7, 0.2, 0.9]), np.eye(3),
                         "hold_snapshot_start_flange", {})
    moved = move_goal(goal, [0.01, 0.0, -0.02])
    assert np.allclose(moved.position_m, [0.71, 0.2, 0.88])
    assert np.allclose(moved.base_R_goal, goal.base_R_goal)


def test_flange_reached_errors():
    goal = CartesianGoal("left", np.array([0.7, 0.2, 0.9]), np.eye(3),
                         "hold_snapshot_start_flange", {})
    achieved = compose_goal_pose(_rz(np.radians(2)), np.array([0.705, 0.2, 0.9]))
    pos_err, rot_err = flange_reached(goal, achieved)
    assert np.isclose(pos_err, 0.005, atol=1e-6)
    assert np.isclose(rot_err, 2.0, atol=1e-3)


def _rz(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])
