"""Unit tests for the Cartesian target interaction state machine (plan 5/7/8)."""

from __future__ import annotations

import dataclasses

import numpy as np
import pytest

from avoidance.cartesian_goal import box_edges
from avoidance.cartesian_target_controller import CartesianTargetController
from avoidance.contracts import AvoidanceError

BOUNDS = np.array([[0.6, -0.1, 0.7], [0.8, 0.3, 0.9]])


@dataclasses.dataclass
class _Prim:
    identifier: int
    role: str
    kind: str
    bounds_m: np.ndarray


@dataclasses.dataclass
class _Scene:
    primitives: list


def _scene():
    return _Scene([_Prim(1, "object", "box", BOUNDS),
                   _Prim(0, "support", "box", np.array([[0.2, -0.7, 0.55], [1.0, 0.7, 0.62]]))])


def ortho(points):
    p = np.atleast_2d(np.asarray(points, float))
    return np.stack([p[:, 0] * 1000.0, -p[:, 1] * 1000.0], axis=1)


def _pick_first_edge(ctrl):
    edge = box_edges(BOUNDS)[0]
    mouse = ortho((edge["p0"] + edge["p1"]) / 2)[0]
    return ctrl.pick(mouse, ortho, view_dir=np.array([0, 0, -1.0]))


def test_load_scene_makes_object_edges_pickable():
    ctrl = CartesianTargetController()
    ctrl.load_scene(_scene(), active_arm="right")
    assert ctrl.active_arm == "right"
    assert _pick_first_edge(ctrl) is True
    assert ctrl.has_target


def test_no_target_before_pick():
    ctrl = CartesianTargetController()
    ctrl.load_scene(_scene())
    assert not ctrl.has_target
    with pytest.raises(AvoidanceError):
        ctrl.goal_position()


def test_goal_position_is_offset_from_edge():
    ctrl = CartesianTargetController(approach_offset_m=0.08)
    ctrl.load_scene(_scene())
    _pick_first_edge(ctrl)
    anchor = ctrl._pick["position_m"]
    assert np.isclose(np.linalg.norm(ctrl.goal_position() - anchor), 0.08)


def test_edit_invalidates_last_verdict():
    ctrl = CartesianTargetController()
    ctrl.load_scene(_scene())
    _pick_first_edge(ctrl)
    ctrl.preview_request("s", "c")
    ctrl.ingest({"request_id": 1, "target_status": "green",
                 "base_T_goal": np.eye(4).tolist(),
                 "ik": {"success": True, "arm_joint_positions_rad": [0.0] * 7}})
    assert ctrl.can_plan
    ctrl.nudge([0.01, 0, 0])            # any edit drops the green verdict
    assert ctrl.last_status == "gray"
    assert not ctrl.can_plan


def test_stale_response_is_dropped():
    ctrl = CartesianTargetController()
    ctrl.load_scene(_scene())
    _pick_first_edge(ctrl)
    ctrl.preview_request("s", "c")     # request_id 1
    ctrl.preview_request("s", "c")     # request_id 2 (fast re-drag)
    accepted_old = ctrl.ingest({"request_id": 1, "target_status": "green",
                                "base_T_goal": np.eye(4).tolist(),
                                "ik": {"success": True, "arm_joint_positions_rad": [0.0] * 7}})
    assert accepted_old is False       # stale id ignored
    assert ctrl.last_status == "gray"
    accepted_new = ctrl.ingest({"request_id": 2, "target_status": "orange",
                                "base_T_goal": np.eye(4).tolist(),
                                "ik": {"success": True, "arm_joint_positions_rad": [0.1] * 7}})
    assert accepted_new is True
    assert ctrl.last_status == "orange"
    assert not ctrl.can_plan           # orange is not plannable


def test_plan_requires_current_green_preview():
    ctrl = CartesianTargetController()
    ctrl.load_scene(_scene())
    _pick_first_edge(ctrl)
    with pytest.raises(AvoidanceError):
        ctrl.plan_request("s", "c")    # nothing previewed yet
    ctrl.preview_request("s", "c")
    ctrl.ingest({"request_id": 1, "target_status": "green",
                 "base_T_goal": np.eye(4).tolist(),
                 "ik": {"success": True, "arm_joint_positions_rad": [0.2] * 7}})
    req = ctrl.plan_request("s", "c")
    assert req["action"] == "plan_cartesian_goal"
    assert req["arm"] == ctrl.active_arm


def test_preview_warm_starts_from_last_feasible():
    ctrl = CartesianTargetController()
    ctrl.load_scene(_scene())
    _pick_first_edge(ctrl)
    r1 = ctrl.preview_request("s", "c")
    assert "seed_arm" not in r1
    ctrl.ingest({"request_id": 1, "target_status": "green",
                 "base_T_goal": np.eye(4).tolist(),
                 "ik": {"success": True, "arm_joint_positions_rad": [0.3] * 7}})
    ctrl.nudge([0.005, 0, 0])
    r2 = ctrl.preview_request("s", "c")
    assert r2["seed_arm"] == [0.3] * 7  # warm-start seed carried forward


def test_flip_reverses_offset_direction():
    ctrl = CartesianTargetController(approach_offset_m=0.05)
    ctrl.load_scene(_scene())
    _pick_first_edge(ctrl)
    before = ctrl.goal_position() - ctrl._pick["position_m"]
    ctrl.toggle_flip()
    after = ctrl.goal_position() - ctrl._pick["position_m"]
    assert np.allclose(after, -before)


def test_to_cartesian_goal_uses_worker_rotation():
    ctrl = CartesianTargetController()
    ctrl.load_scene(_scene(), active_arm="left")
    _pick_first_edge(ctrl)
    ctrl.preview_request("s", "c")
    rot = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1.0]])
    pose = np.eye(4); pose[:3, :3] = rot
    ctrl.ingest({"request_id": 1, "target_status": "green", "base_T_goal": pose.tolist(),
                 "ik": {"success": True, "arm_joint_positions_rad": [0.0] * 7}})
    goal = ctrl.to_cartesian_goal(provenance={"obstacles_sha256": "x"})
    assert np.allclose(goal.base_R_goal, rot)     # rotation comes from the worker
    assert goal.to_dict()["execution_authorized"] is False
    assert goal.tracked_frame == "arm_l_end_link"


def test_projection_helper_matches_matplotlib(tmp_path):
    import matplotlib
    matplotlib.use("Agg")
    from matplotlib.figure import Figure
    from avoidance.cartesian_target_controller import project_world_to_screen

    fig = Figure()
    ax = fig.add_subplot(111, projection="3d")
    ax.set_xlim(0, 1); ax.set_ylim(0, 1); ax.set_zlim(0, 1)
    fig.canvas.draw()
    pts = np.array([[0.2, 0.3, 0.4], [0.7, 0.1, 0.9]])
    screen = project_world_to_screen(ax, pts)
    assert screen.shape == (2, 2)
    assert np.isfinite(screen).all()
