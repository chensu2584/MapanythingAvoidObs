#!/usr/bin/env python3
"""Robot-environment JSON worker for the G2 avoidance GUI."""

from __future__ import annotations

import argparse
import sys
import traceback
from pathlib import Path
from typing import Any

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from avoidance.cartesian_goal import (
    TRACKED_FRAME,
    compose_goal_pose,
    goal_rotation,
)
from avoidance.collision_checker import G2CollisionChecker
from avoidance.contracts import AvoidanceError, read_json, write_json
from avoidance.g2_robot_model import G2RobotModel, load_g2_capture_state
from avoidance.ik_solver import solve_g2_arm_ik
from avoidance.planner import plan_g2_avoidance
from avoidance.planning_scene import load_planning_scene

DEFAULTS_PATH = ROOT / "configs/avoidance_defaults.json"

GROUPS = {
    "body": ("base_link", *(f"body_link{i}" for i in range(1, 6))),
    "head": ("body_link5", *(f"head_link{i}" for i in range(1, 4))),
    "left": ("arm_base_link", *(f"arm_l_link{i}" for i in range(1, 8)), "arm_l_end_link"),
    "right": ("arm_base_link", *(f"arm_r_link{i}" for i in range(1, 8)), "arm_r_end_link"),
}


def skeleton(robot: G2RobotModel, q: np.ndarray) -> dict[str, Any]:
    robot.pin.forwardKinematics(robot.model, robot.data, q)
    robot.pin.updateFramePlacements(robot.model, robot.data)
    return {group: [{"name": name, "position_m": robot.data.oMf[robot.model.getFrameId(name)].translation.tolist()} for name in names] for group, names in GROUPS.items()}


def centers(robot: G2RobotModel, q: np.ndarray) -> list[dict[str, Any]]:
    values = robot.collision_geometry_centers(q)
    return [{"name": str(item.name), "position_m": values[i].tolist()} for i, item in enumerate(robot.collision_model.geometryObjects) if not str(item.name).startswith("gripper_")]


def context(request: dict[str, Any]) -> tuple[Any, ...]:
    scene_path, capture_path = Path(request["scene"]).resolve(), Path(request["capture_state"]).resolve()
    scene = load_planning_scene(scene_path)
    positions, _ = load_g2_capture_state(capture_path)
    robot = G2RobotModel()
    q = robot.configuration_from_positions(positions)
    return scene_path, capture_path, request.get("arm", "left"), scene, robot, q


def _cartesian_goal_pose(robot: G2RobotModel, q: np.ndarray, side: str,
                         request: dict[str, Any]) -> tuple[str, np.ndarray]:
    """Re-derive the flange goal pose authoritatively (never trust GUI cache).

    The rotation is recomputed from the orientation policy against the live
    start-flange FK, so a stale ``base_R_goal`` in the request cannot leak
    through.  ``tracking`` is always the arm-end flange for the demo.
    """
    tracking = TRACKED_FRAME[side]
    start_flange = robot.frame_pose(q, tracking)
    policy = request.get("orientation_policy", "hold_snapshot_start_flange")
    rotation = goal_rotation(policy, start_flange)
    position = np.asarray(request["position_m"], dtype=float)
    if position.shape != (3,) or not np.isfinite(position).all():
        raise AvoidanceError("position_m must be three finite values")
    return tracking, compose_goal_pose(rotation, position)


def _checker(robot: G2RobotModel, scene: Any, defaults: dict[str, Any]) -> G2CollisionChecker:
    cluster = defaults["cluster_scene_planning"]
    return G2CollisionChecker(robot, scene, arm_body_demo=True,
                              environment_inflation_m=cluster["environment_inflation_m"],
                              required_clearance_m=cluster["future_edge_clearance_m"])


def preview_cartesian_goal(request: dict[str, Any]) -> dict[str, Any]:
    """Fast IK + goal collision check for the drag preview (no RRT search)."""
    scene_path, capture_path, side, scene, robot, q = context(request)
    defaults = read_json(DEFAULTS_PATH)
    settings = defaults["g2_planner"]
    tracking, goal_pose = _cartesian_goal_pose(robot, q, side, request)
    checker = _checker(robot, scene, defaults)
    reference = q
    if request.get("seed_arm") is not None:  # warm-start from the last feasible preview
        reference = robot.with_arm_configuration(q, side, np.asarray(request["seed_arm"], float))
    ik = solve_g2_arm_ik(
        robot, reference, side, goal_pose, end_frame=tracking, is_valid=checker.is_valid,
        seed_count=settings["ik_seed_count"],
        position_tolerance_m=settings["ik_position_tolerance_m"],
        rotation_tolerance_deg=settings["ik_rotation_tolerance_deg"],
        random_seed=settings["random_seed"] + 10)
    response: dict[str, Any] = {
        "ok": True, "action": "preview_cartesian_goal", "execution_authorized": False,
        "request_id": request.get("request_id"), "arm": side, "tracked_frame": tracking,
        "goal_type": "flange_pose", "base_T_goal": goal_pose.tolist(),
        "flange_position_m": goal_pose[:3, 3].tolist(), "ik": ik.to_dict(),
    }
    if not ik.success:
        response.update({"target_status": "red", "reason": ik.reason})
        return response
    goal_q = robot.with_arm_configuration(q, side, ik.arm_configuration)
    report = checker.check(goal_q)
    response["goal_collision"] = report.to_dict()
    response["skeleton"] = skeleton(robot, goal_q)
    response["collision_geometry_centers"] = centers(robot, goal_q)
    response["target_status"] = "green" if report.valid else "orange"
    response["reason"] = "reachable_clear" if report.valid else "reachable_but_goal_in_collision"
    return response


def execute(request: dict[str, Any]) -> dict[str, Any]:
    if request["action"] == "preview_cartesian_goal":
        return preview_cartesian_goal(request)
    scene_path, capture_path, side, scene, robot, q = context(request)
    if request["action"] == "plan_cartesian_goal":
        _, goal_pose = _cartesian_goal_pose(robot, q, side, request)
        manifest = plan_g2_avoidance(scene_path=scene_path, capture_state_path=capture_path,
                                     side=side, goal_pose=goal_pose, arm_body_demo=True)
        skeleton_path, center_path = [], []
        for waypoint in manifest.get("path", {}).get("waypoints", []):
            full = robot.with_arm_configuration(q, side, waypoint["arm_joint_positions_rad"])
            skeleton_path.append(skeleton(robot, full)); center_path.append(centers(robot, full))
        return {"ok": True, "action": "plan_cartesian_goal", "execution_authorized": False,
                "request_id": request.get("request_id"), "manifest": manifest,
                "skeleton_path": skeleton_path, "collision_geometry_centers_path": center_path}
    if request["action"] == "describe":
        checker = G2CollisionChecker(robot, scene, arm_body_demo=True)
        lower, upper = robot.arm_limits(side)
        return {"ok": True, "action": "describe", "execution_authorized": False, "arm": side, "start_arm_joint_positions_rad": robot.arm_configuration(q, side).tolist(), "arm_lower_limits_rad": lower.tolist(), "arm_upper_limits_rad": upper.tolist(), "skeleton": skeleton(robot, q), "collision_geometry_centers": centers(robot, q), "start_collision": checker.check(q).to_dict(), "collision_policy": checker.metadata()}
    if request["action"] != "plan":
        raise AvoidanceError("Unsupported worker action")
    manifest = plan_g2_avoidance(scene_path=scene_path, capture_state_path=capture_path, side=side, goal_arm=np.asarray(request["goal_arm"]), arm_body_demo=True)
    skeleton_path, center_path = [], []
    for waypoint in manifest.get("path", {}).get("waypoints", []):
        full = robot.with_arm_configuration(q, side, waypoint["arm_joint_positions_rad"])
        skeleton_path.append(skeleton(robot, full)); center_path.append(centers(robot, full))
    return {"ok": True, "action": "plan", "execution_authorized": False, "manifest": manifest, "skeleton_path": skeleton_path, "collision_geometry_centers_path": center_path}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--request", type=Path, required=True)
    parser.add_argument("--response", type=Path, required=True)
    args = parser.parse_args()
    try:
        write_json(args.response, execute(read_json(args.request)))
        return 0
    except Exception as exc:
        write_json(args.response, {"ok": False, "error": str(exc), "traceback": traceback.format_exc()})
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
