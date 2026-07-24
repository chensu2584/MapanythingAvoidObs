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
from avoidance.collision_checker import G2CollisionChecker
from avoidance.contracts import AvoidanceError, read_json, write_json
from avoidance.g2_robot_model import G2RobotModel, load_g2_capture_state
from avoidance.planner import plan_g2_avoidance
from avoidance.planning_scene import load_planning_scene

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


def execute(request: dict[str, Any]) -> dict[str, Any]:
    scene_path, capture_path, side, scene, robot, q = context(request)
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
