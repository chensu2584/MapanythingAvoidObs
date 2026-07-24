"""Offline G2 arm avoidance orchestration."""

from __future__ import annotations

import datetime as dt
from pathlib import Path
from typing import Any

import numpy as np

from .collision_checker import G2CollisionChecker
from .contracts import AvoidanceError, read_json, sha256_file
from .end_effector_model import DEFAULT_G2_END_EFFECTOR_CONFIG, load_end_effector_model_status
from .g2_robot_model import G2_JOINT_LAYOUT, G2RobotModel, load_g2_capture_state
from .ik_solver import solve_g2_arm_ik
from .planning_scene import load_planning_scene
from .rrt_connect import RRTConnectPlanner


def plan_g2_avoidance(*, scene_path: str | Path, capture_state_path: str | Path, side: str, goal_arm: np.ndarray | None = None, goal_pose: np.ndarray | None = None, goal_source_path: str | Path | None = None, defaults_path: str | Path | None = None, urdf_path: str | Path | None = None, end_effector_config_path: str | Path | None = None, arm_body_demo: bool = False) -> dict[str, Any]:
    if (goal_arm is None) == (goal_pose is None):
        raise AvoidanceError("Provide exactly one goal")
    # A Cartesian goal in arm-body demo mode is permitted, but it can only ever
    # track the arm-end flange -- a confirmed URDF frame -- never the unconfirmed
    # installed-gripper TCP.  ``tracking`` below is forced to the flange for demo
    # mode, so a ``goal_pose`` here is unambiguously a flange target and stays
    # execution_authorized=False like every other demo output.
    root = Path(__file__).resolve().parents[1]
    defaults_file = Path(defaults_path or root / "configs/avoidance_defaults.json").resolve()
    defaults = read_json(defaults_file)
    scene = load_planning_scene(scene_path)
    positions, capture = load_g2_capture_state(capture_state_path)
    robot = G2RobotModel(**({"urdf_path": urdf_path} if urdf_path else {}), joint_limit_margin_rad=float(defaults["g2_planner"]["joint_limit_margin_rad"]))
    start_q = robot.configuration_from_positions(positions)
    config = Path(end_effector_config_path or root / defaults["g2_planner"]["end_effector_model_config"]).resolve()
    ee = load_end_effector_model_status(config).compatibility_report(robot)
    base = {"schema_version": 1, "created_at": dt.datetime.now(dt.timezone.utc).isoformat(), "robot_profile": "g2", "world_frame": "base_link", "active_arm": side, "execution_authorized": False, "inputs": {"scene": str(scene.source_path), "scene_sha256": scene.source_sha256, "capture_state": capture["source"], "capture_state_sha256": capture["sha256"], "defaults": str(defaults_file), "defaults_sha256": sha256_file(defaults_file)}, "robot": robot.metadata(), "end_effector_model": ee}
    if not ee["ready"] and not arm_body_demo:
        return {**base, "status": "blocked", "reason": "installed_end_effector_model_unconfirmed", "execution_blockers": ee["blockers"], "planner": {"success": False, "not_run": True}}
    tracking = f"arm_{'l' if side == 'left' else 'r'}_end_link" if arm_body_demo else ee["tcp_frames"][side]
    cluster, settings = defaults["cluster_scene_planning"], defaults["g2_planner"]
    checker = G2CollisionChecker(robot, scene, environment_inflation_m=cluster["environment_inflation_m"], required_clearance_m=cluster["future_edge_clearance_m"], end_effector_config_path=config, arm_body_demo=arm_body_demo)
    base.update({"planning_scope": checker.metadata()["planning_scope"], "collision_policy": checker.metadata(), "execution_blockers": ["Offline algorithm demo only.", "Installed gripper geometry and TCP are unconfirmed."]})
    start_report = checker.check(start_q)
    base["validation"] = {"start": start_report.to_dict()}
    if not start_report.valid:
        return {**base, "status": "rejected", "reason": "start_configuration_in_collision"}
    if goal_pose is not None:
        ik = solve_g2_arm_ik(
            robot,
            start_q,
            side,
            np.asarray(goal_pose, dtype=float),
            end_frame=tracking,
            is_valid=checker.is_valid,
            seed_count=settings["ik_seed_count"],
            position_tolerance_m=settings["ik_position_tolerance_m"],
            rotation_tolerance_deg=settings["ik_rotation_tolerance_deg"],
            random_seed=settings["random_seed"] + 10,
        )
        base["goal"] = {
            "type": "flange_pose" if arm_body_demo else "tcp_pose",
            "frame": tracking,
            "base_T_goal": np.asarray(goal_pose).tolist(),
            "ik": ik.to_dict(),
        }
        if not ik.success:
            return {**base, "status": "rejected", "reason": "ik_failed"}
        goal_arm = ik.arm_configuration
    goal_arm = np.asarray(goal_arm, dtype=float)
    goal_q = robot.with_arm_configuration(start_q, side, goal_arm)
    goal_report = checker.check(goal_q)
    base["validation"]["goal"] = goal_report.to_dict()
    if goal_pose is None:
        base["goal"] = {"type": "joint_configuration", "joint_names": list(G2_JOINT_LAYOUT[side]), "arm_joint_positions_rad": goal_arm.tolist()}
    if not goal_report.valid:
        return {**base, "status": "rejected", "reason": "goal_configuration_in_collision"}
    lower, upper = robot.arm_limits(side)
    def valid(arm: np.ndarray) -> bool: return checker.is_valid(robot.with_arm_configuration(start_q, side, arm))
    def subdivisions(a: np.ndarray, b: np.ndarray) -> int:
        qa, qb = robot.with_arm_configuration(start_q, side, a), robot.with_arm_configuration(start_q, side, b)
        displacement = np.max(np.linalg.norm(checker.collision_geometry_centers(qb) - checker.collision_geometry_centers(qa), axis=1))
        return max(1, int(np.ceil(displacement / settings["max_link_step_m"])))
    planner = RRTConnectPlanner(lower, upper, valid, extension_step_rad=settings["extension_step_rad"], edge_step_rad=settings["edge_step_rad"], max_iterations=settings["max_iterations"], timeout_s=settings["timeout_s"], goal_bias=settings["goal_bias"], smoothing_attempts=settings["smoothing_attempts"], random_seed=settings["random_seed"], edge_subdivision=subdivisions)
    result = planner.plan(robot.arm_configuration(start_q, side), goal_arm)
    base["planner"] = {**result.to_dict(), "algorithm": "bidirectional_rrt_connect"}
    if not result.success:
        return {**base, "status": "failed", "reason": f"planner_{result.reason}"}
    waypoints, poses = [], []
    for index, arm in enumerate(result.path):
        q = robot.with_arm_configuration(start_q, side, arm)
        report = checker.check(q)
        waypoints.append({"index": index, "arm_joint_positions_rad": arm.tolist(), "minimum_environment_clearance_m": report.minimum_environment_clearance_m})
        poses.append(robot.frame_pose(q, tracking).tolist())
    base["path"] = {"joint_names": list(G2_JOINT_LAYOUT[side]), "waypoints": waypoints, "tracked_frame": tracking, "base_T_tracked_frame": poses, "minimum_environment_clearance_m": min(item["minimum_environment_clearance_m"] for item in waypoints), "dense_recheck_passed": True}
    return {**base, "status": "demo_planned" if arm_body_demo else "planned", "reason": "arm_body_algorithm_demo_complete" if arm_body_demo else "offline_plan_complete"}
