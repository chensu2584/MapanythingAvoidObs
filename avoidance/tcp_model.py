"""TCP calibration semantics and offline gripper-pose reports."""

from __future__ import annotations

import time
from pathlib import Path
from typing import Any

import numpy as np

from .contracts import AvoidanceError, read_json, sha256_file
from .robot_model import (
    SIDE_FRAMES,
    UrdfRobot,
    representative_capture_state,
    transform_from_xyz_rpy,
    wbc_fk_crosscheck,
)


def _three(value: Any, *, label: str) -> np.ndarray:
    result = np.asarray(value, dtype=np.float64)
    if result.shape != (3,) or not np.isfinite(result).all():
        raise AvoidanceError(f"{label} must contain three finite values")
    return result


def load_tcp_calibration(path: str | Path) -> dict[str, Any]:
    path = Path(path).resolve()
    config = read_json(path)
    if config.get("schema_version") != 1:
        raise AvoidanceError("Only TCP calibration schema_version=1 is supported")
    if config.get("world_frame") != "hand_mount_local":
        raise AvoidanceError("TCP calibration world_frame must be hand_mount_local")
    if config.get("matrix_direction") != "hand_T_tcp":
        raise AvoidanceError("TCP calibration matrix_direction must be hand_T_tcp")
    if config.get("translation_unit") != "meter":
        raise AvoidanceError("TCP calibration translation_unit must be meter")
    for side in ("left", "right"):
        if not isinstance(config.get("sides", {}).get(side), dict):
            raise AvoidanceError(f"TCP calibration is missing sides.{side}")
        expected = SIDE_FRAMES[side]["hand"]
        if config["sides"][side].get("mount_frame") != expected:
            raise AvoidanceError(f"TCP {side} mount_frame must be {expected}")
    config["_path"] = str(path)
    config["_sha256"] = sha256_file(path)
    return config


def hand_t_tcp(side_config: dict[str, Any]) -> tuple[np.ndarray, dict[str, Any]]:
    mode = side_config.get("mode")
    if mode == "urdf_reference_plus_measured_correction":
        reference_xyz = _three(
            side_config.get("urdf_reference_translation_m"),
            label="urdf_reference_translation_m",
        )
        reference_rpy = _three(
            side_config.get("urdf_reference_rpy_rad", [0, 0, 0]),
            label="urdf_reference_rpy_rad",
        )
        correction_xyz = _three(
            side_config.get("measured_correction_translation_m"),
            label="measured_correction_translation_m",
        )
        correction_rpy = _three(
            side_config.get("measured_correction_rpy_rad", [0, 0, 0]),
            label="measured_correction_rpy_rad",
        )
        reference = transform_from_xyz_rpy(reference_xyz, reference_rpy)
        correction = transform_from_xyz_rpy(correction_xyz, correction_rpy)
        matrix = reference @ correction
        breakdown = {
            "mode": mode,
            "urdf_reference_translation_m": reference_xyz.tolist(),
            "urdf_reference_rpy_rad": reference_rpy.tolist(),
            "measured_correction_translation_m": correction_xyz.tolist(),
            "measured_correction_rpy_rad": correction_rpy.tolist(),
            "composition": "hand_T_tcp = hand_T_urdf_reference @ reference_T_measured_correction",
        }
    elif mode == "absolute_measured_hand_T_tcp":
        translation = _three(
            side_config.get("measured_translation_m"), label="measured_translation_m"
        )
        rpy = _three(side_config.get("measured_rpy_rad", [0, 0, 0]), label="measured_rpy_rad")
        matrix = transform_from_xyz_rpy(translation, rpy)
        breakdown = {
            "mode": mode,
            "measured_translation_m": translation.tolist(),
            "measured_rpy_rad": rpy.tolist(),
            "composition": "hand_T_tcp = absolute measured transform",
        }
    else:
        raise AvoidanceError(
            "TCP mode must be urdf_reference_plus_measured_correction or "
            "absolute_measured_hand_T_tcp"
        )
    return matrix, breakdown


def build_pose_report(
    robot: UrdfRobot,
    joint_positions: dict[str, float],
    calibration: dict[str, Any],
    *,
    state_report: dict[str, Any],
    wbc_crosscheck: dict[str, Any] | None,
    state_source: dict[str, Any],
) -> dict[str, Any]:
    poses: dict[str, Any] = {}
    all_confirmed = True
    for side in ("left", "right"):
        side_calibration = calibration["sides"][side]
        base_t_hand = robot.base_to_frame(SIDE_FRAMES[side]["hand"], joint_positions)
        local_tcp, breakdown = hand_t_tcp(side_calibration)
        base_t_tcp = base_t_hand @ local_tcp
        confirmed = side_calibration.get("confirmed") is True
        all_confirmed &= confirmed
        poses[side] = {
            "mount_frame": SIDE_FRAMES[side]["hand"],
            "tcp_frame": side_calibration.get("tcp_frame", f"gripper_{side}_tcp"),
            "base_T_hand": base_t_hand.tolist(),
            "hand_T_tcp": local_tcp.tolist(),
            "base_T_tcp": base_t_tcp.tolist(),
            "tcp_position_m": base_t_tcp[:3, 3].tolist(),
            "calibration_confirmed": confirmed,
            "calibration_breakdown": breakdown,
            "measurement_note": side_calibration.get("measurement_note"),
        }
    crosscheck_passed = bool(wbc_crosscheck and wbc_crosscheck.get("passed"))
    stationary = state_report.get("stationary") is True
    return {
        "schema_version": 1,
        "generated_at_unix_ns": time.time_ns(),
        "world_frame": "base_link",
        "matrix_direction": "base_T_frame",
        "translation_unit": "meter",
        "poses": poses,
        "state": state_report,
        "wbc_fk_crosscheck": wbc_crosscheck,
        "calibration": {
            "path": calibration["_path"],
            "sha256": calibration["_sha256"],
            "all_sides_confirmed": all_confirmed,
        },
        "source": state_source,
        "execution_gate": {
            "allowed": all_confirmed and stationary and crosscheck_passed,
            "conditions": {
                "tcp_calibration_confirmed": all_confirmed,
                "robot_state_stationary_or_fresh": stationary,
                "wbc_fk_crosscheck_passed": crosscheck_passed,
            },
            "note": "This stage is read-only; no motion command is implemented here.",
        },
    }


def report_from_capture(
    capture_dir: str | Path,
    *,
    g1_urdf: str | Path,
    tcp_calibration: str | Path,
    defaults: dict[str, Any] | None = None,
) -> dict[str, Any]:
    capture_dir = Path(capture_dir).resolve()
    state_path = capture_dir / "capture_state.json"
    capture_state = read_json(state_path)
    defaults = defaults or {}
    stationarity = defaults.get("stationarity_limits", {})
    positions, state_report = representative_capture_state(
        capture_state,
        max_arm_change_rad=float(stationarity.get("max_arm_component_change_rad", 0.01)),
        max_head_change_deg=float(stationarity.get("max_head_component_change_deg", 0.5)),
        max_waist_pitch_change_rad=float(
            stationarity.get("max_waist_pitch_change_rad", 0.01)
        ),
        max_waist_lift_change_m=float(stationarity.get("max_waist_lift_change_m", 0.005)),
    )
    robot = UrdfRobot(g1_urdf)
    wbc_limits = defaults.get("wbc_fk_limits", {})
    crosscheck = wbc_fk_crosscheck(
        robot,
        positions,
        capture_state,
        max_translation_error_m=float(wbc_limits.get("max_translation_error_m", 5e-4)),
        max_rotation_error_deg=float(wbc_limits.get("max_rotation_error_deg", 0.05)),
    )
    calibration = load_tcp_calibration(tcp_calibration)
    report = build_pose_report(
        robot,
        positions,
        calibration,
        state_report=state_report,
        wbc_crosscheck=crosscheck,
        state_source={
            "mode": "offline_capture",
            "capture_dir": str(capture_dir),
            "capture_state": {"path": str(state_path), "sha256": sha256_file(state_path)},
            "g1_urdf": {
                "path": str(Path(g1_urdf).resolve()),
                "sha256": sha256_file(g1_urdf),
            },
        },
    )
    existing_path = capture_dir / "gripper_poses_base_link.json"
    if existing_path.is_file():
        existing = read_json(existing_path)
        deltas = {}
        for side in ("left", "right"):
            previous = np.asarray(existing.get("poses", {}).get(side, {}).get("position_m"))
            current = np.asarray(report["poses"][side]["tcp_position_m"])
            if previous.shape == (3,) and np.isfinite(previous).all():
                deltas[side] = float(np.linalg.norm(previous - current))
        report["existing_gripper_overlay_crosscheck"] = {
            "path": str(existing_path),
            "sha256": sha256_file(existing_path),
            "tcp_position_delta_m": deltas,
        }
    return report


def report_from_live_sdk(
    *,
    project_root: str | Path,
    g1_urdf: str | Path,
    tcp_calibration: str | Path,
    defaults: dict[str, Any] | None = None,
    warmup_seconds: float = 3.0,
    max_feedback_age_s: float = 1.0,
) -> dict[str, Any]:
    from .sdk_bridge import read_live_snapshot

    defaults = defaults or {}
    snapshot = read_live_snapshot(
        project_root=project_root,
        warmup_seconds=warmup_seconds,
        max_feedback_age_s=max_feedback_age_s,
    )
    from .robot_model import normalize_joint_state

    positions, normalized = normalize_joint_state(snapshot["state"])
    robot = UrdfRobot(g1_urdf)
    synthetic_capture = {
        "wbc_link7_capture": {
            "world_frame": "base_link",
            "pose_direction": "base_T_frame",
            "views": {"live": {"frames": snapshot["wbc"]["frames"]}},
        }
    }
    wbc_limits = defaults.get("wbc_fk_limits", {})
    crosscheck = wbc_fk_crosscheck(
        robot,
        positions,
        synthetic_capture,
        max_translation_error_m=float(wbc_limits.get("max_translation_error_m", 5e-4)),
        max_rotation_error_deg=float(wbc_limits.get("max_rotation_error_deg", 0.05)),
    )
    crosscheck["geometric_passed"] = bool(crosscheck.get("passed"))
    crosscheck["status_timestamp_valid"] = snapshot["wbc"]["status_timestamp_valid"]
    crosscheck["status_fresh"] = snapshot["wbc"]["status_fresh"]
    crosscheck["passed"] = bool(
        crosscheck["geometric_passed"]
        and snapshot["wbc"]["status_fresh"]
        and snapshot["wbc"].get("status_error") in (None, 0, "", {})
    )
    state_report = {
        "mode": "live_sdk_feedback",
        "stationary": bool(snapshot["feedback"]["joint_feedback_fresh"]),
        "fresh": bool(snapshot["feedback"]["joint_feedback_fresh"]),
        "state": normalized,
        "feedback": snapshot["feedback"],
    }
    calibration = load_tcp_calibration(tcp_calibration)
    return build_pose_report(
        robot,
        positions,
        calibration,
        state_report=state_report,
        wbc_crosscheck=crosscheck,
        state_source={
            "mode": "live_read_only_sdk",
            "wbc": snapshot["wbc"],
            "g1_urdf": {
                "path": str(Path(g1_urdf).resolve()),
                "sha256": sha256_file(g1_urdf),
            },
        },
    )
