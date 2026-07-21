"""Read-only bridge to current G1 joint and WBC feedback."""

from __future__ import annotations

import sys
import time
from pathlib import Path
from typing import Any

import numpy as np

from .contracts import AvoidanceError


def _finite(values: Any, length: int, label: str) -> list[float]:
    result = np.asarray(values, dtype=np.float64)
    if result.shape != (length,) or not np.isfinite(result).all():
        raise AvoidanceError(f"Expected {length} finite {label} values, got {values!r}")
    return result.tolist()


def read_live_snapshot(
    *,
    project_root: str | Path,
    warmup_seconds: float = 3.0,
    max_feedback_age_s: float = 1.0,
) -> dict[str, Any]:
    """Read one snapshot without creating a camera or issuing motion commands."""
    project_root = Path(project_root).resolve()
    capture_scripts = project_root / "capture" / "scripts"
    if str(capture_scripts) not in sys.path:
        sys.path.insert(0, str(capture_scripts))
    try:
        from a2d_sdk.robot import RobotController, RobotDds
        from g1_capture_gui import normalize_wbc_link7_frame, robust_robot_dds_class
    except ImportError as exc:
        raise AvoidanceError(
            "G1 SDK is unavailable. Run the live report from an initialized robot shell, "
            "for example after sourcing robot_test/env.sh in the robot conda environment."
        ) from exc

    robot = robust_robot_dds_class(RobotDds)()
    controller = RobotController()
    time.sleep(max(0.0, float(warmup_seconds)))
    arm, arm_timestamp_ns = robot.arm_joint_states()
    head, head_timestamp_ns = robot.head_joint_states()
    waist, waist_timestamp_ns = robot.waist_joint_states()
    captured_at_ns = time.time_ns()
    timestamps = {
        "arm": int(arm_timestamp_ns),
        "head": int(head_timestamp_ns),
        "waist": int(waist_timestamp_ns),
    }
    ages_s = {
        key: abs(captured_at_ns - value) / 1e9 if value > 0 else None
        for key, value in timestamps.items()
    }
    feedback_fresh = all(
        value is not None and value <= float(max_feedback_age_s) for value in ages_s.values()
    )
    status = controller.get_motion_status()
    if not isinstance(status, dict):
        raise AvoidanceError("RobotController returned no WBC motion status")
    frames = status.get("frames")
    if not isinstance(frames, dict):
        raise AvoidanceError("WBC motion status has no frames object")
    normalized_frames = {
        name: normalize_wbc_link7_frame(name, frames.get(name))
        for name in ("arm_left_link7", "arm_right_link7")
    }
    status_timestamp_ns = int(status.get("timestamp", 0) or 0)
    wbc_age_s = (
        abs(captured_at_ns - status_timestamp_ns) / 1e9 if status_timestamp_ns > 0 else None
    )
    wbc_timestamp_fresh = wbc_age_s is not None and wbc_age_s <= float(max_feedback_age_s)
    return {
        "state": {
            "arm_joint_states": _finite(arm, 14, "arm"),
            "head_joint_states": _finite(head, 2, "head"),
            "waist_joint_states": _finite(waist, 2, "waist"),
            "units": {
                "arm": "rad",
                "head": "deg",
                "waist_pitch": "rad",
                "waist_lift": "m",
            },
        },
        "feedback": {
            "captured_at_system_ns": captured_at_ns,
            "state_timestamps_ns": timestamps,
            "state_ages_s": ages_s,
            "max_feedback_age_s": float(max_feedback_age_s),
            "joint_feedback_fresh": feedback_fresh,
        },
        "wbc": {
            "status_timestamp_ns": status_timestamp_ns,
            "status_timestamp_valid": status_timestamp_ns > 0,
            "status_age_s": wbc_age_s,
            "status_fresh": wbc_timestamp_fresh,
            "status_mode": status.get("mode"),
            "status_error": status.get("error"),
            "frames": normalized_frames,
        },
    }
