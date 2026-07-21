#!/usr/bin/env python3
"""Report current/offline G1 TCP positions in ``base_link`` coordinates."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
AVOID_ROOT = SCRIPT_DIR.parent
PROJECT_ROOT = AVOID_ROOT.parent
if str(AVOID_ROOT) not in sys.path:
    sys.path.insert(0, str(AVOID_ROOT))

from avoidance.contracts import AvoidanceError, read_json, write_json
from avoidance.tcp_model import report_from_capture, report_from_live_sdk


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Read-only G1 gripper/TCP pose report in reconstructed base_link"
    )
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--capture-dir", type=Path, help="Offline reconstruction/capture dir")
    source.add_argument("--live", action="store_true", help="Read current robot SDK feedback")
    parser.add_argument("--g1-urdf", type=Path, default=Path("/home/ck/robot_test/G1.urdf"))
    parser.add_argument(
        "--tcp-calibration", type=Path, default=AVOID_ROOT / "configs/tcp_calibration.json"
    )
    parser.add_argument(
        "--defaults", type=Path, default=AVOID_ROOT / "configs/avoidance_defaults.json"
    )
    parser.add_argument("--output", type=Path, help="Optional output JSON path")
    parser.add_argument("--sdk-warmup-seconds", type=float, default=3.0)
    parser.add_argument("--max-feedback-age-seconds", type=float, default=1.0)
    args = parser.parse_args()
    try:
        defaults = read_json(args.defaults)
        if args.live:
            report = report_from_live_sdk(
                project_root=PROJECT_ROOT,
                g1_urdf=args.g1_urdf,
                tcp_calibration=args.tcp_calibration,
                defaults=defaults,
                warmup_seconds=args.sdk_warmup_seconds,
                max_feedback_age_s=args.max_feedback_age_seconds,
            )
        else:
            report = report_from_capture(
                args.capture_dir,
                g1_urdf=args.g1_urdf,
                tcp_calibration=args.tcp_calibration,
                defaults=defaults,
            )
        output = args.output
        if output is None and args.capture_dir is not None:
            output = args.capture_dir / "avoidance" / "gripper_pose_report.json"
        if output is not None:
            write_json(output, report)
        for side in ("left", "right"):
            pose = report["poses"][side]
            print(
                f"{side}: base_link TCP = "
                f"{[round(float(v), 6) for v in pose['tcp_position_m']]} m; "
                f"calibration_confirmed={pose['calibration_confirmed']}"
            )
            breakdown = pose["calibration_breakdown"]
            if breakdown["mode"] == "urdf_reference_plus_measured_correction":
                print(
                    f"  URDF reference {breakdown['urdf_reference_translation_m']} m + "
                    f"measured correction {breakdown['measured_correction_translation_m']} m"
                )
        gate = report["execution_gate"]
        print(f"execution_gate.allowed={gate['allowed']} (read-only report)")
        if output is not None:
            print(f"saved: {output.resolve()}")
        return 0
    except (AvoidanceError, OSError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
