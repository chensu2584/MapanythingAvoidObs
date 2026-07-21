#!/usr/bin/env python3
"""Build an auditable 5 cm-inflated G1 operation map."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
AVOID_ROOT = SCRIPT_DIR.parent
if str(AVOID_ROOT) not in sys.path:
    sys.path.insert(0, str(AVOID_ROOT))

from avoidance.contracts import AvoidanceError, read_json, sha256_file, write_json
from avoidance.map_io import load_voxel_map
from avoidance.operation_map import analyze_self_filter, build_and_write_operation_map
from avoidance.robot_model import UrdfRobot, representative_capture_state
from avoidance.shell_review import (
    SELECTION_SCOPE_ALL,
    build_review_contract,
    load_and_validate_review,
    review_selection_scope,
)
from avoidance.tcp_model import report_from_capture


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Create a base_link operation map with audited self filtering and inflation"
    )
    parser.add_argument("--input", type=Path, required=True, help="Output dir, voxels.npz/GLB, or views.npz")
    parser.add_argument("--output-dir", type=Path, help="Default: <capture>/avoidance")
    parser.add_argument("--capture-state", type=Path, help="Override capture_state.json location")
    parser.add_argument("--source-voxel-size", type=float, default=0.02)
    parser.add_argument("--inflation-m", type=float, help="Default from config (0.05 m)")
    parser.add_argument("--self-surface-margin-m", type=float, help="Default from config")
    parser.add_argument("--ambiguity-shell-m", type=float, help="Default from config")
    parser.add_argument(
        "--approve-self-filter",
        action="store_true",
        help=(
            "Explicitly approve removal of magenta robot-self candidates. Without this flag, "
            "a provisional unfiltered inflated map and audit preview are produced."
        ),
    )
    parser.add_argument(
        "--self-filter-review",
        type=Path,
        help=(
            "Completed hash-bound review JSON. Supports legacy yellow-shell reviews and "
            "full-scene exact manual reviews."
        ),
    )
    parser.add_argument("--g1-urdf", type=Path, default=Path("/home/ck/robot_test/G1.urdf"))
    parser.add_argument(
        "--tcp-calibration", type=Path, default=AVOID_ROOT / "configs/tcp_calibration.json"
    )
    parser.add_argument(
        "--defaults", type=Path, default=AVOID_ROOT / "configs/avoidance_defaults.json"
    )
    args = parser.parse_args()
    try:
        defaults = read_json(args.defaults)
        if defaults.get("world_frame") != "base_link" or defaults.get(
            "translation_unit"
        ) != "meter":
            raise AvoidanceError("Avoidance defaults must use base_link and meter")
        loaded = load_voxel_map(args.input, source_voxel_size=args.source_voxel_size)
        capture_dir = loaded.capture_dir
        state_path = args.capture_state or (
            capture_dir / "capture_state.json" if capture_dir is not None else None
        )
        if state_path is None or not state_path.is_file():
            raise AvoidanceError(
                "Stage 2 self filtering requires the matching capture_state.json; "
                "provide --capture-state explicitly."
            )
        output_dir = args.output_dir or state_path.parent / "avoidance"
        tcp_report = report_from_capture(
            state_path.parent,
            g1_urdf=args.g1_urdf,
            tcp_calibration=args.tcp_calibration,
            defaults=defaults,
        )
        capture_state = read_json(state_path)
        stationarity = defaults.get("stationarity_limits", {})
        joint_positions, _ = representative_capture_state(
            capture_state,
            max_arm_change_rad=float(
                stationarity.get("max_arm_component_change_rad", 0.01)
            ),
            max_head_change_deg=float(stationarity.get("max_head_component_change_deg", 0.5)),
            max_waist_pitch_change_rad=float(
                stationarity.get("max_waist_pitch_change_rad", 0.01)
            ),
            max_waist_lift_change_m=float(
                stationarity.get("max_waist_lift_change_m", 0.005)
            ),
        )
        robot = UrdfRobot(args.g1_urdf)
        self_surface_margin_m = float(
            args.self_surface_margin_m
            if args.self_surface_margin_m is not None
            else defaults.get("self_filter_surface_margin_m", 0.005)
        )
        ambiguity_shell_m = float(
            args.ambiguity_shell_m
            if args.ambiguity_shell_m is not None
            else defaults.get("self_filter_ambiguity_shell_m", 0.105)
        )
        approve_self_filter = bool(args.approve_self_filter)
        approved_yellow_mask = None
        review_provenance = None
        selection_scope = "ambiguity_shell"
        if args.self_filter_review is not None:
            review_header = read_json(args.self_filter_review)
            selection_scope = review_selection_scope(review_header)
            review_analysis = analyze_self_filter(
                loaded.voxel_map,
                robot,
                joint_positions,
                surface_margin_m=self_surface_margin_m,
                ambiguity_shell_m=ambiguity_shell_m,
            )
            contract = build_review_contract(
                loaded,
                review_analysis,
                g1_urdf=args.g1_urdf,
                capture_state=state_path,
                selection_scope=selection_scope,
            )
            review, approved_yellow_mask = load_and_validate_review(
                args.self_filter_review,
                contract,
                loaded.voxel_map,
                review_analysis,
                require_complete=True,
            )
            if (
                selection_scope != SELECTION_SCOPE_ALL
                and review.get("core_approved") is not True
            ):
                raise AvoidanceError("Completed review must explicitly approve the purple core")
            if selection_scope == SELECTION_SCOPE_ALL and args.approve_self_filter:
                raise AvoidanceError(
                    "Do not combine a full-scene manual review with --approve-self-filter; "
                    "only explicitly green voxels should be removed in manual mode"
                )
            if selection_scope != SELECTION_SCOPE_ALL:
                approve_self_filter = True
            selected_count = (
                review["selected_robot_voxel_count"]
                if selection_scope == SELECTION_SCOPE_ALL
                else review["selected_yellow_robot_voxel_count"]
            )
            review_provenance = {
                "path": str(args.self_filter_review.resolve()),
                "sha256": sha256_file(args.self_filter_review),
                "candidate_contract_sha256": review[
                    "candidate_contract_sha256"
                ],
                "review_complete": True,
                "selection_scope": selection_scope,
                "selected_robot_voxel_count": selected_count,
            }
        result = build_and_write_operation_map(
            loaded,
            output_dir,
            robot=robot,
            joint_positions=joint_positions,
            tcp_report=tcp_report,
            inflation_m=float(
                args.inflation_m
                if args.inflation_m is not None
                else defaults.get("map_inflation_m", 0.05)
            ),
            self_surface_margin_m=self_surface_margin_m,
            ambiguity_shell_m=ambiguity_shell_m,
            approve_self_filter=approve_self_filter,
            approved_yellow_mask=approved_yellow_mask,
            review_provenance=review_provenance,
            review_selection_scope=selection_scope,
            future_path_clearance_m=float(defaults.get("future_path_clearance_m", 0.02)),
            max_dense_cells=int(defaults.get("max_dense_cells", 100_000_000)),
            defaults_provenance={
                "path": str(args.defaults.resolve()),
                "sha256": sha256_file(args.defaults),
            },
        )
        pose_path = Path(output_dir) / "gripper_pose_report.json"
        write_json(pose_path, tcp_report)
        manifest = result["manifest"]
        print(f"saved: {result['npz_path']}")
        print(f"saved: {result['glb_path']}")
        print(f"saved: {result['preview_path']}")
        print(
            "self filter: "
            f"approved={manifest['self_filter']['approved']}, "
            f"manually_removed={manifest['self_filter']['removed_reviewed_robot_voxel_count']}, "
            f"candidates={manifest['self_filter']['core_candidate_voxel_count']}"
        )
        print(
            "inflation: "
            f"{manifest['inflation']['requested_radius_m']:.3f} m, "
            f"{manifest['inflation']['input_retained_voxel_count']} -> "
            f"{manifest['inflation']['output_occupied_voxel_count']} voxels"
        )
        print(f"planning_ready={manifest['planning_ready']}; execution_ready=False")
        return 0
    except (AvoidanceError, OSError, ValueError, MemoryError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
