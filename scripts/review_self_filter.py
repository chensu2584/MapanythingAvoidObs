#!/usr/bin/env python3
"""Interactively mark exact robot-self voxels from a candidate shell or full scene."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
AVOID_ROOT = SCRIPT_DIR.parent
if str(AVOID_ROOT) not in sys.path:
    sys.path.insert(0, str(AVOID_ROOT))

from avoidance.contracts import AvoidanceError, read_json
from avoidance.map_io import load_voxel_map
from avoidance.operation_map import analyze_self_filter, export_exact_review_preview
from avoidance.robot_model import UrdfRobot, representative_capture_state
from avoidance.shell_review import (
    SELECTION_SCOPE_ALL,
    SELECTION_SCOPE_AMBIGUITY,
    build_review_contract,
    component_summary,
    load_and_validate_review,
    new_review,
    update_review,
    validate_review,
    write_review,
)


def pick_source_points(
    voxel_map,
    analysis,
    selected_mask: np.ndarray,
    *,
    mode: str,
    brush_radius_m: float,
    selection_scope: str,
) -> tuple[np.ndarray, dict[str, int]]:
    try:
        import open3d as o3d
        from scipy.spatial import cKDTree
    except ImportError as exc:
        raise AvoidanceError("Open3D and SciPy are required for the review GUI") from exc

    points = voxel_map.centers
    if voxel_map.colors is not None:
        source_colors = np.asarray(voxel_map.colors, dtype=np.float64) / 255.0
        colors = 0.15 + 0.30 * source_colors
    else:
        colors = np.tile([0.28, 0.28, 0.28], (len(points), 1))
    colors[analysis.core_candidate_mask] = [1.0, 0.0, 0.85]
    colors[analysis.ambiguity_shell_mask] = [1.0, 0.72, 0.0]
    colors[selected_mask] = [0.0, 1.0, 0.25]
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(points)
    cloud.colors = o3d.utility.Vector3dVector(colors)

    all_occupied = selection_scope == SELECTION_SCOPE_ALL
    print("\nOpen3D 审核操作：")
    if all_occupied:
        print("  旋转/缩放到合适视角；按住 Shift + 左键选择任意可见体素种子点。")
    else:
        print("  旋转/缩放到合适视角；按住 Shift + 左键选择黄色种子点。")
    print("  Shift + 右键可撤销最近一次点选；关闭窗口后才会计算笔刷范围。")
    if all_occupied:
        print("  紫色、黄色、灰色均可选择；绿色表示已人工标为机器人自身。")
    else:
        print("  紫色、灰色以及与当前模式不符的点选会被忽略。")
    print(
        f"  mode={mode}, selection_scope={selection_scope}, "
        f"brush_radius={brush_radius_m:.4f} m"
    )
    visualizer = o3d.visualization.VisualizerWithEditing(
        float(voxel_map.voxel_size), False, str(Path.cwd())
    )
    if not visualizer.create_window(
        window_name=(
            "G1 manual self-filter | magenta=core yellow=shell green=selected"
        ),
        width=1500,
        height=950,
    ):
        raise AvoidanceError("Cannot create Open3D window; check DISPLAY/desktop session")
    visualizer.add_geometry(cloud)
    render = visualizer.get_render_option()
    render.point_size = 7.0
    render.background_color = np.asarray([0.025, 0.025, 0.035])
    visualizer.run()
    picked = np.asarray(visualizer.get_picked_points(), dtype=np.int64)
    visualizer.destroy_window()

    selection_domain = (
        np.ones(len(points), dtype=bool)
        if all_occupied
        else analysis.ambiguity_shell_mask
    )
    if mode == "add":
        eligible = selection_domain & ~selected_mask
    else:
        eligible = selection_domain & selected_mask
    valid_picks = picked[(picked >= 0) & (picked < len(points))]
    valid_picks = valid_picks[eligible[valid_picks]] if len(valid_picks) else valid_picks
    changed = np.zeros(len(points), dtype=bool)
    candidate_ids = np.flatnonzero(selection_domain)
    if len(valid_picks) and len(candidate_ids):
        if brush_radius_m <= 0.0:
            changed[valid_picks] = True
        else:
            tree = cKDTree(points[candidate_ids])
            neighborhoods = tree.query_ball_point(points[valid_picks], r=brush_radius_m)
            local_ids = sorted({item for group in neighborhoods for item in group})
            changed[candidate_ids[np.asarray(local_ids, dtype=np.int64)]] = True
        changed &= eligible
    result = selected_mask.copy()
    if mode == "add":
        result |= changed
    else:
        result &= ~changed
    return result, {
        "raw_picked_point_count": int(len(picked)),
        "eligible_seed_count": int(len(valid_picks)),
        "changed_robot_voxel_count": int(np.count_nonzero(changed)),
    }


def confirmed(prompt: str, assume_yes: bool) -> bool:
    if assume_yes:
        return True
    try:
        return input(prompt).strip().lower() in {"y", "yes"}
    except EOFError:
        return False


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Exact self-filter review with candidate-shell or full-scene point picking"
    )
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--capture-state", type=Path)
    parser.add_argument("--review-file", type=Path)
    parser.add_argument("--mode", choices=("add", "remove"), default="add")
    parser.add_argument(
        "--selection-scope",
        choices=("ambiguity-shell", "all-occupied"),
        default="ambiguity-shell",
        help=(
            "ambiguity-shell keeps the legacy yellow-only workflow; all-occupied "
            "allows exact manual selection from every occupied source voxel"
        ),
    )
    parser.add_argument("--brush-radius-m", type=float, default=0.02)
    parser.add_argument("--approve-core", action="store_true")
    parser.add_argument("--mark-complete", action="store_true")
    parser.add_argument("--operator-note", default=None)
    parser.add_argument("--no-gui", action="store_true")
    parser.add_argument(
        "--view-only",
        action="store_true",
        help="Open the colored review without changing or saving any selection",
    )
    parser.add_argument("--yes", action="store_true", help="Save without terminal confirmation")
    parser.add_argument("--g1-urdf", type=Path, default=Path("/home/ck/robot_test/G1.urdf"))
    parser.add_argument(
        "--defaults", type=Path, default=AVOID_ROOT / "configs/avoidance_defaults.json"
    )
    args = parser.parse_args()
    try:
        selection_scope = args.selection_scope.replace("-", "_")
        if args.brush_radius_m < 0.0:
            raise AvoidanceError("--brush-radius-m cannot be negative")
        if args.view_only and args.no_gui:
            raise AvoidanceError("--view-only requires the GUI; do not combine it with --no-gui")
        defaults = read_json(args.defaults)
        loaded = load_voxel_map(args.input)
        state_path = args.capture_state or loaded.capture_dir / "capture_state.json"
        if not state_path.is_file():
            raise AvoidanceError("Matching capture_state.json is required")
        capture_state = read_json(state_path)
        stationarity = defaults.get("stationarity_limits", {})
        positions, state_report = representative_capture_state(
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
        if not state_report.get("stationary"):
            raise AvoidanceError("Capture is not stationary enough for a self-filter review")
        robot = UrdfRobot(args.g1_urdf)
        analysis = analyze_self_filter(
            loaded.voxel_map,
            robot,
            positions,
            surface_margin_m=float(defaults.get("self_filter_surface_margin_m", 0.005)),
            ambiguity_shell_m=float(
                defaults.get("self_filter_ambiguity_shell_m", 0.105)
            ),
        )
        components = []
        if selection_scope == SELECTION_SCOPE_AMBIGUITY:
            _, components = component_summary(loaded.voxel_map, analysis)
        contract = build_review_contract(
            loaded,
            analysis,
            g1_urdf=args.g1_urdf,
            capture_state=state_path,
            selection_scope=selection_scope,
        )
        default_review_name = (
            "manual_self_filter_review.json"
            if selection_scope == SELECTION_SCOPE_ALL
            else "self_filter_review.json"
        )
        review_path = args.review_file or loaded.capture_dir / "avoidance" / default_review_name
        if review_path.is_file():
            review, selected_mask = load_and_validate_review(
                review_path,
                contract,
                loaded.voxel_map,
                analysis,
                require_complete=False,
            )
        else:
            review = new_review(contract, components=components)
            selected_mask = np.zeros(len(loaded.voxel_map.indices), dtype=bool)

        pick_report = {
            "raw_picked_point_count": 0,
            "eligible_seed_count": 0,
            "changed_robot_voxel_count": 0,
        }
        before_count = int(np.count_nonzero(selected_mask))
        if not args.no_gui:
            selected_mask, pick_report = pick_source_points(
                loaded.voxel_map,
                analysis,
                selected_mask,
                mode=args.mode,
                brush_radius_m=float(args.brush_radius_m),
                selection_scope=selection_scope,
            )
        if args.view_only:
            print(
                "View-only check finished; review JSON and GLB were not modified. "
                f"saved_green={before_count}, review_complete={review.get('review_complete')}"
            )
            return 0
        after_count = int(np.count_nonzero(selected_mask))
        core_approved = (
            True
            if args.approve_core and selection_scope == SELECTION_SCOPE_AMBIGUITY
            else None
        )
        if (
            selection_scope == SELECTION_SCOPE_AMBIGUITY
            and args.mark_complete
            and not (core_approved or review.get("core_approved"))
        ):
            raise AvoidanceError("A complete review requires explicit core approval")
        updated = update_review(
            review,
            loaded.voxel_map,
            selected_mask,
            core_approved=core_approved,
            mark_complete=args.mark_complete,
            operator_note=args.operator_note,
        )
        validate_review(
            updated,
            contract,
            loaded.voxel_map,
            analysis,
            require_complete=args.mark_complete,
        )
        selectable_count = (
            len(loaded.voxel_map.indices)
            if selection_scope == SELECTION_SCOPE_ALL
            else analysis.report["ambiguity_shell_voxel_count"]
        )
        print(f"robot voxels selected: {before_count} -> {after_count} / {selectable_count}")
        print(
            f"picked={pick_report['raw_picked_point_count']}, "
            f"eligible_seeds={pick_report['eligible_seed_count']}, "
            f"changed={pick_report['changed_robot_voxel_count']}"
        )
        print(
            f"selection_scope={selection_scope}, "
            f"core_approved={updated.get('core_approved', 'not-used')}, "
            f"review_complete={updated['review_complete']}"
        )
        # ``--mark-complete`` is itself an explicit write instruction.  In a
        # ``conda run``/CI-style no-TTY invocation, asking for a second stdin
        # confirmation only causes an EOF and loses the requested update.
        explicit_noninteractive_save = bool(
            args.yes or (args.no_gui and args.mark_complete)
        )
        if not confirmed(
            f"Save review to {review_path.resolve()}? [y/N] ",
            explicit_noninteractive_save,
        ):
            print("Review not saved.")
            return 1
        review_path.parent.mkdir(parents=True, exist_ok=True)
        preview_path = review_path.with_name(
            "manual_self_filter_review_base_link.glb"
            if selection_scope == SELECTION_SCOPE_ALL
            else "self_filter_review_base_link.glb"
        )
        preview = export_exact_review_preview(
            preview_path,
            loaded.voxel_map,
            analysis,
            selected_mask,
            selection_scope=selection_scope,
        )
        write_review(review_path, updated)
        print(f"saved atomically: {review_path.resolve()}")
        print(
            f"saved atomically: {preview_path.resolve()} "
            f"(green={preview['selected_robot_voxel_count']})"
        )
        return 0
    except (AvoidanceError, OSError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
