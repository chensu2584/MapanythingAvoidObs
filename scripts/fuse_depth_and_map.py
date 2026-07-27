#!/usr/bin/env python3
"""Fuse direct-depth and MapAnything reconstructions into one planning cloud.

Both inputs are first put through the same cleanup that
``simplify_g2_snapshots.py`` applies before fitting planning primitives --
workspace (table XY) crop, gripper removal, DBSCAN denoise and table crop -- so
the fused cloud is built from denoised, range-limited geometry rather than raw
voxels.  Fusing raw clouds would carry background walls, floor and speckle into
the result and then hand them to the planner.

Depth is then the geometry truth; MapAnything only fills what the depth camera
could not see.  Gripper voxels are carved with the operator-measured box
anchored to the wrist cameras (see :mod:`avoidance.gripper_volume`) -- not the
URDF omnipicker, which is not the gripper installed on this robot.

Camera extrinsics come from the depth capture, per the operator's instruction.

Expected layout (as in ``7.24Exp``)::

    <root>/depth/<snapshot>/direct_depth_voxels.npz
    <root>/map/<snapshot>/voxels.npz
    <root>/in/<snapshot>/camera_extrinsics.json

Example::

    PYTHONPATH=Avoid python Avoid/scripts/fuse_depth_and_map.py --root 7.24Exp
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
AVOID_ROOT = SCRIPT_DIR.parent
if str(AVOID_ROOT) not in sys.path:
    sys.path.insert(0, str(AVOID_ROOT))

from avoidance.contracts import AvoidanceError, read_json, write_json  # noqa: E402
from avoidance.depth_fusion import (  # noqa: E402
    DEFAULT_SNAP_DISTANCE_M,
    fuse,
    load_voxel_cloud,
    save_fused,
)
from avoidance.occupancy_fusion import (  # noqa: E402
    DEFAULT_SURFACE_TOLERANCE_M,
    DepthView,
    fuse_with_occupancy,
)
from avoidance.voxel_cleanup import (  # noqa: E402
    DEFAULT_CLUSTER_EPS_M,
    DEFAULT_MIN_CLUSTER,
    DEFAULT_TABLE_THICKNESS_M,
    DEFAULT_TABLE_XY_BOUNDS,
    clean_cloud,
)

DEPTH_INTRINSICS = "intrinsic_head_front_depth.json"


def head_depth_view(root: Path, snapshot: str, args) -> DepthView | None:
    """Load the head depth image as a free-space evidence source."""
    import cv2

    image_path = root / args.input_dirname / snapshot / "head_depth_raw16.png"
    extrinsics_path = root / args.input_dirname / snapshot / "camera_extrinsics.json"
    intrinsics_path = Path(args.sensor_dir).expanduser() / DEPTH_INTRINSICS
    if not (image_path.is_file() and intrinsics_path.is_file()):
        return None
    raw = cv2.imread(str(image_path), cv2.IMREAD_UNCHANGED)
    if raw is None:
        return None
    depth = raw.astype(np.float32) * args.raw_depth_scale
    depth[(raw == 0) | (raw == 65535)] = 0.0          # sentinel values carry no return
    calibration = read_json(intrinsics_path)
    K = np.array([[calibration["Fx"], 0.0, calibration["Cx"]],
                  [0.0, calibration["Fy"], calibration["Cy"]],
                  [0.0, 0.0, 1.0]], dtype=float)
    extrinsics = read_json(extrinsics_path).get("extrinsics", {})
    entry = extrinsics.get("head_depth") or extrinsics.get("head_rgb")
    if not isinstance(entry, dict) or "matrix" not in entry:
        return None
    return DepthView(depth_m=depth, K=K,
                     base_T_camera=np.asarray(entry["matrix"], dtype=float),
                     max_depth_m=args.max_depth, name="head")
from avoidance.gripper_volume import (  # noqa: E402
    ANCHORS,
    DEFAULT_CENTRE_DISTANCE_M,
    DEFAULT_HEIGHT_M,
    DEFAULT_LENGTH_M,
    DEFAULT_PITCH_DEG,
    DEFAULT_WIDTH_M,
    gripper_boxes,
    remove_gripper_voxels,
)

WRIST_KEYS = ("hand_left_rgb", "hand_right_rgb")

# Provenance tinting for the viewer: truth reads green, model fill reads amber.
DEPTH_TINT = np.array([60, 200, 90], dtype=np.float64)
MAP_TINT = np.array([245, 150, 40], dtype=np.float64)


def fused_glb(result, marker_document, *, tint_strength: float,
              gripper_boxes_to_draw=()) -> "object":
    """Build the fused voxel GLB, matching the repo's viewer conventions.

    Same 180-degree X flip as ``reconstruct_depth_voxels.py`` and
    ``scene_simplify.py`` so this overlays the other derived GLBs, and the same
    robot reference markers.  Voxels keep their captured colour but are tinted
    toward green/amber by provenance, so depth truth and MapAnything fill stay
    distinguishable in the viewer.
    """
    import trimesh
    from g2_glb_markers import add_marker_geometry

    centers = result.points_m
    colors = result.colors.astype(np.float64)
    tint = np.where(result.depth_mask[:, None], DEPTH_TINT, MAP_TINT)
    blended = np.clip(colors * (1.0 - tint_strength) + tint * tint_strength, 0, 255)

    cube = trimesh.creation.box(extents=[result.voxel_size_m * 0.95] * 3)
    vertices = (centers[:, None, :] + cube.vertices[None, :, :]).reshape(-1, 3)
    faces = (cube.faces[None]
             + (np.arange(len(centers)) * len(cube.vertices))[:, None, None]).reshape(-1, 3)
    mesh = trimesh.Trimesh(
        vertices=vertices, faces=faces,
        vertex_colors=np.repeat(blended.astype(np.uint8), len(cube.vertices), axis=0),
        process=False)
    mesh.apply_transform(trimesh.transformations.rotation_matrix(np.pi, [1, 0, 0]))

    scene = trimesh.Scene()
    scene.add_geometry(mesh, geom_name="fused_voxels")
    for box in gripper_boxes_to_draw:
        # wireframe-ish translucent shell showing what was carved out as gripper
        shell = trimesh.creation.box(extents=box.size_m)
        transform = np.eye(4)
        transform[:3, :3] = box.axes
        transform[:3, 3] = box.centre_m
        shell.apply_transform(transform)
        shell.apply_transform(trimesh.transformations.rotation_matrix(np.pi, [1, 0, 0]))
        shell.visual.vertex_colors = np.array([235, 60, 60, 90], dtype=np.uint8)
        scene.add_geometry(shell, geom_name=f"gripper_removal_{box.side}")
    return add_marker_geometry(scene, marker_document)


def import_pipeline(explicit: str | None):
    """Locate MapAnythingPipeline so table detection stays a single source of truth."""
    candidates = []
    if explicit:
        candidates.append(Path(explicit))
    if os.environ.get("MAPANYTHING_PIPELINE"):
        candidates.append(Path(os.environ["MAPANYTHING_PIPELINE"]))
    candidates.append(AVOID_ROOT.parent / "MapAnythingPipeline")
    for path in candidates:
        if (path / "scene_simplify.py").is_file():
            sys.path.insert(0, str(path))
            import scene_simplify  # type: ignore
            return scene_simplify
    raise AvoidanceError(
        "cannot find scene_simplify.py; pass --pipeline or set MAPANYTHING_PIPELINE")


def head_height(extrinsics_path: Path) -> float | None:
    """Head-camera height, used to band-limit the table-height search."""
    extrinsics = read_json(extrinsics_path).get("extrinsics", {})
    entry = extrinsics.get("head_rgb")
    if isinstance(entry, dict) and "matrix" in entry:
        return float(np.asarray(entry["matrix"], dtype=float)[2, 3])
    return None


def wrist_poses(extrinsics_path: Path) -> dict[str, np.ndarray]:
    """Read wrist camera poses from the DEPTH capture's extrinsics."""
    document = read_json(extrinsics_path)
    extrinsics = document.get("extrinsics", document)
    poses = {}
    for key in WRIST_KEYS:
        entry = extrinsics.get(key)
        if isinstance(entry, dict) and "matrix" in entry:
            poses[key] = np.asarray(entry["matrix"], dtype=float)
    if not poses:
        raise AvoidanceError(f"no wrist camera extrinsics in {extrinsics_path}")
    return poses


def process(root: Path, snapshot: str, args, pipeline) -> dict:
    depth_npz = root / "depth" / snapshot / args.depth_filename
    map_npz = root / "map" / snapshot / args.map_filename
    extrinsics = root / args.input_dirname / snapshot / "camera_extrinsics.json"
    for path in (depth_npz, map_npz, extrinsics):
        if not path.is_file():
            raise AvoidanceError(f"missing input: {path}")

    gripper_kwargs = dict(anchor=args.gripper_anchor,
                          centre_distance_m=args.gripper_centre_distance,
                          length_m=args.gripper_length, width_m=args.gripper_width,
                          height_m=args.gripper_height, pitch_deg=args.gripper_pitch,
                          forward_offset_m=args.gripper_forward_offset,
                          down_offset_m=args.gripper_down_offset,
                          margin_m=args.gripper_margin)
    if args.gripper_long_axis_rotation is not None:
        gripper_kwargs["long_axis_rotation_deg"] = args.gripper_long_axis_rotation
    boxes = gripper_boxes(wrist_poses(extrinsics), **gripper_kwargs)
    depth_points, depth_colors, depth_vs, depth_frame = load_voxel_cloud(depth_npz)
    map_points, map_colors, map_vs, map_frame = load_voxel_cloud(map_npz)
    if depth_frame != map_frame:
        raise AvoidanceError(f"frame mismatch: depth={depth_frame} map={map_frame}")
    if not np.isclose(depth_vs, map_vs, rtol=1e-3):
        raise AvoidanceError(f"voxel size mismatch: depth={depth_vs} map={map_vs}")

    # Same cleanup simplify_g2_snapshots.py applies before fitting primitives, so
    # the fusion consumes denoised, range-limited geometry rather than raw voxels.
    cleanup_kwargs = dict(
        gripper_boxes=boxes, table_xy_bounds=args.table_xy_bounds,
        cluster_eps_m=args.cluster_eps, min_cluster=args.min_cluster,
        table_thickness_m=args.table_thickness, head_z_m=head_height(extrinsics),
        support_surface_fn=pipeline.find_support_surface)
    depth_clean = clean_cloud(depth_points, depth_colors, label="depth", **cleanup_kwargs)
    map_clean = clean_cloud(map_points, map_colors, label="map", **cleanup_kwargs)
    for cleaned in (depth_clean, map_clean):
        if not len(cleaned.points_m):
            raise AvoidanceError(
                f"cleanup removed every voxel from the {cleaned.report['label']} cloud")

    shared = {"world_frame": depth_frame, "depth_source": str(depth_npz.resolve()),
              "map_source": str(map_npz.resolve()), "inputs_were_cleaned": True}
    if args.method == "occupancy":
        view = head_depth_view(root, snapshot, args)
        if view is None:
            raise AvoidanceError(
                f"--method occupancy needs {snapshot}'s head_depth_raw16.png and "
                f"{DEPTH_INTRINSICS} under --sensor-dir")
        result = fuse_with_occupancy(
            depth_clean.points_m, depth_clean.colors,
            map_clean.points_m, map_clean.colors, views=[view],
            voxel_size_m=float(depth_vs), snap_distance_m=args.snap_distance,
            surface_tolerance_m=args.surface_tolerance, metadata=shared)
    else:
        result = fuse(depth_clean.points_m, depth_clean.colors,
                      map_clean.points_m, map_clean.colors,
                      voxel_size_m=float(depth_vs), snap_distance_m=args.snap_distance,
                      metadata=shared)
    out_dir = (args.out_root or root / "fused") / snapshot
    fused_path = save_fused(result, out_dir / "fused_voxels.npz")

    glb_path = None
    glb_error = None
    if not args.no_glb:
        try:
            from g2_glb_markers import build_marker_document
            marker_kwargs = {"joint_state_path": extrinsics}
            if args.urdf:
                marker_kwargs["urdf_path"] = Path(args.urdf).expanduser().resolve()
            marker_document = build_marker_document(extrinsics, **marker_kwargs)
            scene = fused_glb(result, marker_document, tint_strength=args.tint_strength,
                              gripper_boxes_to_draw=() if args.no_gripper_shell else boxes)
            glb_path = out_dir / "fused_voxels.glb"
            scene.export(glb_path)
        except Exception as exc:                     # markers are optional decoration
            glb_error = f"{type(exc).__name__}: {exc}"

    report = dict(result.report)
    report.update({
        "snapshot": snapshot,
        "extrinsics_source_file": str(extrinsics.resolve()),
        "cleanup": {"depth": depth_clean.report, "map": map_clean.report,
                    "note": "same workspace crop / gripper removal / denoise / table crop "
                            "as simplify_g2_snapshots.py, applied before fusion"},
        "gripper_removal": {
            "anchor": args.gripper_anchor,
            "centre_distance_m": args.gripper_centre_distance,
            "size_m": [args.gripper_length, args.gripper_width, args.gripper_height],
            "pitch_deg": args.gripper_pitch,
            "margin_m": args.gripper_margin,
            "boxes": [box.to_dict() for box in boxes],
            "note": "operator-measured proxy anchored to the wrist camera; not a confirmed TCP",
        },
        "outputs": {
            "fused_voxels": str(fused_path.resolve()),
            "fused_glb": str(glb_path.resolve()) if glb_path else None,
            "glb_error": glb_error,
        },
    })
    write_json(out_dir / "fusion_report.json", report)
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--root", type=Path, required=True,
                        help="folder holding depth/ map/ in/ subdirectories")
    parser.add_argument("--snapshot", action="append", dest="snapshots",
                        help="snapshot name; repeatable (default: all under depth/)")
    parser.add_argument("--out-root", type=Path, default=None,
                        help="default: <root>/fused")
    parser.add_argument("--depth-filename", default="direct_depth_voxels.npz")
    parser.add_argument("--map-filename", default="voxels.npz")
    parser.add_argument("--input-dirname", default="in")
    parser.add_argument("--method", choices=("occupancy", "surface"), default="occupancy",
                        help="occupancy: use depth free-space evidence and grade confidence "
                             "(default); surface: the older surface-distance rule only")
    parser.add_argument("--surface-tolerance", type=float, default=DEFAULT_SURFACE_TOLERANCE_M,
                        help="band around the measured surface counted as occupied, metres")
    parser.add_argument("--sensor-dir", default="G2_parameters/sensor",
                        help="folder holding intrinsic_head_front_depth.json")
    parser.add_argument("--raw-depth-scale", type=float, default=0.001,
                        help="uint16 depth -> metres (G2 stores millimetres)")
    parser.add_argument("--max-depth", type=float, default=3.0,
                        help="ignore depth returns beyond this, metres")
    parser.add_argument("--snap-distance", type=float, default=DEFAULT_SNAP_DISTANCE_M,
                        help="map voxels closer than this to depth are dropped as duplicates")
    parser.add_argument("--pipeline", default=None,
                        help="MapAnythingPipeline checkout (else sibling / MAPANYTHING_PIPELINE)")
    parser.add_argument("--table-xy-bounds", type=float, nargs=4,
                        metavar=("X_MIN", "X_MAX", "Y_MIN", "Y_MAX"),
                        default=list(DEFAULT_TABLE_XY_BOUNDS),
                        help="workspace rectangle kept before fusion (default: measured 3box table)")
    parser.add_argument("--no-workspace-crop", action="store_true",
                        help="keep off-table background (not recommended for planning)")
    parser.add_argument("--cluster-eps", type=float, default=DEFAULT_CLUSTER_EPS_M,
                        help="DBSCAN neighbourhood radius, metres")
    parser.add_argument("--min-cluster", type=int, default=DEFAULT_MIN_CLUSTER,
                        help="clusters smaller than this are treated as noise")
    parser.add_argument("--table-thickness", type=float, default=DEFAULT_TABLE_THICKNESS_M,
                        help="metres below the table top kept before cropping")
    parser.add_argument("--gripper-anchor", choices=ANCHORS, default="optical")
    parser.add_argument("--gripper-centre-distance", type=float,
                        default=DEFAULT_CENTRE_DISTANCE_M,
                        help="camera centre -> gripper box CENTRE, metres")
    parser.add_argument("--gripper-length", type=float, default=DEFAULT_LENGTH_M)
    parser.add_argument("--gripper-width", type=float, default=DEFAULT_WIDTH_M)
    parser.add_argument("--gripper-height", type=float, default=DEFAULT_HEIGHT_M)
    parser.add_argument("--gripper-pitch", type=float, default=DEFAULT_PITCH_DEG)
    parser.add_argument("--gripper-forward-offset", type=float, default=0.0,
                        help="extra shift along the wrist camera's optical axis, metres")
    parser.add_argument("--gripper-down-offset", type=float, default=0.0,
                        help="extra shift straight down in base_link, metres")
    parser.add_argument("--gripper-long-axis-rotation", type=float, default=None,
                        help="quarter-turn of the long edge toward the table, degrees (default 90)")
    parser.add_argument("--gripper-margin", type=float, default=0.02,
                        help="grow the gripper box on every side, metres")
    parser.add_argument("--no-glb", action="store_true", help="skip the viewer GLB")
    parser.add_argument("--no-gripper-shell", action="store_true",
                        help="omit the translucent gripper-removal boxes from the GLB")
    parser.add_argument("--urdf", default=None,
                        help="G2 URDF for the flange reference markers (default: repo layout)")
    parser.add_argument("--tint-strength", type=float, default=0.55,
                        help="0=captured colour only, 1=pure provenance colour")
    args = parser.parse_args()
    if not 0.0 <= args.tint_strength <= 1.0:
        print("ERROR: --tint-strength must be within [0, 1]", file=sys.stderr)
        return 2
    args.table_xy_bounds = None if args.no_workspace_crop else tuple(args.table_xy_bounds)
    try:
        root = args.root.expanduser().resolve()
        pipeline = import_pipeline(args.pipeline)
        snapshots = args.snapshots or sorted(
            p.name for p in (root / "depth").iterdir()
            if p.is_dir() and (p / args.depth_filename).is_file())
        if not snapshots:
            raise AvoidanceError(f"no snapshots with {args.depth_filename} under {root}/depth")
        for snapshot in snapshots:
            report = process(root, snapshot, args, pipeline)
            depth_stages = report["cleanup"]["depth"]["stages"]
            map_stages = report["cleanup"]["map"]["stages"]
            print(f"{snapshot}:")
            print(f"    cleaned depth {depth_stages['input']} -> {report['depth_voxels']}, "
                  f"map {map_stages['input']} -> {report['map_voxels']}")
            if args.method == "occupancy":
                print(f"    fused {report['fused_voxels']} "
                      f"(fill: occluded {report['map_admitted_occluded']} + "
                      f"unseen {report['map_admitted_unseen']}, "
                      f"snapped {report['map_snapped_to_depth']}, "
                      f"REJECTED in free space {report['map_rejected_in_observed_free_space']})")
            else:
                print(f"    fused {report['fused_voxels']} "
                      f"(map fill {report['map_voxels_admitted_as_fill']}, "
                      f"snapped {report['map_voxels_snapped_to_depth']})")
            if report["outputs"]["fused_glb"]:
                print(f"    glb: {report['outputs']['fused_glb']}")
            elif report["outputs"]["glb_error"]:
                print(f"    glb skipped: {report['outputs']['glb_error']}", file=sys.stderr)
        return 0
    except (AvoidanceError, OSError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
