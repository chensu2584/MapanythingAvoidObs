#!/usr/bin/env python3
"""Adapt a G2 reconstruction into an Avoid planning scene (plan section 11, M1).

``MapAnythingPipeline/scene_simplify.py`` cleans a ``voxels.npz`` (spatial
gripper removal, DBSCAN denoise, Z-histogram table detection, per-cluster AABB
boxes) but writes its own ``obstacles.json`` schema.  ``avoidance.planning_scene``
needs each primitive tagged with ``primitive`` (box/cylinder) and ``role``
(support/object).  This script reuses the pipeline's tested geometry and adds
that classification layer so the avoidance GUI/planner can load the 3box data:

  * the table slab becomes a single ``support`` box (largest cluster at the
    detected table height);
  * every cluster standing above the table becomes an ``object`` box.

It stays deliberately box-only for the first version (the GUI edge-picking is
box-only too); cylinder fitting is left for later.  ``box_inflation_m`` is written
as a visualisation hint only -- planning applies its own inflation and must not
double-count it (plan section 9, phase C).

Reuses the pipeline by importing it; point ``--pipeline`` at the checkout or set
``MAPANYTHING_PIPELINE`` if the default sibling path does not apply.
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

import numpy as np
import trimesh
from sklearn.cluster import DBSCAN

SCRIPT_DIR = Path(__file__).resolve().parent
AVOID_ROOT = SCRIPT_DIR.parent
if str(AVOID_ROOT) not in sys.path:
    sys.path.insert(0, str(AVOID_ROOT))

from avoidance.contracts import AvoidanceError, read_json, sha256_file, write_json  # noqa: E402
from avoidance.planning_scene import load_planning_scene  # noqa: E402
from g2_glb_markers import (  # noqa: E402
    MarkerError,
    add_marker_geometry,
    build_marker_document,
)


def _import_pipeline(explicit: str | None):
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
        "cannot find scene_simplify.py; pass --pipeline or set MAPANYTHING_PIPELINE"
    )


def resolve_extrinsics(snap: Path) -> Path:
    """Find a self-contained camera-pose source for a processed snapshot."""
    local = snap / "camera_extrinsics.json"
    if local.is_file():
        return local.resolve()

    exported = snap / "camera_poses_used_for_export.json"
    if exported.is_file():
        return exported.resolve()

    wrapped = snap.parent / f"{snap.name}_input" / "camera_extrinsics.json"
    if wrapped.is_file():
        return wrapped.resolve()

    direct_source = resolve_direct_depth_source(snap)
    if direct_source is not None:
        for name in (
            "camera_poses_opencv_cam2world.json",
            "camera_extrinsics.json",
        ):
            candidate = direct_source / name
            if candidate.is_file():
                return candidate.resolve()

    capture_state = snap / "capture_state.json"
    if capture_state.is_file():
        source = read_json(capture_state).get("source")
        if source:
            source_path = Path(source).expanduser()
            if not source_path.is_absolute():
                source_path = capture_state.parent / source_path
            source_path = source_path.resolve()
            if source_path.is_file():
                if source_path.parent.name != snap.name:
                    raise AvoidanceError(
                        f"capture source snapshot mismatch for {snap.name}: {source_path}"
                    )
                return source_path

    raise AvoidanceError(
        f"missing camera poses for {snap.name}; checked snapshot-local export/extrinsics, "
        "sibling _input, direct-depth manifest, and capture_state.source"
    )


def resolve_direct_depth_source(snap: Path) -> Path | None:
    """Resolve the registered/raw snapshot recorded by a direct-depth output."""
    manifest_path = snap / "direct_depth_manifest.json"
    if not manifest_path.is_file():
        return None
    source_value = read_json(manifest_path).get("input_directory")
    if not isinstance(source_value, str) or not source_value:
        raise AvoidanceError(f"{manifest_path} has no input_directory")
    source = Path(source_value).expanduser()
    if not source.is_absolute():
        source = (manifest_path.parent / source).resolve()
    else:
        source = source.resolve()
    if source.name != snap.name:
        raise AvoidanceError(
            f"direct-depth source snapshot mismatch for {snap.name}: {source}"
        )
    if not source.is_dir():
        raise AvoidanceError(f"direct-depth source directory is missing: {source}")
    return source


def resolve_capture_state(snap: Path) -> Path | None:
    local = snap / "capture_state.json"
    if local.is_file():
        return local.resolve()
    direct_source = resolve_direct_depth_source(snap)
    if direct_source is not None:
        source_state = direct_source / "capture_state.json"
        if source_state.is_file():
            return source_state.resolve()
    return None


def read_camera_centers(pipeline, pose_path: Path):
    """Read wrist centers/head height from export-pose or legacy schemas."""
    document = read_json(pose_path)
    poses = document.get("poses")
    if isinstance(poses, dict):
        if (
            document.get("matrix_direction") != "camera_to_world"
            or document.get("world_frame") != "base_link"
            or document.get("translation_unit") != "meter"
        ):
            raise AvoidanceError(
                f"{pose_path} must declare camera_to_world/base_link/meter"
            )
        matrices = {}
        for key in ("head", "hand_left", "hand_right"):
            matrix = np.asarray(poses.get(key), dtype=float)
            if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
                raise AvoidanceError(f"{pose_path} has invalid pose {key}")
            matrices[key] = matrix
        wrists = [
            matrices["hand_left"][:3, 3],
            matrices["hand_right"][:3, 3],
        ]
        return wrists, float(matrices["head"][2, 3]), "export_camera_poses"

    wrists, head_z = pipeline.read_camera_centers(str(pose_path))
    return wrists, head_z, "legacy_camera_extrinsics"


def simplify_snapshot(pipeline, voxels_path: Path, extrinsics_path: Path, out_dir: Path,
                      *, gripper_radius, min_count, cluster_eps, min_cluster,
                      obstacle_height, table_thickness, box_inflate,
                      table_xy_bounds=None, capture_state=None):
    world, colors, conf, counts, source_views, meta = pipeline.load_voxels(str(voxels_path))
    if meta["world_frame"] != "base_link":
        raise AvoidanceError(f"{voxels_path} is not base_link")
    keep = np.ones(len(world), bool)
    table_xy_input_voxels = int(len(world))
    if table_xy_bounds is not None:
        x_min, x_max, y_min, y_max = table_xy_bounds
        keep &= (
            (world[:, 0] >= x_min)
            & (world[:, 0] <= x_max)
            & (world[:, 1] >= y_min)
            & (world[:, 1] <= y_max)
        )
        table_xy_input_voxels = int(keep.sum())
    centers, head_z, camera_pose_schema = read_camera_centers(
        pipeline, extrinsics_path
    )
    keep &= pipeline.remove_gripper(world, centers, gripper_radius)
    if min_count > 1:
        keep &= counts >= min_count
    # DBSCAN denoise (drop tiny floating clusters), reusing the pipeline's rule
    idx = np.where(keep)[0]
    labels = DBSCAN(eps=cluster_eps, min_samples=2).fit_predict(world[idx])
    for lab in set(labels):
        m = labels == lab
        if lab < 0 or m.sum() < min_cluster:
            keep[idx[m]] = False
    W, C = world[keep], colors[keep]

    band = (head_z - 1.0, head_z - 0.4) if head_z is not None else None
    table_top_z, mode_z, _ = pipeline.find_support_surface(W, z_band=band)

    table_min_z = table_top_z - table_thickness
    if table_xy_bounds is not None:
        vertical_keep = W[:, 2] >= table_min_z
        retained_indices = np.where(keep)[0]
        keep[retained_indices[~vertical_keep]] = False
        W, C = W[vertical_keep], C[vertical_keep]
        if len(W) == 0:
            raise AvoidanceError(
                f"table crop removed every voxel from {voxels_path}"
            )

    voxel_size = float(meta["voxel_size"])
    fit_eps = max(cluster_eps, voxel_size * 2.5)
    primitives = []
    support, _ = pipeline.fit_support_box(
        W,
        C,
        mode_z,
        table_top_z,
        eps=fit_eps,
        voxel_size=voxel_size,
        thickness_m=table_thickness,
    )
    if support is not None:
        primitives.append({
            "id": 0,
            "primitive": "box",
            "role": "support",
            "center_m": support["center_m"],
            "size_m": support["size_m"],
            "color": support["color"],
            "voxel_count": support["voxel_count"],
        })
        footprint_lo = np.asarray(support["min_m"])[:2] - 2 * voxel_size
        footprint_hi = np.asarray(support["max_m"])[:2] + 2 * voxel_size
        in_footprint = np.all(
            (W[:, :2] >= footprint_lo) & (W[:, :2] <= footprint_hi), axis=1
        )
    else:
        in_footprint = np.ones(len(W), dtype=bool)

    obstacle_mask = in_footprint & (W[:, 2] > table_top_z)
    boxes, _ = pipeline.axis_aligned_boxes(W[obstacle_mask], C[obstacle_mask],
                                           eps=fit_eps, min_samples=2,
                                           min_cluster=min_cluster,
                                           support_top_z=table_top_z,
                                           min_height_m=obstacle_height,
                                           voxel_size=voxel_size,
                                           primitive_mode="box")
    for b in boxes:
        primitives.append({"id": len(primitives), "primitive": "box", "role": "object",
                           "center_m": b["center_m"], "size_m": b["size_m"],
                           "color": b["color"], "voxel_count": b["voxel_count"]})

    provenance = {
        "voxels_sha256": sha256_file(voxels_path),
        "extrinsics_sha256": sha256_file(extrinsics_path),
        "extrinsics_path": str(extrinsics_path.resolve()),
        "voxel_input_filename": voxels_path.name,
        "camera_pose_schema": camera_pose_schema,
        "voxel_size_m": float(meta["voxel_size"]),
        "table_top_z_m": round(float(table_top_z), 4),
        "table_min_z_m": round(float(table_min_z), 4),
        "table_xy_input_voxels": table_xy_input_voxels,
        "input_voxels": int(len(world)),
        "kept_voxels": int(keep.sum()),
        "removed_voxels": int(len(world) - keep.sum()),
        "simplification_parameters": {
            "gripper_radius_m": float(gripper_radius),
            "minimum_observation_count": int(min_count),
            "cluster_eps_m": float(cluster_eps),
            "minimum_cluster_voxels": int(min_cluster),
            "minimum_obstacle_height_m": float(obstacle_height),
            "table_thickness_m": float(table_thickness),
            "table_xy_bounds_base_link_m": (
                [float(value) for value in table_xy_bounds]
                if table_xy_bounds is not None
                else None
            ),
            "crop_below_table": table_xy_bounds is not None,
            "primitive_mode": "box",
            "visualization_box_inflation_m": float(box_inflate),
        },
    }
    if capture_state and Path(capture_state).is_file():
        provenance["capture_state_sha256"] = sha256_file(capture_state)

    marker_document = build_marker_document(
        extrinsics_path,
        Path(capture_state) if capture_state else None,
    )
    provenance["visualization_markers"] = {
        "urdf_source": marker_document["urdf_source"],
        "urdf_sha256": marker_document["urdf_sha256"],
        "pose_source": marker_document["pose_source"],
        "pose_source_sha256": marker_document["pose_source_sha256"],
        "joint_state_source": marker_document["joint_state_source"],
        "semantics": marker_document["semantics"],
    }

    out_dir.mkdir(parents=True, exist_ok=True)
    cleaned_glb_path = out_dir / "cleaned_voxels.glb"
    obstacles_glb_path = out_dir / "obstacles.glb"
    cleaned_scene = trimesh.Scene()
    cleaned_scene.add_geometry(
        pipeline.voxels_to_glb(W, C, voxel_size),
        geom_name="cleaned_voxels",
    )
    add_marker_geometry(cleaned_scene, marker_document).export(cleaned_glb_path)
    obstacle_scene = pipeline.primitives_to_glb(
        primitives,
        inflate_m=box_inflate,
        markers=[],
    )
    add_marker_geometry(obstacle_scene, marker_document).export(obstacles_glb_path)

    document = {
        "world_frame": "base_link",
        "unit": "meter",
        "box_inflation_m": float(box_inflate),  # visualisation hint only
        "boxes": primitives,
        "markers": marker_document["markers"],
        "provenance": provenance,
        "visualization_outputs": {
            "viewer_transform": "rotate_x_180_degrees",
            "cleaned_voxels_glb": {
                "path": cleaned_glb_path.name,
                "sha256": sha256_file(cleaned_glb_path),
            },
            "obstacles_glb": {
                "path": obstacles_glb_path.name,
                "sha256": sha256_file(obstacles_glb_path),
                "object_inflation_m": float(box_inflate),
                "support_inflation_m": 0.0,
            },
        },
    }
    obstacles_path = out_dir / "obstacles.json"
    write_json(obstacles_path, document)
    # fail closed: the output must satisfy the planning-scene contract
    scene = load_planning_scene(obstacles_path)
    return scene, document


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scene-root", type=Path, required=True,
                    help="folder holding snapshot_* reconstruction dirs")
    ap.add_argument("--pipeline", type=Path, default=None,
                    help="MapAnythingPipeline checkout (else sibling / MAPANYTHING_PIPELINE)")
    ap.add_argument(
        "--voxel-filename",
        default="voxels.npz",
        help="snapshot-local voxel archive to simplify (default: voxels.npz)",
    )
    ap.add_argument(
        "--gripper-radius",
        type=float,
        default=0.15,
        help="proxy removal radius around each hand camera (default: 0.15 m)",
    )
    ap.add_argument("--min-count", type=int, default=1)
    ap.add_argument("--cluster-eps", type=float, default=0.03)
    ap.add_argument("--min-cluster", type=int, default=24)
    ap.add_argument("--obstacle-height", type=float, default=0.03)
    ap.add_argument("--table-thickness", type=float, default=0.06)
    ap.add_argument(
        "--table-xy-bounds",
        type=float,
        nargs=4,
        metavar=("X_MIN", "X_MAX", "Y_MIN", "Y_MAX"),
        help="retain only this base_link XY rectangle and points at/above the table slab",
    )
    ap.add_argument("--box-inflate", type=float, default=0.08)
    args = ap.parse_args()
    if Path(args.voxel_filename).name != args.voxel_filename:
        ap.error("--voxel-filename must be a filename, not a path")
    if args.table_xy_bounds is not None:
        x_min, x_max, y_min, y_max = args.table_xy_bounds
        if not np.isfinite(args.table_xy_bounds).all() or x_min >= x_max or y_min >= y_max:
            ap.error("--table-xy-bounds requires finite X_MIN < X_MAX and Y_MIN < Y_MAX")
    try:
        pipeline = _import_pipeline(str(args.pipeline) if args.pipeline else None)
        snaps = sorted(p for p in args.scene_root.iterdir()
                       if p.is_dir() and (p / args.voxel_filename).is_file()
                       and not p.name.endswith("_input"))
        if not snaps:
            raise AvoidanceError(
                f"no snapshot_*/{args.voxel_filename} under {args.scene_root}"
            )
        for snap in snaps:
            ext = resolve_extrinsics(snap)
            capture_state = resolve_capture_state(snap)
            scene, doc = simplify_snapshot(
                pipeline, snap / args.voxel_filename, ext, snap,
                gripper_radius=args.gripper_radius, min_count=args.min_count,
                cluster_eps=args.cluster_eps, min_cluster=args.min_cluster,
                obstacle_height=args.obstacle_height, table_thickness=args.table_thickness,
                box_inflate=args.box_inflate,
                table_xy_bounds=args.table_xy_bounds,
                capture_state=capture_state)
            roles = [p["role"] for p in doc["boxes"]]
            print(f"{snap.name}: {len(doc['boxes'])} primitives "
                  f"(support={roles.count('support')}, object={roles.count('object')}) "
                  f"-> {snap / 'obstacles.json'}")
        return 0
    except (AvoidanceError, MarkerError, OSError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
