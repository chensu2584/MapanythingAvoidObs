#!/usr/bin/env python3
"""Reconstruct G2 depth captures directly into a base_link voxel GLB.

This is an intentionally MapAnything-free comparison path. It supports:

1. Raw G2 snapshots containing ``camera_extrinsics.json`` and one or more
   ``*_depth_raw16.png`` files. Raw uint16 depth is decoded as millimeters,
   back-projected with the depth-camera intrinsics, transformed by
   ``base_T_camera``, and reprojected into the paired RGB camera for color.
2. Preprocessed snapshots containing ``registered_depth.npz``, undistorted
   images/K files, and ``camera_poses_opencv_cam2world.json``.

Each output contains ``direct_depth_voxels.npz``,
``direct_depth_voxels.glb``, and ``direct_depth_manifest.json``.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import trimesh

from g2_glb_markers import (
    MarkerError,
    add_marker_geometry,
    build_marker_document,
)


VIEW_BITS = {"head": 1, "hand_left": 2, "hand_right": 4}
RAW_VIEW_PAIRS = (
    ("head", "head_depth", "head_rgb"),
    ("hand_left", "hand_left_depth", "hand_left_rgb"),
    ("hand_right", "hand_right_depth", "hand_right_rgb"),
)


class ReconstructionError(RuntimeError):
    pass


@dataclass(frozen=True)
class PointBatch:
    view: str
    points_base_m: np.ndarray
    colors_rgb: np.ndarray
    valid_depth_pixels: int
    used_points: int
    minimum_depth_m: float
    maximum_depth_m: float
    depth_source: Path
    color_source: Path


def read_json(path: Path) -> dict[str, Any]:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ReconstructionError(f"cannot read JSON {path}: {exc}") from exc


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def matrix4(value: Any, label: str) -> np.ndarray:
    result = np.asarray(value, dtype=np.float64)
    if result.shape != (4, 4) or not np.isfinite(result).all():
        raise ReconstructionError(f"{label} must be a finite 4x4 matrix")
    if not np.allclose(result[3], [0, 0, 0, 1], atol=1e-7):
        raise ReconstructionError(f"{label} has an invalid homogeneous row")
    return result


def intrinsic(record: dict[str, Any], label: str) -> tuple[np.ndarray, np.ndarray]:
    try:
        fx, fy = float(record["Fx"]), float(record["Fy"])
        cx, cy = float(record["Cx"]), float(record["Cy"])
    except (KeyError, TypeError, ValueError) as exc:
        raise ReconstructionError(f"{label} has invalid intrinsics") from exc
    if min(fx, fy) <= 0 or not np.isfinite([fx, fy, cx, cy]).all():
        raise ReconstructionError(f"{label} has invalid focal values")
    camera_matrix = np.array(
        [[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    distortion = np.array(
        [
            float(record.get("k1", 0.0)),
            float(record.get("k2", 0.0)),
            float(record.get("p1", 0.0)),
            float(record.get("p2", 0.0)),
            float(record.get("k3", 0.0)),
        ],
        dtype=np.float64,
    )
    return camera_matrix, distortion


def depth_pixels(
    depth_m: np.ndarray,
    valid: np.ndarray,
    *,
    minimum_depth_m: float,
    maximum_depth_m: float,
    pixel_stride: int,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    rows, cols = np.indices(depth_m.shape)
    mask = (
        valid
        & np.isfinite(depth_m)
        & (depth_m >= minimum_depth_m)
        & (depth_m <= maximum_depth_m)
    )
    if pixel_stride > 1:
        mask &= (rows % pixel_stride == 0) & (cols % pixel_stride == 0)
    return cols[mask].astype(np.float64), rows[mask].astype(np.float64), depth_m[mask]


def backproject(
    u: np.ndarray,
    v: np.ndarray,
    depth_m: np.ndarray,
    camera_matrix: np.ndarray,
    distortion: np.ndarray,
) -> np.ndarray:
    pixels = np.stack([u, v], axis=1).reshape(-1, 1, 2)
    normalized = cv2.undistortPoints(
        pixels, camera_matrix, distortion
    ).reshape(-1, 2)
    return np.column_stack(
        [normalized[:, 0] * depth_m, normalized[:, 1] * depth_m, depth_m]
    )


def transform_points(points: np.ndarray, transform: np.ndarray) -> np.ndarray:
    return points @ transform[:3, :3].T + transform[:3, 3]


def colorize_from_rgb_camera(
    points_base: np.ndarray,
    image_bgr: np.ndarray,
    base_t_rgb: np.ndarray,
    camera_matrix: np.ndarray,
    distortion: np.ndarray,
) -> np.ndarray:
    rgb_t_base = np.linalg.inv(base_t_rgb)
    points_rgb = transform_points(points_base, rgb_t_base)
    projected, _ = cv2.projectPoints(
        points_rgb,
        np.zeros(3),
        np.zeros(3),
        camera_matrix,
        distortion,
    )
    uv = np.rint(projected.reshape(-1, 2)).astype(np.int64)
    height, width = image_bgr.shape[:2]
    inside = (
        (points_rgb[:, 2] > 0)
        & (uv[:, 0] >= 0)
        & (uv[:, 0] < width)
        & (uv[:, 1] >= 0)
        & (uv[:, 1] < height)
    )
    colors = np.full((len(points_base), 3), 160, dtype=np.uint8)
    colors[inside] = image_bgr[uv[inside, 1], uv[inside, 0], ::-1]
    return colors


def raw_batches(
    snapshot: Path,
    *,
    minimum_depth_m: float,
    maximum_depth_m: float,
    pixel_stride: int,
    raw_depth_scale_m: float,
) -> list[PointBatch]:
    metadata_path = snapshot / "camera_extrinsics.json"
    metadata = read_json(metadata_path)
    convention = metadata.get("convention", {})
    if convention:
        direction = convention.get("extrinsic_direction")
        if direction is not None and direction != "base_T_camera":
            raise ReconstructionError(f"{metadata_path} must use base_T_camera")
        if direction is None and "base_T_camera" not in convention:
            raise ReconstructionError(
                f"{metadata_path} does not declare the extrinsic direction"
            )
        axes = convention.get("camera_axes")
        if axes is not None and "OpenCV RDF" not in axes:
            raise ReconstructionError(
                f"{metadata_path} must use OpenCV RDF camera axes"
            )
    captures = metadata.get("captures", {})
    extrinsics = metadata.get("extrinsics", {})
    batches = []
    for view, depth_key, rgb_key in RAW_VIEW_PAIRS:
        depth_record = captures.get(depth_key)
        rgb_record = captures.get(rgb_key)
        if not isinstance(depth_record, dict) or not isinstance(rgb_record, dict):
            continue
        depth_path = snapshot / depth_record.get("saved_path", f"{depth_key}_raw16.png")
        rgb_path = snapshot / rgb_record.get("saved_path", f"{rgb_key}.png")
        if not depth_path.is_file() or not rgb_path.is_file():
            continue
        depth_raw = cv2.imread(str(depth_path), cv2.IMREAD_UNCHANGED)
        image_bgr = cv2.imread(str(rgb_path), cv2.IMREAD_COLOR)
        if depth_raw is None or depth_raw.dtype != np.uint16 or depth_raw.ndim != 2:
            raise ReconstructionError(f"{depth_path} must be a uint16 depth PNG")
        if image_bgr is None:
            raise ReconstructionError(f"cannot read RGB image {rgb_path}")
        depth_m = depth_raw.astype(np.float32) * raw_depth_scale_m
        raw_valid = (depth_raw > 0) & (depth_raw < np.iinfo(np.uint16).max)
        u, v, z = depth_pixels(
            depth_m,
            raw_valid,
            minimum_depth_m=minimum_depth_m,
            maximum_depth_m=maximum_depth_m,
            pixel_stride=pixel_stride,
        )
        if len(z) == 0:
            continue
        depth_k, depth_distortion = intrinsic(
            depth_record.get("intrinsic", {}), f"{depth_key} intrinsics"
        )
        rgb_k, rgb_distortion = intrinsic(
            rgb_record.get("intrinsic", {}), f"{rgb_key} intrinsics"
        )
        base_t_depth = matrix4(
            extrinsics.get(depth_key, {}).get("matrix"),
            f"{depth_key} base_T_camera",
        )
        base_t_rgb = matrix4(
            extrinsics.get(rgb_key, {}).get("matrix"),
            f"{rgb_key} base_T_camera",
        )
        points_base = transform_points(
            backproject(u, v, z, depth_k, depth_distortion),
            base_t_depth,
        )
        colors = colorize_from_rgb_camera(
            points_base,
            image_bgr,
            base_t_rgb,
            rgb_k,
            rgb_distortion,
        )
        batches.append(
            PointBatch(
                view,
                points_base,
                colors,
                int(raw_valid.sum()),
                len(points_base),
                float(z.min()),
                float(z.max()),
                depth_path,
                rgb_path,
            )
        )
    if not batches:
        raise ReconstructionError(f"no usable raw depth/RGB pairs in {snapshot}")
    return batches


def registered_batches(
    snapshot: Path,
    *,
    minimum_depth_m: float,
    maximum_depth_m: float,
    pixel_stride: int,
) -> list[PointBatch]:
    depth_path = snapshot / "registered_depth.npz"
    pose_path = snapshot / "camera_poses_opencv_cam2world.json"
    poses_document = read_json(pose_path)
    if (
        poses_document.get("matrix_direction") != "camera_to_world"
        or poses_document.get("world_frame") != "base_link"
        or poses_document.get("translation_unit") != "meter"
    ):
        raise ReconstructionError(
            f"{pose_path} must declare camera_to_world/base_link/meter"
        )
    poses = poses_document.get("poses", {})
    try:
        archive = np.load(depth_path, allow_pickle=False)
    except (OSError, ValueError) as exc:
        raise ReconstructionError(f"cannot read {depth_path}: {exc}") from exc
    if (
        str(archive["world_frame"]) != "base_link"
        or str(archive["translation_unit"]) != "meter"
    ):
        raise ReconstructionError(f"{depth_path} must declare base_link/meter")
    batches = []
    for view in [str(item) for item in archive["views"].tolist()]:
        depth_key = f"{view}_depth_z"
        valid_key = f"{view}_depth_valid"
        image_path = snapshot / f"{view}.png"
        k_path = snapshot / f"{view}_K.json"
        if depth_key not in archive or valid_key not in archive:
            raise ReconstructionError(f"{depth_path} is missing {view} depth arrays")
        image_bgr = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        if image_bgr is None:
            raise ReconstructionError(f"cannot read {image_path}")
        depth_m = np.asarray(archive[depth_key], dtype=np.float32)
        valid = np.asarray(archive[valid_key], dtype=bool)
        if depth_m.shape != valid.shape or depth_m.shape != image_bgr.shape[:2]:
            raise ReconstructionError(f"{view} registered depth/image shape mismatch")
        k_document = read_json(k_path)
        camera_matrix = np.asarray(k_document.get("K"), dtype=np.float64)
        if camera_matrix.shape != (3, 3) or not np.isfinite(camera_matrix).all():
            raise ReconstructionError(f"{k_path} has invalid K")
        u, v, z = depth_pixels(
            depth_m,
            valid,
            minimum_depth_m=minimum_depth_m,
            maximum_depth_m=maximum_depth_m,
            pixel_stride=pixel_stride,
        )
        if len(z) == 0:
            continue
        points_camera = backproject(
            u, v, z, camera_matrix, np.zeros(5, dtype=np.float64)
        )
        points_base = transform_points(
            points_camera,
            matrix4(poses.get(view), f"{view} camera_to_world"),
        )
        ui, vi = u.astype(np.int64), v.astype(np.int64)
        colors = image_bgr[vi, ui, ::-1].copy()
        batches.append(
            PointBatch(
                view,
                points_base,
                colors,
                int(valid.sum()),
                len(points_base),
                float(z.min()),
                float(z.max()),
                depth_path,
                image_path,
            )
        )
    if not batches:
        raise ReconstructionError(f"no usable registered depth views in {snapshot}")
    return batches


def detect_mode(snapshot: Path) -> str | None:
    if (snapshot / "camera_extrinsics.json").is_file() and any(
        snapshot.glob("*_depth_raw16.png")
    ):
        return "raw_uint16_depth"
    if (
        (snapshot / "registered_depth.npz").is_file()
        and (snapshot / "camera_poses_opencv_cam2world.json").is_file()
    ):
        return "registered_depth_npz"
    return None


def discover_snapshots(input_path: Path) -> tuple[list[Path], bool]:
    input_path = input_path.expanduser().resolve()
    if detect_mode(input_path):
        return [input_path], True
    snapshots = [
        path
        for path in sorted(input_path.glob("snapshot_*"))
        if path.is_dir() and detect_mode(path)
    ]
    if not snapshots:
        raise ReconstructionError(f"no supported G2 depth snapshots under {input_path}")
    return snapshots, False


def voxelize(
    batches: list[PointBatch],
    *,
    voxel_size_m: float,
    minimum_points_per_voxel: int,
) -> dict[str, np.ndarray]:
    points = np.concatenate([batch.points_base_m for batch in batches], axis=0)
    colors = np.concatenate([batch.colors_rgb for batch in batches], axis=0)
    source_bits = np.concatenate(
        [
            np.full(len(batch.points_base_m), VIEW_BITS[batch.view], dtype=np.uint8)
            for batch in batches
        ]
    )
    global_indices = np.floor(points / voxel_size_m).astype(np.int64)
    minimum_index = global_indices.min(axis=0)
    local_indices = global_indices - minimum_index
    unique, inverse, counts = np.unique(
        local_indices, axis=0, return_inverse=True, return_counts=True
    )
    color_sums = np.zeros((len(unique), 3), dtype=np.float64)
    np.add.at(color_sums, inverse, colors)
    voxel_colors = np.clip(
        np.rint(color_sums / counts[:, None]), 0, 255
    ).astype(np.uint8)
    voxel_sources = np.zeros(len(unique), dtype=np.uint8)
    np.bitwise_or.at(voxel_sources, inverse, source_bits)
    keep = counts >= minimum_points_per_voxel
    indices = unique[keep].astype(np.int32)
    origin = minimum_index.astype(np.float64) * voxel_size_m
    centers = origin + (indices.astype(np.float64) + 0.5) * voxel_size_m
    return {
        "indices": indices,
        "origin": origin,
        "dims": (unique.max(axis=0) + 1).astype(np.int32),
        "colors": voxel_colors[keep],
        "counts": counts[keep].astype(np.int32),
        "source_views": voxel_sources[keep],
        "centers": centers,
    }


def voxels_to_glb(
    centers: np.ndarray,
    colors: np.ndarray,
    voxel_size_m: float,
    marker_document: dict[str, Any],
) -> trimesh.Scene:
    cube = trimesh.creation.box(extents=[voxel_size_m * 0.95] * 3)
    vertices = (
        centers[:, None, :] + cube.vertices[None, :, :]
    ).reshape(-1, 3)
    faces = (
        cube.faces[None]
        + (np.arange(len(centers)) * len(cube.vertices))[:, None, None]
    ).reshape(-1, 3)
    vertex_colors = np.repeat(colors, len(cube.vertices), axis=0)
    mesh = trimesh.Trimesh(
        vertices=vertices,
        faces=faces,
        vertex_colors=vertex_colors,
        process=False,
    )
    mesh.apply_transform(
        trimesh.transformations.rotation_matrix(np.pi, [1, 0, 0])
    )
    scene = trimesh.Scene()
    scene.add_geometry(mesh, geom_name="direct_depth_voxels")
    return add_marker_geometry(scene, marker_document)


def output_directory(
    input_path: Path,
    snapshot: Path,
    *,
    single_snapshot: bool,
    explicit_out_root: Path | None,
) -> Path:
    if explicit_out_root is not None:
        root = explicit_out_root.expanduser().resolve()
        return root if single_snapshot else root / snapshot.name
    if single_snapshot:
        return snapshot / "direct_depth_reconstruction"
    root = input_path.parent if input_path.name == "undistorted" else input_path
    return root / "direct_depth_reconstruction" / snapshot.name


def reconstruct_snapshot(
    snapshot: Path,
    output_dir: Path,
    *,
    voxel_size_m: float,
    minimum_depth_m: float,
    maximum_depth_m: float,
    pixel_stride: int,
    raw_depth_scale_m: float,
    minimum_points_per_voxel: int,
) -> dict[str, Any]:
    mode = detect_mode(snapshot)
    if mode == "raw_uint16_depth":
        batches = raw_batches(
            snapshot,
            minimum_depth_m=minimum_depth_m,
            maximum_depth_m=maximum_depth_m,
            pixel_stride=pixel_stride,
            raw_depth_scale_m=raw_depth_scale_m,
        )
    elif mode == "registered_depth_npz":
        batches = registered_batches(
            snapshot,
            minimum_depth_m=minimum_depth_m,
            maximum_depth_m=maximum_depth_m,
            pixel_stride=pixel_stride,
        )
    else:
        raise ReconstructionError(f"unsupported snapshot format: {snapshot}")

    voxels = voxelize(
        batches,
        voxel_size_m=voxel_size_m,
        minimum_points_per_voxel=minimum_points_per_voxel,
    )
    if len(voxels["indices"]) == 0:
        raise ReconstructionError(f"voxel filtering removed every point in {snapshot}")

    output_dir.mkdir(parents=True, exist_ok=True)
    npz_path = output_dir / "direct_depth_voxels.npz"
    glb_path = output_dir / "direct_depth_voxels.glb"
    manifest_path = output_dir / "direct_depth_manifest.json"
    pose_path = (
        snapshot / "camera_extrinsics.json"
        if mode == "raw_uint16_depth"
        else snapshot / "camera_poses_opencv_cam2world.json"
    )
    capture_state_path = snapshot / "capture_state.json"
    marker_document = build_marker_document(
        pose_path,
        capture_state_path if capture_state_path.is_file() else None,
    )
    np.savez_compressed(
        npz_path,
        indices=voxels["indices"],
        origin=voxels["origin"],
        dims=voxels["dims"],
        voxel_size=np.float32(voxel_size_m),
        colors=voxels["colors"],
        conf=np.ones(len(voxels["indices"]), dtype=np.float32),
        counts=voxels["counts"],
        source_views=voxels["source_views"],
        world_frame=np.array("base_link"),
        translation_unit=np.array("meter"),
        reconstruction_method=np.array("direct_metric_depth_backprojection"),
    )
    voxels_to_glb(
        voxels["centers"],
        voxels["colors"],
        voxel_size_m,
        marker_document,
    ).export(glb_path)

    bounds = np.stack(
        [voxels["centers"].min(axis=0), voxels["centers"].max(axis=0)]
    )
    manifest = {
        "schema_version": 1,
        "method": "direct_metric_depth_backprojection",
        "input_mode": mode,
        "snapshot": snapshot.name,
        "input_directory": str(snapshot),
        "world_frame": "base_link",
        "translation_unit": "meter",
        "camera_axes": "OpenCV RDF: +X right, +Y down, +Z forward",
        "visualization_markers": marker_document,
        "parameters": {
            "voxel_size_m": voxel_size_m,
            "minimum_depth_m": minimum_depth_m,
            "maximum_depth_m": maximum_depth_m,
            "pixel_stride": pixel_stride,
            "raw_depth_scale_m": raw_depth_scale_m,
            "minimum_points_per_voxel": minimum_points_per_voxel,
        },
        "views": [
            {
                "view": batch.view,
                "depth_source": str(batch.depth_source),
                "depth_sha256": sha256_file(batch.depth_source),
                "color_source": str(batch.color_source),
                "color_sha256": sha256_file(batch.color_source),
                "valid_depth_pixels_before_range_filter": batch.valid_depth_pixels,
                "used_points": batch.used_points,
                "minimum_used_depth_m": batch.minimum_depth_m,
                "maximum_used_depth_m": batch.maximum_depth_m,
            }
            for batch in batches
        ],
        "result": {
            "input_point_count": int(
                sum(batch.used_points for batch in batches)
            ),
            "voxel_count": int(len(voxels["indices"])),
            "bounds_base_link_m": bounds.round(6).tolist(),
            "viewer_transform": "rotate_x_180_degrees",
            "npz": {
                "path": npz_path.name,
                "sha256": sha256_file(npz_path),
            },
            "glb": {
                "path": glb_path.name,
                "sha256": sha256_file(glb_path),
                "size_bytes": glb_path.stat().st_size,
            },
        },
        "limitations": [
            "This reconstruction uses measured depth only and does not infer occluded surfaces.",
            "The 3box registered-depth inputs currently contain the head view only.",
            "No robot self-filter, table crop, DBSCAN denoise, or obstacle fitting is applied.",
            "The GLB uses a viewer-only 180 degree X rotation; NPZ remains in base_link.",
            "The hand meshes are visualization-only; flange reference centers are not measured gripper TCPs.",
        ],
    }
    manifest_path.write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return manifest


def positive_float(value: str) -> float:
    result = float(value)
    if not np.isfinite(result) or result <= 0:
        raise argparse.ArgumentTypeError("value must be a positive finite number")
    return result


def positive_int(value: str) -> int:
    result = int(value)
    if result <= 0:
        raise argparse.ArgumentTypeError("value must be positive")
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--input",
        type=Path,
        required=True,
        help="one raw/registered snapshot or a directory containing snapshot_*",
    )
    parser.add_argument("--out-root", type=Path)
    parser.add_argument("--voxel-size", type=positive_float, default=0.01)
    parser.add_argument("--min-depth", type=positive_float, default=0.15)
    parser.add_argument("--max-depth", type=positive_float, default=3.0)
    parser.add_argument("--pixel-stride", type=positive_int, default=1)
    parser.add_argument("--raw-depth-scale", type=positive_float, default=0.001)
    parser.add_argument("--min-points-per-voxel", type=positive_int, default=1)
    args = parser.parse_args()
    if args.min_depth >= args.max_depth:
        parser.error("--min-depth must be smaller than --max-depth")
    try:
        snapshots, single = discover_snapshots(args.input)
        for snapshot in snapshots:
            out = output_directory(
                args.input.expanduser().resolve(),
                snapshot,
                single_snapshot=single,
                explicit_out_root=args.out_root,
            )
            manifest = reconstruct_snapshot(
                snapshot,
                out,
                voxel_size_m=args.voxel_size,
                minimum_depth_m=args.min_depth,
                maximum_depth_m=args.max_depth,
                pixel_stride=args.pixel_stride,
                raw_depth_scale_m=args.raw_depth_scale,
                minimum_points_per_voxel=args.min_points_per_voxel,
            )
            result = manifest["result"]
            print(
                f"{snapshot.name}: views={len(manifest['views'])}, "
                f"points={result['input_point_count']}, "
                f"voxels={result['voxel_count']} -> {out / result['glb']['path']}"
            )
        return 0
    except (MarkerError, ReconstructionError, OSError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
