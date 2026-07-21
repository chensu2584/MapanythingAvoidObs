"""Validated MapAnything voxel/source loading for operation-map construction."""

from __future__ import annotations

import dataclasses
from pathlib import Path
from typing import Any

import numpy as np

from .contracts import AvoidanceError, VoxelMap, read_json, sha256_file


VIEW_NAMES = ("head", "hand_left", "hand_right")


@dataclasses.dataclass(frozen=True)
class LoadedVoxelMap:
    voxel_map: VoxelMap
    requested_input: Path
    effective_input: Path
    source_kind: str
    provenance: dict[str, Any]
    capture_dir: Path | None


def _scalar_string(value: np.ndarray | Any) -> str | None:
    array = np.asarray(value)
    if array.size != 1:
        return None
    item = array.reshape(()).item()
    if isinstance(item, bytes):
        return item.decode("utf-8")
    return str(item)


def _frame_evidence(directory: Path, npz: Any | None = None) -> dict[str, Any]:
    directory = directory.resolve()
    if npz is not None and "world_frame" in npz.files:
        frame = _scalar_string(npz["world_frame"])
        unit = _scalar_string(npz["translation_unit"]) if "translation_unit" in npz.files else None
        if frame == "base_link" and unit in (None, "meter"):
            return {
                "world_frame": frame,
                "translation_unit": unit or "meter",
                "source": "embedded_npz_metadata",
            }
        raise AvoidanceError(
            f"Embedded voxel metadata must be base_link/meter, got {frame!r}/{unit!r}"
        )
    pose_path = directory / "camera_poses_used_for_export.json"
    if pose_path.is_file():
        pose = read_json(pose_path)
        if pose.get("world_frame") == "base_link":
            return {
                "world_frame": "base_link",
                "translation_unit": "meter",
                "source": str(pose_path.resolve()),
                "sha256": sha256_file(pose_path),
            }
    manifest_path = directory / "pose_conversion_manifest.json"
    if manifest_path.is_file():
        manifest = read_json(manifest_path)
        contract = manifest.get("output_contract", {})
        if contract.get("world_frame") == "base_link" and contract.get(
            "translation_unit", "meter"
        ) in {"meter", "meters", "m"}:
            return {
                "world_frame": "base_link",
                "translation_unit": "meter",
                "source": str(manifest_path.resolve()),
                "sha256": sha256_file(manifest_path),
            }
    raise AvoidanceError(
        f"Cannot prove that {directory} uses metric base_link coordinates. "
        "Provide embedded metadata or a matching pose export manifest."
    )


def _load_sparse_npz(path: Path, requested: Path, source_kind: str) -> LoadedVoxelMap:
    try:
        with np.load(path, allow_pickle=False) as data:
            required = {"indices", "origin", "voxel_size", "dims"}
            missing = sorted(required - set(data.files))
            if missing:
                raise AvoidanceError(f"{path} is missing voxel fields: {missing}")
            evidence = _frame_evidence(path.parent, data)
            values = {
                key: np.asarray(data[key]).copy() if key in data.files else None
                for key in ("colors", "counts", "conf", "labels", "label_scores")
            }
            voxel_map = VoxelMap(
                indices=np.asarray(data["indices"], dtype=np.int32),
                origin=np.asarray(data["origin"], dtype=np.float64),
                voxel_size=float(np.asarray(data["voxel_size"]).reshape(())),
                dims=np.asarray(data["dims"], dtype=np.int64),
                **values,
            )
    except (OSError, ValueError) as exc:
        if isinstance(exc, AvoidanceError):
            raise
        raise AvoidanceError(f"Cannot load voxel NPZ {path}: {exc}") from exc
    return LoadedVoxelMap(
        voxel_map=voxel_map,
        requested_input=requested.resolve(),
        effective_input=path.resolve(),
        source_kind=source_kind,
        provenance={
            "requested_path": str(requested.resolve()),
            "effective_path": str(path.resolve()),
            "effective_sha256": sha256_file(path),
            "frame_evidence": evidence,
        },
        capture_dir=path.parent,
    )


def _voxelize_views(path: Path, requested: Path, voxel_size: float) -> LoadedVoxelMap:
    if not np.isfinite(voxel_size) or voxel_size <= 0.0:
        raise AvoidanceError("A positive --source-voxel-size is required for views.npz")
    evidence = _frame_evidence(path.parent)
    try:
        with np.load(path, allow_pickle=False) as data:
            point_parts, color_parts, conf_parts = [], [], []
            have_conf = all(f"{name}_conf" in data.files for name in VIEW_NAMES)
            for name in VIEW_NAMES:
                required = [f"{name}_mask", f"{name}_pts3d", f"{name}_img"]
                missing = [key for key in required if key not in data.files]
                if missing:
                    raise AvoidanceError(f"{path} is missing view fields: {missing}")
                mask = np.asarray(data[f"{name}_mask"], dtype=bool)
                point_parts.append(np.asarray(data[f"{name}_pts3d"], dtype=np.float32)[mask])
                color_parts.append(np.asarray(data[f"{name}_img"], dtype=np.uint8)[mask])
                if have_conf:
                    conf_parts.append(np.asarray(data[f"{name}_conf"], dtype=np.float32)[mask])
    except (OSError, ValueError) as exc:
        if isinstance(exc, AvoidanceError):
            raise
        raise AvoidanceError(f"Cannot load views NPZ {path}: {exc}") from exc
    points = np.concatenate(point_parts, axis=0)
    colors = np.concatenate(color_parts, axis=0)
    confidence = np.concatenate(conf_parts, axis=0) if have_conf else None
    finite = np.isfinite(points).all(axis=1)
    points, colors = points[finite], colors[finite]
    confidence = confidence[finite] if confidence is not None else None
    if not len(points):
        raise AvoidanceError("views.npz contains no finite masked points")
    origin = points.min(axis=0).astype(np.float64)
    extent = points.max(axis=0).astype(np.float64) - origin
    dims = np.maximum(np.floor(extent / voxel_size).astype(np.int64) + 1, 1)
    indices = np.floor((points - origin) / voxel_size).astype(np.int64)
    np.clip(indices, 0, dims - 1, out=indices)
    flat = np.ravel_multi_index(indices.T, tuple(int(v) for v in dims))
    unique, inverse, counts = np.unique(flat, return_inverse=True, return_counts=True)
    output_indices = np.stack(np.unravel_index(unique, tuple(dims)), axis=1).astype(np.int32)
    sums = np.zeros((len(unique), 3), dtype=np.float64)
    for channel in range(3):
        sums[:, channel] = np.bincount(
            inverse, weights=colors[:, channel], minlength=len(unique)
        )
    output_colors = np.clip(sums / counts[:, None], 0, 255).astype(np.uint8)
    if confidence is not None:
        output_conf = np.full(len(unique), -np.inf, dtype=np.float32)
        np.maximum.at(output_conf, inverse, confidence)
    else:
        output_conf = np.full(len(unique), np.nan, dtype=np.float32)
    voxel_map = VoxelMap(
        indices=output_indices,
        origin=origin,
        voxel_size=voxel_size,
        dims=dims,
        colors=output_colors,
        counts=counts.astype(np.int32),
        conf=output_conf,
        labels=np.zeros(len(unique), dtype=np.int32),
        label_scores=np.zeros(len(unique), dtype=np.float32),
    )
    return LoadedVoxelMap(
        voxel_map=voxel_map,
        requested_input=requested.resolve(),
        effective_input=path.resolve(),
        source_kind="views_npz_voxelized_in_memory",
        provenance={
            "requested_path": str(requested.resolve()),
            "effective_path": str(path.resolve()),
            "effective_sha256": sha256_file(path),
            "frame_evidence": evidence,
            "source_point_count": int(len(points)),
            "source_voxel_size_m": float(voxel_size),
        },
        capture_dir=path.parent,
    )


def load_voxel_map(input_path: str | Path, *, source_voxel_size: float = 0.02) -> LoadedVoxelMap:
    requested = Path(input_path).expanduser()
    if not requested.exists():
        raise AvoidanceError(f"Map input does not exist: {requested}")
    if requested.is_dir():
        voxel_path = requested / "voxels.npz"
        if voxel_path.is_file():
            return _load_sparse_npz(voxel_path, requested, "output_directory_voxels_npz")
        views_path = requested / "views.npz"
        if views_path.is_file():
            return _voxelize_views(views_path, requested, source_voxel_size)
        raise AvoidanceError(f"Directory has neither voxels.npz nor views.npz: {requested}")
    suffix = requested.suffix.lower()
    if suffix == ".npz" and requested.name == "views.npz":
        return _voxelize_views(requested, requested, source_voxel_size)
    if suffix == ".npz":
        return _load_sparse_npz(requested, requested, "voxel_npz")
    if suffix in {".glb", ".gltf"}:
        paired = requested.with_name("voxels.npz")
        if not paired.is_file():
            raise AvoidanceError(
                "A standalone GLB is visualization geometry, not a verifiable occupancy grid. "
                "Place its matching voxels.npz beside it or provide views.npz."
            )
        loaded = _load_sparse_npz(paired, requested, "glb_with_paired_voxels_npz")
        provenance = dict(loaded.provenance)
        provenance["requested_glb_sha256"] = sha256_file(requested)
        return dataclasses.replace(loaded, provenance=provenance)
    raise AvoidanceError(f"Unsupported map input type: {requested}")
