"""Shared data contracts and provenance helpers for avoidance stages 1 and 2."""

from __future__ import annotations

import dataclasses
import hashlib
import json
import os
from pathlib import Path
from typing import Any

import numpy as np


class AvoidanceError(RuntimeError):
    """Raised when an input cannot safely satisfy an avoidance contract."""


def sha256_file(path: str | Path) -> str:
    path = Path(path)
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def read_json(path: str | Path) -> dict[str, Any]:
    path = Path(path)
    try:
        with path.open(encoding="utf-8") as handle:
            value = json.load(handle)
    except (OSError, json.JSONDecodeError) as exc:
        raise AvoidanceError(f"Cannot read JSON {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise AvoidanceError(f"Expected a JSON object in {path}")
    return value


def write_json(path: str | Path, value: Any) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp")
    with temporary.open("w", encoding="utf-8") as handle:
        json.dump(value, handle, indent=2, ensure_ascii=False)
        handle.write("\n")
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(temporary, path)


def rigid_matrix(value: Any, *, label: str) -> np.ndarray:
    matrix = np.asarray(value, dtype=np.float64)
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        raise AvoidanceError(f"{label} must be a finite 4x4 matrix")
    if not np.allclose(matrix[3], [0.0, 0.0, 0.0, 1.0], atol=1e-8):
        raise AvoidanceError(f"{label} has an invalid homogeneous bottom row")
    rotation = matrix[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5):
        raise AvoidanceError(f"{label} rotation is not orthonormal")
    if not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-5):
        raise AvoidanceError(f"{label} rotation determinant is not +1")
    return matrix


def rotation_error_deg(first: np.ndarray, second: np.ndarray) -> float:
    relative = first[:3, :3].T @ second[:3, :3]
    cosine = np.clip((np.trace(relative) - 1.0) / 2.0, -1.0, 1.0)
    return float(np.degrees(np.arccos(cosine)))


@dataclasses.dataclass(frozen=True)
class VoxelMap:
    """Sparse occupied cells in a metric axis-aligned ``base_link`` grid."""

    indices: np.ndarray
    origin: np.ndarray
    voxel_size: float
    dims: np.ndarray
    colors: np.ndarray | None = None
    counts: np.ndarray | None = None
    conf: np.ndarray | None = None
    labels: np.ndarray | None = None
    label_scores: np.ndarray | None = None
    world_frame: str = "base_link"
    translation_unit: str = "meter"

    def __post_init__(self) -> None:
        indices = np.asarray(self.indices, dtype=np.int32)
        origin = np.asarray(self.origin, dtype=np.float64)
        dims = np.asarray(self.dims, dtype=np.int64)
        if indices.ndim != 2 or indices.shape[1:] != (3,):
            raise AvoidanceError(f"indices must have shape (N,3), got {indices.shape}")
        if origin.shape != (3,) or not np.isfinite(origin).all():
            raise AvoidanceError("origin must contain three finite values")
        if dims.shape != (3,) or np.any(dims <= 0):
            raise AvoidanceError("dims must contain three positive integers")
        if not np.isfinite(self.voxel_size) or float(self.voxel_size) <= 0.0:
            raise AvoidanceError("voxel_size must be positive and finite")
        if self.world_frame != "base_link" or self.translation_unit != "meter":
            raise AvoidanceError("Voxel maps must use base_link and meter")
        if len(indices) and (np.any(indices < 0) or np.any(indices >= dims)):
            raise AvoidanceError("occupied indices fall outside dims")
        if len(indices):
            flat = np.ravel_multi_index(indices.T, tuple(int(v) for v in dims))
            if len(np.unique(flat)) != len(flat):
                raise AvoidanceError("occupied indices must be unique")
        object.__setattr__(self, "indices", indices)
        object.__setattr__(self, "origin", origin)
        object.__setattr__(self, "dims", dims)
        for name in ("colors", "counts", "conf", "labels", "label_scores"):
            value = getattr(self, name)
            if value is not None and len(value) != len(indices):
                raise AvoidanceError(
                    f"{name} length {len(value)} does not match indices length {len(indices)}"
                )

    @property
    def centers(self) -> np.ndarray:
        return self.origin + (self.indices.astype(np.float64) + 0.5) * self.voxel_size

    @property
    def dense_cell_count(self) -> int:
        return int(np.prod(self.dims, dtype=np.int64))
