"""Validated clustered-scene input for avoidance planning."""

from __future__ import annotations

import dataclasses
from pathlib import Path
from typing import Any

import numpy as np

from .contracts import AvoidanceError, read_json, rigid_matrix, sha256_file


@dataclasses.dataclass(frozen=True)
class PlanningPrimitive:
    identifier: int
    role: str
    kind: str
    center_m: np.ndarray
    size_m: np.ndarray
    color: tuple[int, int, int]
    radius_m: float | None = None
    height_m: float | None = None
    source: dict[str, Any] = dataclasses.field(default_factory=dict, repr=False)

    @property
    def bounds_m(self) -> np.ndarray:
        return np.stack((self.center_m - self.size_m / 2, self.center_m + self.size_m / 2))


@dataclasses.dataclass(frozen=True)
class PlanningMarker:
    identifier: str
    kind: str
    center_m: np.ndarray
    pose_matrix: np.ndarray
    color: tuple[int, int, int]


@dataclasses.dataclass(frozen=True)
class PlanningScene:
    source_path: Path
    source_sha256: str
    primitives: tuple[PlanningPrimitive, ...]
    markers: tuple[PlanningMarker, ...]
    visualization_inflation_m: float
    world_frame: str = "base_link"
    translation_unit: str = "meter"

    @property
    def bounds_m(self) -> np.ndarray:
        bounds = np.stack([item.bounds_m for item in self.primitives])
        return np.stack((bounds[:, 0].min(0), bounds[:, 1].max(0)))


def _vector(value: Any, label: str) -> np.ndarray:
    result = np.asarray(value, dtype=np.float64)
    if result.shape != (3,) or not np.isfinite(result).all():
        raise AvoidanceError(f"{label} must contain three finite values")
    return result


def load_planning_scene(input_path: str | Path) -> PlanningScene:
    path = Path(input_path).expanduser()
    if path.is_dir():
        path = path / "obstacles.json"
    elif path.suffix.lower() in {".glb", ".gltf"}:
        path = path.with_name("obstacles.json")
    path = path.resolve()
    document = read_json(path)
    if document.get("world_frame") != "base_link" or document.get("unit") != "meter":
        raise AvoidanceError("Planning scene must declare base_link/meter")
    raw_items = document.get("boxes")
    if not isinstance(raw_items, list) or not raw_items:
        raise AvoidanceError("Planning scene requires non-empty boxes")
    primitives = []
    ids = set()
    for raw in raw_items:
        identifier = raw.get("id")
        kind = raw.get("primitive")
        role = raw.get("role")
        if not isinstance(identifier, int) or identifier in ids:
            raise AvoidanceError("Primitive ids must be unique integers")
        if kind not in {"box", "cylinder"} or role not in {"support", "object"}:
            raise AvoidanceError("Only support/object box/cylinder primitives are supported")
        ids.add(identifier)
        center = _vector(raw.get("center_m"), f"primitive {identifier} center")
        size = _vector(raw.get("size_m"), f"primitive {identifier} size")
        if np.any(size <= 0):
            raise AvoidanceError("Primitive sizes must be positive")
        radius = float(raw["radius_m"]) if kind == "cylinder" else None
        height = float(raw["height_m"]) if kind == "cylinder" else None
        if kind == "cylinder":
            if not np.allclose(_vector(raw.get("axis"), "cylinder axis"), [0, 0, 1]):
                raise AvoidanceError("Cylinders must be vertical")
            if radius <= 0 or height <= 0:
                raise AvoidanceError("Cylinder dimensions must be positive")
        color = tuple(int(v) for v in raw.get("color", [180, 80, 80]))
        primitives.append(PlanningPrimitive(identifier, role, kind, center, size, color, radius, height, dict(raw)))
    markers = []
    for raw in document.get("markers", []):
        pose = rigid_matrix(raw.get("pose_matrix"), label=f"marker {raw.get('id')}")
        center = _vector(raw.get("center_m"), "marker center")
        if not np.allclose(center, pose[:3, 3], atol=1e-4):
            raise AvoidanceError("Marker center and pose disagree")
        markers.append(PlanningMarker(str(raw["id"]), str(raw["kind"]), center, pose, tuple(raw.get("color", [255, 255, 255]))))
    inflation = float(document.get("box_inflation_m", 0.0))
    if not np.isfinite(inflation) or inflation < 0:
        raise AvoidanceError("box_inflation_m must be non-negative")
    return PlanningScene(path, sha256_file(path), tuple(primitives), tuple(markers), inflation)


def primitive_signed_distance(points: np.ndarray, primitive: PlanningPrimitive) -> np.ndarray:
    values = np.asarray(points, dtype=np.float64)
    scalar = values.shape == (3,)
    values = np.atleast_2d(values)
    local = values - primitive.center_m
    if primitive.kind == "box":
        delta = np.abs(local) - primitive.size_m / 2
        result = np.linalg.norm(np.maximum(delta, 0), axis=1) + np.minimum(np.max(delta, axis=1), 0)
    else:
        radial = np.linalg.norm(local[:, :2], axis=1) - primitive.radius_m
        axial = np.abs(local[:, 2]) - primitive.height_m / 2
        delta = np.stack((radial, axial), axis=1)
        result = np.linalg.norm(np.maximum(delta, 0), axis=1) + np.minimum(np.max(delta, axis=1), 0)
    return result[0] if scalar else result
