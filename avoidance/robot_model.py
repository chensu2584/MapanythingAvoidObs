"""Dependency-light G1 URDF FK and primitive collision queries."""

from __future__ import annotations

import dataclasses
import math
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

import numpy as np

from .contracts import AvoidanceError, rigid_matrix, rotation_error_deg


G1_JOINT_LAYOUT = {
    "waist": ("idx01_waist_lift_joint", "idx02_waist_pitch_joint"),
    "head": ("idx03_head_yaw_joint", "idx04_head_pitch_joint"),
    "left": tuple(f"idx{i:02d}_left_arm_joint{i - 4}" for i in range(5, 12)),
    "right": tuple(f"idx{i:02d}_right_arm_joint{i - 11}" for i in range(12, 19)),
}

SIDE_FRAMES = {
    "left": {"link7": "arm_left_link7", "hand": "hand_left_base_link"},
    "right": {"link7": "arm_right_link7", "hand": "hand_right_base_link"},
}


def _vector(text: str | None, default: tuple[float, float, float] = (0, 0, 0)) -> np.ndarray:
    values = np.asarray(default if text is None else [float(v) for v in text.split()])
    if values.shape != (3,) or not np.isfinite(values).all():
        raise AvoidanceError(f"Expected a finite three-vector, got {text!r}")
    return values.astype(np.float64)


def transform_from_xyz_rpy(xyz: np.ndarray, rpy: np.ndarray) -> np.ndarray:
    roll, pitch, yaw = (float(v) for v in rpy)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.asarray([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=np.float64)
    ry = np.asarray([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=np.float64)
    rz = np.asarray([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=np.float64)
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = rz @ ry @ rx
    result[:3, 3] = xyz
    return result


def rotation_about_axis(axis: np.ndarray, angle: float) -> np.ndarray:
    x, y, z = (float(v) for v in axis)
    skew = np.asarray([[0, -z, y], [z, 0, -x], [-y, x, 0]], dtype=np.float64)
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = (
        np.eye(3) + math.sin(angle) * skew + (1.0 - math.cos(angle)) * (skew @ skew)
    )
    return result


@dataclasses.dataclass(frozen=True)
class Joint:
    name: str
    kind: str
    parent: str
    child: str
    origin: np.ndarray
    axis: np.ndarray


@dataclasses.dataclass(frozen=True)
class CollisionPrimitive:
    name: str
    link: str
    kind: str
    link_t_primitive: np.ndarray
    size: np.ndarray


@dataclasses.dataclass(frozen=True)
class WorldPrimitive:
    name: str
    link: str
    kind: str
    base_t_primitive: np.ndarray
    size: np.ndarray


class UrdfRobot:
    """URDF tree supporting fixed, revolute, continuous and prismatic joints."""

    def __init__(self, path: str | Path):
        self.path = Path(path).resolve()
        try:
            root = ET.parse(self.path).getroot()
        except (OSError, ET.ParseError) as exc:
            raise AvoidanceError(f"Cannot read URDF {self.path}: {exc}") from exc
        self.links = {str(node.get("name")) for node in root.findall("link") if node.get("name")}
        self.joints_by_child: dict[str, Joint] = {}
        self.joints_by_name: dict[str, Joint] = {}
        for node in root.findall("joint"):
            kind = str(node.get("type"))
            if kind not in {"fixed", "revolute", "continuous", "prismatic"}:
                continue
            parent_node, child_node = node.find("parent"), node.find("child")
            if parent_node is None or child_node is None:
                raise AvoidanceError(f"URDF joint {node.get('name')} has no parent/child")
            name = str(node.get("name"))
            parent, child = str(parent_node.get("link")), str(child_node.get("link"))
            origin_node = node.find("origin")
            xyz = _vector(origin_node.get("xyz") if origin_node is not None else None)
            rpy = _vector(origin_node.get("rpy") if origin_node is not None else None)
            axis_node = node.find("axis")
            axis = _vector(axis_node.get("xyz") if axis_node is not None else None, (1, 0, 0))
            norm = float(np.linalg.norm(axis))
            if kind != "fixed" and norm <= 0.0:
                raise AvoidanceError(f"URDF joint {name} has a zero axis")
            if norm > 0.0:
                axis /= norm
            joint = Joint(name, kind, parent, child, transform_from_xyz_rpy(xyz, rpy), axis)
            if child in self.joints_by_child:
                raise AvoidanceError(f"URDF link {child} has multiple parent joints")
            self.joints_by_child[child] = joint
            self.joints_by_name[name] = joint

        self.collision_primitives: list[CollisionPrimitive] = []
        for link_node in root.findall("link"):
            link = str(link_node.get("name"))
            for index, collision in enumerate(link_node.findall("collision")):
                origin_node = collision.find("origin")
                xyz = _vector(origin_node.get("xyz") if origin_node is not None else None)
                rpy = _vector(origin_node.get("rpy") if origin_node is not None else None)
                geometry = collision.find("geometry")
                if geometry is None:
                    continue
                shape = None
                kind = ""
                size = np.zeros(3, dtype=np.float64)
                if geometry.find("sphere") is not None:
                    shape = geometry.find("sphere")
                    kind = "sphere"
                    size[0] = float(shape.get("radius"))
                elif geometry.find("cylinder") is not None:
                    shape = geometry.find("cylinder")
                    kind = "cylinder"
                    size[:2] = [float(shape.get("radius")), float(shape.get("length"))]
                elif geometry.find("box") is not None:
                    shape = geometry.find("box")
                    kind = "box"
                    size = _vector(shape.get("size"))
                else:
                    raise AvoidanceError(
                        f"Unsupported non-primitive collision geometry on link {link}; "
                        "stage 2 requires sphere/cylinder/box"
                    )
                if np.any(size < 0.0) or not np.any(size > 0.0):
                    raise AvoidanceError(f"Invalid {kind} collision size on link {link}")
                name = collision.get("name") or f"{link}_collision_{index}"
                self.collision_primitives.append(
                    CollisionPrimitive(
                        str(name), link, kind, transform_from_xyz_rpy(xyz, rpy), size
                    )
                )

    def base_to_frame(
        self, frame: str, joint_positions: dict[str, float], base: str = "base_link"
    ) -> np.ndarray:
        chain: list[Joint] = []
        cursor = frame
        visited: set[str] = set()
        while cursor != base:
            if cursor in visited:
                raise AvoidanceError(f"Cycle found while resolving URDF frame {frame}")
            visited.add(cursor)
            joint = self.joints_by_child.get(cursor)
            if joint is None:
                raise AvoidanceError(f"No URDF chain from {base} to {frame}; stopped at {cursor}")
            chain.append(joint)
            cursor = joint.parent
        matrix = np.eye(4, dtype=np.float64)
        for joint in reversed(chain):
            matrix = matrix @ joint.origin
            value = float(joint_positions.get(joint.name, 0.0))
            if joint.kind in {"revolute", "continuous"}:
                matrix = matrix @ rotation_about_axis(joint.axis, value)
            elif joint.kind == "prismatic":
                motion = np.eye(4, dtype=np.float64)
                motion[:3, 3] = joint.axis * value
                matrix = matrix @ motion
        return matrix

    def world_primitives(self, joint_positions: dict[str, float]) -> list[WorldPrimitive]:
        link_poses: dict[str, np.ndarray] = {}
        result = []
        for primitive in self.collision_primitives:
            if primitive.link not in link_poses:
                link_poses[primitive.link] = self.base_to_frame(
                    primitive.link, joint_positions
                )
            result.append(
                WorldPrimitive(
                    primitive.name,
                    primitive.link,
                    primitive.kind,
                    link_poses[primitive.link] @ primitive.link_t_primitive,
                    primitive.size,
                )
            )
        return result


def normalize_joint_state(state: dict[str, Any]) -> tuple[dict[str, float], dict[str, Any]]:
    try:
        arms = np.asarray(state["arm_joint_states"], dtype=np.float64)
        head_raw = np.asarray(state["head_joint_states"], dtype=np.float64)
        waist = np.asarray(state["waist_joint_states"], dtype=np.float64)
    except (KeyError, TypeError, ValueError) as exc:
        raise AvoidanceError("Robot state requires numeric arm/head/waist joint arrays") from exc
    if arms.shape != (14,) or head_raw.shape != (2,) or waist.shape != (2,):
        raise AvoidanceError(
            f"Expected arm/head/waist shapes (14,)/(2,)/(2,), got "
            f"{arms.shape}/{head_raw.shape}/{waist.shape}"
        )
    if not np.isfinite(np.r_[arms, head_raw, waist]).all():
        raise AvoidanceError("Robot joint state contains non-finite values")
    units = state.get("units", {})
    if not isinstance(units, dict):
        raise AvoidanceError("Robot state units must be an object")
    if units.get("arm", "rad") != "rad":
        raise AvoidanceError("Arm state unit must be rad")
    if units.get("waist_pitch", "rad") != "rad" or units.get("waist_lift", "m") != "m":
        raise AvoidanceError("Waist units must be rad and m")
    head_unit = units.get("head")
    if head_unit not in {"rad", "deg"}:
        raise AvoidanceError("Head unit must be explicitly rad or deg")
    head = np.radians(head_raw) if head_unit == "deg" else head_raw
    positions = {
        G1_JOINT_LAYOUT["waist"][0]: float(waist[1]),
        G1_JOINT_LAYOUT["waist"][1]: float(waist[0]),
        G1_JOINT_LAYOUT["head"][0]: float(head[0]),
        G1_JOINT_LAYOUT["head"][1]: float(head[1]),
    }
    positions.update(zip(G1_JOINT_LAYOUT["left"], (float(v) for v in arms[:7])))
    positions.update(zip(G1_JOINT_LAYOUT["right"], (float(v) for v in arms[7:])))
    return positions, {
        "arm_joint_states_rad": arms.tolist(),
        "head_joint_states_rad": head.tolist(),
        "waist_pitch_rad": float(waist[0]),
        "waist_lift_m": float(waist[1]),
        "urdf_joint_positions": positions,
    }


def representative_capture_state(
    capture_state: dict[str, Any],
    *,
    max_arm_change_rad: float = 0.01,
    max_head_change_deg: float = 0.5,
    max_waist_pitch_change_rad: float = 0.01,
    max_waist_lift_change_m: float = 0.005,
) -> tuple[dict[str, float], dict[str, Any]]:
    """Return median capture configuration and a fail-closed stationarity report."""
    views = capture_state.get("views")
    if not isinstance(views, dict) or not views:
        positions, normalized = normalize_joint_state(capture_state)
        return positions, {"mode": "shared_state", "stationary": True, "state": normalized}
    units = capture_state.get("units", {})
    normalized_views = []
    names = []
    for name, value in views.items():
        if not isinstance(value, dict):
            continue
        merged = dict(value)
        merged["units"] = value.get("units", units)
        _, normalized = normalize_joint_state(merged)
        normalized_views.append(normalized)
        names.append(name)
    if not normalized_views:
        raise AvoidanceError("capture_state.views contains no valid robot state")
    arm = np.asarray([v["arm_joint_states_rad"] for v in normalized_views])
    head = np.asarray([v["head_joint_states_rad"] for v in normalized_views])
    waist = np.asarray(
        [[v["waist_pitch_rad"], v["waist_lift_m"]] for v in normalized_views]
    )
    changes = {
        "max_arm_component_change_rad": float(np.max(np.ptp(arm, axis=0))),
        "max_head_component_change_deg": float(np.degrees(np.max(np.ptp(head, axis=0)))),
        "max_waist_pitch_change_rad": float(np.ptp(waist[:, 0])),
        "max_waist_lift_change_m": float(np.ptp(waist[:, 1])),
    }
    limits = {
        "max_arm_component_change_rad": float(max_arm_change_rad),
        "max_head_component_change_deg": float(max_head_change_deg),
        "max_waist_pitch_change_rad": float(max_waist_pitch_change_rad),
        "max_waist_lift_change_m": float(max_waist_lift_change_m),
    }
    stationary = all(changes[key] <= limits[key] for key in changes)
    representative = {
        "arm_joint_states": np.median(arm, axis=0).tolist(),
        "head_joint_states": np.median(head, axis=0).tolist(),
        "waist_joint_states": [
            float(np.median(waist[:, 0])),
            float(np.median(waist[:, 1])),
        ],
        "units": {
            "arm": "rad",
            "head": "rad",
            "waist_pitch": "rad",
            "waist_lift": "m",
        },
    }
    positions, normalized = normalize_joint_state(representative)
    return positions, {
        "mode": "median_of_timestamp_aligned_views",
        "view_names": names,
        "stationary": stationary,
        "changes": changes,
        "limits": limits,
        "state": normalized,
    }


def wbc_fk_crosscheck(
    robot: UrdfRobot,
    joint_positions: dict[str, float],
    capture_state: dict[str, Any],
    *,
    max_translation_error_m: float = 5e-4,
    max_rotation_error_deg: float = 0.05,
) -> dict[str, Any]:
    wbc = capture_state.get("wbc_link7_capture")
    if not isinstance(wbc, dict):
        return {"available": False, "passed": False, "reason": "missing wbc_link7_capture"}
    if wbc.get("world_frame") != "base_link" or wbc.get("pose_direction") != "base_T_frame":
        return {"available": False, "passed": False, "reason": "invalid WBC frame contract"}
    views = wbc.get("views", {})
    checks: dict[str, Any] = {}
    for side, frame_config in SIDE_FRAMES.items():
        frame_name = frame_config["link7"]
        candidates = []
        for view in views.values() if isinstance(views, dict) else []:
            if not isinstance(view, dict):
                continue
            frame = view.get("frames", {}).get(frame_name, {})
            if isinstance(frame, dict) and frame.get("base_T_frame") is not None:
                candidates.append(rigid_matrix(frame["base_T_frame"], label=f"WBC {frame_name}"))
        if not candidates:
            checks[side] = {"available": False, "passed": False}
            continue
        translations = np.asarray([m[:3, 3] for m in candidates])
        reference_index = int(np.argmin(np.linalg.norm(translations - np.median(translations, axis=0), axis=1)))
        reference = candidates[reference_index]
        fk = robot.base_to_frame(frame_name, joint_positions)
        translation_error = float(np.linalg.norm(fk[:3, 3] - reference[:3, 3]))
        angle_error = rotation_error_deg(fk, reference)
        checks[side] = {
            "available": True,
            "passed": translation_error <= max_translation_error_m
            and angle_error <= max_rotation_error_deg,
            "sample_count": len(candidates),
            "translation_error_m": translation_error,
            "rotation_error_deg": angle_error,
        }
    return {
        "available": all(value.get("available") for value in checks.values()),
        "passed": all(value.get("passed") for value in checks.values()),
        "limits": {
            "translation_error_m": max_translation_error_m,
            "rotation_error_deg": max_rotation_error_deg,
        },
        "sides": checks,
    }


def points_inside_primitive(
    points_base: np.ndarray, primitive: WorldPrimitive, margin_m: float
) -> np.ndarray:
    points = np.asarray(points_base, dtype=np.float64)
    local = (points - primitive.base_t_primitive[:3, 3]) @ primitive.base_t_primitive[:3, :3]
    margin = float(margin_m)
    if primitive.kind == "sphere":
        return np.linalg.norm(local, axis=1) <= primitive.size[0] + margin
    if primitive.kind == "box":
        return np.all(np.abs(local) <= primitive.size / 2.0 + margin, axis=1)
    if primitive.kind == "cylinder":
        radius, length = primitive.size[:2]
        return (np.linalg.norm(local[:, :2], axis=1) <= radius + margin) & (
            np.abs(local[:, 2]) <= length / 2.0 + margin
        )
    raise AvoidanceError(f"Unsupported primitive kind {primitive.kind}")


def classify_robot_voxels(
    points_base: np.ndarray,
    primitives: list[WorldPrimitive],
    *,
    core_margin_m: float,
    ambiguity_shell_m: float,
) -> tuple[np.ndarray, np.ndarray, dict[str, int]]:
    """Return core self candidates, retained ambiguity shell, and link counts."""
    if core_margin_m < 0.0 or ambiguity_shell_m < 0.0:
        raise AvoidanceError("Self-filter margins cannot be negative")
    core = np.zeros(len(points_base), dtype=bool)
    shell = np.zeros(len(points_base), dtype=bool)
    link_masks: dict[str, np.ndarray] = {}
    for primitive in primitives:
        primitive_core = points_inside_primitive(points_base, primitive, core_margin_m)
        primitive_outer = points_inside_primitive(
            points_base, primitive, core_margin_m + ambiguity_shell_m
        )
        core |= primitive_core
        shell |= primitive_outer & ~primitive_core
        if primitive.link not in link_masks:
            link_masks[primitive.link] = primitive_core.copy()
        else:
            link_masks[primitive.link] |= primitive_core
    shell &= ~core
    link_counts = {
        link: int(np.count_nonzero(mask)) for link, mask in sorted(link_masks.items())
    }
    return core, shell, dict(sorted(link_counts.items()))
