#!/usr/bin/env python3
"""Shared G2 pose markers for reconstruction and planning-scene GLBs.

The URDF describes an omnipicker that is not the gripper installed on the
captured robot.  Consequently, the end-effector markers below stop at each
``arm_*_end_link`` flange.  The palm/finger meshes are intentionally simple
visual aids and must never be used as collision or TCP geometry.
"""

from __future__ import annotations

import hashlib
import json
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

import numpy as np
import trimesh


DEFAULT_URDF = (
    Path(__file__).resolve().parents[2]
    / "G2"
    / "G2_parameters"
    / "G2_t2_crs_omnipicker"
    / "urdf"
    / "G2_t2_crs_omnipicker.urdf"
)
VIEWER_FLIP = trimesh.transformations.rotation_matrix(np.pi, [1.0, 0.0, 0.0])
MARKER_COLORS = {
    "origin": [245, 245, 245],
    "head_camera": [255, 205, 40],
    "left_hand_camera": [40, 205, 245],
    "right_hand_camera": [205, 80, 245],
    "left_gripper_reference_center": [55, 205, 95],
    "right_gripper_reference_center": [245, 125, 45],
}


class MarkerError(RuntimeError):
    pass


def _read_json(path: Path) -> dict[str, Any]:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise MarkerError(f"cannot read JSON {path}: {exc}") from exc


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _vector(text: str | None, default: tuple[float, float, float]) -> np.ndarray:
    if not text:
        return np.asarray(default, dtype=np.float64)
    value = np.fromstring(text, sep=" ", dtype=np.float64)
    if value.shape != (3,) or not np.isfinite(value).all():
        raise MarkerError(f"invalid URDF vector: {text!r}")
    return value


def _rpy_matrix(rpy: np.ndarray) -> np.ndarray:
    roll, pitch, yaw = rpy
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=np.float64)
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=np.float64)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=np.float64)
    return rz @ ry @ rx


def _origin_transform(element: ET.Element | None) -> np.ndarray:
    transform = np.eye(4)
    if element is None:
        return transform
    transform[:3, :3] = _rpy_matrix(
        _vector(element.get("rpy"), (0.0, 0.0, 0.0))
    )
    transform[:3, 3] = _vector(element.get("xyz"), (0.0, 0.0, 0.0))
    return transform


def _axis_rotation(axis: np.ndarray, angle: float) -> np.ndarray:
    norm = np.linalg.norm(axis)
    if norm <= 1e-12:
        raise MarkerError("revolute URDF joint has a zero axis")
    x, y, z = axis / norm
    c, s = np.cos(angle), np.sin(angle)
    cross = 1.0 - c
    result = np.eye(4)
    result[:3, :3] = np.array(
        [
            [c + x * x * cross, x * y * cross - z * s, x * z * cross + y * s],
            [y * x * cross + z * s, c + y * y * cross, y * z * cross - x * s],
            [z * x * cross - y * s, z * y * cross + x * s, c + z * z * cross],
        ],
        dtype=np.float64,
    )
    return result


def flange_poses_from_urdf(
    urdf_path: Path, joint_positions: dict[str, Any]
) -> dict[str, np.ndarray]:
    """Compute both arm flange poses from base_link using only the URDF tree."""
    try:
        root = ET.parse(urdf_path).getroot()
    except (OSError, ET.ParseError) as exc:
        raise MarkerError(f"cannot parse URDF {urdf_path}: {exc}") from exc

    joints: dict[str, dict[str, Any]] = {}
    for element in root.findall("joint"):
        child = element.find("child")
        parent = element.find("parent")
        if child is None or parent is None:
            continue
        child_link = child.get("link")
        parent_link = parent.get("link")
        if not child_link or not parent_link:
            continue
        joints[child_link] = {
            "name": element.get("name", ""),
            "type": element.get("type", "fixed"),
            "parent": parent_link,
            "origin": _origin_transform(element.find("origin")),
            "axis": _vector(
                element.find("axis").get("xyz")
                if element.find("axis") is not None
                else None,
                (1.0, 0.0, 0.0),
            ),
        }

    def pose_for(target: str) -> np.ndarray:
        chain = []
        current = target
        seen = set()
        while current != "base_link":
            if current in seen or current not in joints:
                raise MarkerError(f"cannot trace URDF chain base_link -> {target}")
            seen.add(current)
            joint = joints[current]
            chain.append(joint)
            current = joint["parent"]
        pose = np.eye(4)
        for joint in reversed(chain):
            pose = pose @ joint["origin"]
            if joint["type"] in {"revolute", "continuous"}:
                name = joint["name"]
                if name not in joint_positions:
                    raise MarkerError(f"joint state is missing {name}")
                angle = float(joint_positions[name])
                if not np.isfinite(angle):
                    raise MarkerError(f"joint state {name} is not finite")
                pose = pose @ _axis_rotation(joint["axis"], angle)
            elif joint["type"] == "prismatic":
                name = joint["name"]
                if name not in joint_positions:
                    raise MarkerError(f"joint state is missing {name}")
                distance = float(joint_positions[name])
                motion = np.eye(4)
                motion[:3, 3] = joint["axis"] * distance
                pose = pose @ motion
            elif joint["type"] != "fixed":
                raise MarkerError(f"unsupported URDF joint type {joint['type']!r}")
        return pose

    return {
        "left": pose_for("arm_l_end_link"),
        "right": pose_for("arm_r_end_link"),
    }


def _camera_poses(document: dict[str, Any], pose_path: Path) -> dict[str, np.ndarray]:
    poses = document.get("poses")
    if isinstance(poses, dict):
        if (
            document.get("matrix_direction") != "camera_to_world"
            or document.get("world_frame") != "base_link"
            or document.get("translation_unit") != "meter"
        ):
            raise MarkerError(
                f"{pose_path} must declare camera_to_world/base_link/meter"
            )
        keys = {
            "head_camera": "head",
            "left_hand_camera": "hand_left",
            "right_hand_camera": "hand_right",
        }
        sources = poses
    else:
        convention = document.get("convention", {})
        direction = convention.get("extrinsic_direction")
        if direction is not None and direction != "base_T_camera":
            raise MarkerError(f"{pose_path} does not use base_T_camera")
        if direction is None and "base_T_camera" not in convention:
            raise MarkerError(f"{pose_path} does not use base_T_camera")
        keys = {
            "head_camera": "head_rgb",
            "left_hand_camera": "hand_left_rgb",
            "right_hand_camera": "hand_right_rgb",
        }
        sources = document.get("extrinsics", document)

    result = {}
    for marker_id, key in keys.items():
        record = sources.get(key)
        value = record.get("matrix") if isinstance(record, dict) else record
        matrix = np.asarray(value, dtype=np.float64)
        if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
            raise MarkerError(f"{pose_path} has no valid {key} camera pose")
        result[marker_id] = matrix
    return result


def build_marker_document(
    pose_path: Path,
    joint_state_path: Path | None = None,
    urdf_path: Path = DEFAULT_URDF,
) -> dict[str, Any]:
    """Build JSON-safe marker poses and explicit semantic limitations."""
    pose_path = Path(pose_path).resolve()
    urdf_path = Path(urdf_path).resolve()
    pose_document = _read_json(pose_path)
    cameras = _camera_poses(pose_document, pose_path)

    state_document = (
        _read_json(Path(joint_state_path).resolve())
        if joint_state_path is not None and Path(joint_state_path).is_file()
        else pose_document
    )
    joint_positions = state_document.get(
        "joint_positions_rad", pose_document.get("joint_positions_rad")
    )
    if not isinstance(joint_positions, dict):
        raise MarkerError(
            f"no joint_positions_rad in {joint_state_path or pose_path}"
        )
    flanges = flange_poses_from_urdf(urdf_path, joint_positions)

    poses = {
        "origin": np.eye(4),
        **cameras,
        "left_gripper_reference_center": flanges["left"],
        "right_gripper_reference_center": flanges["right"],
    }
    kinds = {
        "origin": "robot_coordinate_origin",
        "head_camera": "camera",
        "left_hand_camera": "camera",
        "right_hand_camera": "camera",
        "left_gripper_reference_center": "flange_reference",
        "right_gripper_reference_center": "flange_reference",
    }
    markers = []
    for marker_id, pose in poses.items():
        markers.append(
            {
                "id": marker_id,
                "role": "marker",
                "kind": kinds[marker_id],
                "center_m": pose[:3, 3].round(6).tolist(),
                "pose_matrix": pose.round(9).tolist(),
                "color": MARKER_COLORS[marker_id],
                "collision_geometry": False,
            }
        )
    return {
        "schema_version": 1,
        "world_frame": "base_link",
        "unit": "meter",
        "markers": markers,
        "pose_source": str(pose_path),
        "pose_source_sha256": _sha256(pose_path),
        "joint_state_source": (
            str(Path(joint_state_path).resolve())
            if joint_state_path is not None and Path(joint_state_path).is_file()
            else str(pose_path)
        ),
        "urdf_source": str(urdf_path),
        "urdf_sha256": _sha256(urdf_path),
        "semantics": {
            "gripper_reference": "arm_l_end_link/arm_r_end_link flange poses",
            "installed_gripper_tcp_known": False,
            "simplified_hands": "visualization-only approximate palm and two fingers",
            "collision_geometry": False,
        },
    }


def _colored(mesh: trimesh.Trimesh, color: list[int], alpha: int) -> trimesh.Trimesh:
    mesh.visual.vertex_colors = np.array([*color, alpha], dtype=np.uint8)
    return mesh


def _placed(
    mesh: trimesh.Trimesh, pose: np.ndarray, local_offset: tuple[float, float, float]
) -> trimesh.Trimesh:
    local = np.eye(4)
    local[:3, 3] = local_offset
    mesh.apply_transform(pose @ local)
    mesh.apply_transform(VIEWER_FLIP)
    return mesh


def add_marker_geometry(
    scene: trimesh.Scene,
    marker_document: dict[str, Any],
) -> trimesh.Scene:
    """Add named display-only G2 markers to a viewer-flipped GLB scene."""
    lookup = {marker["id"]: marker for marker in marker_document["markers"]}

    for marker_id in (
        "origin",
        "head_camera",
        "left_hand_camera",
        "right_hand_camera",
    ):
        marker = lookup[marker_id]
        pose = np.asarray(marker["pose_matrix"], dtype=np.float64)
        color = marker["color"]
        radius = 0.022 if marker_id == "origin" else 0.018
        center = _colored(
            trimesh.creation.icosphere(subdivisions=2, radius=radius), color, 255
        )
        center.apply_translation(pose[:3, 3])
        center.apply_transform(VIEWER_FLIP)
        scene.add_geometry(center, geom_name=f"marker_{marker_id}_center")

        axes = trimesh.creation.axis(
            origin_size=radius * 0.45,
            transform=pose,
            axis_radius=0.0025,
            axis_length=0.18 if marker_id == "origin" else 0.09,
        )
        axes.apply_transform(VIEWER_FLIP)
        scene.add_geometry(axes, geom_name=f"marker_{marker_id}_axes")

        if marker_id != "origin":
            body = _colored(
                trimesh.creation.box(extents=[0.055, 0.038, 0.028]), color, 235
            )
            _placed(body, pose, (0.0, 0.0, 0.0))
            scene.add_geometry(body, geom_name=f"marker_{marker_id}_body")

    for side in ("left", "right"):
        marker_id = f"{side}_gripper_reference_center"
        marker = lookup[marker_id]
        pose = np.asarray(marker["pose_matrix"], dtype=np.float64)
        color = marker["color"]
        center = _colored(
            trimesh.creation.icosphere(subdivisions=2, radius=0.025), color, 255
        )
        center.apply_translation(pose[:3, 3])
        center.apply_transform(VIEWER_FLIP)
        scene.add_geometry(center, geom_name=f"marker_{marker_id}")

        axes = trimesh.creation.axis(
            origin_size=0.01,
            transform=pose,
            axis_radius=0.0025,
            axis_length=0.10,
        )
        axes.apply_transform(VIEWER_FLIP)
        scene.add_geometry(axes, geom_name=f"marker_{marker_id}_axes")

        palm = _colored(
            trimesh.creation.box(extents=[0.09, 0.065, 0.045]), color, 205
        )
        _placed(palm, pose, (0.0, 0.0, 0.04))
        scene.add_geometry(palm, geom_name=f"approximate_{side}_hand_palm")
        for finger_name, x_offset in (("finger_1", -0.032), ("finger_2", 0.032)):
            finger = _colored(
                trimesh.creation.box(extents=[0.022, 0.025, 0.105]), color, 205
            )
            _placed(finger, pose, (x_offset, 0.0, 0.112))
            scene.add_geometry(
                finger,
                geom_name=f"approximate_{side}_hand_{finger_name}",
            )
    return scene
