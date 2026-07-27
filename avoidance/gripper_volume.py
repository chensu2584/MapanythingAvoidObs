"""Operator-defined gripper volume for removing the robot's own hand voxels.

The G2 URDF ships an ``omnipicker`` end effector that is NOT the gripper bolted
to this robot, so its collision meshes cannot be used to delete gripper voxels
(they sit in the wrong place and have the wrong shape).  What *is* trustworthy
is the wrist-camera extrinsic, which ground-truth landmark reprojection has
verified.  This module therefore models the installed gripper as a simple box
anchored to the wrist camera, using dimensions measured by the operator.

Definition (operator supplied, 2026-07-27):

  * the box CENTRE lies ``centre_distance_m`` (default 7 cm) from the camera
    centre along the gripper direction -- i.e. the 15 cm length is centred there
    (7.5 cm each way) and likewise the width;
  * the box is ``length_m`` (15 cm, mount -> fingertip) along that direction,
    ``width_m`` (10 cm) across, and ``height_m`` (6 cm) thick;
  * the direction is 45 deg below the camera's forward axis.

"45 deg below" admits two reasonable anchors, so both are provided and the
caller picks:

  ``optical``  pitch the camera's optical axis (+Z) 45 deg toward the camera's
               own down axis (+Y).  Purely a function of the verified camera
               extrinsic; follows wrist roll exactly.
  ``world``    take the camera forward direction, project it onto the base_link
               horizontal plane, then tilt 45 deg toward world -Z (gravity).
               Less sensitive to wrist roll, but diverges from the real gripper
               whenever the wrist is rolled away from upright.

Nothing here is a confirmed TCP.  It is a conservative removal proxy for the
robot's own hand, not a grasp frame, and must not be published as gripper
kinematics.
"""

from __future__ import annotations

import dataclasses
from typing import Any

import numpy as np

# Operator-measured dimensions (metres).
DEFAULT_CENTRE_DISTANCE_M = 0.07   # camera centre -> box centre
DEFAULT_LENGTH_M = 0.15            # mount -> fingertip, along the gripper axis
DEFAULT_WIDTH_M = 0.10             # across the fingers
DEFAULT_HEIGHT_M = 0.06            # thickness
DEFAULT_PITCH_DEG = 45.0           # below the camera forward axis
ANCHORS = ("optical", "world")


@dataclasses.dataclass(frozen=True)
class GripperBox:
    """An oriented box approximating one installed gripper."""

    side: str
    anchor: str
    centre_m: np.ndarray          # box centre in base_link
    axes: np.ndarray              # 3x3, columns = unit length/width/height axes
    size_m: np.ndarray            # (length, width, height)
    camera_centre_m: np.ndarray
    direction: np.ndarray         # unit gripper direction (camera -> fingertip)

    def contains(self, points: np.ndarray) -> np.ndarray:
        """Boolean mask of points inside the box (base_link metres)."""
        pts = np.atleast_2d(np.asarray(points, dtype=np.float64))
        local = (pts - self.centre_m) @ self.axes      # project onto box axes
        return np.all(np.abs(local) <= self.size_m / 2 + 1e-9, axis=1)

    def corners(self) -> np.ndarray:
        """The eight corners, for visualisation/export."""
        half = self.size_m / 2
        signs = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)],
                         dtype=np.float64)
        return self.centre_m + (signs * half) @ self.axes.T

    def to_dict(self) -> dict[str, Any]:
        return {
            "side": self.side,
            "anchor": self.anchor,
            "centre_m": self.centre_m.round(6).tolist(),
            "size_m": self.size_m.round(6).tolist(),
            "axes_columns_length_width_height": self.axes.round(6).tolist(),
            "camera_centre_m": self.camera_centre_m.round(6).tolist(),
            "direction": self.direction.round(6).tolist(),
            "semantics": "operator_measured_removal_proxy_not_a_confirmed_tcp",
        }


def _orthonormal_basis(direction: np.ndarray, hint: np.ndarray) -> np.ndarray:
    """Build a right-handed basis whose first column is ``direction``."""
    length_axis = direction / np.linalg.norm(direction)
    width_axis = hint - (hint @ length_axis) * length_axis
    norm = np.linalg.norm(width_axis)
    if norm < 1e-6:                      # hint parallel to direction; pick any normal
        alternative = np.array([0.0, 0.0, 1.0])
        if abs(alternative @ length_axis) > 0.9:
            alternative = np.array([1.0, 0.0, 0.0])
        width_axis = alternative - (alternative @ length_axis) * length_axis
        norm = np.linalg.norm(width_axis)
    width_axis /= norm
    height_axis = np.cross(length_axis, width_axis)
    return np.column_stack((length_axis, width_axis, height_axis))


def gripper_direction(base_T_camera: np.ndarray, anchor: str = "optical",
                      pitch_deg: float = DEFAULT_PITCH_DEG) -> tuple[np.ndarray, np.ndarray]:
    """Return (unit gripper direction, width-axis hint) in base_link.

    ``optical`` rotates the camera's +Z (forward) toward its +Y (down) by
    ``pitch_deg``; ``world`` tilts the horizontal part of camera-forward toward
    world -Z by the same angle.
    """
    pose = np.asarray(base_T_camera, dtype=np.float64)
    right, down, forward = pose[:3, 0], pose[:3, 1], pose[:3, 2]
    angle = np.radians(pitch_deg)
    if anchor == "optical":
        direction = np.cos(angle) * forward + np.sin(angle) * down
        hint = right
    elif anchor == "world":
        horizontal = forward.copy()
        horizontal[2] = 0.0
        if np.linalg.norm(horizontal) < 1e-6:   # camera looks straight up/down
            horizontal = right.copy()
            horizontal[2] = 0.0
        horizontal /= np.linalg.norm(horizontal)
        direction = np.cos(angle) * horizontal + np.sin(angle) * np.array([0.0, 0.0, -1.0])
        hint = np.cross(np.array([0.0, 0.0, 1.0]), horizontal)
    else:
        raise ValueError(f"unknown gripper anchor {anchor!r}; expected one of {ANCHORS}")
    return direction / np.linalg.norm(direction), hint


def gripper_box(base_T_camera: np.ndarray, side: str, *, anchor: str = "optical",
                centre_distance_m: float = DEFAULT_CENTRE_DISTANCE_M,
                length_m: float = DEFAULT_LENGTH_M, width_m: float = DEFAULT_WIDTH_M,
                height_m: float = DEFAULT_HEIGHT_M,
                pitch_deg: float = DEFAULT_PITCH_DEG,
                margin_m: float = 0.0) -> GripperBox:
    """Build the removal box for one wrist camera.

    ``margin_m`` grows the box on every side; use it to be deliberately
    conservative about deleting the robot's own hand.
    """
    pose = np.asarray(base_T_camera, dtype=np.float64)
    if pose.shape != (4, 4) or not np.isfinite(pose).all():
        raise ValueError("base_T_camera must be a finite 4x4 matrix")
    direction, hint = gripper_direction(pose, anchor=anchor, pitch_deg=pitch_deg)
    camera_centre = pose[:3, 3]
    centre = camera_centre + direction * centre_distance_m
    axes = _orthonormal_basis(direction, hint)
    size = np.array([length_m, width_m, height_m], dtype=np.float64) + 2 * margin_m
    return GripperBox(side, anchor, centre, axes, size, camera_centre, direction)


def gripper_boxes(camera_poses: dict[str, np.ndarray], **kwargs) -> list[GripperBox]:
    """Build boxes for whichever wrist cameras are present.

    ``camera_poses`` maps ``"left"``/``"right"`` (or the raw
    ``hand_left_rgb``/``hand_right_rgb`` keys) to 4x4 base_T_camera matrices.
    """
    aliases = {"left": ("left", "hand_left", "hand_left_rgb"),
               "right": ("right", "hand_right", "hand_right_rgb")}
    boxes = []
    for side, keys in aliases.items():
        for key in keys:
            if key in camera_poses:
                boxes.append(gripper_box(camera_poses[key], side, **kwargs))
                break
    if not boxes:
        raise ValueError("no wrist camera poses found for gripper removal")
    return boxes


def remove_gripper_voxels(points: np.ndarray, boxes) -> np.ndarray:
    """Keep-mask dropping every point inside any gripper box."""
    pts = np.asarray(points, dtype=np.float64)
    keep = np.ones(len(pts), dtype=bool)
    for box in boxes:
        keep &= ~box.contains(pts)
    return keep
