"""G2 Pinocchio model and joint helpers."""

from __future__ import annotations

import hashlib
import tempfile
from pathlib import Path
from typing import Any, Iterable

import numpy as np

from .contracts import AvoidanceError, read_json, sha256_file

G2_JOINT_LAYOUT = {
    "body": tuple(f"idx{i:02d}_body_joint{i}" for i in range(1, 6)),
    "head": tuple(f"idx{i:02d}_head_joint{i - 10}" for i in range(11, 14)),
    "left": tuple(f"idx{i:02d}_arm_l_joint{i - 20}" for i in range(21, 28)),
    "right": tuple(f"idx{i:02d}_arm_r_joint{i - 60}" for i in range(61, 68)),
}
G2_REQUIRED_CAPTURE_JOINTS = tuple(name for group in ("body", "head", "left", "right") for name in G2_JOINT_LAYOUT[group])
G2_END_FRAMES = {"left": "gripper_l_center_link", "right": "gripper_r_center_link"}
DEFAULT_G2_URDF = Path(__file__).resolve().parents[2] / "G2/G2_parameters/G2_t2_crs_omnipicker/urdf/G2_t2_crs_omnipicker.urdf"


def _pin() -> Any:
    try:
        import pinocchio
        return pinocchio
    except ImportError as exc:
        raise AvoidanceError("G2 planning requires Pinocchio in the robot environment") from exc


def load_g2_capture_state(path: str | Path) -> tuple[dict[str, float], dict[str, Any]]:
    requested = Path(path).expanduser()
    if requested.is_dir():
        requested /= "capture_state.json"
    doc = read_json(requested)
    if doc.get("robot_profile") != "g2" or doc.get("world_frame") != "base_link":
        raise AvoidanceError("G2 capture state must declare g2/base_link")
    raw = doc.get("joint_positions_rad", {})
    missing = sorted(set(G2_REQUIRED_CAPTURE_JOINTS) - set(raw))
    if missing:
        raise AvoidanceError(f"G2 capture state is missing joints: {missing}")
    values = {name: float(raw[name]) for name in G2_REQUIRED_CAPTURE_JOINTS}
    if not np.isfinite(list(values.values())).all():
        raise AvoidanceError("G2 capture joints must be finite")
    return values, {"source": str(requested.resolve()), "sha256": sha256_file(requested), "joint_count": len(values), "kinematic_validation": doc.get("kinematic_validation")}


class G2RobotModel:
    def __init__(self, urdf_path: str | Path = DEFAULT_G2_URDF, *, joint_limit_margin_rad: float = 0.02):
        self.pin = _pin()
        self.urdf_path = Path(urdf_path).expanduser().resolve()
        self.joint_limit_margin_rad = float(joint_limit_margin_rad)
        parameter_root = self.urdf_path.parent.parent
        source = self.urdf_path.read_text().replace("package://genie_robot_description/meshes/", (parameter_root / "mesh").as_uri() + "/")
        with tempfile.NamedTemporaryFile("w", suffix=".urdf") as temp:
            temp.write(source)
            temp.flush()
            self.model, self.collision_model, self.visual_model = self.pin.buildModelsFromUrdf(temp.name)
        self.data = self.model.createData()
        self.geometry_data = self.pin.GeometryData(self.collision_model)
        self.urdf_sha256 = sha256_file(self.urdf_path)
        paths = sorted({Path(str(item.meshPath)).resolve() for item in self.collision_model.geometryObjects if Path(str(item.meshPath)).is_file()})
        digest = hashlib.sha256()
        for path in paths:
            try:
                relative = path.relative_to(parameter_root)
            except ValueError:
                relative = path
            digest.update(str(relative).encode())
            digest.update(bytes.fromhex(sha256_file(path)))
        self.collision_mesh_paths = tuple(paths)
        self.collision_mesh_bundle_sha256 = digest.hexdigest()
        self._joint_ids = {name: int(self.model.getJointId(name)) for name in G2_REQUIRED_CAPTURE_JOINTS}
        self._arm_indices = {side: np.asarray([self.model.joints[self._joint_ids[name]].idx_q for name in G2_JOINT_LAYOUT[side]], dtype=int) for side in ("left", "right")}
        self._arm_v_indices = {side: np.asarray([self.model.joints[self._joint_ids[name]].idx_v for name in G2_JOINT_LAYOUT[side]], dtype=int) for side in ("left", "right")}

    def neutral_configuration(self) -> np.ndarray:
        return np.asarray(self.pin.neutral(self.model), dtype=float)

    def configuration_from_positions(self, positions: dict[str, float]) -> np.ndarray:
        q = self.neutral_configuration()
        for name, value in positions.items():
            jid = int(self.model.getJointId(name))
            if jid:
                q[int(self.model.joints[jid].idx_q)] = float(value)
        self.validate_configuration(q)
        return q

    def validate_configuration(self, q: np.ndarray) -> None:
        q = np.asarray(q)
        if q.shape != (self.model.nq,) or not np.isfinite(q).all():
            raise AvoidanceError("Invalid G2 configuration")
        for side in ("left", "right"):
            arm = self.arm_configuration(q, side)
            lower, upper = self.arm_limits(side)
            if np.any(arm < lower) or np.any(arm > upper):
                raise AvoidanceError(f"G2 {side} arm violates joint limits")

    def arm_limits(self, side: str) -> tuple[np.ndarray, np.ndarray]:
        indices = self._arm_indices[side]
        return (np.asarray(self.model.lowerPositionLimit[indices]) + self.joint_limit_margin_rad, np.asarray(self.model.upperPositionLimit[indices]) - self.joint_limit_margin_rad)

    def arm_configuration(self, q: np.ndarray, side: str) -> np.ndarray:
        return np.asarray(q)[self._arm_indices[side]].copy()

    def with_arm_configuration(self, q: np.ndarray, side: str, arm: Iterable[float]) -> np.ndarray:
        values = np.asarray(tuple(arm), dtype=float)
        lower, upper = self.arm_limits(side)
        if values.shape != (7,) or np.any(values < lower) or np.any(values > upper):
            raise AvoidanceError("G2 arm configuration violates joint limits")
        result = np.asarray(q).copy()
        result[self._arm_indices[side]] = values
        return result

    def frame_pose(self, q: np.ndarray, frame: str) -> np.ndarray:
        self.pin.forwardKinematics(self.model, self.data, q)
        self.pin.updateFramePlacements(self.model, self.data)
        placement = self.data.oMf[self.model.getFrameId(frame)]
        result = np.eye(4)
        result[:3, :3] = placement.rotation
        result[:3, 3] = placement.translation
        return result

    def collision_geometry_centers(self, q: np.ndarray) -> np.ndarray:
        self.pin.updateGeometryPlacements(self.model, self.data, self.collision_model, self.geometry_data, q)
        return np.asarray([item.translation for item in self.geometry_data.oMg])

    def joint_positions(self, q: np.ndarray, names: Iterable[str]) -> dict[str, float]:
        return {name: float(q[self.model.joints[self._joint_ids[name]].idx_q]) for name in names}

    def metadata(self) -> dict[str, Any]:
        return {"robot_profile": "g2", "urdf": str(self.urdf_path), "urdf_sha256": self.urdf_sha256, "collision_mesh_file_count": len(self.collision_mesh_paths), "collision_mesh_bundle_sha256": self.collision_mesh_bundle_sha256, "configuration_size": int(self.model.nq), "velocity_size": int(self.model.nv), "collision_geometry_count": int(self.collision_model.ngeoms), "active_arm_joints": {side: list(G2_JOINT_LAYOUT[side]) for side in ("left", "right")}, "end_frames": G2_END_FRAMES, "joint_limit_margin_rad": self.joint_limit_margin_rad}


def load_arm_goal(path: str | Path, side: str) -> np.ndarray:
    doc = read_json(path)
    raw = doc.get("arm_joint_positions_rad")
    if raw is None:
        raw = [doc["joint_positions_rad"][name] for name in G2_JOINT_LAYOUT[side]]
    result = np.asarray(raw, dtype=float)
    if result.shape != (7,) or not np.isfinite(result).all():
        raise AvoidanceError("Joint goal must contain seven finite radians")
    return result


def load_pose_goal(path: str | Path) -> np.ndarray:
    matrix = np.asarray(read_json(path).get("base_T_goal"), dtype=float)
    if matrix.shape != (4, 4):
        raise AvoidanceError("Pose goal requires a 4x4 base_T_goal")
    return matrix
