#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import socket
import subprocess
import sys
import time
from datetime import datetime
from typing import Any
from urllib.parse import urlparse


DEFAULT_G2_BODY_JOINTS_RAD = {
    "idx01_body_joint1": -0.19447103125832138,
    "idx02_body_joint2": 0.07993353640933853,
    "idx03_body_joint3": 0.9248275754282509,
    "idx04_body_joint4": 0.0,
    "idx05_body_joint5": 0.0,
}

HEAD_URDF_JOINTS = {
    "yaw": "idx11_head_joint1",
    "roll": "idx12_head_joint2",
    "pitch": "idx13_head_joint3",
}

ARM_LEFT_URDF_JOINTS = [
    "idx21_arm_l_joint1",
    "idx22_arm_l_joint2",
    "idx23_arm_l_joint3",
    "idx24_arm_l_joint4",
    "idx25_arm_l_joint5",
    "idx26_arm_l_joint6",
    "idx27_arm_l_joint7",
]

ARM_RIGHT_URDF_JOINTS = [
    "idx61_arm_r_joint1",
    "idx62_arm_r_joint2",
    "idx63_arm_r_joint3",
    "idx64_arm_r_joint4",
    "idx65_arm_r_joint5",
    "idx66_arm_r_joint6",
    "idx67_arm_r_joint7",
]

LINKS_FOR_VALIDATION = [
    "arm_l_end_link",
    "arm_r_end_link",
    "gripper_l_camera_link",
    "gripper_r_camera_link",
    "head_link3",
]

CAPTURE_INTRINSICS = {
    "head_rgb": "intrinsic_head_front_rgb.json",
    "head_depth": "intrinsic_head_front_depth.json",
    "hand_left_rgb": "intrinsic_hand_left_rgb.json",
    "hand_right_rgb": "intrinsic_hand_right_rgb.json",
    "hand_left_depth": "intrinsic_hand_left_depth.json",
    "hand_right_depth": "intrinsic_hand_right_depth.json",
}

CAMERA_META = {
    "head_rgb": ("head front RGB", "kHeadColor", "color", "head_rgb.png"),
    "head_depth": ("head front depth", "kHeadDepth", "depth", "head_depth_raw16.png"),
    "hand_left_rgb": ("left hand RGB", "kHandLeftColor", "color", "hand_left_rgb.png"),
    "hand_right_rgb": ("right hand RGB", "kHandRightColor", "color", "hand_right_rgb.png"),
    "hand_left_depth": ("left hand depth", "kHandLeftDepth", "depth", "hand_left_depth_raw16.png"),
    "hand_right_depth": ("right hand depth", "kHandRightDepth", "depth", "hand_right_depth_raw16.png"),
}


def repo_root() -> Path:
    """Return the MapAnythingTest workspace containing Avoid and G2."""
    return Path(__file__).resolve().parents[2]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Capture G2 head/hand camera data and calibrated base_T_camera extrinsics."
    )
    parser.add_argument("--g2-root", type=Path, default=repo_root() / "G2")
    parser.add_argument("--output-root", type=Path, default=repo_root() / "G2")
    parser.add_argument("--session-dir", type=Path, default=None)
    parser.add_argument("--count", type=int, default=1)
    parser.add_argument("--interval-sec", type=float, default=0.0)
    parser.add_argument("--timeout-ms", type=float, default=1000.0)
    parser.add_argument("--sync-threshold-ms", type=float, default=50.0)
    parser.add_argument("--capture-hand-depth", action="store_true")
    parser.add_argument("--allow-missing-images", action="store_true")
    parser.add_argument("--body-joints-json", type=str, default="")
    parser.add_argument("--allow-missing-live-joints", action="store_true")
    parser.add_argument("--no-dds-env", action="store_true")
    parser.add_argument("--skip-discovery-check", action="store_true")
    parser.add_argument("--discovery-timeout-sec", type=float, default=2.0)
    parser.add_argument("--validate-existing-snapshot", type=Path, default=None)
    return parser.parse_args()


def load_json(path: Path) -> Any:
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def write_json(path: Path, value: Any) -> None:
    with path.open("w", encoding="utf-8") as f:
        json.dump(value, f, indent=2, ensure_ascii=False)
        f.write("\n")


def now_tag() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def setup_dds_env() -> None:
    try:
        from corobot.utils.dds_setting import dds_env_set

        dds_env_set()
        return
    except Exception:
        pass

    lines = subprocess.check_output(["ip", "-o", "-4", "addr", "list"], text=True).splitlines()
    local_ips = [line.split()[3].split("/")[0] for line in lines if "10.42.0." in line]
    if not local_ips:
        raise RuntimeError("no local 10.42.0.* address found; cannot set DDS robot env")
    os.environ["LOCATOR_IP"] = local_ips[0]
    os.environ["AORTA_DISCOVERY_URI"] = "http://10.42.0.101:2379"
    os.environ["PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION"] = "python"


def check_discovery_endpoint(timeout_sec: float) -> None:
    uri = os.environ.get("AORTA_DISCOVERY_URI", "http://10.42.0.101:2379")
    parsed = urlparse(uri)
    host = parsed.hostname
    port = parsed.port or (443 if parsed.scheme == "https" else 80)
    if not host:
        raise RuntimeError(f"invalid AORTA_DISCOVERY_URI: {uri}")
    try:
        with socket.create_connection((host, port), timeout=timeout_sec):
            return
    except OSError as exc:
        raise RuntimeError(f"cannot connect to Aorta discovery endpoint {host}:{port}: {exc}") from exc


def quat_wxyz_to_matrix(q: dict[str, float]):
    import numpy as np

    w, x, y, z = (float(q[k]) for k in ("w", "x", "y", "z"))
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm == 0:
        raise ValueError("zero quaternion")
    w, x, y, z = w / norm, x / norm, y / norm, z / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def matrix_from_param(path: Path):
    import numpy as np

    obj = load_json(path)
    m = np.eye(4, dtype=float)
    m[:3, :3] = quat_wxyz_to_matrix(obj["rotation"])
    m[:3, 3] = [float(obj["translation"][axis]) for axis in ("x", "y", "z")]
    return m


def matrix_to_quat_xyzw(rotation):
    trace = float(rotation[0, 0] + rotation[1, 1] + rotation[2, 2])
    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (rotation[2, 1] - rotation[1, 2]) / s
        y = (rotation[0, 2] - rotation[2, 0]) / s
        z = (rotation[1, 0] - rotation[0, 1]) / s
    elif rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
        s = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
        w = (rotation[2, 1] - rotation[1, 2]) / s
        x = 0.25 * s
        y = (rotation[0, 1] + rotation[1, 0]) / s
        z = (rotation[0, 2] + rotation[2, 0]) / s
    elif rotation[1, 1] > rotation[2, 2]:
        s = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
        w = (rotation[0, 2] - rotation[2, 0]) / s
        x = (rotation[0, 1] + rotation[1, 0]) / s
        y = 0.25 * s
        z = (rotation[1, 2] + rotation[2, 1]) / s
    else:
        s = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
        w = (rotation[1, 0] - rotation[0, 1]) / s
        x = (rotation[0, 2] + rotation[2, 0]) / s
        y = (rotation[1, 2] + rotation[2, 1]) / s
        z = 0.25 * s
    return [float(x), float(y), float(z), float(w)]


def serialize_transform(matrix) -> dict[str, Any]:
    import numpy as np

    inv = np.linalg.inv(matrix)
    return {
        "matrix": matrix.tolist(),
        "translation_xyz_m": [float(x) for x in matrix[:3, 3]],
        "quaternion_xyzw": matrix_to_quat_xyzw(matrix[:3, :3]),
        "inverse_matrix": inv.tolist(),
    }


def transform_error(a, b) -> dict[str, float]:
    import numpy as np

    dt = float(np.linalg.norm(a[:3, 3] - b[:3, 3]))
    r = a[:3, :3].T @ b[:3, :3]
    c = (float(np.trace(r)) - 1.0) / 2.0
    c = max(-1.0, min(1.0, c))
    return {"translation_error_m": dt, "rotation_error_deg": math.degrees(math.acos(c))}


class G2Kinematics:
    def __init__(self, urdf_path: Path):
        import pinocchio as pin

        self.pin = pin
        self.model = pin.buildModelFromUrdf(str(urdf_path))
        self.data = self.model.createData()

    def forward(self, joint_positions: dict[str, float]) -> tuple[dict[str, Any], list[str]]:
        import numpy as np

        q = self.pin.neutral(self.model)
        missing = []
        for name, value in joint_positions.items():
            joint_id = self.model.getJointId(name)
            if joint_id == 0:
                missing.append(name)
                continue
            if self.model.nqs[joint_id] != 1:
                missing.append(name)
                continue
            q[self.model.idx_qs[joint_id]] = float(value)
        self.pin.forwardKinematics(self.model, self.data, q)
        self.pin.updateFramePlacements(self.model, self.data)

        result = {}
        for link_name in LINKS_FOR_VALIDATION:
            frame_id = self.model.getFrameId(link_name)
            if frame_id >= len(self.model.frames):
                missing.append(link_name)
                continue
            se3 = self.data.oMf[frame_id]
            matrix = np.eye(4, dtype=float)
            matrix[:3, :3] = se3.rotation
            matrix[:3, 3] = se3.translation
            result[link_name] = matrix
        return result, missing


def sensor_files(sensor_dir: Path) -> list[str]:
    return [str(path.relative_to(sensor_dir.parent)) for path in sorted(sensor_dir.glob("*.json"))]


def load_body_joints(value: str) -> dict[str, float]:
    if not value:
        return dict(DEFAULT_G2_BODY_JOINTS_RAD)
    maybe_path = Path(value)
    parsed = load_json(maybe_path) if maybe_path.exists() else json.loads(value)
    joints = dict(DEFAULT_G2_BODY_JOINTS_RAD)
    for name, joint_value in parsed.items():
        joints[name] = float(joint_value)
    return joints


def build_extrinsics(fk_base_t_link: dict[str, Any], sensor_dir: Path) -> dict[str, Any]:
    head_rgb = fk_base_t_link["head_link3"] @ matrix_from_param(sensor_dir / "extrinsic_end_T_head_front_rgbd.json")
    hand_left_rgb = fk_base_t_link["arm_l_end_link"] @ matrix_from_param(sensor_dir / "extrinsic_end_T_hand_left_rgbd.json")
    hand_right_rgb = fk_base_t_link["arm_r_end_link"] @ matrix_from_param(sensor_dir / "extrinsic_end_T_hand_right_rgbd.json")
    return {
        "head_rgb": head_rgb,
        "head_depth": head_rgb @ matrix_from_param(sensor_dir / "extrinsic_head_front_rgb_T_depth.json"),
        "hand_left_rgb": hand_left_rgb,
        "hand_right_rgb": hand_right_rgb,
        "hand_left_depth": hand_left_rgb @ matrix_from_param(sensor_dir / "extrinsic_hand_left_rgb_T_depth.json"),
        "hand_right_depth": hand_right_rgb @ matrix_from_param(sensor_dir / "extrinsic_hand_right_rgb_T_depth.json"),
    }


def validation_from_fk(fk_base_t_link: dict[str, Any], extrinsics: dict[str, Any]) -> dict[str, Any]:
    return {
        "fk_vs_sdk_tf": {},
        "sensor_calibration_vs_urdf_visual_camera_link_info": {
            "hand_left_rgb": transform_error(extrinsics["hand_left_rgb"], fk_base_t_link["gripper_l_camera_link"])
            | {"meaning": "calibrated optical/RGBD frame compared with the URDF camera link; informational only"},
            "hand_right_rgb": transform_error(extrinsics["hand_right_rgb"], fk_base_t_link["gripper_r_camera_link"])
            | {"meaning": "calibrated optical/RGBD frame compared with the URDF camera link; informational only"},
        },
        "notes": [
            "base_T_camera is computed from G2 URDF FK and G2_parameters/sensor extrinsics.",
            "Current a2d_sdk path exposes camera and joint DDS streams but not the SDK TF tree; fk_vs_sdk_tf is empty unless a TF backend is added.",
        ],
    }


def validate_existing_snapshot(snapshot: Path, g2_root: Path) -> int:
    import numpy as np

    meta_path = snapshot / "camera_extrinsics.json"
    meta = load_json(meta_path)
    urdf = g2_root / "G2_parameters/G2_t2_crs_omnipicker/urdf/G2_t2_crs_omnipicker.urdf"
    sensor_dir = g2_root / "G2_parameters/sensor"
    kin = G2Kinematics(urdf)
    fk, missing = kin.forward({name: float(value) for name, value in meta["joint_positions_rad"].items()})
    extrinsics = build_extrinsics(fk, sensor_dir)
    max_translation = 0.0
    max_rotation = 0.0
    print(f"validate {snapshot}")
    if missing:
        print("missing:", ", ".join(missing))
    for name, matrix in extrinsics.items():
        old = np.array(meta["extrinsics"][name]["matrix"], dtype=float)
        err = transform_error(matrix, old)
        max_translation = max(max_translation, err["translation_error_m"])
        max_rotation = max(max_rotation, err["rotation_error_deg"])
        print(f"{name}: {err['translation_error_m']:.12g} m, {err['rotation_error_deg']:.12g} deg")
    print(f"max: {max_translation:.12g} m, {max_rotation:.12g} deg")
    return 0 if max_translation < 1e-9 and max_rotation < 1e-6 and not missing else 1


def wait_for_image(camera: Any, camera_name: str, timeout_sec: float):
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        image, timestamp = camera.get_latest_image(camera_name)
        if image is not None and timestamp is not None:
            return image, int(timestamp)
        time.sleep(0.02)
    return None, None


def nearest_image(camera: Any, camera_name: str, timestamp_ns: int):
    image, timestamp = camera.get_image_nearest(camera_name, timestamp_ns)
    if timestamp is not None:
        timestamp = int(timestamp)
    return image, timestamp


def wait_for_nearest_state(robot: Any, method_name: str, timestamp_ns: int, timeout_sec: float):
    method = getattr(robot, method_name)
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        values, state_timestamp = method(timestamp_ns)
        if values is not None and state_timestamp is not None:
            return list(values), int(state_timestamp)
        time.sleep(0.02)
    return None, None


def collect_live_joint_positions(
    robot: Any,
    timestamp_ns: int,
    timeout_sec: float,
    body_joints: dict[str, float],
    allow_missing: bool,
) -> tuple[dict[str, float], list[str], dict[str, Any]]:
    joint_positions = dict(body_joints)
    missing = []
    sources = {}

    head, head_ts = wait_for_nearest_state(robot, "head_joint_states_nearest", timestamp_ns, timeout_sec)
    if head is None or len(head) < 2:
        missing.extend([HEAD_URDF_JOINTS["yaw"], HEAD_URDF_JOINTS["pitch"]])
    else:
        joint_positions[HEAD_URDF_JOINTS["yaw"]] = math.radians(float(head[0]))
        joint_positions[HEAD_URDF_JOINTS["roll"]] = 0.0
        joint_positions[HEAD_URDF_JOINTS["pitch"]] = math.radians(float(head[1]))
        sources["head_dds_deg"] = {"timestamp_ns": head_ts, "values": head}

    arm, arm_ts = wait_for_nearest_state(robot, "arm_joint_states_nearest", timestamp_ns, timeout_sec)
    if arm is None or len(arm) < 14:
        missing.extend(ARM_LEFT_URDF_JOINTS + ARM_RIGHT_URDF_JOINTS)
    else:
        for name, value in zip(ARM_LEFT_URDF_JOINTS + ARM_RIGHT_URDF_JOINTS, arm[:14]):
            joint_positions[name] = float(value)
        sources["arm_dds_rad"] = {"timestamp_ns": arm_ts, "values": arm[:14]}

    if missing and not allow_missing:
        raise RuntimeError("missing live joints: " + ", ".join(missing))
    for name in missing:
        joint_positions.setdefault(name, 0.0)
    return joint_positions, missing, sources


def save_depth_visual(depth, output_path: Path) -> None:
    import cv2
    import numpy as np

    depth = depth.squeeze()
    valid = depth[(depth > 0) & (depth < 65535)]
    if valid.size == 0:
        visual = np.zeros(depth.shape, dtype=np.uint8)
    else:
        hi = float(np.percentile(valid, 99.0))
        lo = float(np.percentile(valid, 1.0))
        if hi <= lo:
            hi = float(valid.max())
            lo = float(valid.min())
        scale = 255.0 / max(hi - lo, 1.0)
        visual = np.clip((depth.astype(np.float32) - lo) * scale, 0, 255).astype(np.uint8)
    color = cv2.applyColorMap(visual, cv2.COLORMAP_TURBO)
    cv2.imwrite(str(output_path), color)


def save_capture_images(snapshot_dir: Path, frames: dict[str, tuple[Any, int | None]]) -> dict[str, Any]:
    import cv2
    import numpy as np

    captures = {}
    for name, (frame, timestamp_ns) in frames.items():
        description, camera_type, kind, saved_path = CAMERA_META[name]
        entry = {
            "description": description,
            "camera_type": camera_type,
            "kind": kind,
            "timestamp_ns": timestamp_ns,
            "saved_path": saved_path,
            "error": "",
            "shape": list(frame.shape) if frame is not None else None,
        }
        if frame is None:
            entry["error"] = "no frame captured"
        elif kind == "depth":
            depth = frame.squeeze().astype(np.uint16, copy=False)
            cv2.imwrite(str(snapshot_dir / saved_path), depth)
            if name == "head_depth":
                save_depth_visual(depth, snapshot_dir / "head_depth_visual.png")
        else:
            cv2.imwrite(str(snapshot_dir / saved_path), frame)
        captures[name] = entry
    return captures


def write_validation_summary(path: Path, validation: dict[str, Any]) -> None:
    lines = ["Validation summary"]
    lines.append("fk_vs_sdk_tf:")
    if validation.get("fk_vs_sdk_tf"):
        for name, err in validation["fk_vs_sdk_tf"].items():
            lines.append(f"  {name}: translation={err['translation_error_m']:.4f} m, rotation={err['rotation_error_deg']:.2f} deg")
    else:
        lines.append("  unavailable")
    lines.append("sensor_calibration_vs_urdf_visual_camera_link_info (INFO only, not pass/fail):")
    for name, err in validation.get("sensor_calibration_vs_urdf_visual_camera_link_info", {}).items():
        lines.append(f"  {name}: translation={err['translation_error_m']:.4f} m, rotation={err['rotation_error_deg']:.2f} deg")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def capture_one(
    camera: Any,
    robot: Any,
    snapshot_dir: Path,
    args: argparse.Namespace,
    kin: G2Kinematics,
    sensor_dir: Path,
    urdf: Path,
    body_joints: dict[str, float],
) -> Path:
    timeout_sec = args.timeout_ms / 1000.0
    head_frame, head_ts = wait_for_image(camera, "head", timeout_sec)
    if head_frame is None or head_ts is None:
        raise RuntimeError("timeout waiting for head camera")

    frames = {
        "head_rgb": (head_frame, head_ts),
        "head_depth": nearest_image(camera, "head_depth", head_ts),
        "hand_left_rgb": nearest_image(camera, "hand_left", head_ts),
        "hand_right_rgb": nearest_image(camera, "hand_right", head_ts),
    }
    if args.capture_hand_depth:
        frames["hand_left_depth"] = nearest_image(camera, "hand_left_depth", head_ts)
        frames["hand_right_depth"] = nearest_image(camera, "hand_right_depth", head_ts)

    missing_images = [name for name, (frame, frame_ts) in frames.items() if frame is None or frame_ts is None]
    if missing_images and not args.allow_missing_images:
        raise RuntimeError("missing required images: " + ", ".join(missing_images))
    sync_ms = {
        name: abs(int(frame_ts) - int(head_ts)) / 1e6
        for name, (_frame, frame_ts) in frames.items()
        if frame_ts is not None
    }
    late_images = [name for name, delta_ms in sync_ms.items() if delta_ms > args.sync_threshold_ms]
    if late_images and not args.allow_missing_images:
        formatted = ", ".join(f"{name}={sync_ms[name]:.3f}ms" for name in late_images)
        raise RuntimeError(f"camera sync exceeds {args.sync_threshold_ms:.3f} ms: {formatted}")

    joint_positions, missing_live, joint_sources = collect_live_joint_positions(
        robot, head_ts, timeout_sec, body_joints, args.allow_missing_live_joints
    )
    fk, missing_fk = kin.forward(joint_positions)
    extrinsic_mats = build_extrinsics(fk, sensor_dir)
    validation = validation_from_fk(fk, extrinsic_mats)

    captures = save_capture_images(snapshot_dir, frames)
    for name, entry in captures.items():
        intrinsic_file = CAPTURE_INTRINSICS.get(name)
        if intrinsic_file:
            entry["intrinsic"] = load_json(sensor_dir / intrinsic_file)

    validation["camera_sync_ms_vs_head_rgb"] = sync_ms
    validation["camera_sync_threshold_ms"] = args.sync_threshold_ms
    validation["joint_sources"] = joint_sources
    validation["missing_live_joints_using_zero"] = missing_live
    validation["missing_fk_names"] = missing_fk

    meta = {
        "created_at": datetime.now().isoformat(timespec="seconds"),
        "script": Path(__file__).name,
        "output_dir": str(snapshot_dir),
        "base_link": "base_link",
        "urdf": str(urdf.relative_to(args.g2_root)),
        "sensor_dir": str(sensor_dir.relative_to(args.g2_root)),
        "sensor_files_seen": sensor_files(sensor_dir),
        "camera_timeout_ms": args.timeout_ms,
        "pose_source": "fk_from_a2d_dds_joints_and_g2_sensor_params",
        "joint_positions_rad": joint_positions,
        "missing_joints_using_zero": missing_live + missing_fk,
        "captures": captures,
        "extrinsics": {name: serialize_transform(matrix) for name, matrix in extrinsic_mats.items()},
        "sdk_tf_base_T_link": {},
        "validation": validation,
        "convention": {
            "extrinsic_direction": "base_T_camera",
            "camera_axes": "OpenCV RDF: +X right, +Y down, +Z forward",
            "length_unit": "meter",
            "depth_unit": "uint16 millimeter for *_depth_raw16.png",
        },
        "fk_base_T_link": {name: serialize_transform(matrix) for name, matrix in fk.items()},
    }
    write_json(snapshot_dir / "camera_extrinsics.json", meta)
    write_validation_summary(snapshot_dir / "validation_summary.txt", validation)
    return snapshot_dir


def run_live_capture(args: argparse.Namespace) -> int:
    if not args.no_dds_env:
        setup_dds_env()
    if not args.skip_discovery_check:
        check_discovery_endpoint(args.discovery_timeout_sec)

    from a2d_sdk.robot import CosineCamera, RobotDds

    g2_root = args.g2_root.resolve()
    sensor_dir = g2_root / "G2_parameters/sensor"
    urdf = g2_root / "G2_parameters/G2_t2_crs_omnipicker/urdf/G2_t2_crs_omnipicker.urdf"
    body_joints = load_body_joints(args.body_joints_json)
    kin = G2Kinematics(urdf)

    camera_names = ["head", "head_depth", "hand_left", "hand_right"]
    if args.capture_hand_depth:
        camera_names += ["hand_left_depth", "hand_right_depth"]
    camera = CosineCamera(camera_names)
    robot = RobotDds()

    session_dir = args.session_dir
    if session_dir is None:
        session_dir = args.output_root / f"session_{now_tag()}"
    session_dir.mkdir(parents=True, exist_ok=True)

    try:
        for idx in range(1, args.count + 1):
            tag = now_tag()
            snapshot_dir = session_dir / f"snapshot_{tag}_{idx:04d}"
            snapshot_dir.mkdir(parents=True, exist_ok=False)
            saved = capture_one(camera, robot, snapshot_dir, args, kin, sensor_dir, urdf, body_joints)
            print(saved)
            if idx < args.count and args.interval_sec > 0:
                time.sleep(args.interval_sec)
    finally:
        try:
            camera.close()
        finally:
            robot.shutdown()
    return 0


def main() -> int:
    args = parse_args()
    if args.validate_existing_snapshot is not None:
        return validate_existing_snapshot(args.validate_existing_snapshot.resolve(), args.g2_root.resolve())
    if args.count < 1:
        raise ValueError("--count must be >= 1")
    return run_live_capture(args)


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        sys.exit(1)
