"""Fail-closed G2 installed end-effector contract."""

from __future__ import annotations

import dataclasses
from pathlib import Path
from typing import Any

from .contracts import AvoidanceError, read_json, sha256_file

DEFAULT_G2_END_EFFECTOR_CONFIG = Path(__file__).resolve().parents[1] / "configs" / "g2_end_effector_model.json"


@dataclasses.dataclass(frozen=True)
class EndEffectorStatus:
    path: Path
    document: dict[str, Any]

    def compatibility_report(self, robot: Any) -> dict[str, Any]:
        doc = self.document
        tcp = doc.get("tcp_frames", {})
        checks = {
            "confirmed": doc.get("confirmed") is True,
            "urdf_matches_installed": doc.get("urdf_matches_installed") is True,
            "collision_geometry_confirmed": doc.get("collision_geometry_confirmed") is True,
            "tcp_frames_confirmed": doc.get("tcp_frames_confirmed") is True,
            "robot_urdf_sha256_matches": doc.get("urdf_sha256") == robot.urdf_sha256,
            "collision_mesh_bundle_sha256_matches": doc.get("collision_mesh_bundle_sha256") == robot.collision_mesh_bundle_sha256,
            "tcp_frames_present": all(isinstance(tcp.get(side), str) and tcp.get(side) for side in ("left", "right")),
            "tcp_frames_exist_in_robot": all(isinstance(tcp.get(side), str) and robot.model.existFrame(tcp.get(side)) for side in ("left", "right")),
            "declared_blockers_clear": not doc.get("blockers"),
            "required_next_inputs_clear": not doc.get("required_next_inputs"),
        }
        blockers = list(doc.get("blockers", []))
        labels = {
            "confirmed": "End-effector configuration is not confirmed.",
            "urdf_matches_installed": "URDF end-effector does not match the installed gripper.",
            "collision_geometry_confirmed": "Installed gripper collision geometry is not confirmed.",
            "tcp_frames_confirmed": "Installed gripper TCP frames are not confirmed.",
            "tcp_frames_present": "Left/right installed gripper TCP frames are missing.",
            "tcp_frames_exist_in_robot": "Configured TCP frames do not exist in the robot model.",
            "declared_blockers_clear": "End-effector contract still declares unresolved blockers.",
            "required_next_inputs_clear": "End-effector contract still declares required inputs.",
        }
        blockers.extend(labels[key] for key, value in checks.items() if not value and key in labels)
        return {
            "ready": all(checks.values()),
            "source": str(self.path),
            "source_sha256": sha256_file(self.path),
            "urdf_end_effector_model": doc.get("urdf_end_effector_model"),
            "installed_end_effector_model": doc.get("installed_end_effector_model"),
            "checks": checks,
            "tcp_frames": {"left": tcp.get("left"), "right": tcp.get("right")},
            "blockers": blockers,
            "required_next_inputs": list(doc.get("required_next_inputs", [])),
        }

    def require_compatible(self, robot: Any) -> dict[str, Any]:
        report = self.compatibility_report(robot)
        if not report["ready"]:
            raise AvoidanceError("G2 end-effector model is not planning-ready: " + "; ".join(report["blockers"]))
        return report


def load_end_effector_model_status(path: str | Path = DEFAULT_G2_END_EFFECTOR_CONFIG) -> EndEffectorStatus:
    resolved = Path(path).expanduser().resolve()
    return EndEffectorStatus(resolved, read_json(resolved))
