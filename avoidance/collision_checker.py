"""HPP-FCL checks for the G2 model and clustered scenes."""

from __future__ import annotations

import dataclasses
from pathlib import Path
from typing import Any

import numpy as np

from .contracts import AvoidanceError, read_json, sha256_file
from .end_effector_model import DEFAULT_G2_END_EFFECTOR_CONFIG, load_end_effector_model_status


@dataclasses.dataclass(frozen=True)
class CollisionReport:
    valid: bool
    self_collisions: tuple[tuple[str, str], ...]
    environment_collisions: tuple[tuple[str, int], ...]
    minimum_environment_clearance_m: float
    checked_self_pairs: int
    checked_environment_pairs: int

    def to_dict(self) -> dict[str, Any]:
        return dataclasses.asdict(self)


class G2CollisionChecker:
    def __init__(self, robot: Any, scene: Any, *, environment_inflation_m: float = 0.08, required_clearance_m: float = 0.02, allowed_self_collisions_path: str | Path | None = None, end_effector_config_path: str | Path = DEFAULT_G2_END_EFFECTOR_CONFIG, arm_body_demo: bool = False):
        import hppfcl
        self.hppfcl = hppfcl
        self.robot, self.scene = robot, scene
        self.environment_inflation_m = float(environment_inflation_m)
        self.required_clearance_m = float(required_clearance_m)
        self.total_environment_margin_m = self.environment_inflation_m + self.required_clearance_m
        self.arm_body_demo = bool(arm_body_demo)
        status = load_end_effector_model_status(end_effector_config_path)
        self.end_effector_report = status.compatibility_report(robot) if arm_body_demo else status.require_compatible(robot)
        self.ignored_geometry_indices = frozenset(i for i, g in enumerate(robot.collision_model.geometryObjects) if arm_body_demo and str(g.name).startswith("gripper_"))
        self.ignored_geometry_names = tuple(str(robot.collision_model.geometryObjects[i].name) for i in sorted(self.ignored_geometry_indices))
        self.allowed_self_collisions_path = Path(allowed_self_collisions_path or Path(__file__).resolve().parents[1] / "configs/g2_allowed_self_collisions.json").resolve()
        document = read_json(self.allowed_self_collisions_path)
        if document.get("urdf_sha256") != robot.urdf_sha256 or document.get("collision_mesh_bundle_sha256") != robot.collision_mesh_bundle_sha256:
            raise AvoidanceError("Allowed collision matrix model hash mismatch")
        self.allowed_self_collisions = {frozenset((item["geometry_a"], item["geometry_b"])) for item in document["pairs"]}
        self._configure_pairs()
        self.environment = tuple(self._environment(item) for item in scene.primitives)
        self.collision_request = hppfcl.CollisionRequest()
        self.distance_request = hppfcl.DistanceRequest()

    def _configure_pairs(self) -> None:
        model = self.robot.collision_model
        model.removeAllCollisionPairs()
        model.addAllCollisionPairs()
        retained = []
        for pair in model.collisionPairs:
            if pair.first in self.ignored_geometry_indices or pair.second in self.ignored_geometry_indices:
                continue
            a, b = model.geometryObjects[pair.first], model.geometryObjects[pair.second]
            adjacent = self.robot.model.parents[a.parentJoint] == b.parentJoint or self.robot.model.parents[b.parentJoint] == a.parentJoint
            if not adjacent and frozenset((str(a.name), str(b.name))) not in self.allowed_self_collisions:
                retained.append((pair.first, pair.second))
        model.removeAllCollisionPairs()
        for a, b in retained:
            model.addCollisionPair(self.robot.pin.CollisionPair(a, b))
        self.robot.geometry_data = self.robot.pin.GeometryData(model)

    def _environment(self, primitive: Any) -> tuple[Any, Any, Any]:
        margin = self.total_environment_margin_m
        geometry = self.hppfcl.Box(*(primitive.size_m + 2 * margin)) if primitive.kind == "box" else self.hppfcl.Cylinder(primitive.radius_m + margin, primitive.height_m + 2 * margin)
        return primitive, geometry, self.hppfcl.Transform3f(np.eye(3), primitive.center_m)

    def _transform(self, placement: Any) -> Any:
        return self.hppfcl.Transform3f(placement.rotation, placement.translation)

    def check(self, q: np.ndarray, *, stop_at_first: bool = False) -> CollisionReport:
        self.robot.validate_configuration(q)
        self.robot.pin.updateGeometryPlacements(self.robot.model, self.robot.data, self.robot.collision_model, self.robot.geometry_data, q)
        self_hits, env_hits, checked_self, checked_env = [], [], 0, 0
        minimum = float("inf")
        for pair in self.robot.collision_model.collisionPairs:
            checked_self += 1
            a, b = self.robot.collision_model.geometryObjects[pair.first], self.robot.collision_model.geometryObjects[pair.second]
            result = self.hppfcl.CollisionResult()
            if self.hppfcl.collide(a.geometry, self._transform(self.robot.geometry_data.oMg[pair.first]), b.geometry, self._transform(self.robot.geometry_data.oMg[pair.second]), self.collision_request, result):
                self_hits.append((str(a.name), str(b.name)))
                if stop_at_first:
                    return CollisionReport(False, tuple(self_hits), (), float("-inf"), checked_self, 0)
        for index, geometry in enumerate(self.robot.collision_model.geometryObjects):
            if index in self.ignored_geometry_indices:
                continue
            transform = self._transform(self.robot.geometry_data.oMg[index])
            for primitive, obstacle, obstacle_transform in self.environment:
                checked_env += 1
                result = self.hppfcl.CollisionResult()
                if self.hppfcl.collide(geometry.geometry, transform, obstacle, obstacle_transform, self.collision_request, result):
                    env_hits.append((str(geometry.name), primitive.identifier))
                    minimum = min(minimum, 0.0)
                    if stop_at_first:
                        return CollisionReport(False, tuple(self_hits), tuple(env_hits), minimum, checked_self, checked_env)
                else:
                    distance = self.hppfcl.DistanceResult()
                    minimum = min(minimum, float(self.hppfcl.distance(geometry.geometry, transform, obstacle, obstacle_transform, self.distance_request, distance)))
        return CollisionReport(not self_hits and not env_hits, tuple(self_hits), tuple(env_hits), minimum, checked_self, checked_env)

    def is_valid(self, q: np.ndarray) -> bool:
        return self.check(q, stop_at_first=True).valid

    def collision_geometry_centers(self, q: np.ndarray) -> np.ndarray:
        centers = self.robot.collision_geometry_centers(q)
        return centers[[i for i in range(len(centers)) if i not in self.ignored_geometry_indices]]

    def metadata(self) -> dict[str, Any]:
        return {"backend": "hpp-fcl", "planning_scope": "arm_body_demo_excludes_unconfirmed_end_effector" if self.arm_body_demo else "complete_confirmed_robot", "execution_valid": not self.arm_body_demo, "robot_collision_geometry_count": int(self.robot.collision_model.ngeoms), "checked_robot_collision_geometry_count": int(self.robot.collision_model.ngeoms) - len(self.ignored_geometry_indices), "ignored_collision_geometries": list(self.ignored_geometry_names), "self_collision_pair_count": len(self.robot.collision_model.collisionPairs), "allowed_self_collision_pair_count": len(self.allowed_self_collisions), "allowed_self_collisions": str(self.allowed_self_collisions_path), "allowed_self_collisions_sha256": sha256_file(self.allowed_self_collisions_path), "environment_primitive_count": len(self.environment), "environment_inflation_m": self.environment_inflation_m, "required_clearance_m": self.required_clearance_m, "combined_collision_margin_m": self.total_environment_margin_m, "end_effector_model": self.end_effector_report}
