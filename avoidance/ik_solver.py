"""Bounded multi-seed inverse kinematics for one G2 arm."""

from __future__ import annotations

import dataclasses
from typing import Any, Callable

import numpy as np


@dataclasses.dataclass(frozen=True)
class IKResult:
    success: bool
    arm_configuration: np.ndarray | None
    attempts: int
    position_error_m: float
    rotation_error_deg: float
    reason: str

    def to_dict(self) -> dict[str, Any]:
        return {
            "success": self.success,
            "attempts": self.attempts,
            "position_error_m": self.position_error_m,
            "rotation_error_deg": self.rotation_error_deg,
            "reason": self.reason,
            "arm_joint_positions_rad": (
                self.arm_configuration.tolist()
                if self.arm_configuration is not None
                else None
            ),
        }


def solve_g2_arm_ik(
    robot: Any,
    reference_q: np.ndarray,
    side: str,
    goal_pose: np.ndarray,
    *,
    end_frame: str,
    is_valid: Callable[[np.ndarray], bool],
    seed_count: int = 12,
    position_tolerance_m: float = 0.005,
    rotation_tolerance_deg: float = 2.0,
    random_seed: int = 17,
) -> IKResult:
    goal = robot.pin.SE3(goal_pose[:3, :3], goal_pose[:3, 3])
    frame_id = robot.model.getFrameId(end_frame)
    lower, upper = robot.arm_limits(side)
    initial = robot.arm_configuration(reference_q, side)
    rng = np.random.default_rng(random_seed)
    seeds = [initial] + [rng.uniform(lower, upper) for _ in range(max(0, seed_count - 1))]
    best = (float("inf"), float("inf"), None)
    for seed in seeds:
        arm = seed.copy()
        for _ in range(250):
            q = robot.with_arm_configuration(reference_q, side, arm)
            robot.pin.forwardKinematics(robot.model, robot.data, q)
            robot.pin.updateFramePlacements(robot.model, robot.data)
            current = robot.data.oMf[frame_id]
            error = robot.pin.log6(current.inverse() * goal)
            position = float(np.linalg.norm(error.linear))
            rotation = float(np.degrees(np.linalg.norm(error.angular)))
            if position + rotation / 180 < best[0] + best[1] / 180:
                best = (position, rotation, arm.copy())
            if position <= position_tolerance_m and rotation <= rotation_tolerance_deg:
                if is_valid(q):
                    return IKResult(True, arm.copy(), len(seeds), position, rotation, "converged")
                break
            jacobian = robot.pin.computeFrameJacobian(
                robot.model,
                robot.data,
                q,
                frame_id,
                robot.pin.ReferenceFrame.LOCAL,
            )[:, robot._arm_v_indices[side]]
            delta = np.linalg.solve(
                jacobian @ jacobian.T + 1e-5 * np.eye(6),
                error.vector,
            )
            arm = np.clip(arm + 0.35 * jacobian.T @ delta, lower, upper)
    return IKResult(False, None, len(seeds), best[0], best[1], "no_collision_free_solution")
