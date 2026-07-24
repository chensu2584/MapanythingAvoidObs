"""Deterministic bidirectional RRT-Connect for a seven-joint arm."""

from __future__ import annotations

import dataclasses
import time
from typing import Any, Callable

import numpy as np

from .contracts import AvoidanceError


@dataclasses.dataclass(frozen=True)
class RRTResult:
    success: bool
    path: tuple[np.ndarray, ...]
    iterations: int
    sampled_nodes: int
    collision_checks: int
    elapsed_s: float
    reason: str

    def to_dict(self) -> dict[str, Any]:
        return {"success": self.success, "waypoint_count": len(self.path), "iterations": self.iterations, "sampled_nodes": self.sampled_nodes, "collision_checks": self.collision_checks, "elapsed_s": self.elapsed_s, "reason": self.reason}


class _Tree:
    def __init__(self, root: np.ndarray, start: bool):
        self.nodes, self.parents, self.start = [root.copy()], [-1], start
    def nearest(self, target: np.ndarray) -> int:
        return int(np.argmin(np.linalg.norm(np.asarray(self.nodes) - target, axis=1)))
    def add(self, node: np.ndarray, parent: int) -> int:
        self.nodes.append(node.copy()); self.parents.append(parent); return len(self.nodes) - 1
    def trace(self, index: int) -> list[np.ndarray]:
        result = []
        while index >= 0:
            result.append(self.nodes[index]); index = self.parents[index]
        return list(reversed(result))


class RRTConnectPlanner:
    def __init__(self, lower_limits: np.ndarray, upper_limits: np.ndarray, is_valid: Callable[[np.ndarray], bool], *, extension_step_rad: float = .18, edge_step_rad: float = .04, max_iterations: int = 4000, timeout_s: float = 15, goal_bias: float = .12, smoothing_attempts: int = 120, random_seed: int = 7, edge_subdivision: Callable[[np.ndarray, np.ndarray], int] | None = None):
        self.lower, self.upper = np.asarray(lower_limits), np.asarray(upper_limits)
        if self.lower.shape != (7,) or self.upper.shape != (7,):
            raise AvoidanceError("RRT limits must contain seven values")
        self.is_valid_callback, self.extension_step_rad, self.edge_step_rad = is_valid, float(extension_step_rad), float(edge_step_rad)
        self.max_iterations, self.timeout_s, self.goal_bias = int(max_iterations), float(timeout_s), float(goal_bias)
        self.smoothing_attempts, self.rng, self.edge_subdivision = int(smoothing_attempts), np.random.default_rng(random_seed), edge_subdivision
        self.collision_checks = 0

    def _valid(self, state: np.ndarray) -> bool:
        self.collision_checks += 1
        return bool(self.is_valid_callback(state))

    def _steps(self, a: np.ndarray, b: np.ndarray) -> int:
        result = max(1, int(np.ceil(np.max(np.abs(b - a)) / self.edge_step_rad)))
        return max(result, int(self.edge_subdivision(a, b))) if self.edge_subdivision else result

    def edge_is_valid(self, a: np.ndarray, b: np.ndarray) -> bool:
        return all(self._valid(a + alpha * (b - a)) for alpha in np.linspace(0, 1, self._steps(a, b) + 1)[1:])

    def _extend(self, tree: _Tree, target: np.ndarray) -> tuple[str, int]:
        parent = tree.nearest(target); source = tree.nodes[parent]; delta = target - source; distance = np.linalg.norm(delta)
        if distance < 1e-12: return "reached", parent
        node = np.clip(source + delta * min(1, self.extension_step_rad / distance), self.lower, self.upper)
        if not self.edge_is_valid(source, node): return "trapped", parent
        index = tree.add(node, parent)
        return ("reached" if np.linalg.norm(node - target) < 1e-9 else "advanced"), index

    def _connect(self, tree: _Tree, target: np.ndarray) -> tuple[str, int]:
        while True:
            status, index = self._extend(tree, target)
            if status != "advanced": return status, index

    def _densify(self, path: list[np.ndarray]) -> tuple[np.ndarray, ...]:
        result = [path[0]]
        for a, b in zip(path, path[1:]):
            result.extend(a + alpha * (b - a) for alpha in np.linspace(0, 1, self._steps(a, b) + 1)[1:])
        return tuple(result)

    def plan(self, start: np.ndarray, goal: np.ndarray) -> RRTResult:
        began = time.monotonic(); self.collision_checks = 0
        start, goal = np.asarray(start, dtype=float), np.asarray(goal, dtype=float)
        if not self._valid(start): return RRTResult(False, (), 0, 0, self.collision_checks, time.monotonic()-began, "start_in_collision")
        if not self._valid(goal): return RRTResult(False, (), 0, 0, self.collision_checks, time.monotonic()-began, "goal_in_collision")
        if np.linalg.norm(goal-start) < 1e-12: return RRTResult(True, (start,), 0, 1, self.collision_checks, time.monotonic()-began, "already_at_goal")
        if self.edge_is_valid(start, goal): return RRTResult(True, self._densify([start, goal]), 0, 2, self.collision_checks, time.monotonic()-began, "direct_path")
        a, b = _Tree(start, True), _Tree(goal, False)
        for iteration in range(1, self.max_iterations + 1):
            if time.monotonic() - began > self.timeout_s: break
            sample = b.nodes[0] if self.rng.random() < self.goal_bias else self.rng.uniform(self.lower, self.upper)
            status_a, ia = self._extend(a, sample)
            if status_a != "trapped":
                status_b, ib = self._connect(b, a.nodes[ia])
                if status_b == "reached":
                    pa, pb = a.trace(ia), b.trace(ib)
                    path = pa + list(reversed(pb[:-1])) if a.start else pb + list(reversed(pa[:-1]))
                    return RRTResult(True, self._densify(path), iteration, len(a.nodes)+len(b.nodes), self.collision_checks, time.monotonic()-began, "connected")
            a, b = b, a
        return RRTResult(False, (), iteration, len(a.nodes)+len(b.nodes), self.collision_checks, time.monotonic()-began, "timeout")
