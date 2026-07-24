"""Stateful glue between the GUI and the Cartesian goal-point workflow.

The GUI owns pixels and events; ``cartesian_goal`` owns pure geometry.  This
controller sits between them and holds the interaction state described in the
plan (sections 5, 7, 8): which primitive edge was picked, the approach offset
and normal flip, the debounce request id, and the latest worker verdict.  It is
matplotlib-only for the projection helper (imported lazily) and otherwise pure,
so the state machine can be unit-tested with an injected projection.

The controller never trusts itself for rotation: it sends the flange *position*
and an orientation *policy* to the worker, which recomputes the goal rotation
against live forward kinematics and returns the authoritative ``base_T_goal``.
"""

from __future__ import annotations

from typing import Any, Callable

import numpy as np

from .cartesian_goal import (
    CartesianGoal,
    anchor_to_goal,
    box_edges,
    edge_outward_normal,
    pick_edge,
)
from .contracts import AvoidanceError

ProjectFn = Callable[[np.ndarray], np.ndarray]


def project_world_to_screen(ax: Any, points: np.ndarray) -> np.ndarray:
    """Project base_link points onto a Matplotlib 3D axes' display pixels.

    Used as the ``project_fn`` for :func:`avoidance.cartesian_goal.pick_edge` so
    a mouse click in the 3D view can be matched against primitive edges.
    """
    from mpl_toolkits.mplot3d import proj3d

    pts = np.atleast_2d(np.asarray(points, dtype=float))
    proj = ax.get_proj()
    out = np.empty((len(pts), 2))
    for i, (x, y, z) in enumerate(pts):
        xs, ys, _ = proj3d.proj_transform(x, y, z, proj)
        out[i] = ax.transData.transform((xs, ys))
    return out


class CartesianTargetController:
    """Interaction state for placing one Cartesian flange goal."""

    def __init__(self, *, approach_offset_m: float = 0.08,
                 orientation_policy: str = "hold_snapshot_start_flange",
                 pixel_threshold: float = 12.0, pickable_roles=("object", "support")):
        self.approach_offset_m = float(approach_offset_m)
        self.orientation_policy = orientation_policy
        self.pixel_threshold = float(pixel_threshold)
        self.pickable_roles = set(pickable_roles)
        self.active_arm = "left"
        self._edges: dict[int, list[dict[str, Any]]] = {}
        self._pick: dict[str, Any] | None = None
        self._flip = False
        self._request_id = 0
        self._pending_id: int | None = None
        self.last_status: str = "gray"           # gray/green/orange/red (plan section 8)
        self.last_base_T_goal: np.ndarray | None = None
        self.last_feasible_arm: list[float] | None = None
        self.last_reason: str = ""

    # --- scene / selection -------------------------------------------------
    def load_scene(self, scene: Any, active_arm: str = "left") -> None:
        self.active_arm = active_arm
        self._edges = {}
        for prim in scene.primitives:
            if prim.role in self.pickable_roles and prim.kind == "box":
                self._edges[int(prim.identifier)] = box_edges(prim.bounds_m)
        self._reset_target()

    def _reset_target(self) -> None:
        self._pick = None
        self._flip = False
        self._pending_id = None
        self.last_status = "gray"
        self.last_base_T_goal = None
        self.last_feasible_arm = None
        self.last_reason = ""

    @property
    def has_target(self) -> bool:
        return self._pick is not None

    def pick(self, mouse_xy, project_fn: ProjectFn, view_dir=None) -> bool:
        """Pick the nearest primitive edge; returns True if something was hit."""
        if not self._edges:
            return False
        hit = pick_edge(self._edges, mouse_xy, project_fn, self.pixel_threshold)
        if hit is None:
            return False
        hit["view_dir"] = None if view_dir is None else np.asarray(view_dir, float)
        self._pick = hit
        self._flip = False
        self.last_status = "gray"
        self.last_base_T_goal = None
        return True

    # --- target geometry (visual position; rotation is worker-authoritative)
    def _outward_normal(self) -> np.ndarray:
        return edge_outward_normal(self._pick["face_normals"],
                                   view_dir=self._pick.get("view_dir"), flip=self._flip)

    def goal_position(self) -> np.ndarray:
        if self._pick is None:
            raise AvoidanceError("no target selected")
        return anchor_to_goal(self._pick["position_m"], self._outward_normal(),
                              self.approach_offset_m)

    def nudge(self, delta_xyz) -> None:
        """Shift the anchor by delta (metres) so the goal tracks the XYZ steppers."""
        if self._pick is None:
            raise AvoidanceError("no target selected")
        self._pick["position_m"] = np.asarray(self._pick["position_m"], float) + np.asarray(delta_xyz, float)
        self._invalidate()

    def set_offset(self, offset_m: float) -> None:
        if offset_m < 0:
            raise AvoidanceError("approach offset must be non-negative")
        self.approach_offset_m = float(offset_m)
        self._invalidate()

    def toggle_flip(self) -> None:
        self._flip = not self._flip
        self._invalidate()

    def _invalidate(self) -> None:
        """Any target edit invalidates the last worker verdict (plan section 8)."""
        self.last_status = "gray"
        self.last_base_T_goal = None
        self._pending_id = None

    # --- worker protocol (debounced preview, then plan) --------------------
    def preview_request(self, scene_path: str, capture_state_path: str) -> dict[str, Any]:
        if self._pick is None:
            raise AvoidanceError("no target selected")
        self._request_id += 1
        self._pending_id = self._request_id
        request = {
            "action": "preview_cartesian_goal", "scene": str(scene_path),
            "capture_state": str(capture_state_path), "arm": self.active_arm,
            "position_m": self.goal_position().tolist(),
            "orientation_policy": self.orientation_policy,
            "request_id": self._request_id,
        }
        if self.last_feasible_arm is not None:
            request["seed_arm"] = list(self.last_feasible_arm)  # warm-start IK
        return request

    def ingest(self, response: dict[str, Any]) -> bool:
        """Apply a worker preview response iff it is the latest one (drop stale)."""
        rid = response.get("request_id")
        if rid is None or rid != self._pending_id:
            return False  # a newer request superseded this one
        self.last_status = response.get("target_status", "red")
        self.last_reason = response.get("reason", "")
        if response.get("base_T_goal") is not None:
            self.last_base_T_goal = np.asarray(response["base_T_goal"], float)
        ik = response.get("ik", {})
        if ik.get("success") and ik.get("arm_joint_positions_rad") is not None:
            self.last_feasible_arm = list(ik["arm_joint_positions_rad"])
        return True

    @property
    def can_plan(self) -> bool:
        """Plan only from a settled, green, up-to-date preview (plan section 8)."""
        return (self._pick is not None and self.last_status == "green"
                and self._pending_id == self._request_id and self.last_base_T_goal is not None)

    def plan_request(self, scene_path: str, capture_state_path: str) -> dict[str, Any]:
        if not self.can_plan:
            raise AvoidanceError("plan requires a current green preview")
        return {
            "action": "plan_cartesian_goal", "scene": str(scene_path),
            "capture_state": str(capture_state_path), "arm": self.active_arm,
            "position_m": self.goal_position().tolist(),
            "orientation_policy": self.orientation_policy,
            "request_id": self._request_id,
        }

    def to_cartesian_goal(self, provenance: dict[str, str] | None = None) -> CartesianGoal:
        """Freeze the current green target into a saveable goal (uses the
        worker-returned rotation, never a locally guessed one)."""
        if self._pick is None or self.last_base_T_goal is None:
            raise AvoidanceError("no confirmed target to save")
        normal = self._outward_normal()
        anchor = {
            "primitive_id": int(self._pick["primitive_id"]),
            "feature": "box_edge", "edge_id": int(self._pick["edge_id"]),
            "position_m": np.asarray(self._pick["position_m"], float),
            "outward_normal": normal, "approach_offset_m": self.approach_offset_m,
        }
        return CartesianGoal(self.active_arm, self.goal_position(),
                             self.last_base_T_goal[:3, :3], self.orientation_policy,
                             anchor, provenance or {})
