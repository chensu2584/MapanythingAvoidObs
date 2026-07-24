"""Cartesian goal-point interaction for the G2 arm-body avoidance demo.

Implements the geometry and data contract behind the target-point workflow in
``G2_3BOX_CARTESIAN_GOAL_PLAN.md`` (sections 5 and 6): pick an anchor on a
planning primitive's edge, push it out along an approach normal, hold the
snapshot's start-flange rotation, and serialise a hash-bindable Cartesian goal.

Deliberately dependency-light -- numpy only, no pinocchio/hppfcl -- so the
picking maths and the contract can be unit-tested without the planning stack.
Anything needing forward kinematics (the start-flange pose) is passed in as a
4x4 matrix by the caller (GUI/worker), never computed here.

Semantics locked for the first version (plan section 1):
  * the tracked frame is the active arm's ``arm_l_end_link`` / ``arm_r_end_link``
    flange -- NOT a confirmed gripper TCP;
  * the goal rotation defaults to the snapshot's start-flange rotation;
  * XYZ is metric ``base_link``;
  * this is ``arm_body_demo`` and ``execution_authorized`` is always False.
"""

from __future__ import annotations

import dataclasses
from typing import Any, Callable

import numpy as np

from .contracts import AvoidanceError, rotation_error_deg

SCHEMA_VERSION = 1
PLANNING_SCOPE = "arm_body_demo_excludes_unconfirmed_end_effector"
TRACKED_FRAME = {"left": "arm_l_end_link", "right": "arm_r_end_link"}
ORIENTATION_POLICIES = {
    "hold_snapshot_start_flange",
    "hold_last_feasible",
}
ProjectFn = Callable[[np.ndarray], np.ndarray]  # world (N,3) -> screen (N,2) px


# --- box primitive edges ---------------------------------------------------
# The six axis-aligned face outward normals, indexed by (axis, sign).
_FACE_NORMAL = {
    (0, -1): np.array([-1.0, 0.0, 0.0]), (0, 1): np.array([1.0, 0.0, 0.0]),
    (1, -1): np.array([0.0, -1.0, 0.0]), (1, 1): np.array([0.0, 1.0, 0.0]),
    (2, -1): np.array([0.0, 0.0, -1.0]), (2, 1): np.array([0.0, 0.0, 1.0]),
}


def box_edges(bounds: np.ndarray) -> list[dict[str, Any]]:
    """Return the 12 edges of an axis-aligned box.

    ``bounds`` is ``[[xmin,ymin,zmin],[xmax,ymax,zmax]]``.  Each edge carries its
    two 3D endpoints and the outward normals of the two faces it borders, so an
    approach direction can be derived later.
    """
    bounds = np.asarray(bounds, dtype=np.float64)
    if bounds.shape != (2, 3) or not np.isfinite(bounds).all():
        raise AvoidanceError("box bounds must be a finite (2,3) [min,max] array")
    if np.any(bounds[1] < bounds[0]):
        raise AvoidanceError("box max must be >= min on every axis")
    edges: list[dict[str, Any]] = []
    for axis in range(3):
        other = [a for a in range(3) if a != axis]
        for s0 in (-1, 1):
            for s1 in (-1, 1):
                p0 = np.empty(3)
                p1 = np.empty(3)
                p0[axis], p1[axis] = bounds[0, axis], bounds[1, axis]
                for oa, sgn in zip(other, (s0, s1)):
                    val = bounds[1, oa] if sgn > 0 else bounds[0, oa]
                    p0[oa] = p1[oa] = val
                normals = [_FACE_NORMAL[(other[0], s0)], _FACE_NORMAL[(other[1], s1)]]
                edges.append({"edge_id": len(edges), "p0": p0, "p1": p1,
                              "face_normals": normals})
    return edges


# --- edge picking ----------------------------------------------------------
def _closest_param_on_segment(a: np.ndarray, b: np.ndarray, p: np.ndarray) -> tuple[float, float]:
    """Closest point of segment a-b (2D) to point p: return (param t in [0,1], distance)."""
    ab = b - a
    denom = float(ab @ ab)
    if denom < 1e-12:
        return 0.0, float(np.linalg.norm(p - a))
    t = float(np.clip((p - a) @ ab / denom, 0.0, 1.0))
    closest = a + t * ab
    return t, float(np.linalg.norm(p - closest))


def pick_edge(primitive_edges: dict[int, list[dict[str, Any]]], mouse_xy,
              project_fn: ProjectFn, pixel_threshold: float = 12.0):
    """Pick the primitive edge nearest the mouse in the current projected view.

    ``primitive_edges`` maps primitive id -> the list from ``box_edges``.
    ``project_fn`` maps world points (N,3) to screen pixels (N,2) -- the GUI
    supplies its live Matplotlib projection; tests supply a synthetic one.

    Returns a dict with the picked primitive/edge, the segment parameter, the
    recovered 3D anchor and the edge's face normals, or ``None`` when nothing is
    within ``pixel_threshold``.  The 3D anchor is the endpoints linearly blended
    by the 2D-closest parameter -- an approximation that is refined afterwards by
    explicit XYZ editing (plan section 5.1/5.2), so it need not be exact under
    perspective.
    """
    mouse = np.asarray(mouse_xy, dtype=np.float64)
    best = None
    for pid, edges in primitive_edges.items():
        for edge in edges:
            screen = project_fn(np.stack([edge["p0"], edge["p1"]]))
            t, dist = _closest_param_on_segment(screen[0], screen[1], mouse)
            if dist <= pixel_threshold and (best is None or dist < best["pixel_distance"]):
                anchor = edge["p0"] + t * (edge["p1"] - edge["p0"])
                best = {"primitive_id": pid, "edge_id": edge["edge_id"],
                        "segment_param": t, "pixel_distance": dist,
                        "position_m": anchor, "face_normals": edge["face_normals"]}
    return best


def edge_outward_normal(face_normals, view_dir: np.ndarray | None = None,
                        flip: bool = False) -> np.ndarray:
    """Approach normal for a picked edge: normalised sum of its two face normals.

    When ``view_dir`` (camera -> scene) is given the normal is oriented to face
    the camera so the approach comes from the visible side; ``flip`` inverts it.
    """
    normal = np.sum(face_normals, axis=0)
    norm = np.linalg.norm(normal)
    if norm < 1e-9:
        raise AvoidanceError("degenerate edge normal")
    normal = normal / norm
    if view_dir is not None and normal @ np.asarray(view_dir, float) > 0:
        normal = -normal  # point back toward the camera, not away from it
    if flip:
        normal = -normal
    return normal


def anchor_to_goal(anchor: np.ndarray, outward_normal: np.ndarray,
                   approach_offset_m: float) -> np.ndarray:
    """Push the on-surface anchor out to a reachable, non-colliding goal point.

    The anchor sits on the obstacle boundary, so a flange sent there would be in
    contact; the goal is offset along the outward normal by ``approach_offset_m``
    (plan section 5.3).  The offset is an arm-body demo standoff, not a real
    gripper length.
    """
    if approach_offset_m < 0:
        raise AvoidanceError("approach_offset_m must be non-negative")
    return np.asarray(anchor, float) + np.asarray(outward_normal, float) * float(approach_offset_m)


# --- rotation policy -------------------------------------------------------
def goal_rotation(policy: str, start_flange_pose: np.ndarray,
                  last_feasible_pose: np.ndarray | None = None) -> np.ndarray:
    """Resolve the 3x3 goal rotation for the chosen orientation policy."""
    if policy == "hold_snapshot_start_flange":
        return np.asarray(start_flange_pose, float)[:3, :3].copy()
    if policy == "hold_last_feasible":
        if last_feasible_pose is None:
            return np.asarray(start_flange_pose, float)[:3, :3].copy()
        return np.asarray(last_feasible_pose, float)[:3, :3].copy()
    raise AvoidanceError(f"unknown orientation policy: {policy!r}")


def compose_goal_pose(rotation: np.ndarray, position: np.ndarray) -> np.ndarray:
    """Build the 4x4 base_T_goal from a rotation and a position."""
    pose = np.eye(4)
    pose[:3, :3] = np.asarray(rotation, float)
    pose[:3, 3] = np.asarray(position, float)
    return pose


# --- goal contract ---------------------------------------------------------
@dataclasses.dataclass(frozen=True)
class CartesianGoal:
    """A saveable, hash-bindable Cartesian flange goal (plan section 6)."""

    active_arm: str
    position_m: np.ndarray
    base_R_goal: np.ndarray
    orientation_policy: str
    anchor: dict[str, Any]
    provenance: dict[str, str] = dataclasses.field(default_factory=dict)

    def __post_init__(self) -> None:
        if self.active_arm not in TRACKED_FRAME:
            raise AvoidanceError("active_arm must be 'left' or 'right'")
        if self.orientation_policy not in ORIENTATION_POLICIES:
            raise AvoidanceError(f"unknown orientation policy: {self.orientation_policy!r}")
        pos = np.asarray(self.position_m, float)
        if pos.shape != (3,) or not np.isfinite(pos).all():
            raise AvoidanceError("position_m must be three finite values")
        rot = np.asarray(self.base_R_goal, float)
        if rot.shape != (3, 3) or not np.allclose(rot.T @ rot, np.eye(3), atol=1e-5) \
                or not np.isclose(np.linalg.det(rot), 1.0, atol=1e-5):
            raise AvoidanceError("base_R_goal must be a valid rotation")
        object.__setattr__(self, "position_m", pos)
        object.__setattr__(self, "base_R_goal", rot)

    @property
    def tracked_frame(self) -> str:
        return TRACKED_FRAME[self.active_arm]

    def goal_pose(self) -> np.ndarray:
        return compose_goal_pose(self.base_R_goal, self.position_m)

    def to_dict(self) -> dict[str, Any]:
        anchor = dict(self.anchor)
        for key in ("position_m", "outward_normal"):
            if key in anchor:
                anchor[key] = np.asarray(anchor[key], float).round(6).tolist()
        return {
            "schema_version": SCHEMA_VERSION,
            "robot_profile": "g2",
            "world_frame": "base_link",
            "translation_unit": "meter",
            "planning_scope": PLANNING_SCOPE,
            "active_arm": self.active_arm,
            "tracked_frame": self.tracked_frame,
            "position_m": self.position_m.round(6).tolist(),
            "orientation_policy": self.orientation_policy,
            "base_R_goal": self.base_R_goal.round(8).tolist(),
            "anchor": anchor,
            "execution_authorized": False,  # always -- gripper TCP unconfirmed
            "provenance": dict(self.provenance),
        }


def build_goal_from_pick(active_arm: str, pick: dict[str, Any], approach_offset_m: float,
                         start_flange_pose: np.ndarray, *, view_dir=None, flip_normal=False,
                         orientation_policy: str = "hold_snapshot_start_flange",
                         provenance: dict[str, str] | None = None) -> CartesianGoal:
    """Assemble a CartesianGoal from an edge pick plus an approach offset."""
    normal = edge_outward_normal(pick["face_normals"], view_dir=view_dir, flip=flip_normal)
    goal_pos = anchor_to_goal(pick["position_m"], normal, approach_offset_m)
    rotation = goal_rotation(orientation_policy, start_flange_pose)
    anchor = {
        "primitive_id": int(pick["primitive_id"]),
        "feature": "box_edge",
        "edge_id": int(pick["edge_id"]),
        "position_m": np.asarray(pick["position_m"], float),
        "outward_normal": normal,
        "approach_offset_m": float(approach_offset_m),
    }
    return CartesianGoal(active_arm, goal_pos, rotation, orientation_policy, anchor,
                         provenance or {})


def move_goal(goal: CartesianGoal, delta_xyz) -> CartesianGoal:
    """Return a goal nudged by delta (metres) -- backs the XYZ steppers/sliders."""
    return dataclasses.replace(goal, position_m=goal.position_m + np.asarray(delta_xyz, float))


def flange_reached(goal: CartesianGoal, achieved_pose: np.ndarray):
    """Position/rotation error between the goal and an IK-achieved flange pose."""
    pos_err = float(np.linalg.norm(goal.position_m - np.asarray(achieved_pose, float)[:3, 3]))
    rot_err = rotation_error_deg(goal.goal_pose(), np.asarray(achieved_pose, float))
    return pos_err, rot_err
