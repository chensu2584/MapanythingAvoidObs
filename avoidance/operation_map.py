"""Auditable robot self-filtering and conservative metric voxel inflation."""

from __future__ import annotations

import dataclasses
import math
import time
from pathlib import Path
from typing import Any

import numpy as np

from .contracts import AvoidanceError, VoxelMap, sha256_file, write_json
from .map_io import LoadedVoxelMap
from .robot_model import UrdfRobot, classify_robot_voxels


@dataclasses.dataclass(frozen=True)
class SelfFilterAnalysis:
    core_candidate_mask: np.ndarray
    ambiguity_shell_mask: np.ndarray
    report: dict[str, Any]


@dataclasses.dataclass(frozen=True)
class InflatedMap:
    voxel_map: VoxelMap
    cell_kind: np.ndarray
    source_retained_indices: np.ndarray
    structure: np.ndarray
    padding_cells: int
    report: dict[str, Any]


def analyze_self_filter(
    voxel_map: VoxelMap,
    robot: UrdfRobot,
    joint_positions: dict[str, float],
    *,
    surface_margin_m: float,
    ambiguity_shell_m: float,
) -> SelfFilterAnalysis:
    voxel_half_diagonal = math.sqrt(3.0) * voxel_map.voxel_size / 2.0
    effective_core_margin = float(surface_margin_m) + voxel_half_diagonal
    primitives = robot.world_primitives(joint_positions)
    core, shell, link_counts = classify_robot_voxels(
        voxel_map.centers,
        primitives,
        core_margin_m=effective_core_margin,
        ambiguity_shell_m=ambiguity_shell_m,
    )
    return SelfFilterAnalysis(
        core_candidate_mask=core,
        ambiguity_shell_mask=shell,
        report={
            "collision_primitive_count": len(primitives),
            "source_occupied_voxel_count": len(voxel_map.indices),
            "core_candidate_voxel_count": int(np.count_nonzero(core)),
            "ambiguity_shell_voxel_count": int(np.count_nonzero(shell)),
            "surface_margin_m": float(surface_margin_m),
            "voxel_half_diagonal_m": voxel_half_diagonal,
            "effective_core_center_margin_m": effective_core_margin,
            "ambiguity_shell_m": float(ambiguity_shell_m),
            "candidate_counts_by_link": link_counts,
            "semantics": {
                "core_candidate": (
                    "voxel center is inside a robot collision primitive expanded by the "
                    "configured surface margin plus voxel half-diagonal"
                ),
                "ambiguity_shell": (
                    "near-robot voxel outside the core candidate; retained even after approval"
                ),
            },
        },
    )


def _dilation_structure(radius_m: float, voxel_size: float) -> tuple[np.ndarray, int, dict[str, Any]]:
    if not np.isfinite(radius_m) or radius_m < 0.0:
        raise AvoidanceError("Inflation radius must be finite and non-negative")
    padding = int(math.floor(radius_m / voxel_size)) + 1 if radius_m > 0 else 0
    coordinates = np.arange(-padding, padding + 1, dtype=np.int64)
    x, y, z = np.meshgrid(coordinates, coordinates, coordinates, indexing="ij")
    offsets = np.stack([x, y, z], axis=-1)
    # Minimum separation between two closed voxel cubes at this index offset.
    axis_gaps = np.maximum(np.abs(offsets) - 1, 0).astype(np.float64) * voxel_size
    minimum_cube_distance = np.linalg.norm(axis_gaps, axis=-1)
    structure = minimum_cube_distance <= radius_m + 1e-12
    included = minimum_cube_distance[structure]
    excluded = minimum_cube_distance[~structure]
    center_distances = np.linalg.norm(offsets[structure], axis=1) * voxel_size
    return structure, padding, {
        "requested_radius_m": float(radius_m),
        "method": "cube_to_cube_euclidean_minkowski_dilation",
        "included_offset_count": int(np.count_nonzero(structure)),
        "max_included_minimum_cube_distance_m": float(included.max(initial=0.0)),
        "min_excluded_minimum_cube_distance_m": (
            float(excluded.min()) if len(excluded) else None
        ),
        "realized_radius_quantization_bracket_m": {
            "largest_included_cube_gap": float(included.max(initial=0.0)),
            "smallest_excluded_cube_gap": float(excluded.min()) if len(excluded) else None,
        },
        "max_included_center_distance_m": float(center_distances.max(initial=0.0)),
        "padding_cells_each_side": padding,
        "padding_m_each_side": padding * voxel_size,
    }


def inflate_voxels(
    voxel_map: VoxelMap,
    retained_mask: np.ndarray,
    *,
    radius_m: float,
    max_dense_cells: int,
) -> InflatedMap:
    retained_mask = np.asarray(retained_mask, dtype=bool)
    if retained_mask.shape != (len(voxel_map.indices),):
        raise AvoidanceError("retained_mask shape does not match source voxel count")
    retained_indices = voxel_map.indices[retained_mask]
    if not len(retained_indices):
        raise AvoidanceError("Self-filter removed every occupied voxel; refusing empty map")
    structure, padding, dilation_report = _dilation_structure(radius_m, voxel_map.voxel_size)
    output_dims = voxel_map.dims + 2 * padding
    dense_cells = int(np.prod(output_dims, dtype=np.int64))
    if dense_cells > int(max_dense_cells):
        raise AvoidanceError(
            f"Inflated dense work grid needs {dense_cells} cells, exceeding "
            f"max_dense_cells={max_dense_cells}. Crop the workspace or raise the audited limit."
        )
    try:
        from scipy import ndimage
    except ImportError as exc:
        raise AvoidanceError("SciPy is required for operation-map dilation") from exc
    dense = np.zeros(tuple(int(v) for v in output_dims), dtype=bool)
    shifted = retained_indices.astype(np.int64) + padding
    dense[tuple(shifted.T)] = True
    dilated = ndimage.binary_dilation(dense, structure=structure, iterations=1)
    output_indices = np.argwhere(dilated).astype(np.int32)
    output_flat = np.ravel_multi_index(output_indices.T, tuple(int(v) for v in output_dims))
    retained_flat = np.ravel_multi_index(shifted.T, tuple(int(v) for v in output_dims))
    retained_flat_sorted = np.sort(retained_flat)
    positions = np.searchsorted(retained_flat_sorted, output_flat)
    original = (positions < len(retained_flat_sorted)) & (
        retained_flat_sorted[np.minimum(positions, len(retained_flat_sorted) - 1)] == output_flat
    )
    cell_kind = np.where(original, 1, 2).astype(np.uint8)
    colors = np.empty((len(output_indices), 3), dtype=np.uint8)
    colors[:] = [230, 72, 72]
    source_colors = (
        np.asarray(voxel_map.colors, dtype=np.uint8)[retained_mask]
        if voxel_map.colors is not None
        else np.tile(np.asarray([150, 150, 150], dtype=np.uint8), (len(shifted), 1))
    )
    retained_order = np.argsort(retained_flat)
    matched = np.flatnonzero(original)
    if len(matched):
        source_lookup = retained_order[positions[matched]]
        colors[matched] = source_colors[source_lookup]
    output = VoxelMap(
        indices=output_indices,
        origin=voxel_map.origin - padding * voxel_map.voxel_size,
        voxel_size=voxel_map.voxel_size,
        dims=output_dims,
        colors=colors,
    )
    dilation_report.update(
        {
            "input_retained_voxel_count": int(len(retained_indices)),
            "output_occupied_voxel_count": int(len(output_indices)),
            "new_inflation_voxel_count": int(np.count_nonzero(cell_kind == 2)),
            "work_grid_dims": output_dims.tolist(),
            "work_grid_cell_count": dense_cells,
            "output_origin_m": output.origin.tolist(),
        }
    )
    return InflatedMap(output, cell_kind, retained_indices, structure, padding, dilation_report)


def nearest_voxel_distance_report(voxel_map: VoxelMap, tcp_report: dict[str, Any]) -> dict[str, Any]:
    centers = voxel_map.centers
    half_diagonal = math.sqrt(3.0) * voxel_map.voxel_size / 2.0
    result = {}
    for side in ("left", "right"):
        point = np.asarray(tcp_report["poses"][side]["tcp_position_m"], dtype=np.float64)
        distances = np.linalg.norm(centers - point, axis=1)
        minimum = float(distances.min()) if len(distances) else float("inf")
        result[side] = {
            "minimum_center_distance_m": minimum,
            "conservative_minimum_distance_to_voxel_cube_m": max(0.0, minimum - half_diagonal),
            "inside_or_intersects_occupied_voxel": minimum <= half_diagonal,
        }
    return result


def _cube_mesh(indices: np.ndarray, colors: np.ndarray, voxel_size: float, origin: np.ndarray):
    import trimesh

    cube = trimesh.creation.box(extents=[voxel_size * 0.98] * 3)
    cube_vertices = np.asarray(cube.vertices)
    cube_faces = np.asarray(cube.faces)
    centers = origin + (indices.astype(np.float64) + 0.5) * voxel_size
    count = len(indices)
    vertices = (centers[:, None, :] + cube_vertices[None, :, :]).reshape(-1, 3)
    faces = (
        cube_faces[None, :, :]
        + (np.arange(count) * len(cube_vertices))[:, None, None]
    ).reshape(-1, 3)
    rgba = np.c_[colors, np.full(count, 255, dtype=np.uint8)]
    face_colors = np.repeat(rgba, len(cube_faces), axis=0)
    mesh = trimesh.Trimesh(vertices=vertices, faces=faces, process=False)
    mesh.visual.face_colors = face_colors
    return mesh


def _atomic_scene_export(scene: Any, path: Path) -> None:
    temporary = path.with_name(f".{path.stem}.tmp{path.suffix}")
    scene.export(temporary)
    temporary.replace(path)


def export_operation_glb(
    path: Path,
    inflated: InflatedMap,
    tcp_report: dict[str, Any],
) -> dict[str, Any]:
    try:
        import trimesh
        from scipy import ndimage
    except ImportError as exc:
        raise AvoidanceError("Trimesh and SciPy are required for GLB export") from exc
    shape = tuple(int(v) for v in inflated.voxel_map.dims)
    dense = np.zeros(shape, dtype=bool)
    dense[tuple(inflated.voxel_map.indices.T)] = True
    interior = ndimage.binary_erosion(dense, structure=np.ones((3, 3, 3), dtype=bool))
    surface_mask = dense & ~interior
    surface_indices = np.argwhere(surface_mask).astype(np.int32)
    all_flat = np.ravel_multi_index(inflated.voxel_map.indices.T, shape)
    surface_flat = np.ravel_multi_index(surface_indices.T, shape)
    positions = np.searchsorted(all_flat, surface_flat)
    surface_colors = inflated.voxel_map.colors[positions]
    scene = trimesh.Scene()
    scene.add_geometry(
        _cube_mesh(
            surface_indices,
            surface_colors,
            inflated.voxel_map.voxel_size,
            inflated.voxel_map.origin,
        ),
        geom_name="operation_map_surface_base_link",
    )
    scene.add_geometry(
        trimesh.creation.axis(origin_size=0.015, axis_radius=0.004, axis_length=0.2),
        geom_name="base_link_axes",
    )
    marker_colors = {"left": [255, 160, 0, 255], "right": [0, 220, 255, 255]}
    for side in ("left", "right"):
        pose = np.asarray(tcp_report["poses"][side]["base_T_tcp"], dtype=np.float64)
        marker = trimesh.creation.icosphere(subdivisions=2, radius=0.018)
        marker.apply_translation(pose[:3, 3])
        marker.visual.face_colors = np.tile(marker_colors[side], (len(marker.faces), 1))
        axes = trimesh.creation.axis(origin_size=0.006, axis_radius=0.002, axis_length=0.07)
        axes.apply_transform(pose)
        scene.add_geometry(marker, geom_name=f"tcp_{side}_center")
        scene.add_geometry(axes, geom_name=f"tcp_{side}_axes")
    _atomic_scene_export(scene, path)
    return {
        "coordinate_convention": "raw base_link; no viewer-only 180-degree X flip",
        "surface_voxel_count": int(len(surface_indices)),
        "full_occupancy_is_in_npz": True,
    }


def export_self_filter_preview(
    path: Path, voxel_map: VoxelMap, analysis: SelfFilterAnalysis
) -> dict[str, Any]:
    import trimesh

    colors = (
        np.asarray(voxel_map.colors, dtype=np.uint8).copy()
        if voxel_map.colors is not None
        else np.tile(np.asarray([130, 130, 130], dtype=np.uint8), (len(voxel_map.indices), 1))
    )
    colors[analysis.ambiguity_shell_mask] = [255, 205, 0]
    colors[analysis.core_candidate_mask] = [255, 0, 220]
    scene = trimesh.Scene()
    scene.add_geometry(
        _cube_mesh(voxel_map.indices, colors, voxel_map.voxel_size, voxel_map.origin),
        geom_name="source_with_self_filter_audit",
    )
    scene.add_geometry(
        trimesh.creation.axis(origin_size=0.015, axis_radius=0.004, axis_length=0.2),
        geom_name="base_link_axes",
    )
    _atomic_scene_export(scene, path)
    return {
        "coordinate_convention": "raw base_link; no viewer-only 180-degree X flip",
        "colors": {
            "magenta": "core robot-self candidate; removed only after explicit approval",
            "yellow": "ambiguity shell; always retained",
            "other": "retained source occupancy",
        },
    }


def export_exact_review_preview(
    path: Path,
    voxel_map: VoxelMap,
    analysis: SelfFilterAnalysis,
    selected_robot_mask: np.ndarray,
    *,
    selection_scope: str = "ambiguity_shell",
) -> dict[str, Any]:
    """Export source occupancy with exact operator-selected voxels shown in green."""
    import trimesh

    selected = np.asarray(selected_robot_mask, dtype=bool)
    if selected.shape != (len(voxel_map.indices),):
        raise AvoidanceError("Exact review preview selection has the wrong shape")
    if selection_scope not in {"ambiguity_shell", "all_occupied"}:
        raise AvoidanceError(f"Unsupported review selection scope: {selection_scope}")
    if selection_scope == "ambiguity_shell" and np.any(
        selected & ~analysis.ambiguity_shell_mask
    ):
        raise AvoidanceError("Exact review preview selected a non-yellow voxel")
    colors = (
        np.asarray(voxel_map.colors, dtype=np.uint8).copy()
        if voxel_map.colors is not None
        else np.tile(np.asarray([110, 110, 110], dtype=np.uint8), (len(voxel_map.indices), 1))
    )
    colors = np.clip(colors.astype(np.float32) * 0.45, 0, 255).astype(np.uint8)
    colors[analysis.core_candidate_mask] = [255, 0, 220]
    colors[analysis.ambiguity_shell_mask] = [255, 190, 0]
    colors[selected] = [0, 255, 70]
    scene = trimesh.Scene()
    scene.add_geometry(
        _cube_mesh(voxel_map.indices, colors, voxel_map.voxel_size, voxel_map.origin),
        geom_name="exact_manual_self_filter_review",
    )
    scene.add_geometry(
        trimesh.creation.axis(origin_size=0.015, axis_radius=0.004, axis_length=0.2),
        geom_name="base_link_axes",
    )
    _atomic_scene_export(scene, path)
    return {
        "path": str(path.resolve()),
        "coordinate_convention": "raw base_link; no viewer-only 180-degree X flip",
        "selection_scope": selection_scope,
        "selected_robot_voxel_count": int(np.count_nonzero(selected)),
        "selected_yellow_robot_voxel_count": int(
            np.count_nonzero(selected & analysis.ambiguity_shell_mask)
        ),
        "colors": {
            "magenta": "automatic core hint only; not selected unless green",
            "yellow": "automatic ambiguity hint only; not selected unless green",
            "green": "exact occupied voxel selected manually as robot self",
            "dim_context": "ordinary source scene occupancy",
        },
    }


def build_and_write_operation_map(
    loaded: LoadedVoxelMap,
    output_dir: str | Path,
    *,
    robot: UrdfRobot,
    joint_positions: dict[str, float],
    tcp_report: dict[str, Any],
    inflation_m: float,
    self_surface_margin_m: float,
    ambiguity_shell_m: float,
    approve_self_filter: bool,
    approved_yellow_mask: np.ndarray | None,
    review_provenance: dict[str, Any] | None,
    review_selection_scope: str,
    future_path_clearance_m: float,
    max_dense_cells: int,
    defaults_provenance: dict[str, Any],
) -> dict[str, Any]:
    output_dir = Path(output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    source = loaded.voxel_map
    analysis = analyze_self_filter(
        source,
        robot,
        joint_positions,
        surface_margin_m=self_surface_margin_m,
        ambiguity_shell_m=ambiguity_shell_m,
    )
    if review_selection_scope not in {"ambiguity_shell", "all_occupied"}:
        raise AvoidanceError(
            f"Unsupported review selection scope: {review_selection_scope}"
        )
    if approved_yellow_mask is None:
        approved_yellow_mask = np.zeros(len(source.indices), dtype=bool)
    selected_review_mask = np.asarray(approved_yellow_mask, dtype=bool)
    if selected_review_mask.shape != (len(source.indices),):
        raise AvoidanceError("Exact review mask has the wrong shape")
    if review_selection_scope == "ambiguity_shell":
        if np.any(selected_review_mask & ~analysis.ambiguity_shell_mask):
            raise AvoidanceError("Exact yellow review selected a non-yellow source voxel")
        if np.any(selected_review_mask) and not approve_self_filter:
            raise AvoidanceError("Exact yellow selection requires approved purple core")
    elif review_provenance is None:
        raise AvoidanceError("Full-scene manual selection requires completed review provenance")
    manual_full_review = bool(
        review_selection_scope == "all_occupied" and review_provenance is not None
    )
    self_filter_authorized = bool(approve_self_filter or manual_full_review)
    removal_mask = np.zeros(len(source.indices), dtype=bool)
    if approve_self_filter:
        removal_mask |= analysis.core_candidate_mask
    removal_mask |= selected_review_mask
    retained = ~removal_mask
    inflated = inflate_voxels(
        source,
        retained,
        radius_m=inflation_m,
        max_dense_cells=max_dense_cells,
    )
    npz_path = output_dir / "operation_map.npz"
    np.savez_compressed(
        npz_path,
        indices=inflated.voxel_map.indices,
        origin=inflated.voxel_map.origin.astype(np.float64),
        voxel_size=np.float64(inflated.voxel_map.voxel_size),
        dims=inflated.voxel_map.dims.astype(np.int32),
        colors=inflated.voxel_map.colors,
        cell_kind=inflated.cell_kind,
        world_frame=np.asarray("base_link"),
        translation_unit=np.asarray("meter"),
        outside_grid_policy=np.asarray("occupied"),
        inside_unoccupied_policy=np.asarray("assumed_free_not_raycast_verified"),
    )
    preview_path = output_dir / "self_filter_candidates_base_link.glb"
    preview_report = export_self_filter_preview(preview_path, source, analysis)
    glb_path = output_dir / "operation_map_base_link.glb"
    glb_report = export_operation_glb(glb_path, inflated, tcp_report)
    start_clearance = analyze_self_filter(
        inflated.voxel_map,
        robot,
        joint_positions,
        surface_margin_m=future_path_clearance_m,
        ambiguity_shell_m=0.0,
    )
    start_collision_voxels = int(
        np.count_nonzero(start_clearance.core_candidate_mask)
    )
    self_report = dict(analysis.report)
    self_report.update(
        {
            "approval_required": True,
            "approved": self_filter_authorized,
            "filter_applied": self_filter_authorized,
            "selection_scope": review_selection_scope,
            "automatic_core_removal_approved": bool(approve_self_filter),
            "removed_core_candidate_voxel_count": int(
                np.count_nonzero(removal_mask & analysis.core_candidate_mask)
            ),
            "retained_ambiguity_shell_voxel_count": int(
                np.count_nonzero(analysis.ambiguity_shell_mask & ~removal_mask)
            ),
            "ambiguity_review_mode": (
                "exact_full_scene_manual_voxel_selection"
                if manual_full_review
                else (
                    "exact_hash_bound_voxel_selection"
                    if review_provenance
                    else "retain_all"
                )
            ),
            "removed_reviewed_yellow_voxel_count": int(
                np.count_nonzero(selected_review_mask & analysis.ambiguity_shell_mask)
            ),
            "removed_reviewed_robot_voxel_count": int(
                np.count_nonzero(selected_review_mask)
            ),
            "review_provenance": review_provenance,
            "preview": {"path": str(preview_path), **preview_report},
            "tcp_nearest_occupancy_before_filter": nearest_voxel_distance_report(
                source, tcp_report
            ),
            "tcp_nearest_occupancy_after_inflation": nearest_voxel_distance_report(
                inflated.voxel_map, tcp_report
            ),
            "robot_state_stationary": bool(tcp_report["state"].get("stationary")),
            "wbc_fk_crosscheck_passed": bool(
                tcp_report.get("wbc_fk_crosscheck", {}).get("passed")
            ),
            "start_robot_vs_operation_map": {
                "required_clearance_m": float(future_path_clearance_m),
                "collision_or_clearance_violation_voxel_count": start_collision_voxels,
                "passed": start_collision_voxels == 0,
                "note": (
                    "Checks all G1 collision primitives against the final inflated map; "
                    "voxel half-diagonal is included conservatively."
                ),
            },
        }
    )
    self_report_path = output_dir / "self_filter_report.json"
    write_json(self_report_path, self_report)
    planning_ready = bool(
        self_filter_authorized
        and self_report["robot_state_stationary"]
        and self_report["wbc_fk_crosscheck_passed"]
        and self_report["start_robot_vs_operation_map"]["passed"]
    )
    manifest = {
        "schema_version": 1,
        "generated_at_unix_ns": time.time_ns(),
        "stage": "early_experiment_operation_map",
        "world_frame": "base_link",
        "translation_unit": "meter",
        "input": loaded.provenance,
        "source_kind": loaded.source_kind,
        "source_grid": {
            "origin_m": source.origin.tolist(),
            "voxel_size_m": source.voxel_size,
            "dims": source.dims.tolist(),
            "occupied_voxel_count": len(source.indices),
        },
        "self_filter": {
            "approved": self_filter_authorized,
            "applied": self_filter_authorized,
            "selection_scope": review_selection_scope,
            "ambiguity_review_mode": self_report["ambiguity_review_mode"],
            "removed_reviewed_yellow_voxel_count": self_report[
                "removed_reviewed_yellow_voxel_count"
            ],
            "removed_reviewed_robot_voxel_count": self_report[
                "removed_reviewed_robot_voxel_count"
            ],
            "review_provenance": review_provenance,
            "core_candidate_voxel_count": self_report["core_candidate_voxel_count"],
            "ambiguity_shell_voxel_count": self_report["ambiguity_shell_voxel_count"],
            "retained_ambiguity_shell_voxel_count": self_report[
                "retained_ambiguity_shell_voxel_count"
            ],
            "start_robot_clearance_check": self_report[
                "start_robot_vs_operation_map"
            ],
            "report": str(self_report_path),
        },
        "inflation": inflated.report,
        "unknown_space_policy": {
            "outside_grid": "occupied",
            "unoccupied_inside_grid": "assumed_free_not_raycast_verified",
        },
        "coordinate_rendering": glb_report,
        "planning_ready": planning_ready,
        "execution_ready": False,
        "execution_blockers": [
            "Stages 3+ (IK, collision checking, path planning, trajectory execution) are not implemented.",
            "TCP measurement remains an independent execution gate.",
        ],
        "defaults": defaults_provenance,
        "outputs": {
            "operation_map_npz": str(npz_path),
            "operation_map_npz_sha256": sha256_file(npz_path),
            "operation_map_glb": str(glb_path),
            "operation_map_glb_sha256": sha256_file(glb_path),
            "self_filter_preview_glb": str(preview_path),
            "self_filter_preview_glb_sha256": sha256_file(preview_path),
            "self_filter_report": str(self_report_path),
        },
    }
    manifest_path = output_dir / "operation_map_manifest.json"
    write_json(manifest_path, manifest)
    return {
        "manifest": manifest,
        "manifest_path": str(manifest_path),
        "npz_path": str(npz_path),
        "glb_path": str(glb_path),
        "self_filter_report_path": str(self_report_path),
        "preview_path": str(preview_path),
    }
