"""Exact, hash-bound operator review for shell-only or full-scene self filtering."""

from __future__ import annotations

import hashlib
import json
import time
from pathlib import Path
from typing import Any

import numpy as np

from .contracts import AvoidanceError, VoxelMap, read_json, sha256_file, write_json
from .map_io import LoadedVoxelMap
from .operation_map import SelfFilterAnalysis

SELECTION_SCOPE_AMBIGUITY = "ambiguity_shell"
SELECTION_SCOPE_ALL = "all_occupied"


def _canonical_sha256(value: Any) -> str:
    payload = json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=True
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def build_review_contract(
    loaded: LoadedVoxelMap,
    analysis: SelfFilterAnalysis,
    *,
    g1_urdf: str | Path,
    capture_state: str | Path,
    selection_scope: str = SELECTION_SCOPE_AMBIGUITY,
) -> dict[str, Any]:
    if selection_scope not in {SELECTION_SCOPE_AMBIGUITY, SELECTION_SCOPE_ALL}:
        raise AvoidanceError(f"Unsupported review selection scope: {selection_scope}")
    voxel_map = loaded.voxel_map
    report = analysis.report
    contract = {
        "schema_version": 1,
        "world_frame": "base_link",
        "translation_unit": "meter",
        "source_voxels": {
            "path": str(loaded.effective_input.resolve()),
            "sha256": loaded.provenance["effective_sha256"],
            "origin_m": voxel_map.origin.tolist(),
            "voxel_size_m": float(voxel_map.voxel_size),
            "dims": voxel_map.dims.tolist(),
            "occupied_voxel_count": len(voxel_map.indices),
        },
        "robot_state": {
            "capture_state_path": str(Path(capture_state).resolve()),
            "capture_state_sha256": sha256_file(capture_state),
            "g1_urdf_path": str(Path(g1_urdf).resolve()),
            "g1_urdf_sha256": sha256_file(g1_urdf),
        },
        "candidate_parameters": {
            "surface_margin_m": report["surface_margin_m"],
            "voxel_half_diagonal_m": report["voxel_half_diagonal_m"],
            "effective_core_center_margin_m": report[
                "effective_core_center_margin_m"
            ],
            "ambiguity_shell_m": report["ambiguity_shell_m"],
        },
        "candidate_counts": {
            "core": report["core_candidate_voxel_count"],
            "ambiguity_shell": report["ambiguity_shell_voxel_count"],
        },
    }
    # Keep the legacy ambiguity-shell contract byte-for-byte compatible with
    # reviews already created before full-scene manual selection was added.
    if selection_scope != SELECTION_SCOPE_AMBIGUITY:
        contract["selection_scope"] = selection_scope
    return contract


def contract_sha256(contract: dict[str, Any]) -> str:
    return _canonical_sha256(contract)


def component_summary(
    voxel_map: VoxelMap, analysis: SelfFilterAnalysis
) -> tuple[np.ndarray, list[dict[str, Any]]]:
    """Label yellow candidates with 26-connectivity in a tight temporary grid."""
    yellow_indices = voxel_map.indices[analysis.ambiguity_shell_mask]
    source_component_ids = np.zeros(len(voxel_map.indices), dtype=np.int32)
    if not len(yellow_indices):
        return source_component_ids, []
    try:
        from scipy import ndimage
    except ImportError as exc:
        raise AvoidanceError("SciPy is required to label review components") from exc
    lower = yellow_indices.min(axis=0).astype(np.int64)
    upper = yellow_indices.max(axis=0).astype(np.int64)
    shape = tuple(int(v) for v in upper - lower + 1)
    dense = np.zeros(shape, dtype=bool)
    local = yellow_indices.astype(np.int64) - lower
    dense[tuple(local.T)] = True
    labels, count = ndimage.label(dense, structure=np.ones((3, 3, 3), dtype=bool))
    yellow_labels = labels[tuple(local.T)].astype(np.int32)
    source_component_ids[analysis.ambiguity_shell_mask] = yellow_labels

    purple = np.zeros(tuple(int(v) for v in voxel_map.dims), dtype=bool)
    purple_indices = voxel_map.indices[analysis.core_candidate_mask]
    if len(purple_indices):
        purple[tuple(purple_indices.T)] = True
    touching = ndimage.binary_dilation(purple, structure=np.ones((3, 3, 3), dtype=bool))
    summaries = []
    for component_id in range(1, count + 1):
        member_mask = yellow_labels == component_id
        members = yellow_indices[member_mask]
        centers = voxel_map.origin + (members.astype(np.float64) + 0.5) * voxel_map.voxel_size
        summaries.append(
            {
                "component_id": component_id,
                "voxel_count": int(len(members)),
                "touches_core_26_neighbor": bool(np.any(touching[tuple(members.T)])),
                "grid_min": members.min(axis=0).tolist(),
                "grid_max": members.max(axis=0).tolist(),
                "world_center_min_m": centers.min(axis=0).tolist(),
                "world_center_max_m": centers.max(axis=0).tolist(),
            }
        )
    summaries.sort(key=lambda item: (-item["voxel_count"], item["component_id"]))
    return source_component_ids, summaries


def new_review(
    contract: dict[str, Any],
    *,
    core_approved: bool = False,
    operator_note: str = "",
    components: list[dict[str, Any]] | None = None,
) -> dict[str, Any]:
    now = time.time_ns()
    selection_scope = contract.get("selection_scope", SELECTION_SCOPE_AMBIGUITY)
    if selection_scope == SELECTION_SCOPE_ALL:
        return {
            "schema_version": 2,
            "review_kind": "exact_occupied_voxel_review",
            "selection_scope": SELECTION_SCOPE_ALL,
            "candidate_contract_sha256": contract_sha256(contract),
            "candidate_contract": contract,
            "created_at_unix_ns": now,
            "updated_at_unix_ns": now,
            "review_complete": False,
            "selected_robot_voxel_count": 0,
            "selected_robot_indices": [],
            "unselected_voxel_policy": "retain_as_scene",
            "operator_note": str(operator_note),
            "component_summary": components or [],
        }
    return {
        "schema_version": 1,
        "review_kind": "exact_mixed_ambiguity_voxel_review",
        "candidate_contract_sha256": contract_sha256(contract),
        "candidate_contract": contract,
        "created_at_unix_ns": now,
        "updated_at_unix_ns": now,
        "core_approved": bool(core_approved),
        "review_complete": False,
        "selected_yellow_robot_voxel_count": 0,
        "selected_yellow_robot_indices": [],
        "unselected_yellow_policy": "retain_as_scene_or_unapproved",
        "operator_note": str(operator_note),
        "component_summary": components or [],
    }


def _selected_indices(review: dict[str, Any]) -> np.ndarray:
    field = (
        "selected_robot_indices"
        if review_selection_scope(review) == SELECTION_SCOPE_ALL
        else "selected_yellow_robot_indices"
    )
    values = np.asarray(review.get(field, []), dtype=np.int64)
    if values.size == 0:
        return np.empty((0, 3), dtype=np.int64)
    if values.ndim != 2 or values.shape[1] != 3:
        raise AvoidanceError(f"{field} must have shape (N,3)")
    return values


def review_selection_scope(review: dict[str, Any]) -> str:
    if (
        review.get("schema_version") == 2
        and review.get("review_kind") == "exact_occupied_voxel_review"
        and review.get("selection_scope") == SELECTION_SCOPE_ALL
    ):
        return SELECTION_SCOPE_ALL
    return SELECTION_SCOPE_AMBIGUITY


def validate_review(
    review: dict[str, Any],
    contract: dict[str, Any],
    voxel_map: VoxelMap,
    analysis: SelfFilterAnalysis,
    *,
    require_complete: bool,
) -> np.ndarray:
    selection_scope = review_selection_scope(review)
    legacy_review = review.get("schema_version") == 1 and review.get(
        "review_kind"
    ) == "exact_mixed_ambiguity_voxel_review"
    full_scene_review = selection_scope == SELECTION_SCOPE_ALL
    if not (legacy_review or full_scene_review):
        raise AvoidanceError("Unsupported self-filter review schema")
    contract_scope = contract.get("selection_scope", SELECTION_SCOPE_AMBIGUITY)
    if contract_scope != selection_scope:
        raise AvoidanceError("Review selection scope does not match its contract")
    expected = contract_sha256(contract)
    if review.get("candidate_contract_sha256") != expected:
        raise AvoidanceError(
            "Self-filter review does not match the current voxels/URDF/state/parameters"
        )
    if _canonical_sha256(review.get("candidate_contract")) != expected:
        raise AvoidanceError("Self-filter review embeds a modified candidate contract")
    if require_complete and review.get("review_complete") is not True:
        raise AvoidanceError("Self-filter review is still a draft; mark it complete first")
    selected = _selected_indices(review)
    if len(selected):
        if np.any(selected < 0) or np.any(selected >= voxel_map.dims):
            raise AvoidanceError("Selected yellow voxel index falls outside the source grid")
        selected_flat = np.ravel_multi_index(selected.T, tuple(int(v) for v in voxel_map.dims))
        if len(np.unique(selected_flat)) != len(selected_flat):
            raise AvoidanceError("Selected yellow voxel indices contain duplicates")
    else:
        selected_flat = np.empty(0, dtype=np.int64)
    source_flat = np.ravel_multi_index(
        voxel_map.indices.T, tuple(int(v) for v in voxel_map.dims)
    )
    if len(selected_flat) and not np.all(np.isin(selected_flat, source_flat)):
        raise AvoidanceError("Review selected an index that is not an occupied source voxel")
    if selection_scope == SELECTION_SCOPE_AMBIGUITY:
        shell_flat = source_flat[analysis.ambiguity_shell_mask]
        if len(selected_flat) and not np.all(np.isin(selected_flat, shell_flat)):
            raise AvoidanceError("Review selected a voxel outside the current yellow shell")
    selected_mask = np.isin(source_flat, selected_flat)
    count_field = (
        "selected_robot_voxel_count"
        if selection_scope == SELECTION_SCOPE_ALL
        else "selected_yellow_robot_voxel_count"
    )
    declared_count = review.get(count_field)
    if declared_count != int(np.count_nonzero(selected_mask)):
        raise AvoidanceError("Review selected count does not match its exact index list")
    return selected_mask


def load_and_validate_review(
    path: str | Path,
    contract: dict[str, Any],
    voxel_map: VoxelMap,
    analysis: SelfFilterAnalysis,
    *,
    require_complete: bool,
) -> tuple[dict[str, Any], np.ndarray]:
    review = read_json(path)
    selected_mask = validate_review(
        review, contract, voxel_map, analysis, require_complete=require_complete
    )
    return review, selected_mask


def update_review(
    review: dict[str, Any],
    voxel_map: VoxelMap,
    selected_mask: np.ndarray,
    *,
    core_approved: bool | None = None,
    mark_complete: bool = False,
    operator_note: str | None = None,
) -> dict[str, Any]:
    selected_mask = np.asarray(selected_mask, dtype=bool)
    if selected_mask.shape != (len(voxel_map.indices),):
        raise AvoidanceError("Review selection mask has the wrong shape")
    result = dict(review)
    indices = voxel_map.indices[selected_mask]
    if len(indices):
        order = np.lexsort((indices[:, 2], indices[:, 1], indices[:, 0]))
        indices = indices[order]
    selection_scope = review_selection_scope(review)
    if selection_scope == SELECTION_SCOPE_ALL:
        result["selected_robot_indices"] = indices.astype(int).tolist()
        result["selected_robot_voxel_count"] = int(len(indices))
    else:
        result["selected_yellow_robot_indices"] = indices.astype(int).tolist()
        result["selected_yellow_robot_voxel_count"] = int(len(indices))
    if core_approved is not None and selection_scope == SELECTION_SCOPE_AMBIGUITY:
        result["core_approved"] = bool(core_approved)
    if operator_note is not None:
        result["operator_note"] = str(operator_note)
    result["review_complete"] = bool(mark_complete)
    result["updated_at_unix_ns"] = time.time_ns()
    return result


def write_review(path: str | Path, review: dict[str, Any]) -> None:
    write_json(path, review)
