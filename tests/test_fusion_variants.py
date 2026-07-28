"""Tests for the three cleaned output variants exported by the fusion CLI."""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from avoidance.depth_fusion import (
    PROVENANCE_DEPTH,
    PROVENANCE_MAP,
    load_voxel_cloud,
    save_fused,
)

SCRIPT_PATH = Path(__file__).resolve().parents[1] / "scripts" / "fuse_depth_and_map.py"
SPEC = importlib.util.spec_from_file_location("avoid_fuse_depth_and_map", SCRIPT_PATH)
assert SPEC is not None and SPEC.loader is not None
FUSION_SCRIPT = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = FUSION_SCRIPT
SPEC.loader.exec_module(FUSION_SCRIPT)
cleaned_source_result = FUSION_SCRIPT.cleaned_source_result


@pytest.mark.parametrize(
    ("source", "expected_provenance", "expected_policy"),
    (
        ("depth", PROVENANCE_DEPTH, "cleaned_metric_depth_only"),
        ("map", PROVENANCE_MAP, "cleaned_mapanything_only"),
    ),
)
def test_cleaned_source_result_preserves_cleaned_geometry(
    tmp_path, source, expected_provenance, expected_policy
):
    points = np.array(
        [[0.25, -0.69, 0.61], [1.01, 0.70, 0.67]],
        dtype=np.float64,
    )
    colors = np.array([[10, 20, 30], [40, 50, 60]], dtype=np.uint8)
    cleaned = SimpleNamespace(
        points_m=points,
        colors=colors,
        report={
            "parameters": {
                "table_xy_bounds": [0.239, 1.019, -0.694, 0.706],
            }
        },
    )

    result = cleaned_source_result(
        cleaned,
        source=source,
        voxel_size_m=0.01,
        world_frame="base_link",
        source_path=tmp_path / "source.npz",
    )

    assert np.array_equal(result.points_m, points)
    assert np.array_equal(result.colors, colors)
    assert set(result.provenance.tolist()) == {expected_provenance}
    assert result.report["policy"] == expected_policy
    assert result.report["cleanup"] == cleaned.report

    path = save_fused(result, tmp_path / f"{source}.npz")
    reloaded_points, reloaded_colors, voxel_size, frame = load_voxel_cloud(path)
    assert len(reloaded_points) == len(points)
    assert np.array_equal(reloaded_colors, colors)
    assert voxel_size == pytest.approx(0.01)
    assert frame == "base_link"


def test_cleaned_source_result_rejects_unknown_source(tmp_path):
    cleaned = SimpleNamespace(
        points_m=np.ones((1, 3)),
        colors=np.ones((1, 3), dtype=np.uint8),
        report={},
    )
    with pytest.raises(ValueError, match="unknown cleaned source"):
        cleaned_source_result(
            cleaned,
            source="rgb",
            voxel_size_m=0.01,
            world_frame="base_link",
            source_path=tmp_path / "source.npz",
        )
