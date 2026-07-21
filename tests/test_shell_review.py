import tempfile
import unittest
from pathlib import Path

import numpy as np

from avoidance.contracts import AvoidanceError, VoxelMap, sha256_file
from avoidance.map_io import LoadedVoxelMap
from avoidance.operation_map import SelfFilterAnalysis
from avoidance.shell_review import (
    SELECTION_SCOPE_ALL,
    build_review_contract,
    component_summary,
    new_review,
    update_review,
    validate_review,
)


class ShellReviewTests(unittest.TestCase):
    def fixture(self, root: Path):
        voxel_path = root / "voxels.npz"
        state_path = root / "capture_state.json"
        urdf_path = root / "G1.urdf"
        voxel_path.write_bytes(b"voxel-source")
        state_path.write_text("{}", encoding="utf-8")
        urdf_path.write_text("<robot name='x'/>", encoding="utf-8")
        voxel_map = VoxelMap(
            indices=np.asarray(
                [[0, 0, 0], [1, 1, 1], [2, 1, 1], [5, 5, 5]], dtype=np.int32
            ),
            origin=np.zeros(3),
            voxel_size=0.01,
            dims=np.asarray([6, 6, 6]),
        )
        loaded = LoadedVoxelMap(
            voxel_map=voxel_map,
            requested_input=voxel_path,
            effective_input=voxel_path,
            source_kind="test",
            provenance={"effective_sha256": sha256_file(voxel_path)},
            capture_dir=root,
        )
        analysis = SelfFilterAnalysis(
            core_candidate_mask=np.asarray([True, False, False, False]),
            ambiguity_shell_mask=np.asarray([False, True, True, True]),
            report={
                "surface_margin_m": 0.005,
                "voxel_half_diagonal_m": 0.00866,
                "effective_core_center_margin_m": 0.01366,
                "ambiguity_shell_m": 0.105,
                "core_candidate_voxel_count": 1,
                "ambiguity_shell_voxel_count": 3,
            },
        )
        contract = build_review_contract(
            loaded, analysis, g1_urdf=urdf_path, capture_state=state_path
        )
        return voxel_map, analysis, contract

    def test_exact_selection_roundtrip_and_non_shell_rejection(self):
        with tempfile.TemporaryDirectory() as tmp:
            voxel_map, analysis, contract = self.fixture(Path(tmp))
            review = new_review(contract, core_approved=True)
            selected = np.asarray([False, True, False, False])
            complete = update_review(
                review, voxel_map, selected, mark_complete=True
            )
            mask = validate_review(
                complete, contract, voxel_map, analysis, require_complete=True
            )
            np.testing.assert_array_equal(mask, selected)

            bad = dict(complete)
            bad["selected_yellow_robot_indices"] = [[0, 0, 0]]
            with self.assertRaisesRegex(AvoidanceError, "outside the current yellow"):
                validate_review(
                    bad, contract, voxel_map, analysis, require_complete=True
                )

    def test_contract_change_and_draft_fail_closed(self):
        with tempfile.TemporaryDirectory() as tmp:
            voxel_map, analysis, contract = self.fixture(Path(tmp))
            review = new_review(contract, core_approved=True)
            with self.assertRaisesRegex(AvoidanceError, "still a draft"):
                validate_review(
                    review, contract, voxel_map, analysis, require_complete=True
                )

    def test_full_scene_review_accepts_any_occupied_voxel_only(self):
        with tempfile.TemporaryDirectory() as tmp:
            voxel_map, analysis, legacy_contract = self.fixture(Path(tmp))
            contract = dict(legacy_contract)
            contract["selection_scope"] = SELECTION_SCOPE_ALL
            review = new_review(contract)
            self.assertEqual(review["schema_version"], 2)
            selected = np.asarray([True, False, False, True])
            complete = update_review(
                review, voxel_map, selected, mark_complete=True
            )
            mask = validate_review(
                complete, contract, voxel_map, analysis, require_complete=True
            )
            np.testing.assert_array_equal(mask, selected)

            bad = dict(complete)
            bad["selected_robot_indices"] = [[0, 0, 1]]
            bad["selected_robot_voxel_count"] = 1
            with self.assertRaisesRegex(AvoidanceError, "not an occupied source voxel"):
                validate_review(
                    bad, contract, voxel_map, analysis, require_complete=True
                )
            changed = dict(contract)
            changed["candidate_counts"] = dict(contract["candidate_counts"])
            changed["candidate_counts"]["ambiguity_shell"] = 99
            with self.assertRaisesRegex(AvoidanceError, "does not match"):
                validate_review(
                    review, changed, voxel_map, analysis, require_complete=False
                )

    def test_component_summary_keeps_disconnected_regions_separate(self):
        with tempfile.TemporaryDirectory() as tmp:
            voxel_map, analysis, _ = self.fixture(Path(tmp))
            ids, summary = component_summary(voxel_map, analysis)
        self.assertEqual(len(summary), 2)
        self.assertEqual([item["voxel_count"] for item in summary], [2, 1])
        self.assertEqual(ids[1], ids[2])
        self.assertNotEqual(ids[1], ids[3])


if __name__ == "__main__":
    unittest.main()
