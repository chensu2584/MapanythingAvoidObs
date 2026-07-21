import tempfile
import unittest
from pathlib import Path

import numpy as np

from avoidance.contracts import AvoidanceError
from avoidance.map_io import load_voxel_map


class MapIoTests(unittest.TestCase):
    def write_voxels(self, directory: Path, embed_frame: bool = True):
        fields = dict(
            indices=np.asarray([[0, 0, 0]], dtype=np.int32),
            origin=np.zeros(3),
            voxel_size=np.float64(0.02),
            dims=np.asarray([1, 1, 1], dtype=np.int32),
            colors=np.asarray([[1, 2, 3]], dtype=np.uint8),
        )
        if embed_frame:
            fields.update(world_frame=np.asarray("base_link"), translation_unit=np.asarray("meter"))
        np.savez_compressed(directory / "voxels.npz", **fields)

    def test_loads_embedded_base_link_contract(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self.write_voxels(root)
            loaded = load_voxel_map(root)
            self.assertEqual(loaded.voxel_map.world_frame, "base_link")
            self.assertEqual(loaded.source_kind, "output_directory_voxels_npz")

    def test_refuses_npz_with_no_frame_evidence(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self.write_voxels(root, embed_frame=False)
            with self.assertRaisesRegex(AvoidanceError, "Cannot prove"):
                load_voxel_map(root)

    def test_glb_routes_to_canonical_paired_npz(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            self.write_voxels(root)
            (root / "voxels.glb").write_bytes(b"visualization-placeholder")
            loaded = load_voxel_map(root / "voxels.glb")
            self.assertEqual(loaded.effective_input.name, "voxels.npz")
            self.assertEqual(loaded.source_kind, "glb_with_paired_voxels_npz")


if __name__ == "__main__":
    unittest.main()
