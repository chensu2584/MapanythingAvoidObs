import unittest

import numpy as np

from avoidance.contracts import VoxelMap
from avoidance.operation_map import inflate_voxels


class OperationMapTests(unittest.TestCase):
    def test_single_voxel_inflation_is_metric_and_expands_bounds(self):
        source = VoxelMap(
            indices=np.asarray([[0, 0, 0]], dtype=np.int32),
            origin=np.zeros(3),
            voxel_size=0.02,
            dims=np.ones(3, dtype=np.int32),
            colors=np.asarray([[10, 20, 30]], dtype=np.uint8),
        )
        result = inflate_voxels(
            source, np.asarray([True]), radius_m=0.05, max_dense_cells=1_000_000
        )
        self.assertEqual(result.padding_cells, 3)
        np.testing.assert_allclose(result.voxel_map.origin, [-0.06, -0.06, -0.06])
        np.testing.assert_array_equal(result.voxel_map.dims, [7, 7, 7])
        self.assertEqual(len(result.voxel_map.indices), 275)
        offsets = {tuple(index - 3) for index in result.voxel_map.indices}
        self.assertIn((3, 0, 0), offsets)
        self.assertIn((3, 2, 0), offsets)
        self.assertNotIn((3, 3, 0), offsets)
        self.assertEqual(np.count_nonzero(result.cell_kind == 1), 1)

    def test_zero_radius_keeps_source(self):
        source = VoxelMap(
            indices=np.asarray([[1, 1, 1]], dtype=np.int32),
            origin=np.zeros(3),
            voxel_size=0.1,
            dims=np.asarray([3, 3, 3]),
        )
        result = inflate_voxels(
            source, np.asarray([True]), radius_m=0.0, max_dense_cells=100
        )
        np.testing.assert_array_equal(result.voxel_map.indices, source.indices)
        np.testing.assert_array_equal(result.voxel_map.dims, source.dims)


if __name__ == "__main__":
    unittest.main()
