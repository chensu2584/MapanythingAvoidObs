import unittest

import numpy as np

from avoidance.tcp_model import hand_t_tcp


class TcpModelTests(unittest.TestCase):
    def test_reference_plus_measurement_has_explicit_composition(self):
        matrix, report = hand_t_tcp(
            {
                "mode": "urdf_reference_plus_measured_correction",
                "urdf_reference_translation_m": [0, 0, 0.14308],
                "urdf_reference_rpy_rad": [0, 0, 0],
                "measured_correction_translation_m": [0.001, -0.002, 0.003],
                "measured_correction_rpy_rad": [0, 0, 0],
            }
        )
        np.testing.assert_allclose(matrix[:3, 3], [0.001, -0.002, 0.14608])
        self.assertIn("urdf_reference", report["composition"])


if __name__ == "__main__":
    unittest.main()
