import json
import tempfile
import unittest
from pathlib import Path

import numpy as np

from avoidance.robot_model import (
    UrdfRobot,
    classify_robot_voxels,
    normalize_joint_state,
)


class RobotModelTests(unittest.TestCase):
    def test_fk_and_primitive_query(self):
        with tempfile.TemporaryDirectory() as tmp:
            urdf = Path(tmp) / "robot.urdf"
            urdf.write_text(
                """<robot name="test">
                <link name="base_link"/><link name="moving">
                  <collision><origin xyz="0 0 0.5" rpy="0 0 0"/>
                    <geometry><sphere radius="0.1"/></geometry></collision>
                </link>
                <joint name="slide" type="prismatic"><parent link="base_link"/>
                  <child link="moving"/><origin xyz="1 0 0" rpy="0 0 0"/>
                  <axis xyz="0 0 1"/></joint>
                </robot>""",
                encoding="utf-8",
            )
            robot = UrdfRobot(urdf)
            pose = robot.base_to_frame("moving", {"slide": 0.2})
            np.testing.assert_allclose(pose[:3, 3], [1.0, 0.0, 0.2])
            primitive = robot.world_primitives({"slide": 0.2})[0]
            points = np.asarray([[1.0, 0.0, 0.7], [1.3, 0.0, 0.7]])
            core, shell, counts = classify_robot_voxels(
                points, [primitive], core_margin_m=0.01, ambiguity_shell_m=0.2
            )
            np.testing.assert_array_equal(core, [True, False])
            np.testing.assert_array_equal(shell, [False, True])
            self.assertEqual(counts["moving"], 1)

    def test_normalize_sdk_joint_order_and_head_degrees(self):
        positions, report = normalize_joint_state(
            {
                "arm_joint_states": list(range(14)),
                "head_joint_states": [90, -90],
                "waist_joint_states": [0.25, 0.4],
                "units": {
                    "arm": "rad",
                    "head": "deg",
                    "waist_pitch": "rad",
                    "waist_lift": "m",
                },
            }
        )
        self.assertAlmostEqual(positions["idx01_waist_lift_joint"], 0.4)
        self.assertAlmostEqual(positions["idx02_waist_pitch_joint"], 0.25)
        self.assertAlmostEqual(positions["idx03_head_yaw_joint"], np.pi / 2)
        self.assertEqual(positions["idx05_left_arm_joint1"], 0.0)
        self.assertEqual(positions["idx12_right_arm_joint1"], 7.0)
        self.assertAlmostEqual(report["head_joint_states_rad"][1], -np.pi / 2)


if __name__ == "__main__":
    unittest.main()
