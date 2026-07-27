import importlib.util
import unittest
from pathlib import Path
from avoidance.g2_robot_model import G2RobotModel, load_g2_capture_state

ROOT = Path(__file__).resolve().parents[2]
SNAPSHOT = ROOT / "G2/expoutput3/snapshot_20260723_034729_0001"
BACKEND = importlib.util.find_spec("pinocchio") is not None and importlib.util.find_spec("hppfcl") is not None

class CaptureTests(unittest.TestCase):
    def test_capture(self):
        values, report = load_g2_capture_state(SNAPSHOT/"capture_state.json")
        self.assertEqual(len(values), 22); self.assertEqual(report["joint_count"], 22)

@unittest.skipUnless(BACKEND, "robot backend required")
class PlannerTests(unittest.TestCase):
    def setUp(self):
        self.robot=G2RobotModel(); values,_=load_g2_capture_state(SNAPSHOT/"capture_state.json"); self.q=self.robot.configuration_from_positions(values)
    def test_complete_mode_fails_closed(self):
        from avoidance.planner import plan_g2_avoidance
        result=plan_g2_avoidance(scene_path=SNAPSHOT,capture_state_path=SNAPSHOT/"capture_state.json",side="left",goal_arm=self.robot.arm_configuration(self.q,"left"))
        self.assertEqual(result["status"],"blocked")
    def test_demo_excludes_gripper_and_plans(self):
        from avoidance.planner import plan_g2_avoidance
        goal=self.robot.arm_configuration(self.q,"left");goal[0]+=.03
        result=plan_g2_avoidance(scene_path=SNAPSHOT,capture_state_path=SNAPSHOT/"capture_state.json",side="left",goal_arm=goal,arm_body_demo=True)
        self.assertEqual(result["status"],"demo_planned");self.assertFalse(result["execution_authorized"])
        self.assertTrue(all(x.startswith("gripper_") for x in result["collision_policy"]["ignored_collision_geometries"]))
    def test_demo_tracks_cartesian_goal_with_flange(self):
        from avoidance.planner import plan_g2_avoidance
        flange_pose = self.robot.frame_pose(self.q, "arm_l_end_link")
        result = plan_g2_avoidance(
            scene_path=SNAPSHOT,
            capture_state_path=SNAPSHOT/"capture_state.json",
            side="left",
            goal_pose=flange_pose,
            arm_body_demo=True,
        )
        self.assertEqual(result["status"], "demo_planned")
        self.assertEqual(result["goal"]["type"], "flange_pose")
        self.assertEqual(result["goal"]["frame"], "arm_l_end_link")
        self.assertFalse(result["execution_authorized"])
