#!/usr/bin/env python3
"""Generate an offline G2 plan manifest."""
import argparse
import sys
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from avoidance.contracts import AvoidanceError, write_json
from avoidance.g2_robot_model import load_arm_goal, load_pose_goal
from avoidance.planner import plan_g2_avoidance

def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--scene", type=Path, required=True); p.add_argument("--capture-state", type=Path, required=True)
    p.add_argument("--arm", choices=("left","right"), required=True)
    goal = p.add_mutually_exclusive_group(required=True); goal.add_argument("--goal-joints", type=Path); goal.add_argument("--goal-pose", type=Path)
    p.add_argument("--out", type=Path, required=True); p.add_argument("--arm-body-demo", action="store_true")
    args = p.parse_args()
    try:
        result = plan_g2_avoidance(scene_path=args.scene, capture_state_path=args.capture_state, side=args.arm, goal_arm=load_arm_goal(args.goal_joints,args.arm) if args.goal_joints else None, goal_pose=load_pose_goal(args.goal_pose) if args.goal_pose else None, arm_body_demo=args.arm_body_demo)
        write_json(args.out, result)
        print(f"plan manifest: {args.out.resolve()}")
        return 0 if result["status"] in {"planned","demo_planned"} else 2
    except AvoidanceError as exc:
        print(f"error: {exc}", file=sys.stderr); return 2
if __name__ == "__main__": raise SystemExit(main())
