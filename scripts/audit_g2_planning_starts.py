#!/usr/bin/env python3
import argparse,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from avoidance.contracts import write_json
from avoidance.g2_robot_model import G2RobotModel
from avoidance.end_effector_model import load_end_effector_model_status
p=argparse.ArgumentParser();p.add_argument("--input",type=Path,required=True);p.add_argument("--out",type=Path,required=True);a=p.parse_args()
r=G2RobotModel();ee=load_end_effector_model_status().compatibility_report(r);paths=sorted(a.input.glob("snapshot_*"))
items=[{"snapshot":x.name,"valid":False,"not_run":True,"reason":"installed_end_effector_model_unconfirmed"} for x in paths]
write_json(a.out,{"schema_version":1,"status":"blocked","snapshot_count":len(items),"valid_start_count":0,"previous_omnipicker_based_results_valid":False,"end_effector_model":ee,"snapshots":items,"execution_authorized":False})
print(f"wrote {a.out}")
