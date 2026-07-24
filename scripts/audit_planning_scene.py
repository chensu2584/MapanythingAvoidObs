#!/usr/bin/env python3
import argparse, datetime as dt, sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from avoidance.contracts import write_json
from avoidance.planning_scene import load_planning_scene
p=argparse.ArgumentParser();p.add_argument("--input",type=Path,required=True);p.add_argument("--out",type=Path,required=True);p.add_argument("--planning-inflation-m",type=float,default=.08);a=p.parse_args()
paths=sorted(a.input.glob("snapshot_*")) if a.input.is_dir() and not (a.input/"obstacles.json").exists() else [a.input]
items=[]
for path in paths:
 s=load_planning_scene(path);items.append({"snapshot":path.name,"valid":True,"primitive_count":len(s.primitives),"box_count":sum(x.kind=="box" for x in s.primitives),"cylinder_count":sum(x.kind=="cylinder" for x in s.primitives),"marker_count":len(s.markers),"scene_sha256":s.source_sha256})
write_json(a.out,{"schema_version":1,"created_at":dt.datetime.now(dt.timezone.utc).isoformat(),"status":"passed","snapshot_count":len(items),"planning_inflation_m":a.planning_inflation_m,"snapshots":items})
print(f"wrote {a.out}")
