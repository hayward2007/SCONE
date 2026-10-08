import json,gzip,sys,math
import numpy as np
import fcl
from pathlib import Path
sys.path.insert(0,str(Path(__file__).parent))
from rom_mesh import Shape,matrix
p=Path('artifacts/housing/20260915_MARC_v10/R04_DELIVERY');items=json.load(gzip.open(p/'validation_meshes.json.gz','rt'));targets=[Shape(x) for x in items if not x['name'].startswith('LEG')]
base=json.load(open(p.parent/'R03_DELIVERY/final_inventory.json'));m=np.array(next(x['transform'] for x in base['occurrences'] if x['name']=='LEG 1:5+ARC:1')).reshape(4,4);m[:3,3]*=10
m[:3,3]+=m[:3,2]*5
# Tight full-spin envelope, 2.1 mm nominal space; sampling is not a proof for arbitrary combinations.
g=fcl.Cylinder(124.6,14.2);obj=fcl.CollisionObject(g);hits=[]
for deg in list(range(0,93))+[91.985]:
 tm=matrix(deg,'z',(-41.8912974452,-65.140925829,39))@matrix(-.14356,'y',(-41.8912974452,-95.140925829,18.5))@m
 obj.setTransform(fcl.Transform(tm[:3,:3],tm[:3,3]))
 for t in targets:
  if fcl.collide(obj,t.obj,fcl.CollisionRequest(),fcl.CollisionResult()):hits.append([deg,t.name])
r={'yaw_world_deg':[0,91.985],'yaw_step_deg':1,'hip_world_deg':-.14356,'distal_spin':'continuous cylinder','envelope_radial_and_axial_allowance_mm':2.1,'hits':hits};(p/'yaw_path_screen.json').write_text(json.dumps(r,indent=2));print(r)
