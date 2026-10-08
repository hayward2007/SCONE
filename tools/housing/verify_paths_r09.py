import json,gzip,sys,math
import numpy as np
import fcl
from pathlib import Path
sys.path.insert(0,str(Path(__file__).parent));from rom_mesh import Shape,matrix
p=Path('artifacts/housing/20260915_MARC_v10/R09_DELIVERY');items=json.load(gzip.open(p/'validation_meshes.json.gz','rt'));targets=[Shape(x) for x in items if not x['name'].startswith('LEG')]
saved=json.load(open(p/'user_pose_before.json'));old=json.load(open(p.parent/'R03_DELIVERY/final_inventory.json'));now={x['name']:x for x in saved['occurrences']};before={x['name']:x for x in old['occurrences']}
path='LEG 1:5+ARC:1';m=np.array(now[path]['transform']).reshape(4,4);m[:3,3]*=10;m[:3,3]+=m[:3,2]*10
obj=fcl.CollisionObject(fcl.Cylinder(122.5,20),fcl.Transform(m[:3,:3],m[:3,3]));distances=[]
for t in targets:
 distances.append([t.name,float(fcl.distance(obj,t.obj,fcl.DistanceRequest(enable_nearest_points=True),fcl.DistanceResult()))])
basemat=np.array(before[path]['transform']).reshape(4,4);basemat[:3,3]*=10;basemat[:3,3]+=basemat[:3,2]*10
obj=fcl.CollisionObject(fcl.Cylinder(124.6,24.2));hits=[]
for deg in range(93):
 tm=matrix(deg,'z',(-41.8912974452,-65.140925829,39))@matrix(-.14356,'y',(-41.8912974452,-95.140925829,18.5))@basemat
 for side in [-1,1]:
  sm=tm.copy()
  if side>0:sm[1,:]*=-1;sm[:3,0]*=-1
  obj.setTransform(fcl.Transform(sm[:3,:3],sm[:3,3]))
  for t in targets:
   if fcl.collide(obj,t.obj,fcl.CollisionRequest(),fcl.CollisionResult()):hits.append([side,deg,t.name])
r={'basis':'Explicit saved user pose matrices, independent FCL test against final Fusion housing mesh','rear_yaw_world_deg':[0,92],'yaw_step_deg':1,'hip_world_deg':-.14356,'sides':[-1,1],'continuous_distal_spin_at_each_yaw_sample':True,'envelope_allowance_mm':2.1,'hits':hits,'minimum_clearances_at_exact_user_pose_mm':sorted(distances,key=lambda a:a[1]),'limits':'Yaw path sampled at 1deg; arbitrary hip motion, all simultaneous legs and structural deflection not certified'}
(p/'yaw_path_screen.json').write_text(json.dumps(r,indent=2));print('PATH HITS',len(hits),'MIN DISTANCES',r['minimum_clearances_at_exact_user_pose_mm'][:5])
