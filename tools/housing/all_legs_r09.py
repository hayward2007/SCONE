"""Independent mesh check of four legs in the requested critical pose."""
import json,gzip,sys
from pathlib import Path
import numpy as np
import fcl
sys.path.insert(0,str(Path(__file__).parent))
from rom_mesh import Shape,matrix
p=Path('artifacts/housing/20260915_MARC_v10/R09_DELIVERY')
items=json.load(gzip.open(p/'validation_meshes.json.gz','rt'))
saved={x['name']:x for x in json.load(open(p/'user_pose_before.json'))['occurrences']}
old={x['name']:x for x in json.load(open(p.parent/'R03_DELIVERY/final_inventory.json'))['occurrences']}
def mat(x):
 m=np.array(x['transform']).reshape(4,4);m[:3,3]*=10;return m
targets=[Shape(x) for x in items if not x['name'].startswith('LEG')];legs=[];envs=[]
for x in items:
 if not x['name'].startswith('LEG'):continue
 path=x['name'].rsplit('/',1)[0];parent=path.split('+')[0];side=1 if '(미러)' in parent else -1;xc=208.1087025548 if parent in ['LEG 1:1','LEG 1(미러):1'] else -41.8912974452
 undo=mat(old[path])@np.linalg.inv(mat(saved[path]));motion=np.eye(4)
 if '+' in path:motion=matrix(side*90,'z',(xc,side*65.140925829,39))@(np.eye(4) if '+FR07:' in path else matrix(180,'y',(xc,side*95.140925829,18.5)))
 sh=Shape(x);sh.set(motion@undo);legs.append((parent,sh))
 if '+ARC:' in path and x['name'].split('/')[-1].startswith('T'):
  m=motion@mat(old[path]);m[:3,3]+=m[:3,2]*(10 if side<0 else -10);envs.append((parent,fcl.CollisionObject(fcl.Cylinder(124.6,24.2),fcl.Transform(m[:3,:3],m[:3,3]))))
hits=[]
for parent,obj in envs:
 for sh in targets+[s for par,s in legs if par!=parent]:
  if fcl.collide(obj,sh.obj,fcl.CollisionRequest(),fcl.CollisionResult()):hits.append([parent,sh.name])
# Exact user-pose full spin is a body clearance requirement, not permission for all other legs to occupy any position.
actual=mat(saved['LEG 1:5+ARC:1']);actual[:3,3]+=actual[:3,2]*10
obj=fcl.CollisionObject(fcl.Cylinder(122.5,20),fcl.Transform(actual[:3,:3],actual[:3,3]));user_others=[]
for x in items:
 if x['name'].startswith('LEG') and not x['name'].startswith('LEG 1:5'):
  sh=Shape(x)
  if fcl.collide(obj,sh.obj,fcl.CollisionRequest(),fcl.CollisionResult()):user_others.append(x['name'])
result={'four_legs_MX_inward90_hip180_continuous_distal_spin':{'wheel_width_mm':20,'radial_and_axial_allowance_mm':2.1,'hits':hits},'exact_user_rear_pose_full_spin_vs_other_legs_in_saved_positions':user_others,'limits':'Analytic cylinders against 0.03mm native meshes; joint trajectories and arbitrary simultaneous leg combinations are not certified.'}
(p/'all_legs_motion.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
