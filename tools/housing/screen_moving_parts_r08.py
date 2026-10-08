import json,gzip,sys
import numpy as np
from pathlib import Path
sys.path.insert(0,str(Path(__file__).parent));from rom_mesh import Shape,matrix
p=Path('artifacts/housing/20260915_MARC_v10/R08_DELIVERY');items=json.load(gzip.open(p/'validation_meshes.json.gz','rt'));targets=[Shape(x) for x in items if not x['name'].startswith('LEG')]
a={x['name']:x for x in json.load(open(p/'user_pose_before.json'))['occurrences']};b={x['name']:x for x in json.load(open(p.parent/'R03_DELIVERY/final_inventory.json'))['occurrences']}
mov=[]
for x in items:
 if x['name'].startswith('LEG 1:5+') and '+ARC:' not in x['name']:
  path=x['name'].rsplit('/',1)[0];ma=np.array(a[path]['transform']).reshape(4,4);mb=np.array(b[path]['transform']).reshape(4,4);ma[:3,3]*=10;mb[:3,3]*=10
  mov.append((Shape(x),np.linalg.inv(ma@np.linalg.inv(mb)),path))
hits=[]
for deg in range(93):
 ym=matrix(deg,'z',(-41.8912974452,-65.140925829,39));hm=matrix(-.14356,'y',(-41.8912974452,-95.140925829,18.5))
 for sh,undo,path in mov:
  sh.set(ym@(np.eye(4) if '+FR07:' in path else hm)@undo)
  for t in targets:
   if sh.hits(t):hits.append([deg,sh.name,t.name])
r={'yaw_step_deg':1,'yaw_range_deg':[0,92],'hip_deg':-.14356,'method':'FCL meshes 0.03mm; includes surface contact, native volume witness required','hits':hits};(p/'moving_parts_path.json').write_text(json.dumps(r,indent=2));print('HITS',len(hits));print(hits[:12])
