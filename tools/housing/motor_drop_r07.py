import json,gzip,sys,math
import numpy as np
import fcl
from pathlib import Path
sys.path.insert(0,str(Path(__file__).parent));from rom_mesh import Shape
p=Path('artifacts/housing/20260915_MARC_v10/R07_DELIVERY');data=json.load(gzip.open(p/'validation_meshes.json.gz','rt'))
parts=[Shape(x) for x in data if not x['name'].startswith(('LEG','V'))];lower=[s for s in parts if s.name.startswith(('R01 Rear','R02 Front'))]
allsh=[Shape(x) for x in data];camera=next(s for s in allsh if s.name.startswith('V02'));cover=next(s for s in parts if s.name.startswith('S02'))
cases=[s for s in allsh if s.name.startswith('LEG') and '+' not in s.name];hits=[]
for sh in cases:
 side=1 if '(미러)' in sh.name else -1
 for shift in range(0,81,2):
  m=np.eye(4);m[2,3]=shift;sh.set(m)
  for t in lower:
   if sh.hits(t):
    req=fcl.CollisionRequest(num_max_contacts=12,enable_contact=True);res=fcl.CollisionResult();fcl.collide(sh.obj,t.obj,req,res);hits.append([sh.name,shift,t.name,[np.asarray(c.pos).tolist() for c in res.contacts[:6]]])
 sh.set(np.eye(4))
print('MX top drop hits',len(hits),hits[:12],flush=True)
# Compare alternative camera raise points inside detached front cover.
camera_routes=[]
for dx in [-115,-105,-95,-85,-75,-66,-56,-46,-36,-26,-16,-6]:
 hs=[]
 for dz in range(-100,1,2):
  m=np.eye(4);m[:3,3]=[dx,0,dz];camera.set(m)
  if camera.hits(cover):hs.append(dz)
 camera_routes.append([dx,hs])
print('camera routes',[(dx,len(h)) for dx,h in camera_routes],flush=True)
# Static contact pairs include intended seating; unknown native overlaps remain unresolved.
contacts=[]
for i,a in enumerate(parts):
 for b in parts[i+1:]:
  if a.hits(b):
   req=fcl.CollisionRequest(num_max_contacts=20,enable_contact=True);res=fcl.CollisionResult();fcl.collide(a.obj,b.obj,req,res)
   contacts.append([a.name,b.name,[np.asarray(c.pos).tolist() for c in res.contacts[:10]]])
r={'mx_slide_step_mm':2,'mx_top_insertion_range_mm':[0,80],'mx_hits':hits,'camera_raise_routes':camera_routes,'part_contacts':contacts};(p/'assembly_screen.json').write_text(json.dumps(r,indent=2))
print('contacts',contacts,flush=True)
