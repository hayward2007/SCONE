"""Read-only mesh screening, 0.05 mm tessellation. Does not command robot or change CAD."""
from pathlib import Path
import gzip,json,math,time
import numpy as np
import fcl
OUT=Path(__file__).resolve().parents[2]/'artifacts/housing/20260915_MARC_v10'
data=json.load(gzip.open(OUT/'analysis_meshes.json.gz','rt'))
def matrix(deg,axis,point):
 a=math.radians(deg);c,s=math.cos(a),math.sin(a)
 R=np.array([[c,0,s],[0,1,0],[-s,0,c]]) if axis=='y' else np.array([[c,-s,0],[s,c,0],[0,0,1]])
 p=np.array(point);m=np.eye(4);m[:3,:3]=R;m[:3,3]=p-R@p;return m
class Shape:
 def __init__(self,d,offset=(0,0,0)):
  self.name=d['name'];self.v=np.array(d['vertices_mm']).reshape(-1,3)+offset
  self.faces=np.array(d['triangles'],dtype=np.int32).reshape(-1,3)
  self.geom=fcl.BVHModel();self.geom.beginModel(len(self.v),len(self.faces));self.geom.addSubModel(self.v,self.faces);self.geom.endModel()
  self.obj=fcl.CollisionObject(self.geom);self.set(np.eye(4))
 def set(self,m):
  self.obj.setTransform(fcl.Transform(m[:3,:3],m[:3,3]));v=self.v@m[:3,:3].T+m[:3,3];self.lo=v.min(0);self.hi=v.max(0)
 def hits(self,other):
  if np.any(self.hi<other.lo-.01) or np.any(other.hi<self.lo-.01):return False
  return bool(fcl.collide(self.obj,other.obj,fcl.CollisionRequest(),fcl.CollisionResult()))
def check():
 leg=[Shape(v,(250,0,0)) for v in data if v['name'].startswith('LEG 1:5+')]
 arc=[s for s in leg if '+ARC:' in s.name];link=[s for s in leg if '+ARC:' not in s.name]
 floor=[Shape(v) for v in data if v['name'].startswith('ROOT/')]
 added=[Shape(v) for v in data if v['name'].startswith('H')]
 other=[Shape(v) for v in data if v['name'].startswith('LEG') and not v['name'].startswith('LEG 1:1+')]
 fixed=[s for s in other if '+' not in s.name]
 others=[s for s in other if '+' in s.name]
 report={'mesh_tolerance_mm':.05,'yaw_world_deg':90,'hip_step_deg':10,'spin_step_deg':5,'hip':[]}
 # User joint's positive yaw orientation is checked in BOTH world signs.
 for yaw in [90,-90]:
  ym=matrix(yaw,'z',(208.1087025548,-65.140925829,39))
  for hip in range(-180,180,10):
   pm=matrix(hip,'y',(208.1087025548,-95.140925829,18.5));base=ym@pm
   for s in link:s.set(ym if '+FR07:' in s.name else base)
   static={(s.name,t.name) for s in link for t in floor+added+fixed if s.hits(t)}
   row={'yaw_world':yaw,'hip':hip,'static_hits':list(static),'blocked_spin':{},'full_spin_clear_body':True,'full_spin_clear_self':True,'full_spin_clear_others':True}
   for spin in range(0,360,5):
    sm=matrix(spin,'y',(85.6087025548,-138.140925829,18.5))
    for s in arc:s.set(base@sm)
    hits=[]
    for category,targets in [('body',floor+added+fixed),('self',link),('others',others)]:
     pairs=[(s.name,t.name) for s in arc for t in targets if s.hits(t)]
     if pairs:row['full_spin_clear_'+category]=False;hits += [(category,*p) for p in pairs]
    if hits:row['blocked_spin'][str(spin)]=hits
   report['hip'].append(row)
   (OUT/'yaw90_screen.json').write_text(json.dumps(report,ensure_ascii=False))
   print(yaw,hip,len(static),len(row['blocked_spin']),row['full_spin_clear_body'],row['full_spin_clear_self'],row['full_spin_clear_others'],flush=True)
 return report
if __name__=='__main__':check()
