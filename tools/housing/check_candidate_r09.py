"""Independent offline screen of clean cover candidates, using exported R08 hardware."""
import sys,json,gzip
from pathlib import Path
import numpy as np
import trimesh,fcl
sys.path.insert(0,str(Path(__file__).parent))
import faceted_r08 as g
from simple_upper_r09 import SRC,DEST
from rom_mesh import Shape
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.BRep import BRep_Tool
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_FACE,TopAbs_REVERSED
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS
def mesh(q,name):
 BRepMesh_IncrementalMesh(q,.02,False,.15,True).Perform();e=TopExp_Explorer(q,TopAbs_FACE);v=[];f=[]
 while e.More():
  face=TopoDS.Face(e.Current());loc=TopLoc_Location();t=BRep_Tool.Triangulation_s(face,loc);base=len(v)
  for i in range(1,t.NbNodes()+1):v.append(t.Node(i).Transformed(loc.Transformation()).Coord())
  for i in range(1,t.NbTriangles()+1):
   a,b,c=t.Triangle(i).Get();f.append([base+a-1,base+c-1,base+b-1] if face.Orientation()==TopAbs_REVERSED else [base+a-1,base+b-1,base+c-1])
  e.Next()
 m=trimesh.Trimesh(v,f,process=True)
 return dict(name=name,vertices_mm=np.asarray(m.vertices).ravel().tolist(),triangles=np.asarray(m.faces).ravel().tolist(),volume_cm3=g.vol(q)/1000,lumps=len(g.solids(q)))
def hits(a,b):
 if not a.hits(b):return None
 req=fcl.CollisionRequest(num_max_contacts=20,enable_contact=True);res=fcl.CollisionResult();fcl.collide(a.obj,b.obj,req,res)
 return dict(a=a.name,b=b.name,contacts_mm=[np.asarray(x.pos).tolist() for x in res.contacts[:10]])
def run():
 data=json.load(gzip.open(SRC/'validation_meshes.json.gz','rt'));new=[]
 for d in data:
  if d['name'].startswith(('S01 Rear','S02 Front')):
   tag=d['name'][:3];q=g.read('../R09_DELIVERY/'+tag+'_clean.step');d=mesh(q,d['name']);print(tag,'mesh',len(d['triangles'])//3,flush=True)
  new.append(d)
 with gzip.open(DEST/'candidate_meshes.json.gz','wt',encoding='utf-8') as h:json.dump(new,h)
 shapes=[Shape(d) for d in new];covers=[s for s in shapes if s.name.startswith(('S01 Rear','S02 Front'))];others=[s for s in shapes if s not in covers]
 report={'basis':'R09 offline upper STEP candidates; all other geometry from R08 v14 export','mesh_tolerance_mm':.02,'static_contacts':[]}
 for cv in covers:
  for sh in others:
   h=hits(cv,sh)
   if h:report['static_contacts'].append(h)
 cam=next(s for s in shapes if s.name.startswith('V02'));cv=next(s for s in covers if s.name.startswith('S02'));report['camera_entry_hits']=[];report['camera_slide_hits']=[]
 for dz in range(-80,1,2):
  m=np.eye(4);m[:3,3]=[-105,0,dz];cam.set(m)
  if cam.hits(cv):report['camera_entry_hits'].append(dz)
 for dx in range(-105,1):
  m=np.eye(4);m[0,3]=dx;cam.set(m)
  if cam.hits(cv):report['camera_slide_hits'].append(dx)
 cam.set(np.eye(4));report['ffc_contact']=hits(Shape(mesh(g.box(187,218,-18,18,94,100.2),'FFC reserve')),cv)
 (DEST/'upper_candidate_screen.json').write_text(json.dumps(report,indent=2));print(report,flush=True)
if __name__=='__main__':run()
