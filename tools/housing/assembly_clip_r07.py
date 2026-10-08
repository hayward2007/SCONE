"""R07 simple planar B-rep construction, millimetres. No original model edits."""
from pathlib import Path
import json, math, numpy as np
from OCP.gp import gp_Pnt,gp_Vec,gp_Dir,gp_Ax2,gp_Trsf
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakePolygon,BRepBuilderAPI_MakeFace,BRepBuilderAPI_Sewing,BRepBuilderAPI_MakeSolid,BRepBuilderAPI_Transform
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox,BRepPrimAPI_MakeCylinder,BRepPrimAPI_MakeCone
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut,BRepAlgoAPI_Fuse,BRepAlgoAPI_Common
from OCP.BRepOffsetAPI import BRepOffsetAPI_MakeOffsetShape
from OCP.GeomAbs import GeomAbs_Intersection
from OCP.BRepOffset import BRepOffset_Skin
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.BRep import BRep_Tool
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_SOLID,TopAbs_SHELL,TopAbs_FACE
from OCP.TopoDS import TopoDS
from OCP.STEPControl import STEPControl_Reader,STEPControl_Writer,STEPControl_AsIs
from OCP.ShapeUpgrade import ShapeUpgrade_UnifySameDomain
from OCP.ShapeFix import ShapeFix_Solid
BASE=Path('/Users/hayward_kim/Developer/SCONE/artifacts/housing/20260915_MARC_v10/R07_DELIVERY')
def read(n):
 r=STEPControl_Reader();assert r.ReadFile(str(BASE/n))==1;r.TransferRoots();return r.OneShape()
def write(s,n):
 assert BRepCheck_Analyzer(s).IsValid(),n
 w=STEPControl_Writer();w.Transfer(s,STEPControl_AsIs);assert w.Write(str(BASE/n))==1

def clean(s):
 u=ShapeUpgrade_UnifySameDomain(s,True,True,False);u.Build();return u.Shape()
def vol(s):
 p=GProp_GProps();BRepGProp.VolumeProperties_s(s,p);return p.Mass()
def solids(s):
 e=TopExp_Explorer(s,TopAbs_SOLID);a=[]
 while e.More():a.append(TopoDS.Solid(e.Current()));e.Next()
 return a
def boolean(a,b,kind):
 op=kind(a,b);op.SetFuzzyValue(1e-6);op.Build();assert op.IsDone();return op.Shape()
def cut(a,b):return boolean(a,b,BRepAlgoAPI_Cut)
def fuse(a,b):return boolean(a,b,BRepAlgoAPI_Fuse)
def common(a,b):return boolean(a,b,BRepAlgoAPI_Common)
def box(x0,x1,y0,y1,z0,z1):return BRepPrimAPI_MakeBox(gp_Pnt(x0,y0,z0),x1-x0,y1-y0,z1-z0).Shape()
def cyl(a,b,r):
 v=np.array(b)-a;return BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(*a),gp_Dir(*v)),r,float(np.linalg.norm(v))).Shape()
def move(s,x=0,y=0,z=0):
 t=gp_Trsf();t.SetTranslation(gp_Vec(x,y,z));return BRepBuilderAPI_Transform(s,t,True).Shape()
def face(pts):
 p=BRepBuilderAPI_MakePolygon()
 for q in pts:p.Add(gp_Pnt(*q))
 p.Close();return BRepBuilderAPI_MakeFace(p.Wire(),True).Face()
def mirror(h):return h+[(x,-y) for x,y in reversed(h)]
def poly(levels):
 return rowpoly([[(x,y,z) for x,y in h] for z,h in levels])
def rowpoly(rows):
 sew=BRepBuilderAPI_Sewing(1e-6)
 sew.Add(face(list(reversed(rows[0]))));sew.Add(face(rows[-1]))
 for a,b in zip(rows,rows[1:]):
  for i in range(len(a)):
   j=(i+1)%len(a);pts=[a[i],a[j],b[j],b[i]]
   n=np.cross(np.subtract(pts[1],pts[0]),np.subtract(pts[2],pts[0]));err=abs(np.dot(n,np.subtract(pts[3],pts[0])))/max(np.linalg.norm(n),1e-9)
   if err<1e-7:sew.Add(face(pts))
   else:sew.Add(face(pts[:3]));sew.Add(face([pts[0],pts[2],pts[3]]))
 sew.Perform();sh=TopoDS.Shell(sew.SewedShape());s=BRepBuilderAPI_MakeSolid(sh).Solid();fx=ShapeFix_Solid(s);fx.Perform();s=fx.Solid();assert BRepCheck_Analyzer(s).IsValid();return clean(s)
BOTTOM=[(-61.4,-38),(-28,-38),(25,-33),(40,-54),(49,-54),(58,-71),(99,-71),(118,-53),(135,-53),(145,-33),(190,-38),(227.6,-38)]
SEAM=[(-61.4,-43),(-28,-43),(28,-37.5),(36,-57.5),(49,-57.5),(58,-77),(99,-77),(118,-58),(135,-58),(145,-37.5),(190,-43),(227.6,-43)]
CAP=[(-61.4,-43),(-28,-43),(28,-37.5),(36,-57.5),(49,-57.5),(58,-77),(99,-77),(118,-58),(135,-58),(145,-37.5),(190,-43),(227.6,-43)]
SHOULDER=[(-21,-37),(0,-41),(26,-51),(36,-57.5),(49,-57.5),(58,-76),(99,-75),(118,-58),(135,-58),(156,-52),(180,-51),(188,-51)]
ROOF=[(-10,-30),(6,-36),(25,-57),(36,-57),(49,-57),(58,-62),(99,-62),(118,-56),(135,-56),(155,-49),(173,-47),(180,-47)]
def masters():
 side=[(-61.4,-43),(28,-37.5),(36,-57.5),(49,-57.5),(58,-77),(99,-77),(118,-58),(135,-58),(145,-37.5),(227.6,-43)]
 rect56=[(x,-56) for x,y in side];rect48=[(x,-48) for x,y in side]
 levels=[(-.5,mirror(side)),(46,mirror(side)),(56,mirror(rect56)),(96,mirror(rect56)),(104,mirror(rect48))]
 outer=poly(levels);write(outer,'outer_master.step')
 # Offset each vertex against every incident face plane. Triangles remain planar.
 # This conservative construction avoids the general offset kernel's concave self intersections.
 from scipy.optimize import minimize,LinearConstraint
 rows=np.array([[(x,y,z) for x,y in h] for z,h in levels],dtype=float);flat=rows.reshape(-1,3);N=len(rows[0]);adj=[[] for _ in flat]
 facets=[list(reversed(range(N))),list(range(len(flat)-N,len(flat)))]
 for k in range(len(rows)-1):
  for i in range(N):
   j=(i+1)%N;facets += [[k*N+i,k*N+j,(k+1)*N+j],[k*N+i,(k+1)*N+j,(k+1)*N+i]]
 for ids in facets:
  pts=flat[ids];normal=np.cross(pts[1]-pts[0],pts[2]-pts[0])
  if np.linalg.norm(normal)<1e-8:normal=np.cross(pts[len(pts)//2-1]-pts[0],pts[len(pts)//2]-pts[0])
  normal/=np.linalg.norm(normal)
  for j in ids:adj[j].append(normal)
 new=[]
 for v,ns in zip(flat,adj):
  A=np.array(ns);res=minimize(lambda x:float(x@x),-A.sum(axis=0),jac=lambda x:2*x,constraints=[LinearConstraint(A,-np.inf,-3.1)],method='SLSQP',options={'ftol':1e-10,'maxiter':200})
  assert res.success and (A@res.x<=-3.1+1e-7).all();new.append(v+res.x)
 inner=rowpoly(np.array(new).reshape(rows.shape).tolist())
 assert len(solids(inner))==1 and BRepCheck_Analyzer(inner).IsValid(),'Invalid planar inset'
 write(inner,'assembly_clip.step');shell=clean(cut(outer,inner));write(shell,'assembly_shell_unused.step')
 (BASE/'assembly_clip_report.json').write_text(json.dumps(dict(outer_mm3=vol(outer),shell_mm3=vol(shell),wall_mm=2.8,style='Explicit planar facets; 3D constant wall inset; shared upper/lower surface'),indent=2))
if __name__=='__main__':masters()
