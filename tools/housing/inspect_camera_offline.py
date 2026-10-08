from OCP.STEPControl import STEPControl_Reader
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_SOLID,TopAbs_FACE
from OCP.BRepBndLib import BRepBndLib
from OCP.Bnd import Bnd_Box
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from pathlib import Path
p=next(Path('artifacts/housing/20260915_MARC_v10/references').rglob('0619.stp'))
r=STEPControl_Reader();r.ReadFile(str(p));r.TransferRoots();s=r.OneShape();ex=TopExp_Explorer(s,TopAbs_SOLID)
while ex.More():
 q=ex.Current();bb=Bnd_Box();BRepBndLib.Add_s(q,bb);prop=GProp_GProps();BRepGProp.VolumeProperties_s(q,prop)
 print(bb.CornerMin().Coord(),bb.CornerMax().Coord(),prop.Mass())
 ex.Next()
from OCP.TopoDS import TopoDS
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Plane,GeomAbs_Cylinder
ex=TopExp_Explorer(s,TopAbs_FACE);records=[]
while ex.More():
 q=TopoDS.Face(ex.Current());ad=BRepAdaptor_Surface(q);prop=GProp_GProps();BRepGProp.SurfaceProperties_s(q,prop)
 if ad.GetType()==GeomAbs_Plane:
  pl=ad.Plane();records.append((prop.Mass(),pl.Location().Coord(),pl.Axis().Direction().Coord()))
 ex.Next()
print(sorted(records,reverse=True)[:20])
