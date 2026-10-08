from OCP.STEPControl import STEPControl_Reader,STEPControl_Writer,STEPControl_AsIs
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_SOLID
from OCP.TopoDS import TopoDS_Compound
from OCP.BRep import BRep_Builder
from pathlib import Path
import json
p=Path('artifacts/housing/20260915_MARC_v10/references');r=STEPControl_Reader();r.ReadFile(str(p/'orin_3d/P3766-P3768SKU4-P3767ENVELOPE.stp'));r.TransferRoots();s=r.OneShape()
builder=BRep_Builder();comp=TopoDS_Compound();builder.MakeCompound(comp);e=TopExp_Explorer(s,TopAbs_SOLID);n=0;keep=[]
while e.More():
 q=e.Current();b=Bnd_Box();BRepBndLib.AddOptimal_s(q,b);lo=b.CornerMin().Coord();hi=b.CornerMax().Coord();dims=[hi[i]-lo[i] for i in range(3)]
 if max(dims)>8 or any(abs(lo[i]-v)<.001 for i,v in enumerate([-5.5,-18.876875,-4.900021523])) or any(abs(hi[i]-v)<.001 for i,v in enumerate([97.5,72,29.866])):
  builder.Add(comp,q);keep.append([lo,hi])
 n+=1;e.Next()
w=STEPControl_Writer();w.Transfer(comp,STEPControl_AsIs);w.Write(str(p/'orin_mechanical.stp'));(p/'orin_simplification.json').write_text(json.dumps({'original_solids':n,'retained_solids':len(keep),'criterion':'Retain all solids with any dimension >8 mm, and all solids establishing overall extent. Vendor geometry not scaled. Small passives omitted only in mounting reference.'},indent=2));print(n,len(keep),flush=True)
