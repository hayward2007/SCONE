from OCP.STEPControl import STEPControl_Reader
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_SOLID
from pathlib import Path
import json
p=Path('artifacts/housing/20260915_MARC_v10/references')
report={}
for name,path in [('orin',p/'orin_3d/P3766-P3768SKU4-P3767ENVELOPE.stp'),('camera',p/'camera_3d/IMX219-83-Stereo-Camera-3D-Drawing/0619.stp')]:
 r=STEPControl_Reader();r.ReadFile(str(path));r.TransferRoots();s=r.OneShape();b=Bnd_Box();BRepBndLib.AddOptimal_s(s,b);bb=(*b.CornerMin().Coord(),*b.CornerMax().Coord());print(name,bb,flush=True)
 solids=[];e=TopExp_Explorer(s,TopAbs_SOLID)
 while e.More():
  b=Bnd_Box();BRepBndLib.AddOptimal_s(e.Current(),b);solids.append((*b.CornerMin().Coord(),*b.CornerMax().Coord()));e.Next()
 report[name]={'bounds':bb,'solid_bounds':solids};print('solids',len(solids),flush=True)
(p/'vendor_step_inventory.json').write_text(json.dumps(report,indent=2))
