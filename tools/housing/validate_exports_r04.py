import json,zipfile,zlib,struct
from pathlib import Path
from OCP.STEPControl import STEPControl_Reader
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_SOLID
from OCP.BRepCheck import BRepCheck_Analyzer
import zstandard
p=Path('artifacts/housing/20260915_MARC_v10/R04_DELIVERY');records=[]
for fn in sorted(p.glob('*.step')):
 r=STEPControl_Reader();r.ReadFile(str(fn));r.TransferRoots();q=r.OneShape();e=TopExp_Explorer(q,TopAbs_SOLID);count=0
 while e.More():count+=1;e.Next()
 valid=BRepCheck_Analyzer(q).IsValid();assert valid;records.append(dict(file=fn.name,solids=count,valid=valid))
(p/'step_validation.json').write_text(json.dumps(records,indent=2))
f=p/'MARC_Housing_PLA_R04.f3d';count=0
with zipfile.ZipFile(f) as z,open(f,'rb') as h:
 for i in z.infolist():
  h.seek(i.header_offset);header=h.read(30);nl,el=struct.unpack_from('<HH',header,26);h.read(nl+el);raw=h.read(i.compress_size)
  if i.compress_type==93:data=zstandard.ZstdDecompressor().decompress(raw,max_output_size=i.file_size)
  elif i.compress_type==8:data=zlib.decompress(raw,-15)
  elif i.compress_type==0:data=raw
  else:raise ValueError(i.compress_type)
  assert len(data)==i.file_size and zlib.crc32(data)==i.CRC;count+=1
(p/'archive_validation.json').write_text(json.dumps(dict(file=f.name,members_checked=count,crc_ok=True),indent=2));print(records,'F3D CRC',count)
