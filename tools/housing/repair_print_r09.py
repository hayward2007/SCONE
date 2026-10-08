"""Split collinear T junctions from native face tessellation without moving vertices."""
import struct,json,collections,math
from pathlib import Path
out=Path(__file__).resolve().parents[2]/'artifacts/housing/20260915_MARC_v10/R09_DELIVERY/PRINT_PACKAGE';manifest=json.load(open(out/'print_manifest.json'))
for part in manifest:
 p=out/part['filename'];buf=p.read_bytes();faces=[]
 for rec in struct.iter_unpack('<12fH',buf[84:]):faces.append([tuple(rec[i:i+3]) for i in (3,6,9)])
 def counts(faces):
  e=collections.Counter()
  for f in faces:
   for a,b in zip(f,f[1:]+f[:1]):e[tuple(sorted([a,b]))]+=1
  return e
 before=counts(faces);boundary={e for e,n in before.items() if n==1};verts={p for e in boundary for p in e};fixed=[];splits=0
 for face in faces:
  remaining=[face]
  for a,b in zip(face,face[1:]+face[:1]):
   if tuple(sorted([a,b])) not in boundary:continue
   v=[b[k]-a[k] for k in range(3)];L2=sum(x*x for x in v);inside=[]
   for q in verts:
    t=sum((q[k]-a[k])*v[k] for k in range(3))/L2
    if 1e-6<t<1-1e-6 and sum((q[k]-a[k]-t*v[k])**2 for k in range(3))<1e-10:inside.append((t,q))
   if not inside:continue
   new=[]
   for f in remaining:
    found=next((i for i in range(3) if f[i]==a and f[(i+1)%3]==b),None)
    if found is None:new.append(f);continue
    third=f[(found+2)%3];seq=[a]+[q for _,q in sorted(inside)]+[b];new += [[p0,p1,third] for p0,p1 in zip(seq,seq[1:])];splits+=len(inside)
   remaining=new
  fixed+=remaining
 e=counts(fixed);bad=sum(n!=2 for n in e.values());records=[]
 for a,b,c in fixed:
  u=[b[k]-a[k] for k in range(3)];v=[c[k]-a[k] for k in range(3)];n=[u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0]];L=math.sqrt(sum(x*x for x in n));assert L>0
  records.append(struct.pack('<12fH',*[x/L for x in n],*a,*b,*c,0))
 assert bad==0,(part['filename'],bad,[(edge,n) for edge,n in e.items() if n!=2][:12])
 p.write_bytes(buf[:80]+struct.pack('<I',len(records))+b''.join(records));part['nonmanifold_edge_count']=bad;part['triangles']=len(records);part['collinear_boundary_splits']=splits
 print(part['filename'],'closed manifold',len(records),'triangles',splits,'boundary splits')
(out/'print_manifest.json').write_text(json.dumps(manifest,indent=2))
