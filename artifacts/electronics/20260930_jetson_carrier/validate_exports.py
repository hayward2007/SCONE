from pathlib import Path
import json, struct, math, collections
OUT=Path(__file__).parent
parts=json.loads((OUT/'installed_parts.json').read_text())
expected={};counts=collections.Counter()
for p in parts:
 if p['kind']=='print':
  counts[p['component']]+=1;expected[f"{p['component']}_{counts[p['component']]}.stl"]=p
reports=[]
for file in sorted((OUT/'DELIVERY'/'STL').glob('*.stl')):
 data=file.read_bytes();count=struct.unpack_from('<I',data,80)[0];assert len(data)==84+50*count
 lo=[float('inf')]*3;hi=[float('-inf')]*3;edges=collections.Counter();degenerate=0;signed=0
 for i in range(count):
  nums=struct.unpack_from('<12fH',data,84+50*i);verts=[tuple(nums[3+j*3:6+j*3]) for j in range(3)]
  assert all(math.isfinite(v) for pt in verts for v in pt)
  for pt in verts:
   for a in range(3):lo[a]=min(lo[a],pt[a]);hi[a]=max(hi[a],pt[a])
  vs=[tuple(round(v,5) for v in pt) for pt in verts]
  for j in range(3):edges[tuple(sorted((vs[j],vs[(j+1)%3])))]+=1
  a,b,c=verts
  u=[b[k]-a[k] for k in range(3)];v=[c[k]-a[k] for k in range(3)]
  cross=(u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0])
  if sum(x*x for x in cross)<1e-16:degenerate+=1
  signed+=(a[0]*(b[1]*c[2]-b[2]*c[1])-a[1]*(b[0]*c[2]-b[2]*c[0])+a[2]*(b[0]*c[1]-b[1]*c[0]))/6
 exp=expected[file.name];delta=max(abs(v-w) for aa,bb in zip((lo,hi),exp['box']) for v,w in zip(aa,bb));nonmanifold=sum(n!=2 for n in edges.values())
 reports.append({'file':file.name,'triangles':count,'finite':True,'boundary_or_nonmanifold_edges':nonmanifold,'degenerate_triangles':degenerate,'bbox_mm':[lo,hi],'bbox_max_error_mm':delta,'volume_mm3':signed})
 assert not nonmanifold and not degenerate and delta<.05 and signed>0,file.name
assert len(reports)==7
(OUT/'export_validation.json').write_text(json.dumps({'passed':True,'parts':reports},indent=2))
print(json.dumps({'passed':True,'files':len(reports),'triangles':sum(p['triangles'] for p in reports),'max_bbox_error_mm':max(p['bbox_max_error_mm'] for p in reports)}))
