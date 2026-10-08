"""Create millimetre STL files in print orientation from native Fusion tessellations."""
import gzip,json,struct,math,collections
from pathlib import Path
out=Path(__file__).resolve().parents[2]/'artifacts/housing/20260915_MARC_v10/R08_DELIVERY/PRINT_PACKAGE'
items=json.load(gzip.open(out/'manufacturing_meshes.json.gz','rt'));report=[]
for item in items:
 tag=item['tag'];v=[item['vertices_mm'][i:i+3] for i in range(0,len(item['vertices_mm']),3)]
 if tag.startswith(('S','M')):v=[[x,-y,-z] for x,y,z in v];orientation='Flat exterior/top face on bed, part upside down'
 elif tag=='W11':v=[[x,-y,-z] for x,y,z in v];orientation='Original hub mounting face on bed, 18mm seat upward'
 elif tag.startswith('L'):orientation='5mm plate flat on bed, hole axes vertical'
 else:orientation='Lower planar face on bed'
 lo=[min(p[a] for p in v) for a in range(3)];v=[[round(p[a]-lo[a],5) for a in range(3)] for p in v]
 # Weld coincident face-border nodes within 0.0001 mm, far below tessellation tolerance.
 buckets=collections.defaultdict(list);weld=[];tol=.0001
 for p in v:
  cell=tuple(math.floor(x/tol) for x in p);found=None
  for dx in [-1,0,1]:
   for dy in [-1,0,1]:
    for dz in [-1,0,1]:
     for q in buckets[(cell[0]+dx,cell[1]+dy,cell[2]+dz)]:
      if sum((a-b)**2 for a,b in zip(p,q))<=tol*tol:found=q;break
  if found is None:found=p;buckets[cell].append(p)
  weld.append(found)
 v=weld
 tris=[item['triangles'][i:i+3] for i in range(0,len(item['triangles']),3)];records=[];edges=collections.Counter();volume=0
 for tri in tris:
  a,b,c=[v[i] for i in tri];u=[b[i]-a[i] for i in range(3)];w=[c[i]-a[i] for i in range(3)];n=[u[1]*w[2]-u[2]*w[1],u[2]*w[0]-u[0]*w[2],u[0]*w[1]-u[1]*w[0]];L=math.sqrt(sum(x*x for x in n))
  if L<1e-12:continue
  records.append(struct.pack('<12fH',*[x/L for x in n],*a,*b,*c,0))
  pts=[tuple(round(x,5) for x in p) for p in [a,b,c]]
  for p,q in zip(pts,pts[1:]+pts[:1]):edges[tuple(sorted([p,q]))]+=1
  volume+=(a[0]*(b[1]*c[2]-b[2]*c[1])+a[1]*(b[2]*c[0]-b[0]*c[2])+a[2]*(b[0]*c[1]-b[1]*c[0]))/6
 filename=tag+'_'+item['material']+'_mm.stl';(out/filename).write_bytes((tag+' | millimetres | print orientation').encode().ljust(80,b' ')+struct.pack('<I',len(records))+b''.join(records))
 dims=[max(p[a] for p in v) for a in range(3)];bad=sum(n!=2 for n in edges.values());err=abs(abs(volume)/1000/item['volume_cm3']-1)*100
 report.append({'part':item['name'],'filename':filename,'quantity':item['quantity'],'material':item['material'],'orientation':orientation,'dimensions_mm':dims,'fits_256mm':all(x<256 for x in dims),'triangles':len(records),'nonmanifold_edge_count':bad,'mesh_volume_error_pct':err,'native_lumps':item['lumps']})
(out/'print_manifest.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
