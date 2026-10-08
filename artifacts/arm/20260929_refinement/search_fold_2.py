import adsk.core as c, adsk.fusion as f, math,json,time
from pathlib import Path
OUT=Path('/Users/hayward_kim/Developer/SCONE/artifacts/arm/20260929_refinement')
T=f.TemporaryBRepManager.get()
def rot_yz(a,p,roll=0):
 a=math.radians(a);r=math.radians(roll);ca,sa,cr,sr=math.cos(a),math.sin(a),math.cos(r),math.sin(r)
 m=c.Matrix3D.create();m.setWithArray([ca,-sa*sr,-sa*cr,p[0]/10,0,cr,-sr,p[1]/10,sa,ca*sr,ca*cr,p[2]/10,0,0,0,1]);return m
def plus(p,L,a):return [p[0]+L*math.cos(math.radians(a)),p[1],p[2]+L*math.sin(math.radians(a))]
def bb(b):return [[v*10 for v in p.asArray()] for p in [b.boundingBox.minPoint,b.boundingBox.maxPoint]]
def overlap(a,b):return all(min(a[1][i],b[1][i])-max(a[0][i],b[0][i])>0.02 for i in range(3))
def run(_context):
 app=c.Application.get();d=f.Design.cast(app.activeProduct);r=d.rootComponent
 arm=next(x for x in d.allComponents if x.name.startswith('ARM 5DOF'))
 parts={int(o.component.name[3]):[(b.name,T.copy(b)) for b in o.component.bRepBodies if b.isSolid] for o in arm.occurrences}
 ext=[]
 for o in r.allOccurrences:
  if o.fullPathName.startswith('ARM 5DOF') or not o.isVisible:continue
  for b in o.bRepBodies:
   if b.isSolid and b.isVisible and b.boundingBox.maxPoint.z>14.4:
    q=T.copy(b);ext.append((o.component.name+'/'+b.name,q,bb(q)))
 out=[]
 candidates=[(200,80,190,0),(205,85,195,0),(210,90,200,0),(215,95,200,0),(215,95,205,0),(210,90,195,90),(215,95,200,90),(210,85,190,0),(205,80,185,0),(218,98,203,0),(220,100,205,0),(215,90,195,0)]
 for a,b,k,roll in candidates:
  p2=[228,0,230.25];p3=plus(p2,90,a);p4=plus(p3,90,b);p5=plus(p4,65.5,k)
  mats={0:c.Matrix3D.create(),1:rot_yz(0,[228,0,189]),2:rot_yz(a,p2),3:rot_yz(b,p3),4:rot_yz(k,p4),5:rot_yz(k,p5,roll)}
  p6=c.Point3D.create(2.025,0,1.6);p6.transformBy(mats[5]);mats[6]=rot_yz(k,[v*10 for v in p6.asArray()],roll)
  world=[]
  for idx,ps in parts.items():
   for name,q in ps:
    v=T.copy(q);T.transform(v,mats[idx]);world.append((idx,name,v,bb(v)))
  hits=[]
  for i,(idx,name,q,box) in enumerate(world):
   for j,(other,n2,q2,box2) in enumerate(world[:i]):
    if idx==other or not overlap(box,box2):continue
    z=T.copy(q);assert T.booleanOperation(z,q2,f.BooleanTypes.IntersectionBooleanType)
    v=z.volume*1000 if z.isSolid and z.faces.count else 0
    if v>0.05:hits.append([idx,other,name[:20],n2[:20],round(v,2)])
   if idx==0:continue
   for n2,q2,box2 in ext:
    if not overlap(box,box2):continue
    z=T.copy(q);assert T.booleanOperation(z,q2,f.BooleanTypes.IntersectionBooleanType)
    v=z.volume*1000 if z.isSolid and z.faces.count else 0
    if v>0.05:hits.append([idx,'external',name[:20],n2[:50],round(v,2)])
  bounds=[[min(w[3][0][i] for w in world) for i in range(3)],[max(w[3][1][i] for w in world) for i in range(3)]]
  out.append({'angles':[a,b,k,roll],'box':bounds,'hits':hits});(OUT/'fold_candidates_2.json').write_text(json.dumps(out,indent=2));print(out[-1],flush=True)
 print('done')
