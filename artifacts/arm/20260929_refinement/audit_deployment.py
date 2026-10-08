exec(open('/Users/hayward_kim/Developer/SCONE/artifacts/arm/20260929_refinement/search_fold.py').read().split('def run(_context):')[0])
def iv(a,b):
 q=T.copy(a);assert T.booleanOperation(q,b,f.BooleanTypes.IntersectionBooleanType)
 return q.volume*1000 if q.isSolid and q.faces.count else 0

def run(_context):
 app=c.Application.get();d=f.Design.cast(app.activeProduct);r=d.rootComponent;assert app.activeDocument.name.startswith('MARC v4 Arm R1 Folded')
 arm=next(co for co in d.allComponents if co.name.startswith('ARM 5DOF'))
 parts={int(o.component.name[3]):[(b.name,T.copy(b)) for b in o.component.bRepBodies if b.isSolid and b.isLightBulbOn] for o in arm.occurrences}
 ext=[]
 for o in r.allOccurrences:
  if o.fullPathName.startswith('ARM 5DOF') or not o.isVisible:continue
  for b in o.bRepBodies:
   if b.isSolid and b.isVisible and b.boundingBox.maxPoint.z>14.4:
    q=T.copy(b);ext.append((o.component.name+'/'+b.name,q,bb(q)))
 out=[]
 poses=[('stowed',205,90,200,0,0)]
 poses += [('lift_'+str(a),a,90,200,0,0) for a in range(195,119,-15)]
 poses += [('elbow_'+str(b),120,b,b+110,0,0) for b in range(90,-1,-15)]
 poses += [('wrist_'+str(k),120,0,k,0,0) for k in [95,80,65,50,35,20,5,0]]
 poses += [('reach_'+str(a),a,0,0,0,0) for a in [105,90,75,60]]
 for label,a,b,k,roll,jaw in poses:
  p2=[228,0,230.25];p3=plus(p2,90,a);p4=plus(p3,90,b);p5=plus(p4,65.5,k)
  mats={0:c.Matrix3D.create(),1:rot_yz(0,[228,0,189]),2:rot_yz(a,p2),3:rot_yz(b,p3),4:rot_yz(k,p4),5:rot_yz(k,p5,roll)}
  p6=c.Point3D.create(2.025,0,1.6);p6.transformBy(mats[5]);mats[6]=rot_yz(k,[v*10 for v in p6.asArray()],roll)
  mj=rot_yz(jaw,[0,0,0]);base=mats[6].copy();mj.transformBy(base);mats[6]=mj
  world=[]
  for idx,ps in parts.items():
   for name,q in ps:
    v=T.copy(q);T.transform(v,mats[idx]);world.append((idx,name,v,bb(v)))
  hits=[];inherited=[]
  for i,(idx,name,q,box) in enumerate(world):
   for other,n2,q2,box2 in world[:i]:
    if not overlap(box,box2):continue
    volume=iv(q,q2)
    if volume>.05:
     entry=[idx,other,name,n2,round(volume,4)]
     if idx==other and name.startswith('J') and n2.startswith('J') and ('cable cover' in name or 'cable cover' in n2):inherited.append(entry)
     else:hits.append(entry)
   for n2,q2,box2 in ext:
    if not overlap(box,box2):continue
    volume=iv(q,q2)
    if volume>.05:hits.append([idx,'external',name,n2,round(volume,4)])
  bounds=[[min(w[3][0][i] for w in world) for i in range(3)],[max(w[3][1][i] for w in world) for i in range(3)]]
  out.append({'pose':label,'angles':[a,b,k,roll,jaw],'box_mm':bounds,'interferences':hits,'inherited_motor_cover_overlaps':inherited})
  (OUT/'deployment_path_audit.json').write_text(json.dumps(out,ensure_ascii=False,indent=2));print(label,hits,flush=True)
 print('AUDIT_COMPLETE')
