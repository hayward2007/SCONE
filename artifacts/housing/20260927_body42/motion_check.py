exec(open('/Users/hayward_kim/Developer/SCONE/artifacts/housing/20260927_body42/build_mounts.py').read().split('def run(')[0])
def ivol(t,a,b):
 z=t.copy(a);op(t,z,t.copy(b),f.BooleanTypes.IntersectionBooleanType);return z.volume
def overlap(a,b):
 aa=a.boundingBox;bb=b.boundingBox
 return all(getattr(aa.minPoint,k)<=getattr(bb.maxPoint,k) and getattr(bb.minPoint,k)<=getattr(aa.maxPoint,k) for k in ['x','y','z'])
def run(_context: str):
 app=c.Application.get();d=f.Design.cast(app.activeProduct);r=d.rootComponent;t=f.TemporaryBRepManager.get();final=next(b for b in r.bRepBodies if b.name.startswith('Body42 MX28'))
 srcdoc=next(doc for doc in app.documents if doc.name=='MARC Housing PLA R09 Assembled v6');src=f.Design.cast(srcdoc.products.itemByProductType('DesignProductType')).rootComponent.bRepBodies.item(3)
 added=t.copy(final);op(t,added,t.copy(src),f.BooleanTypes.DifferenceBooleanType)
 legs=[o for o in r.allOccurrences if o.fullPathName.startswith('LEG') and '+' not in o.fullPathName and o.isVisible]
 results=[]
 for leg in legs:
  bb=leg.bRepBodies.item(0).boundingBox;xc=(bb.minPoint.x+bb.maxPoint.x)*5;ys=1 if bb.minPoint.y>0 else -1;yc=ys*65.140925828965
  parts=[t.copy(b) for o in r.allOccurrences if o.fullPathName.startswith(leg.fullPathName+'+') and o.isVisible for b in o.bRepBodies if b.isVisible and b.isSolid]
  hits=[];new_blocked=[];clear=0
  for deg in range(0,360,5):
   m=c.Matrix3D.create();m.setToRotation(math.radians(deg),c.Vector3D.create(0,0,1),pt(xc,yc,18.5))
   oldhit=0;newhit=0;addedhit=0
   for part in parts:
    p=t.copy(part);t.transform(p,m)
    if overlap(p,src):oldhit+=ivol(t,src,p)
    if overlap(p,final):newhit+=ivol(t,final,p)
    if overlap(p,added):addedhit+=ivol(t,added,p)
   if oldhit<1e-5:clear+=1
   if oldhit<1e-5 and newhit>1e-5:new_blocked.append(dict(deg=deg,volume_cm3=newhit))
   if addedhit>1e-5:hits.append(dict(deg=deg,added_intersection_cm3=addedhit,original_intersection_cm3=oldhit,final_intersection_cm3=newhit))
  results.append(dict(leg=leg.fullPathName,parts=len(parts),samples=72,original_clear_samples=clear,newly_blocked_samples=new_blocked,added_region_hits=hits))
 report={'method':'Per-leg MX28 z-axis sweep every 5 degrees over full turn; other joint angles held at exact original user pose; body-only collision comparison, no inter-leg validation','results':results}
 (OUT/'motion_validation.json').write_text(json.dumps(report,ensure_ascii=False,indent=2));print(json.dumps(report,ensure_ascii=False))
