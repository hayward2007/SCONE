exec(open('/Users/hayward_kim/Developer/SCONE/artifacts/housing/20260927_body42/build_mounts.py').read().split('def run(')[0])
def xyz(p):return list(v*10 for v in p.asArray())
def ivol(t,a,b):
 z=t.copy(a);op(t,z,t.copy(b),f.BooleanTypes.IntersectionBooleanType);return z.volume

def run(_context: str):
 app=c.Application.get();d=f.Design.cast(app.activeProduct);r=d.rootComponent;t=f.TemporaryBRepManager.get();b=next(b for b in r.bRepBodies if b.name.startswith('Body42 MX28'));srcdoc=next(doc for doc in app.documents if doc.name=='MARC Housing PLA R09 Assembled v6');src=f.Design.cast(srcdoc.products.itemByProductType('DesignProductType')).rootComponent.bRepBodies.item(3)
 report={'solid':b.isSolid,'lumps':b.lumps.count,'volume_cm3':b.volume,'source_volume_cm3':src.volume,'bounds_mm':[xyz(b.boundingBox.minPoint),xyz(b.boundingBox.maxPoint)]}
 holes=json.loads((OUT/'build_report.json').read_text())['holes_mm'];checks=[]
 for x,y in holes:
  ring=cyl(t,x,y,0,3.5,2.5);op(t,ring,cyl(t,x,y,-.01,3.51,1.405),f.BooleanTypes.DifferenceBooleanType)
  v=ivol(t,b,ring);void=ivol(t,b,cyl(t,x,y,-55,-.0001,2.499));through=ivol(t,b,cyl(t,x,y,-1.5,3.5,1.404))
  checks.append(dict(x=x,y=y,web_fill_ratio=v/ring.volume,head_and_driver_obstruction_cm3=void,through_obstruction_cm3=through))
 report['mount_checks']=checks;collisions=[];n=0
 for o in r.allOccurrences:
  if not o.fullPathName.startswith('LEG') or not o.isVisible:continue
  for part in o.bRepBodies:
   if part.isVisible and part.isSolid:
    n+=1;v=ivol(t,b,part)
    if v>1e-6:collisions.append(dict(path=o.fullPathName,body=part.name,volume_cm3=v))
 report['checked_leg_solids']=n;report['static_collisions']=collisions
 original=json.loads((OUT/'inventory.json').read_text());before={a['path']:a['transform'] for a in original['occ']};deltas=[]
 for o in r.allOccurrences:
  if o.fullPathName in before:
   delta=max(abs(a-z) for a,z in zip(before[o.fullPathName],o.transform2.asArray()))
   if delta>1e-9:deltas.append(dict(path=o.fullPathName,delta=delta))
 report['transform_changes']=deltas
 add=t.copy(b);op(t,add,t.copy(src),f.BooleanTypes.DifferenceBooleanType);rem=t.copy(src);op(t,rem,t.copy(b),f.BooleanTypes.DifferenceBooleanType)
 report['added_material_cm3']=add.volume;report['removed_material_cm3']=rem.volume
 # Exact integration of original cavity's rectangular sections, from measured inner planes.
 levels=[-41.5,34.8397459622,114.8397459622,153.5]
 def area(z):
  length=min(410.1226127212,229.597023+2*(z+41.5)/math.sqrt(3))
  width=min(162.6578947283,70.281851658+2*max(0,z-34.8397459622)/math.sqrt(3))
  return length*width
 cavity=sum((z1-z0)/6*(area(z0)+4*area((z0+z1)/2)+area(z1)) for z0,z1 in zip(levels,levels[1:]))/1000
 report['original_cavity_cm3']=cavity;report['retained_original_cavity_lower_bound_percent']=100*(1-add.volume/cavity)
 report['new_feature_health']=[dict(name=ft.name,health=int(ft.healthState),message=ft.errorOrWarningMessage) for ft in r.features if ft.name.startswith(('Body42','Spot inspired'))]
 (OUT/'validation.json').write_text(json.dumps(report,ensure_ascii=False,indent=2));print(json.dumps(report,ensure_ascii=False))
