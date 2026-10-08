exec(open('/Users/hayward_kim/Developer/SCONE/tools/housing/probe_clearance.py',encoding='utf-8').read().split('\ndef run():')[0])
def run():
 app,doc,d=guard();r=d.rootComponent;report={'snapshot':snapshot(doc),'motor_surfaces':[],'envelopes':[]}
 for name in ['LEG 1:5','LEG 1:1']:
  o=r.occurrences.itemByName(name);b=o.bRepBodies.item(0)
  for i,fa in enumerate(b.faces):
   g=fa.geometry;cy=c.Cylinder.cast(g);pl=c.Plane.cast(g)
   if cy:report['motor_surfaces'].append({'occ':name,'face':i,'kind':'cylinder','origin':xyz(cy.origin),'axis':list(cy.axis.asArray()),'r_mm':cy.radius*10,'bounds':bb(fa)})
   elif pl and abs(pl.normal.z)>.99:report['motor_surfaces'].append({'occ':name,'face':i,'kind':'horizontal','origin':xyz(pl.origin),'bounds':bb(fa),'area_mm2':fa.area*100})
 for yaw in [-90,90]:
  for hip in [0,180,-90]:
   for o in r.allOccurrences:
    if not o.fullPathName.startswith('LEG 1:5+ARC:'):continue
    for b in o.bRepBodies:
     if not b.isSolid:continue
     q=move(T.copy(b),x=250) if 'move' in globals() else T.copy(b)
     if 'move' not in globals():m=c.Matrix3D.create();m.translation=vec(25,0,0);T.transform(q,m)
     rotate(q,hip,(0,1,0),(208.1087025548,-95.140925829,18.5));rotate(q,yaw,(0,0,1),(208.1087025548,-65.140925829,39))
     report['envelopes'].append({'yaw':yaw,'hip':hip,'body':b.name,'bbox':bb(q)})
 (OUT/'p03_inspect.json').write_text(json.dumps(report,ensure_ascii=True,indent=2))
 print('P03 INSPECTION READY',len(report['motor_surfaces']))
try:run()
except:print(traceback.format_exc())
