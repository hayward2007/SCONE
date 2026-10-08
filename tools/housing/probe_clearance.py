exec(open('/Users/hayward_kim/Developer/SCONE/tools/housing/check_geometry.py',encoding='utf-8').read().split('\ndef run():')[0])
def rotate(b,deg,axis,p):
 m=c.Matrix3D.create();m.setToRotation(math.radians(deg),vec(*axis),P(*p));T.transform(b,m)
def run():
 app,doc,d=guard();r=d.rootComponent;leg=[]
 for o in r.allOccurrences:
  if not o.fullPathName.startswith('LEG 1:5+'):continue
  for b in o.bRepBodies:
   if b.isSolid:
    q=T.copy(b);m=c.Matrix3D.create();m.translation=vec(25,0,0);T.transform(q,m);leg.append((o.fullPathName+'/'+b.name,q))
 report=[]
 for spin in [0,90]:
  moving={}
  for name,b in leg:
   q=T.copy(b)
   if 'ARC' in name:rotate(q,spin,(0,1,0),(85.6087025548,-138.140925829,18.5))
   if 'FR07' not in name:rotate(q,-90,(0,1,0),(208.1087025548,-95.140925829,18.5))
   rotate(q,-90,(0,0,1),(208.1087025548,-65.140925829,39));moving[name]=q
  arc=moving['LEG 1:5+ARC:1/본체1'];motor=moving['LEG 1:5+XM430:1+X-430_IDLE:1/본체1']
  inter=T.copy(arc);T.booleanOperation(inter,motor,f.BooleanTypes.IntersectionBooleanType)
  report.append({'spin':spin,'hub_overlap_mm3':inter.volume*1000 if inter.isSolid else 0,'hub_overlap_box':bb(inter) if inter.faces.count else None,'arc_box':bb(arc)})
 (OUT/'critical_joint_probe.json').write_text(json.dumps(report,ensure_ascii=False,indent=2));print(report)
try:run()
except:print(traceback.format_exc())
