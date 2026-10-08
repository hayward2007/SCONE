exec(open('/Users/hayward_kim/Developer/SCONE/tools/housing/refine_r03.py',encoding='utf-8').read().split('\ndef run():')[0])
DEST=OUT/'R04_DELIVERY'
def run():
 app,doc,d=guard();r=d.rootComponent;hits=[]
 parts=[(o.name,current(o.component)) for o in r.occurrences if o.name.startswith(('R01 Rear','R02 Front','R03 Smooth','P04 PLA'))]
 for o in r.allOccurrences:
  if not o.fullPathName.startswith('LEG'):continue
  for b in o.bRepBodies:
   if not b.isSolid or not b.isVisible:continue
   for pn,p in parts:
    v=intersection_volume(T.copy(p),T.copy(b))
    if v is None or v>.01:
     q=T.copy(p);intersect(q,T.copy(b));hits.append(dict(part=pn,leg=o.fullPathName,body=b.name,volume_mm3=v,bounds=bb(q)))
 loops=[]
 for path in ['LEG 1:5+LINK:1','LEG 1(미러):2+LINK(미러):1']:
  o=next(o for o in r.allOccurrences if o.fullPathName==path)
  for b in list(o.bRepBodies)[:2]:
   loops.append(dict(path=path,body=b.name,bounds=bb(b),loops=[dict(outer=l.isOuter,edges=[dict(type=e.geometry.objectType,bounds=bb(e)) for e in l.edges]) for face in b.faces if face.geometry.objectType=='adsk::core::Plane' and abs(face.geometry.normal.z)>.99 for l in face.loops]))
 (DEST/'initial_probe.json').write_text(json.dumps(dict(hits=hits,plate_loops=loops),ensure_ascii=True,indent=2));print('R04 INITIAL CLASHES',hits)
try:run()
except:(DEST/'probe_error.txt').write_text(traceback.format_exc(),encoding='utf-8');print(traceback.format_exc())
