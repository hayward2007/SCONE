exec(open('/Users/hayward_kim/Developer/SCONE/tools/housing/build_r05.py',encoding='utf-8').read().split('\ndef run():')[0])
DEST=OUT/'R09_DELIVERY'
def run():
 app,doc,d=guard();r=d.rootComponent;assert doc.name.startswith('MARC Housing PLA R09');assert (DEST/'native_integrated.json').exists()
 report=[]
 for name in ['S01 Rear fairing lid:1','S02 Front camera fairing lid:1']:
  cp=r.occurrences.itemByName(name).component;b=current(cp);edges=[]
  for e in b.edges:
   if not e.startVertex or not e.endVertex:continue
   a,z=e.startVertex.geometry,e.endVertex.geometry
   if abs(a.x-z.x)<1e-6 and abs(a.y-z.y)<1e-6 and abs(abs(a.y)*10-56)<.02 and abs(abs(a.z-z.z)*10-40)<.05 and min(abs(a.x*10+61.4),abs(a.x*10-227.6))<.02:edges.append(e)
  if edges:
   fi=cp.features.filletFeatures.createInput();fi.addConstantRadiusEdgeSet(collection(edges),c.ValueInput.createByString('1 mm'),False);ft=cp.features.filletFeatures.add(fi);ft.name='R09 exterior corner R1'
  report.append(dict(part=name,corner_edges=len(edges),radius_mm=1))
 # Validate existing fixed wiring corridors against the new geometry, without recutting by default.
 parts=[(o.name,current(o.component)) for o in r.occurrences if o.name.startswith(('R01 Rear','R02 Front','S01 Rear','S02 Front','P04 PLA','M01 Rear','M02 Front'))]
 hw=[(o.fullPathName+'/'+b.name,b) for o in r.allOccurrences if o.fullPathName.startswith('V') for b in o.bRepBodies if b.isSolid and b.isLightBulbOn]
 routes=json.loads((OUT/'R08_DELIVERY'/'wiring_clearance.json').read_text())['routes'];records=[]
 for route in routes:
  hits=[];pts=route['centerline_mm']
  for i,(a,b) in enumerate(zip(pts,pts[1:])):
   for tn,t in parts+hw:
    v=intersection_volume(cylinder(a,b,1.6),T.copy(t))
    if v is None or v>.01:hits.append([i,tn,v])
  records.append(dict(name=route['name'],centerline_mm=pts,reserve_diameter_mm=3.2,hits=hits))
 (DEST/'wiring_clearance.json').write_text(json.dumps(dict(routes=records,limitations='Fixed corridor only. Moving cable loops and purchased harness dimensions require physical verification.'),indent=2),encoding='utf-8')
 (DEST/'finish_details.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
 doc.save('R09 clean housing exterior with R1 corners and rechecked wiring corridors')
 print('R09 FINISH',report,'WIRE HITS',[(r['name'],r['hits']) for r in records if r['hits']])
try:run()
except:(DEST/'finish_error.txt').write_text(traceback.format_exc(),encoding='utf-8');print(traceback.format_exc())
