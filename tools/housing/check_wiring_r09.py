exec(open('/Users/hayward_kim/Developer/SCONE/tools/housing/build_r05.py',encoding='utf-8').read().split('\ndef run():')[0])
DEST=OUT/'R09_DELIVERY'
app,doc,d=guard();assert doc.name.startswith('MARC Housing PLA R09');r=d.rootComponent
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
(DEST/'wiring_clearance.json').write_text(json.dumps(dict(routes=records,limitations='Fixed corridor only; physical cable harness and moving service loops remain to validate.'),indent=2),encoding='utf-8')
print('R09 WIRE FAILURES',[(x['name'],x['hits']) for x in records if x['hits']])
