exec(open('/Users/hayward_kim/Developer/SCONE/tools/housing/build_r05.py',encoding='utf-8').read().split('\ndef run():')[0])
DEST=OUT/'R08_DELIVERY'
def run():
 app,doc,d=guard();r=d.rootComponent;assert doc.name.startswith('MARC Housing PLA R08')
 parts=[(o.name,T.copy(current(o.component))) for o in r.occurrences if o.name.startswith(('R01 Rear','R02 Front','S01 Rear','S02 Front','P04 PLA','M01 Rear','M02 Front'))]
 hw=[(o.fullPathName+'/'+b.name,T.copy(b)) for o in r.allOccurrences if o.fullPathName.startswith('V') for b in o.bRepBodies if b.isSolid and b.isLightBulbOn]
 routes=[]
 for s in [-1,1]:
  routes.append(('TTL trunk '+str(s),[(34,s*35,34.3),(137,s*35,34.3)]))
  routes.append(('rear TTL branch '+str(s),[(34,s*35,34.3),(34,s*30,34.3),(25,s*30,30),(-15,s*18,23),(-41.891297,s*24,16)]))
  routes.append(('front TTL branch '+str(s),[(137,s*35,34.3),(141,s*33,34.3),(169,s*33,25),(186,s*33,25),(194,s*18,23),(208.108703,s*24,16)]))
  for xc in [-41.891297,208.108703]:routes.append(('MX feedthrough '+str([xc,s]),[(xc,s*24,16),(xc,s*22,8),(xc,s*22,-8),(xc,s*40,-8)]))
 routes.append(('U2D2 signal to distribution',[(13,0,18),(20,0,25),(28,25,34.3),(135,25,34.3),(139,0,25)]))
 # Open small cable passages through interior webs and the existing feedthrough sites.
 changes=[]
 for n,q in parts:
  if not n.startswith(('R01','R02','S01')):continue
  before=q.volume
  for rn,pts in routes:
   for a,b in zip(pts,pts[1:]):cut(q,cylinder(a,b,2.0))
  assert q.lumps.count==1,(n,q.lumps.count)
  assert before-q.volume<2.0,(n,before-q.volume)
  cp=r.occurrences.itemByName(n).component;replace_final(cp,q,'R08 '+n[:3]+' open internal TTL cable passages');pla(current(cp),(222,174,45));changes.append(dict(part=n,removed_cm3=before-q.volume))
 (DEST/'wire_channel_changes.json').write_text(json.dumps(changes,indent=2))
 pieces=[];records=[]
 for n,pts in routes:
  hits=[]
  for i,(a,b) in enumerate(zip(pts,pts[1:])):
   q=cylinder(a,b,1.6);pieces.append((n+' segment '+str(i),q))
   for tn,t in parts+hw:
    try:v=intersection_volume(T.copy(q),T.copy(t))
    except Exception as e:_=app.activeDocument.name;hits.append([i,tn,None,str(e)]);continue
    if v is None or v>.01:hits.append([i,tn,v])
  records.append(dict(name=n,centerline_mm=pts,reserve_diameter_mm=3.2,hits=hits))
 (DEST/'wiring_clearance.json').write_text(json.dumps(dict(routes=records,note='Fixed cable corridors only; plugs stop before connector interfaces. Moving service loops, FFC bend fatigue and final purchased cable/connector dimensions require physical validation.'),indent=2))
 if any(v['hits'] for v in records):print('WIRE HITS',[(v['name'],v['hits']) for v in records if v['hits']]);return
 cp=compnew(r,'W08 TTL fixed routing reserve - not printed',True);persist(cp,pieces,(177,88,203),'PrismMaterial-022','TTL routing reserve')
 r.occurrences.itemByName(cp.name+':1').isLightBulbOn=False
 doc.save('R08 fixed TTL cable routes and floor feedthrough reserves');print('R08 WIRES DONE')
try:run()
except:(DEST/'wiring_error.txt').write_text(traceback.format_exc(),encoding='utf-8');print(traceback.format_exc())
