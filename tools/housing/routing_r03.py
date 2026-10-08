exec(open('/Users/hayward_kim/Developer/SCONE/tools/housing/refine_r03.py',encoding='utf-8').read().split('\ndef run():')[0])
def run():
 app,doc,d=guard();r=d.rootComponent
 for n in ['R01 Rear smooth PLA chassis:1','R02 Front smooth PLA chassis:1']:
  cp=r.occurrences.itemByName(n).component;q=T.copy(current(cp));xb=bb(q)
  for x in [45,106]:
   if xb[0][0]<x<xb[1][0]:
    for y in [-50,50]:cut(q,box(x-5,x+5,y-1.5,y+1.5,-1,5))
  for x in [0,158]:
   if xb[0][0]<x<xb[1][0]:
    for y in [-19,19]:cut(q,box(x-3,x+3,y-1,y+1,-1,5))
  # Short pack runners keep cells above the floor; straps provide retention.
  a,b=(42,64.85) if n.startswith('R01') else (65.15,109)
  if b>a:
   for y in [-41,41]:union(q,box(a,b,y-2,y+2,1.8,4.3))
  replace_final(cp,q,n[:3]+' production PLA frame')
 routes=[
  ('USB-C external service',1.5,[(36,-42,50.3),(25,-42,51),(9,-17,57),(-27,-17,57)]),
  ('Ethernet external service',2.2,[(36,-27,54.4),(18,-27,56),(3,13,57),(-27,13,57)]),
  ('U2D2 USB',1.4,[(36,-9.6,56),(21,-9.6,56),(16,24,27),(2,29,13),(2,24,13)]),
  ('Jetson regulated DC',1.5,[(36,44,54),(27,44,57),(29,39,80),(137,39,81),(145,18,66)]),
  ('Battery fused power',2.0,[(112,35,31),(117,35,32),(140,25,32),(147,15,22)]),
  ('Camera twin CSI corridor',1.5,[(171,0,101.7),(158,0,98),(143,0,89),(140,24,53),(116,24,48)]),
  ('Lidar power UART',1.2,[(53,-19.3,107),(53,-23,105),(53,-23,98),(37,-28,88),(17,-22,28),(7,-23,17)]),
 ]
 for side in [-1,1]:
  routes.append(('Front servo harness '+str(side),1.8,[(147,side*20,22),(167,side*19,23),(208,side*24,19)]))
  routes.append(('Rear servo harness '+str(side),1.8,[(147,side*20,22),(137,side*38,32),(30,side*38,31),(-7,side*19,23),(-42,side*24,19)]))
 comp=compnew(r,'W01 Wiring route study',True);comp.attributes.add('SCONE_HOUSING','role','routing_reference')
 base=comp.features.baseFeatures.add();base.name='Illustrative cable corridors, connector and bend radii provisional';base.startEdit()
 for label,radius,pts in routes:
  temp=None
  for a,b in zip(pts,pts[1:]):
   seg=cylinder(a,b,radius)
   if temp is None:temp=seg
   else:union(temp,seg)
  for p in pts[1:-1]:union(temp,T.createSphere(P(*p),radius/10))
  b=comp.bRepBodies.add(temp,base);b.name=label
 base.finishEdit()
 for b in comp.bRepBodies:finish_appearance(app,d,b,(60,125,180),'Wire route reference','PrismMaterial-022')
 r.occurrences.itemByName('W01 Wiring route study:1').isLightBulbOn=False
 for o in r.allOccurrences:
  if o.fullPathName.startswith('V'):
   o.isLightBulbOn=True
   for b in o.bRepBodies:b.isLightBulbOn=True
 (OUT/'r03_routes.json').write_text(json.dumps({'status':'Illustrative centerline corridors; exact purchased plug envelopes, flexible-cable bend radii and strain relief require harness prototype','routes':[{'name':n,'assumed_radius_mm':ra,'points_mm':p} for n,ra,p in routes]},indent=2))
 doc.save('R03 battery strap slots, pack support runners and service wiring route study')
 print('R03 ROUTING READY')
try:run()
except:(OUT/'r03_routing_error.txt').write_text(traceback.format_exc(),encoding='utf-8');print(traceback.format_exc())
