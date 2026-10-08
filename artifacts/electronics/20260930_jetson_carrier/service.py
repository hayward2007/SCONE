exec(open('/Users/hayward_kim/Developer/SCONE/artifacts/electronics/20260930_jetson_carrier/geometry.py').read())
def run(_context):
 a=c.Application.get();d=f.Design.cast(next(x for x in a.documents if x.name=='MARC v4 Body v5 BOX v21').products.itemByProductType('DesignProductType'))
 parts=generate(a);fixed=parts[0][1]
 old=[(o.fullPathName+'/'+b.name,T.copy(b)) for o in d.rootComponent.allOccurrences if not any(t in o.fullPathName for t in('E9 ','L Top lids','H Carry handle','ARM v3b')) for b in o.bRepBodies if b.isVisible and b.isSolid and b.boundingBox.intersects(box(100,244,0,56,30,300).boundingBox)]
 old.append(('P1 fixed brace',fixed))
 out={'lift_collisions':[],'driver_collisions':[],'cable_collisions':[],'envelope_collisions':[],'assembly':'Remove the front lid with its attached arm as one unit, and remove the carry handle. Install P1 using four MX28 top screws and two shared Jetson top screws. Assemble boards and shelf on P2 outside the robot; lower cassette and secure its two vertical screws.'}
 for dz in(0,2,5,10,20,40,60,80,100,120):
  for n,q,k in parts[1:]:
   b=T.copy(q);transform(b,0,0,dz,IDENT)
   for on,ob in old:
    v=interference(b,ob)
    if v>.001:out['lift_collisions'].append([dz,n,on,round(v,5)])
 for label,x,y,z,r in [('foot',x,y,44.21,2.1) for x,y in FOOT_HOLES]+[('Jetson',x,10.65,136.51,2.5) for x in(108.6,221.4)]+[('cassette',x,30,134.51,2.1) for x in(107.55,236)]:
  driver=cyl((x,y,z),(x,y,215),r)
  for on,ob in old:
   v=interference(driver,ob)
   if v>.001:out['driver_collisions'].append([label,x,y,on,round(v,5)])
 # PCB component allowance and connector plugs, excluded from exported geometry.
 envelopes=[('power PCB components 15mm',box(164.5,230.5,29.95,44.95,55,101)),('U2D2 TTL plug and bend',box(176,188,28.6,34.4,122.2,139.5)),('U2D2 UART plug and bend',box(176,188,34.5,41.2,122.2,139.5)),('U2D2 RS485 plug and bend',box(176,190.5,24.4,30.2,122.2,139.5)),('USB plug',box(219.5,236,27.5,38.5,115.5,122)),('PHB right angle power plug',box(121,137,33,48,112,127))]
 allparts=old+[(n,q) for n,q,k in parts if k!='official']
 for n,q in envelopes:
  for on,ob in allparts:
   v=interference(q,ob)
   if v>.001:out['envelope_collisions'].append([n,on,round(v,5)])
 out['part_lumps']=[(n,q.lumps.count) for n,q,k in parts if k=='print']
 (OUT/'service_check.json').write_text(json.dumps(out,ensure_ascii=False,indent=2),encoding='utf-8')
 print(json.dumps(out,ensure_ascii=False))
