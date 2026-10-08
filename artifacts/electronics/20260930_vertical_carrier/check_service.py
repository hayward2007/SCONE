exec(open('/Users/hayward_kim/Developer/SCONE/artifacts/electronics/20260930_vertical_carrier/carrier_geometry.py').read())
def run(_context):
 a=c.Application.get();doc=next(x for x in a.documents if x.name=='MARC v4 Body v5 BOX v19');d=f.Design.cast(doc.products.itemByProductType('DesignProductType'))
 ps=generate(a);frame=next(q for n,q,k in ps if n.startswith('P1'))
 area=box(-97,20,-50,55,76,205).boundingBox
 old=[]
 for o in d.rootComponent.allOccurrences:
  if not o.isVisible:continue
  if 'L Top lids' in o.fullPathName or 'M3 LiDAR mast' in o.fullPathName or 'V07 T-mini' in o.fullPathName or 'H Carry handle' in o.fullPathName:continue
  for b in o.bRepBodies:
   if b.isVisible and b.isSolid and b.boundingBox.intersects(area):old.append((o.fullPathName+'/'+b.name,T.copy(b)))
 fixed=old+[('NEW frame',frame)]+[(n,q) for n,q,k in ps if k=='pcb' or n.startswith('P4')]
 module=[(n,q) for n,q,k in ps if n.startswith('P2') or k=='official']
 collisions=[]
 for dz in [0,5,10,20,30,40,50,60]:
  for n,q in module:
   moved=T.copy(q);transform(moved,0,0,dz,((1,0,0),(0,1,0),(0,0,1)))
   for nn,qq in fixed:
    vol=interference(moved,qq)
    if vol>.002:collisions.append({'lift':dz,'moving':n,'fixed':nn,'mm3':round(vol,5)})
 # Vertical driver access to base screws before inserting the module.
 access=[]
 for x in (-70,-50):
  for y in (-40,40):
   tool=cyl((x,y,89.6),(x,y,200),2.3)
   for n,q in fixed:
    vol=interference(tool,q)
    if vol>.002:access.append({'screw':[x,y],'obstacle':n,'mm3':round(vol,5)})
 # PCB front-side component allowance, and low-profile right-angle USB plug clearance.
 envelopes=[('power PCB component allowance 20mm',box(-6.4,13.6,-29,29,95,130)),('USB-C right-angle clearance, 5mm downward projection',box(-76,-62,26,35.5,84.8,90))]
 env=[]
 for n,q in envelopes:
  for nn,qq in fixed:
   vol=interference(q,qq)
   if vol>.002:env.append({'envelope':n,'obstacle':nn,'mm3':round(vol,5)})
 out={'cassette_lift_collisions':collisions,'base_driver_collisions':access,'envelope_collisions':env,'lid_lidar_and_carry_handle_removed_for_service':True,'module_lift_samples_mm':[0,5,10,20,30,40,50,60],'frame_single_solid':frame.lumps.count==1,'part_lumps':[(n,q.lumps.count) for n,q,k in ps if k=='print']}
 (OUT/'service_check.json').write_text(json.dumps(out,ensure_ascii=False,indent=2),encoding='utf-8');print(json.dumps(out,ensure_ascii=False))
