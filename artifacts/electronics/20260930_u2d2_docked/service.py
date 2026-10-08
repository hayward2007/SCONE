exec(open('/Users/hayward_kim/Developer/SCONE/artifacts/electronics/20260930_u2d2_docked/geometry.py').read())
def run(_context):
 a=c.Application.get();doc=next(x for x in a.documents if x.name.startswith('MARC v4 Body v5 BOX v'));d=f.Design.cast(doc.products.itemByProductType('DesignProductType'));src=find_parts(d);parts=proposed(d)
 fixed=parts[0][1];plate=parts[1][1]
 assembly=parts[1:]+[(b.name,T.copy(b),k) for k in('P3','PHB','PCB') for b in src[k].bRepBodies]
 excluded={src[k].fullPathName for k in('P1','P2','P3','P4','PHB','PCB','U2D2')}
 old=[(o.fullPathName+'/'+b.name,T.copy(b)) for o in d.rootComponent.allOccurrences if o.fullPathName not in excluded and not any(t in o.fullPathName for t in('L Top lids','H Carry handle','ARM v3b')) for b in o.bRepBodies if b.isVisible and b.isSolid and b.boundingBox.intersects(box(99,244,0,56,30,270).boundingBox)]
 out={'lift_collisions':[],'fastener_collisions':[],'driver_collisions':[],'dock_face_gap_mm':0.0,'PHB_pose_preserved':True,'fasteners':'4x M2.5x12 with underside nuts in existing body rail D2.6 holes (115/125/225/235,52), body z75..81. Original motor foot removed. Existing Jetson M3x12 x2 retained.','service':'Remove front lid with arm attached and carry handle, disconnect external cables, then lift the cassette.'}
 for dz in(0,10,20,25,30,35,40,60,100,120):
  for n,q,k in assembly:
   moved=T.copy(q);transform(moved,0,0,dz,IDENT)
   targets=[('new P1 frame',fixed)]+(old if dz in(0,30,60,120) else [])
   for on,ob in targets:
    v=interference(moved,ob)
    if v>.001:out['lift_collisions'].append([dz,n,on,round(v,5)])
 for x,y in RAIL_HOLES:
  nut=box(x-2.9,x+2.9,y-2.5,y+2.5,72.8,75)
  shank=cyl((x,y,71.9),(x,y,83.9),1.25)
  for label,q in(('M2.5 nut envelope',nut),('M2.5 bolt shank',shank)):
   for on,ob in old+[('new P1 frame',fixed)]:
    v=interference(q,ob)
    if v>.001:out['fastener_collisions'].append([x,label,on,round(v,5)])
  driver=cyl((x,y,87.1),(x,y,190),2.1)
  for on,ob in old+[('new P1 frame',fixed)]:
   v=interference(driver,ob)
   if v>.001:out['driver_collisions'].append([x,on,round(v,5)])
 (OUT/'service_check.json').write_text(json.dumps(out,ensure_ascii=False,indent=2),encoding='utf-8');print(json.dumps(out,ensure_ascii=False))
