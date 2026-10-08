exec(open('/Users/hayward_kim/Developer/SCONE/tools/housing/build_r04.py',encoding='utf-8').read().split('\ndef run():')[0])
def run():
 app,doc,d=guard();r=d.rootComponent
 cv=current(r.occurrences.itemByName('R03 Smooth PLA sensor cover:1').component)
 cb=next(o for o in r.allOccurrences if o.fullPathName=='V02 Waveshare official camera:1+0619:1').bRepBodies.item(0)
 targets=[(o.fullPathName+'/'+b.name,b) for o in r.allOccurrences if (o.fullPathName.startswith(('R01 Rear','R02 Front','P04 PLA','V01','V03','V04','V05','V06')) or o.fullPathName.startswith('LEG')) for b in o.bRepBodies if b.isSolid and b.isLightBulbOn]
 hits=[]
 for dz in range(100,-1,-2):
  for name,b in [('cover',cv),('camera',cb)]:
   q=move(T.copy(b),z=dz)
   for n,t in targets:
    v=intersection_volume(q,T.copy(t))
    if v is None or v>.01:hits.append([dz,name,n,v])
 route=box(145,176,-18,18,94,100.2);v=intersection_volume(route,T.copy(cv))
 rep=dict(cover_camera_lowering_step_mm=2,cover_camera_lowering_hits=hits,ffc_reserve_box_mm=bb(route),ffc_reserve_cover_overlap_mm3=v,ffc_min_bend_radius_mm='5mm design assumption; verify chosen ribbon and connector',fasteners='4 x M2x6 example; M2 insert OD3.2 L4, pilot3.0x4.2 example; 0.2mm insulating spacing at PCB support pads')
 (DEST/'camera_service.json').write_text(json.dumps(rep,indent=2));print('CAMERA SERVICE',len(hits),'FFC SPACE',v)
try:run()
except:(DEST/'service_error.txt').write_text(traceback.format_exc(),encoding='utf-8');print(traceback.format_exc())
