exec(open('/Users/hayward_kim/Developer/SCONE/tools/housing/build_r04.py',encoding='utf-8').read().split('\ndef run():')[0])
def run():
 app,doc,d=guard();r=d.rootComponent
 covercp=r.occurrences.itemByName('R03 Smooth PLA sensor cover:1').component;cv=T.copy(current(covercp))
 camera=next(o for o in r.allOccurrences if o.fullPathName=='V02 Waveshare official camera:1+0619:1').bRepBodies.item(0)
 # Relieve tiny PCB component protrusions around the four insert bosses, without changing module geometry.
 for dx,dy,dz in [(0,0,0),(-.2,0,0),(0,-.2,0),(0,.2,0),(0,0,-.2),(0,0,.2)]:cut(cv,move(T.copy(camera),x=dx,y=dy,z=dz))
 replace_final(covercp,cv,'R04 cover final camera service clearance')
 rearcp=r.occurrences.itemByName('R01 Rear smooth PLA chassis:1').component;q=T.copy(current(rearcp))
 # Restore structural continuity across the 0.0001mm clipping seam at each lap tongue.
 for y in [-62,62]:union(q,box(63,66,y-7,y+7,0,8))
 cut(q,T.copy(cv));cut(q,move(T.copy(cv),z=-.2));replace_final(rearcp,q,'R04 rear frame connected lap tongues')
 frontcp=r.occurrences.itemByName('R02 Front smooth PLA chassis:1').component;q=T.copy(current(frontcp));cut(q,T.copy(cv));cut(q,move(T.copy(cv),z=-.15));replace_final(frontcp,q,'R04 front cover seam fit')
 traycp=r.occurrences.itemByName('P04 PLA electronics tray:1').component;q=T.copy(current(traycp))
 for side in [-1,1]:cut(q,box(49.4,51.95,60.35 if side>0 else -64.35,64.35 if side>0 else -60.35,36,40))
 cut(q,T.copy(cv));cut(q,move(T.copy(cv),y=.15));cut(q,move(T.copy(cv),y=-.15));replace_final(traycp,q,'R04 single solid tray')
 report={cp.name:dict(lumps=current(cp).lumps.count,volume_cm3=current(cp).volume) for cp in [rearcp,frontcp,covercp,traycp]};(DEST/'refined.json').write_text(json.dumps(report,indent=2));doc.save('R04 assembly tolerances and continuous printable structure');print('R04 REFINED',report)
try:run()
except:(DEST/'refine_error.txt').write_text(traceback.format_exc(),encoding='utf-8');print(traceback.format_exc())
