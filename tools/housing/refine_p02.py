exec(open('/Users/hayward_kim/Developer/SCONE/tools/housing/build_p02.py',encoding='utf-8').read().split('\ndef run():')[0])
def join_native(comp,temps,label):
 target=comp.bRepBodies.item(0);base=comp.features.baseFeatures.add();base.name=label+' tools';base.startEdit()
 for i,temp in enumerate(temps):comp.bRepBodies.add(temp,base).name=label+' tool '+str(i)
 base.finishEdit();ts=[b for b in comp.bRepBodies if b.name.startswith(label+' tool ')]
 inp=comp.features.combineFeatures.createInput(target,collection(ts));inp.operation=f.FeatureOperations.JoinFeatureOperation;feat=comp.features.combineFeatures.add(inp);feat.name=label
 return comp.bRepBodies.item(0)
def replace_ref(comp,temp):
 base=comp.features.baseFeatures.item(0);base.startEdit()
 for b in list(base.bodies):b.deleteMe()
 comp.bRepBodies.add(temp,base);base.finishEdit()
def run():
 app,doc,d=guard();r=d.rootComponent
 rear=r.occurrences.itemByName('P01 Rear PLA chassis:1').component;front=r.occurrences.itemByName('P02 Front PLA chassis:1').component;cover=r.occurrences.itemByName('P03 PLA sensor cover:1').component;tray=r.occurrences.itemByName('P04 PLA electronics tray:1').component
 # Reorient the same 70 x 60 board: 60 across Y, 70 vertical, to free cover seam space.
 cut_native(front,front.bRepBodies.item(0),[box(141.9,151.1,32,39,3.6,72),box(141.9,151.1,-39,-32,3.6,72)],'P02 retire wide board cradle')
 rails=[]
 for y in [-30,30]:
  q=box(142,151,y-3,y+3,3,81);cut(q,box(145.7,147.9,y-2,y+2,6.7,81.5));rails.append(q)
 join_native(front,rails,'P02 rotated PCB cradle')
 replace_ref(r.occurrences.itemByName('V04 Custom PCB 70x60:1').component,box(146,147.6,-30,30,7,77))
 replace_ref(r.occurrences.itemByName('V05 PCB component allowance:1').component,box(147.6,163.6,-28,28,9,75))
 # U2D2 nest gets external ribs and open connector ends, with 0.4 mm fitting clearance.
 q=box(-11,15,-28,28,3,12);cut(q,box(-7.4,11.4,-24.4,24.4,6,25))
 for y in [-28,28]:cut(q,box(-4,8,y-5,y+5,8,25))
 join_native(rear,[q],'P01 U2D2 nest roots')
 cut_native(rear,rear.bRepBodies.item(0),[box(-7.4,11.4,-24.4,24.4,6,25)],'P01 U2D2 fitting clearance')
 # Pack remains a design envelope; insulation, BMS and plug must fit this envelope.
 pack=box(36.7,115.3,-47.3,47.3,4.2,32.8)
 cut_native(rear,rear.bRepBodies.item(0),[T.copy(pack)],'P01 battery clearance')
 cut_native(front,front.bRepBodies.item(0),[T.copy(pack),box(145.7,147.9,-30.3,30.3,6.8,77.3),box(147.4,164,-28.4,28.4,8.6,75.4)],'P02 payload fitting clearance')
 # Apply wheel clearance after adding every structural post, including its wider root.
 cut_native(front,front.bRepBodies.item(0),[box(122.3,138.3,-200,200,-30,24.5)],'P02 complete wheel sweep relief')
 # Keep original roof silhouette while giving the camera 0.2 mm fitting room.
 cut_native(cover,cover.bRepBodies.item(0),[box(165.8,169,-42.8,42.8,77.45,101.85),box(145.7,147.9,-30.3,30.3,6.8,77.3)],'P03 camera board clearance')
 # Offset cover skins in four directions to clear PLA seam and tray corners by 0.25 mm.
 skin=T.copy(cover.bRepBodies.item(0))
 for comp,label in [(rear,'P01'),(front,'P02'),(tray,'P04')]:
  ts=[move(T.copy(skin),x=x,y=y) for x,y in [(0,0),(.25,0),(-.25,0),(0,.25),(0,-.25)]]
  cut_native(comp,comp.bRepBodies.item(0),ts,label+' cover fitting margin')
 for comp in [rear,front,cover,tray]:
  for b in comp.bRepBodies:pla(b,(222,174,45) if comp==cover else (42,47,53))
 doc.save('P02 fitting clearances, corrected PCB orientation, complete distal sweep relief')
 print('P02 REFINED')
try:run()
except:(OUT/'p02_refine_error.txt').write_text(traceback.format_exc(),encoding='utf-8');print(traceback.format_exc())
