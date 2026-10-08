exec(open('/Users/hayward_kim/Developer/SCONE/tools/housing/refine_r03.py',encoding='utf-8').read().split('\ndef run():')[0])
DEST=OUT/'R04_DELIVERY'
def rectround(x0,x1,y0,y1,z0,z1,rad):
 q=union(box(x0+rad,x1-rad,y0,y1,z0,z1),box(x0,x1,y0+rad,y1-rad,z0,z1))
 for x in [x0+rad,x1-rad]:
  for y in [y0+rad,y1-rad]:union(q,cylinder((x,y,z0),(x,y,z1),rad))
 return q

def run():
 app,doc,d=guard();r=d.rootComponent;assert doc.name.startswith('MARC Housing PLA R04')
 assert not doc.attributes.itemByName('SCONE_HOUSING','R04_built')
 # Keep the user's posed leg transforms; source-component geometry is edited only in this independent copy.
 linkrecords=[]
 for path in ['LEG 1:1+LINK:1','LEG 1(미러):1+LINK(미러):1']:
  o=next(o for o in r.allOccurrences if o.fullPathName==path);cp=o.component;originals=list(o.bRepBodies)[:2];temps=[]
  for i,b in enumerate(originals):
   q=T.copy(b);lo,hi=bb(q);ym=(lo[1]+hi[1])/2;cut(q,rectround(132.5,160.5,ym-7,ym+7,lo[2]-1,hi[2]+1,4))
   inv=o.transform2.copy();inv.invert();T.transform(q,inv);temps.append(('R04 plate '+str(i+1),q))
  olds=list(cp.bRepBodies);base=cp.features.baseFeatures.add();base.name='R04 separate 5mm plates - no center box';base.startEdit()
  for name,q in temps:nb=cp.bRepBodies.add(q,base);nb.name=name;pla(nb,(68,73,80))
  base.finishEdit()
  for b in olds:b.isLightBulbOn=False
  linkrecords.append(dict(component=cp.name,window_mm=[28,14],corner_radius_mm=4,thickness_mm=5,plates_per_leg=2,volume_cm3=[q.volume for _,q in temps],motor_centers_unchanged=True))
 (DEST/'link_design.json').write_text(json.dumps(linkrecords,indent=2))
 # A closed concave waist replaces the rear shoulder where the user's rotated wheel crossed the body.
 bottom=mirrorpts([(-24,-33),(14,-33),(25,-54),(49,-54),(58,-71),(97,-71),(118,-53),(135,-53),(145,-33),(190,-33)])
 mid=mirrorpts([(-24,-35),(14,-35),(25,-56),(49,-56),(58,-75),(99,-75),(118,-56),(135,-56),(145,-35),(190,-35)])
 seam=mirrorpts([(-24,-37.5),(14,-37.5),(25,-57.5),(49,-57.5),(58,-77),(99,-77),(118,-58),(135,-58),(145,-37.5),(190,-37.5)])
 shoulder=mirrorpts([(-21,-37),(14,-46),(25,-57.5),(49,-57.5),(58,-76),(99,-75),(118,-58),(135,-58),(156,-52),(188,-51)])
 roof=mirrorpts([(-10,-30),(14,-42),(25,-57),(49,-57),(58,-62),(99,-62),(118,-56),(135,-56),(155,-49),(180,-47)])
 lowcp,outer=sculpt(r,'R40 R04 lower waist master',[(-.5,bottom,5),(14.5,mid,5),(33.5,seam,5)],2.6,True)
 low=T.copy(current(lowcp));rearcp=r.occurrences.itemByName('R01 Rear smooth PLA chassis:1').component;rear=T.copy(current(rearcp))
 mask=T.copy(outer)
 for q in [box(-80,-21,-100,100,-2,100),box(188,240,-100,100,-2,100),box(-80,250,-100,100,33.5,100)]:union(mask,q)
 intersect(rear,mask);union(rear,intersect(low,box(-100,64.85,-110,110,-10,100)))
 # Remove now-obsolete rear tray pedestals; move them clear of the user's wheel sweep.
 for y in [-59,59]:cut(rear,box(33.5,44.5,y-5.5,y+5.5,2.6,40))
 for y in [-59,59]:union(rear,cylinder((57,y,2.5),(57,y,36),4.8));cut(rear,cylinder((57,y,28),(57,y,38),1.25))
 replace_final(rearcp,rear,'R04 rear chassis with closed wheel waist')
 # Rebuild cover skin so the narrowed waist stays closed and smooth; retain ports and lid interfaces.
 master,cvouter=sculpt(r,'R41 R04 cover master',[(33.8,seam,5),(53,shoulder,7),(104,roof,9)],2.6,False)
 cv=T.copy(current(master))
 for x,y in [(38.9,-14.1),(67.1,14.1)]:cut(cv,cylinder((x,y,98),(x,y,106),1.15))
 cut(cv,box(48,58,-27,-19,99,106))
 for x in [79,85,91,97,103,109,115]:cut(cv,box(x-1.5,x+1.5,-24,26,98,110))
 for side in [-1,1]:
  for x in [-8,1,10,153,162,171,180]:cut(cv,box(x-1.3,x+1.3,side*48-12,side*48+12,59,74))
 cut(cv,box(-35,8,-24,-9,53,62));cut(cv,box(-35,8,3,23,47,68))
 for x in [10,168]:
  for y in [-31,31]:
   union(cv,box(x-4,x+4,-44 if y<0 else 31,-31 if y<0 else 44,35,43));cut(cv,cylinder((x,-44 if y<0 else 25,38),(x,-25 if y<0 else 44,38),1.65))
 # Camera mounts on its INNER four real PCB holes, from behind, with cover removed.
 # Move whole official camera down 8mm to allow latch/FFC bend access above the module.
 camera=r.occurrences.itemByName('V02 Waveshare official camera:1');tr=camera.transform2;tr.translation=vec(tr.translation.x,tr.translation.y,tr.translation.z-.8);camera.transform2=tr
 for y in [-10.5,10.5]:
  for z in [71.7,85.15]:
   post=T.createCylinderOrCone(P(167.8,y,z),.34,P(195,y,z),.52);intersect(post,T.copy(cvouter));union(cv,post)
   cut(cv,cylinder((167.5,y,z),(172.0,y,z),1.5));cut(cv,cylinder((167.4,y,z),(177,y,z),1.0))
 for y in [-30.05,30.05]:cut(cv,cylinder((166,y,78),(201,y,78),12))
 # Minimum board / component assembly allowance at final location.
 cut(cv,box(165.8,167.79,-42.8,42.8,69.45,93.85))
 for side in [-1,1]:cut(cv,cylinder((113.1087026,side*187.64092583,18.5),(142.5087026,side*187.64092583,18.5),124.8))
 covercp=r.occurrences.itemByName('R03 Smooth PLA sensor cover:1').component;replace_final(covercp,cv,'R04 serviceable camera cover')
 traycp=r.occurrences.itemByName('P04 PLA electronics tray:1').component;tray=T.copy(current(traycp))
 # Smooth skin occupies the old square tray corners. Preserve actual Jetson fixing positions.
 cut(tray,T.copy(cv))
 for side in [-1,1]:cut(tray,box(20,49.8,54.2 if side>0 else -70,70 if side>0 else -54.2,35,52))
 for y in [-59,59]:
  union(tray,cylinder((57,y,36.3),(57,y,39.5),4.5));cut(tray,cylinder((57,y,35),(57,y,41),1.65))
 replace_final(traycp,tray,'R04 tray with relocated rear supports')
 for name in ['R40 R04 lower waist master:1','R41 R04 cover master:1','CHECK MX28 90 HIP180:1']:
  o=r.occurrences.itemByName(name)
  if o:o.isLightBulbOn=False
 for cp in d.allComponents:cp.isJointsFolderLightBulbOn=False;cp.isSketchFolderLightBulbOn=False;cp.isConstructionFolderLightBulbOn=False
 doc.attributes.add('SCONE_HOUSING','R04_built','1');doc.save('R04 smaller plate windows, serviceable camera and user-pose rear waist')
 (DEST/'build_done.json').write_text(json.dumps(dict(parts=[dict(name=cp.name,lumps=current(cp).lumps.count,volume_cm3=current(cp).volume) for cp in [rearcp,covercp,traycp]],camera_shift_z_mm=-8,camera_mount_holes_mm=[[167.8,y,z] for y in [-10.5,10.5] for z in [71.7,85.15]],rear_tray_mount_mm=[[57,y] for y in [-59,59]]),indent=2))
 print('R04 BUILT')
try:run()
except:(DEST/'build_error.txt').write_text(traceback.format_exc(),encoding='utf-8');print(traceback.format_exc())
