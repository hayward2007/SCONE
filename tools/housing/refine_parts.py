exec(open('/Users/hayward_kim/Developer/SCONE/tools/housing/geometry.py').read())
def replace_component(root,name):
 old=root.occurrences.itemByName(name+':1');assert old and old.component.attributes.itemByName('SCONE_HOUSING','owned');old.deleteMe()
 return compnew(root,name)
def run():
 app,doc,d=guard();r=d.rootComponent
 # Remove non-load-carrying plate areas while retaining closed beam sections and mount pads.
 comp=r.occurrences.itemByName('H02 Aluminium backbone:1').component
 cut_native(comp,comp.bRepBodies.item(0),[box(39,127,-68,68,-2,1),box(-20,23,-10,10,-2,1),box(141,182,-10,10,-2,1)],'H02 lightweight plate windows')
 tray=replace_component(r,'H03 Removable Jetson carrier')
 b=box(30,131,-74,74,29.8,31.8);cut(b,box(40,121,-62,62,29,33))
 for y in [-49,49]:union(b,box(30,115,y-4,y+4,29.8,31.8))
 for x in [40,106]:
  for y in [-47,47]:
   union(b,cylinder((x,y,29.8),(x,y,38),3.5));cut(b,cylinder((x,y,29),(x,y,39),1.6))
 for x in [38,128]:
  for y in [-71,71]:
   union(b,cylinder((x,y,3.5),(x,y,31.8),2.2));cut(b,cylinder((x,y,3),(x,y,33),1.15))
 persist(tray,[('Lift-out frame - provisional stand-off pattern',b)],(81,89,98),'PrismMaterial-231','Anodized aluminium')
 cp=replace_component(r,'H04 Raised PCB carrier')
 b=box(109,182,-46,46,47.5,49.5);cut(b,box(116,177,-38,38,47,50))
 for x in [117,177]:
  for y in [-41,41]:
   union(b,box(x-4,x+4,y-5,y+5,47.5,49.5));cut(b,box(x-1.6,x+1.6,y-3,y+3,47,50))
 parts=[('PCB slotted frame 70 x 90 mm reserved',b)]
 for x in [142,180]:
  for y in [-30,30]:parts.append(('M3 tubular pillar',cut(cylinder((x,y,3.5),(x,y,47.5),3),cylinder((x,y,3),(x,y,48),1.6))))
 persist(cp,parts,(81,89,98),'PrismMaterial-231','Anodized aluminium')
 cp=replace_component(r,'H05 Battery retention and lid anchors');parts=[]
 for x in [38,128]:
  q=box(x-1,x+1,-74,74,4,26.5)
  for y in [-71,71]:cut(q,cylinder((x,y,3),(x,y,28),2.6))
  parts.append(('Battery end restraint with pillar clearances',q))
 parts.append(('Removable central battery strap bridge',box(38,128,-3,3,26.5,28.5)))
 for x in [10,168]:
  for side in [-1,1]:
   y=side*32;post=box(x-4,x+4,y-3,y+3,3.5,40)
   cut(post,cylinder((x,side*26,38),(x,side*44,38),1.5));cut(post,cylinder((x,y,3),(x,y,17),1.25));parts.append(('Side-access M3 lid anchor',post))
 persist(cp,parts)
 cp=replace_component(r,'H06 Sensor mounts')
 b=cylinder((53,0,104),(53,0,112),24);cut(b,cylinder((53,0,103),(53,0,113),6))
 for x in [38,68]:
  for y in [-15,15]:cut(b,cylinder((x,y,103),(x,y,113),1.6))
 parts=[('Lidar adapter - pattern provisional',b)]
 for y in [-40,40]:
  p=box(157,163,y-3,y+3,74,100);union(p,box(153,165,y-5,y+5,100,101.8));cut(p,cylinder((156,y,95),(164,y,95),1.6));cut(p,cylinder((158,y,99),(158,y,105),1.6));parts.append(('Camera bracket mounted to removable roof',p))
 persist(cp,parts)
 # Move a reference envelope only; the real board carrier remains editable for actual mount data.
 orin=r.occurrences.itemByName('R01 Orin Nano RESERVED 79x100x50:1');m=orin.transform2;m.translation=vec(.15,0,0);orin.transform2=m
 orin.component.attributes.itemByName('SCONE_HOUSING','bounds_mm').value=json.dumps([30.5,109.5,-50,50,38,88])
 service=r.occurrences.itemByName('R04 Rear connector service space:1');service.isLightBulbOn=False
 # Trim the explicitly reserved cable volume away from the two side anchors.
 cut_native(service.component,service.component.bRepBodies.item(0),[box(-1,30,-31,-28,37,76),box(-1,30,28,31,37,76)],'R04 anchor clearance')
 # Mounting holes in the COPY's retained floor and matching aluminium underplate.
 holes=[(38,-71,1.25),(38,71,1.25),(128,-71,1.25),(128,71,1.25)]+[(x,y,1.6) for x in [142,180] for y in [-30,30]]+[(x,y,1.6) for x in [10,168] for y in [-32,32]]
 cut_native(r,r.bRepBodies.itemByName('본체12'),[cylinder((x,y,-3),(x,y,4),rad) for x,y,rad in holes],'H07 carrier and anchor mounting holes')
 comp=r.occurrences.itemByName('H02 Aluminium backbone:1').component
 cut_native(comp,comp.bRepBodies.item(0),[cylinder((x,y,-11),(x,y,1),rad) for x,y,rad in holes],'H02 corresponding carrier holes')
 shell=r.occurrences.itemByName('H01 Upper shell:1').component
 cut_native(shell,shell.bRepBodies.item(0),[cylinder((x,y,99),(x,y,113),1.6) for x in [38,68] for y in [-15,15]]+[cylinder((158,y,99),(158,y,106),1.6) for y in [-40,40]],'H01 sensor bracket mounting holes')
 for cp in d.allComponents:cp.isSketchFolderLightBulbOn=False;cp.isConstructionFolderLightBulbOn=False;cp.isJointsFolderLightBulbOn=False
 doc.save('Housing A01 packaging clearance refinement, mass reduction, matched mounting holes')
 print('REFINEMENT SAVED')
try:run()
except:(OUT/'refine_error.txt').write_text(traceback.format_exc());print(traceback.format_exc())
