exec(open('/Users/hayward_kim/Developer/SCONE/tools/housing/build_r03.py',encoding='utf-8').read().split('\ndef run():')[0])
exec(open('/Users/hayward_kim/Developer/SCONE/tools/housing/probe_clearance.py',encoding='utf-8').read().split('\ndef run():')[0])
def current(comp):return [b for b in comp.bRepBodies if b.isLightBulbOn][-1]
def replace_final(comp,q,name):
 olds=list(comp.bRepBodies);base=comp.features.baseFeatures.add();base.name=name;base.startEdit();b=comp.bRepBodies.add(q,base);b.name=name;pla(b,(222,174,45) if 'cover' in comp.name else (42,47,53));base.finishEdit()
 for b in olds:b.isLightBulbOn=False
 return current(comp)
def run():
 app,doc,d=guard();r=d.rootComponent;names=['R01 Rear smooth PLA chassis:1','R02 Front smooth PLA chassis:1','R03 Smooth PLA sensor cover:1','P04 PLA electronics tray:1'];comps=[r.occurrences.itemByName(n).component for n in names];rear,front,cover,tray=[T.copy(current(cp)) for cp in comps]
 # Trim inherited square walls to the same smooth outside as the new lower skin.
 seam=mirrorpts([(-24,-37.5),(28,-37.5),(36,-77),(99,-77),(118,-58),(135,-58),(145,-37.5),(190,-37.5)])
 bottom=mirrorpts([(-24,-33),(25,-33),(40,-71),(97,-71),(118,-53),(135,-53),(145,-33),(190,-33)])
 mid=mirrorpts([(-24,-35),(27,-35),(38,-75),(99,-75),(118,-56),(135,-56),(145,-35),(190,-35)])
 master,mask=sculpt(r,'R00 Outer clipping master',[(-.5,bottom,7),(14.5,mid,7),(33.5,seam,5)],None)
 for q in [box(-70,-21,-100,100,-2,100),box(188,235,-100,100,-2,100),box(-80,250,-100,100,33.5,100)]:union(mask,q)
 intersect(rear,T.copy(mask));intersect(front,T.copy(mask))
 # Clear the stepped split joint after adding the lower skin.
 cut(front,T.copy(rear));cut(front,box(64,83.3,-69.3,-54.7,-2,8.3));cut(front,box(64,83.3,54.7,69.3,-2,8.3))
 # Real fixed motor surface relief; original case and all existing hole coordinates remain untouched.
 details=[]
 for on,target in [('LEG 1:5',rear),('LEG 1(미러):2',rear),('LEG 1:1',front),('LEG 1(미러):1',front)]:
  b=r.occurrences.itemByName(on).bRepBodies.item(0);tool=T.copy(b);iv=T.copy(target);intersect(iv,T.copy(tool));details.append({'case':on,'overlap_box':bb(iv),'volume_mm3':iv.volume*1000})
  cut(target,T.copy(tool));cut(target,move(T.copy(tool),z=.2))
 # Fixture clearance for the actual hip motor at yaw inward90, hip180.
 for o in r.allOccurrences:
  if not o.fullPathName.startswith('LEG 1:5+XM430:2+'):continue
  for b in o.bRepBodies:
   if not b.isSolid:continue
   q=move(T.copy(b),x=250);rotate(q,180,(0,1,0),(208.1087025548,-95.140925829,18.5));rotate(q,-90,(0,0,1),(208.1087025548,-65.140925829,39))
   iv=intersection_volume(front,q)
   if iv and iv>.01:
    inter=T.copy(front);intersect(inter,T.copy(q));lo,hi=bb(inter);details.append({'hip_part':o.fullPathName,'overlap_box':[lo,hi],'volume_mm3':iv})
    # Symmetric, smooth cylinder relief with 1 mm allowance around each overlap witness.
    x0,x1=lo[0]-1,hi[0]+1;z0,z1=lo[2]-1,hi[2]+1
    for sign in [-1,1]:
     y0,y1=sorted([sign*lo[1],sign*hi[1]]);cut(front,box(x0,x1,y0-1,y1+1,z0,z1))
 # Camera circuit board fit; trim tray's obsolete outer corner to the narrowed waist.
 cut(cover,box(165.8,169,-42.8,42.8,77.45,101.85));cut(tray,T.copy(cover))
 for sign in [-1,1]:cut(tray,box(113,132,55.0 if sign>0 else -70,70 if sign>0 else -55,35,52))
 # Move the front tray fixing inward onto the elevated bridge, clear of both pack and wheel sweep.
 for y in [-50,50]:
  union(front,cylinder((125,y,25),(125,y,36),4.8));cut(front,cylinder((125,y,27),(125,y,38),1.25))
  union(tray,cylinder((125,y,36.3),(125,y,39.5),4.5));cut(tray,cylinder((125,y,35),(125,y,41),1.65))
 # Show only current hardware and printed parts, keep old concepts stored but hidden.
 for cp,q in zip(comps,[rear,front,cover,tray]):replace_final(cp,q,cp.name[:3]+' R03 fitted print body')
 r.occurrences.itemByName(master.name+':1').isLightBulbOn=False
 for o in r.occurrences:
  if o.name.startswith('V') or o.name in names:o.isLightBulbOn=True
  tag=o.component.attributes.itemByName('SCONE_HOUSING','role')
  if tag and tag.value=='retired':o.isLightBulbOn=False
 for cp in d.allComponents:cp.isJointsFolderLightBulbOn=False;cp.isSketchFolderLightBulbOn=False;cp.isConstructionFolderLightBulbOn=False
 (OUT/'r03_refine.json').write_text(json.dumps(details,ensure_ascii=True,indent=2))
 doc.save('R03 smooth lower surfaces trimmed, exact case supports and hip180 clearance fitted')
 print('R03 FITTED',len(details))
try:run()
except:(OUT/'r03_refine_error.txt').write_text(traceback.format_exc(),encoding='utf-8');print(traceback.format_exc())
