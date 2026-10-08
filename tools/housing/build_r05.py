exec(open('/Users/hayward_kim/Developer/SCONE/tools/housing/build_r04.py',encoding='utf-8').read().split('\ndef run():')[0])
DEST=OUT/'R05_DELIVERY'
def run():
 app,doc,d=guard();r=d.rootComponent;assert doc.name.startswith('MARC Housing PLA R05')
 # Common perimeter makes the four shell quadrants form one continuous exterior.
 bottom=mirrorpts([(-61.4,-38),(-28,-38),(25,-33),(40,-54),(49,-54),(58,-71),(97,-71),(118,-53),(135,-53),(145,-33),(190,-38),(227.6,-38)])
 mid=mirrorpts([(-61.4,-41),(-28,-41),(27,-35),(38,-56),(49,-56),(58,-75),(99,-75),(118,-56),(135,-56),(145,-35),(190,-41),(227.6,-41)])
 seam=mirrorpts([(-61.4,-43),(-28,-43),(28,-37.5),(36,-57.5),(49,-57.5),(58,-77),(99,-77),(118,-58),(135,-58),(145,-37.5),(190,-43),(227.6,-43)])
 shoulder_low=mirrorpts([(-61.4,-42),(-28,-43),(22,-43),(36,-57.5),(49,-57.5),(58,-77),(99,-77),(118,-58),(135,-58),(151,-46),(188,-49),(227.6,-42)])
 shoulder_high=mirrorpts([(-52,-40),(-28,-43),(20,-46),(35,-57.5),(49,-57.5),(58,-76),(99,-75),(118,-58),(135,-58),(154,-50),(188,-51),(218,-40)])
 shoulder=mirrorpts([(-21,-37),(18,-46),(35,-57.5),(49,-57.5),(58,-76),(99,-75),(118,-58),(135,-58),(156,-52),(188,-51)])
 roof=mirrorpts([(-10,-30),(14,-42),(25,-57),(49,-57),(58,-62),(99,-62),(118,-56),(135,-56),(155,-49),(180,-47)])
 lc,_=sculpt(r,'R50 shared lower fairing',[(-.5,bottom,5),(14.5,mid,5),(33.5,seam,5)],2.6,True);low=T.copy(current(lc))
 uc,cvouter=sculpt(r,'R51 shared upper fairing',[(33.8,seam,5),(46,shoulder_low,6),(50,shoulder_high,6),(56,shoulder,7),(104,roof,9)],2.6,False);cv=T.copy(current(uc))
 # Recreate sensor features on the shared skin, with the verified rear-assembled R04 camera position.
 for x,y in [(38.9,-14.1),(67.1,14.1)]:cut(cv,cylinder((x,y,98),(x,y,106),1.15))
 cut(cv,box(48,58,-27,-19,99,106))
 for x in [79,85,91,97,103,109,115]:cut(cv,box(x-1.5,x+1.5,-24,26,98,110))
 for side in [-1,1]:
  for x in [-8,1,10,153,162,171,180]:cut(cv,box(x-1.3,x+1.3,side*48-12,side*48+12,59,74))
 cut(cv,box(-75,8,-24,-9,53,62));cut(cv,box(-75,8,3,23,47,68))
 for y in [-10.5,10.5]:
  for z in [71.7,85.15]:
   post=T.createCylinderOrCone(P(167.8,y,z),.34,P(205,y,z),.55);intersect(post,T.copy(cvouter));union(cv,post)
   cut(cv,cylinder((167.5,y,z),(172,y,z),1.5));cut(cv,cylinder((167.4,y,z),(177,y,z),1.0))
 for y in [-30.05,30.05]:cut(cv,cylinder((166,y,78),(240,y,78),12))
 cut(cv,box(165.8,167.79,-42.8,42.8,69.45,93.85))
 camera=next(o for o in r.allOccurrences if o.fullPathName=='V02 Waveshare official camera:1+0619:1').bRepBodies.item(0)
 for dx,dy,dz in [(0,0,0),(-.2,0,0),(0,-.2,0),(0,.2,0),(0,0,-.2),(0,0,.2)]:cut(cv,move(T.copy(camera),x=dx,y=dy,z=dz))
 # Access eight lid screws from the underside. Upper lids conceal all 16 original MX28 top mounting heads.
 lower=[]
 for name,x0,x1 in [('R01 Rear smooth PLA chassis:1',-80,64.85),('R02 Front smooth PLA chassis:1',65.15,250)]:
  cp=r.occurrences.itemByName(name).component;q=T.copy(current(cp))
  patch=T.copy(low);intersect(patch,box(x0,x1,-100,100,-2,34))
  # Only add changed end skins; central waist was already fitted to the user wheel pose.
  intersect(patch,box(-80,30,-100,100,-2,34) if x0<0 else box(142,250,-100,100,-2,34));union(q,patch)
  for xc in [10,168]:
   if x0<xc<x1:
    for side in [-1,1]:cut(q,box(xc-5.3,xc+5.3,25.7 if side>0 else -45.3,45.3 if side>0 else -25.7,33.4,43.3))
  # Exact unchanged fixed cases and .2mm assembly relief.
  for parent in (['LEG 1:5','LEG 1(미러):2'] if x0<0 else ['LEG 1:1','LEG 1(미러):1']):
   b=r.occurrences.itemByName(parent).bRepBodies.item(0)
   for dz in [0,.2]:cut(q,move(T.copy(b),z=dz));cut(cv,move(T.copy(b),z=dz))
  lower.append((cp,q))
 # Four rear/front underside fixing stations; each is mirrored left/right.
 fasteners=[]
 for x,yabs in [(-14,25),(70,69),(97,69),(180,25)]:
  q=lower[0 if x<65 else 1][1]
  for side in [-1,1]:
   y=side*yabs;union(q,cylinder((x,y,0),(x,y,33.5),4.8));cut(q,cylinder((x,y,-1),(x,y,35),1.65));cut(q,cylinder((x,y,-1),(x,y,2.5),3.3))
   root=union(cylinder((x,y,33.8),(x,y,43),4.8),box(x-4.8,x+4.8,-80 if side<0 else y,y if side<0 else 80,35,42));intersect(root,T.copy(cvouter));union(cv,root)
   cut(cv,cylinder((x,y,33.5),(x,y,39.2),2.0));cut(cv,cylinder((x,y,33.4),(x,y,42.5),1.4));fasteners.append([x,y])
 # 24x13 motor connector access, 14x10 plug feedthrough, integrated cable-tie anchors.
 for cp,q in lower:
  xc=-41.8912974452 if cp.name.startswith('R01') else 208.1087025548
  for side in [-1,1]:
   cut(q,box(xc-12,xc+12,18 if side>0 else -29,29 if side>0 else -18,9,22))
   cut(q,rectround(xc-7,xc+7,side*20-5,side*20+5,-2,7,2))
   for dx in [-11,11]:
    anchor=box(xc+dx-3.5,xc+dx+3.5,side*15-4,side*15+4,1.5,7)
    cut(anchor,box(xc+dx-2,xc+dx+2,side*15-5,side*15+5,3.4,5.6));union(q,anchor)
  # Preserve the previously checked front wheel clearance, including wider tire candidate.
  for side in [-1,1]:cut(q,cylinder((113.1087026,side*187.640925829,18.5),(142.5087026,side*187.640925829,18.5),124.8))
 for side in [-1,1]:cut(cv,cylinder((113.1087026,side*187.640925829,18.5),(142.5087026,side*187.640925829,18.5),124.8))
 # Independently printable lids share the same loft surface and meet with a 0.3mm seam.
 back=T.copy(cv);intersect(back,box(-100,82.85,-120,120,30,160));front=T.copy(cv);intersect(front,box(83.15,260,-120,120,30,160))
 for name,q in [('S01 Rear fairing lid',back),('S02 Front camera fairing lid',front)]:part(r,name,q,(222,174,45))
 for cp,q in lower:replace_final(cp,q,'R05 '+cp.name+' TTL frame')
 # The shallow, open 8mm-wide tray underside channels accept flat 3x21AWG looms before assembly.
 tc=r.occurrences.itemByName('P04 PLA electronics tray:1').component;tq=T.copy(current(tc))
 for side in [-1,1]:
  cut(tq,box(32,132,side*35-4,side*35+4,35.5,37.3));cut(tq,box(49.4,51.95,60.35 if side>0 else -64.35,64.35 if side>0 else -60.35,36,40))
 replace_final(tc,tq,'R05 tray TTL loom channels')
 for o in r.occurrences:
  if o.name.startswith(('R03 Smooth','R4','R50','R51','CHECK')):o.isLightBulbOn=False
 for cp in d.allComponents:cp.isJointsFolderLightBulbOn=False;cp.isSketchFolderLightBulbOn=False;cp.isConstructionFolderLightBulbOn=False
 doc.save('R05 full motor bolt fairing, underside lid screws, TTL plug access and loom channels')
 (DEST/'build_done.json').write_text(json.dumps(dict(underside_lid_screw_xy_mm=fasteners,example_lid_screw='M3x35 with example M3 insert OD4.6 L5 / pilot4.0',lid_split_mm=[82.85,83.15],motor_connector_window_mm=[24,13],floor_feedthrough_mm=[14,10],tray_flat_loom_channels_mm=[8,1],ttl_confirmed=True,source_motor_centers_changed=False),indent=2));print('R05 BUILT')
try:run()
except:(DEST/'build_error.txt').write_text(traceback.format_exc(),encoding='utf-8');print(traceback.format_exc())
