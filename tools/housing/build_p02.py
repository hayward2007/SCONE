exec(open('/Users/hayward_kim/Developer/SCONE/tools/housing/geometry.py',encoding='utf-8').read())

def pla(body,color=(42,47,53)):
 app=c.Application.get();d=f.Design.cast(app.activeProduct)
 finish_appearance(app,d,body,color,'PLA mass proxy - unvalidated strength','PrismMaterial-022')
 mat=body.material;mat.materialProperties.itemById('structural_Density').value=1240
 mat.materialProperties.itemById('physmat_Comments').value='PLA nominal density 1240 kg/m3 for mass only. Generic base mechanical properties are NOT qualified FDM PLA strength data. No structural or thermal certification.'
 # Independent color per part while sharing mass-only material.
 name='PLA ochre' if color[0]>100 else 'PLA graphite'
 a=d.appearances.itemByName(name)
 if not a:a=d.appearances.addByCopy(mat.appearance,name)
 for p in a.appearanceProperties:
  if p.id in ('opaque_albedo','surface_albedo'):
   try:p.value=c.Color.create(*color,255)
   except:pass
 body.appearance=a

def part(root,name,temp,color=(42,47,53),ref=False):
 comp=compnew(root,name,ref);comp.attributes.add('SCONE_HOUSING','owned','P02')
 base=comp.features.baseFeatures.add();base.name=name+' master';base.startEdit()
 b=comp.bRepBodies.add(temp,base);b.name=name
 if not ref:pla(b,color)
 base.finishEdit()
 if not ref:
  for b in comp.bRepBodies:pla(b,color)
 return comp

def move(temp,x=0,y=0,z=0):
 m=c.Matrix3D.create();m.translation=vec(x/10,y/10,z/10);T.transform(temp,m);return temp

def beam(a,b,width,z0,z1):
 ax,ay=a;bx,by=b;dx=bx-ax;dy=by-ay;L=math.hypot(dx,dy)
 return T.createBox(c.OrientedBoundingBox3D.create(P((ax+bx)/2,(ay+by)/2,(z0+z1)/2),vec(dx/L,dy/L,0),vec(-dy/L,dx/L,0),L/10,width/10,(z1-z0)/10))

def run():
 app,doc,d=guard();root=d.rootComponent
 assert doc.attributes.itemByName('SCONE_HOUSING','revision_P02')
 assert not root.occurrences.itemByName('P01 Rear PLA chassis:1'),'P02 parts already exist; inspect before revision'
 oldfloor=root.bRepBodies.itemByName('본체12');frame=T.copy(oldfloor)
 # Retain all original motor mounting interfaces and hole locations.
 footprint=intersect(T.copy(oldfloor),box(-100,260,-100,100,0,3.5))
 union(frame,move(T.copy(footprint),z=-.5))
 for side in [-1,1]:
  for a,b in [((0,side*29),(28,side*29)),((28,side*29),(42,side*62)),((42,side*62),(122,side*62)),((143,side*29),(184,side*29))]:union(frame,beam(a,b,4,3,18))
  union(frame,box(44,85,side*62-7,side*62+7,0,18))
  # Thicken battery-side walls inward without moving outer interface surfaces.
  union(frame,box(37,129,side*76-1.8,side*76+1.8,3,28))
 # Elevated bridge across the distal-wheel sweep; rounded outer cylinder is checked separately.
 union(frame,box(116,145,-76,76,25,29))
 union(frame,box(116,120,-75,75,3,29));union(frame,box(141,145,-37,37,3,29))
 cut(frame,box(122.3,138.3,-250,250,-30,24.5))
 # Mounting posts for the single removable electronics tray.
 for x in [39,117]:
  for y in [-59,59]:
   union(frame,cylinder((x,y,3),(x,y,36),4.8))
   union(frame,T.createCylinderOrCone(P(x,y,3),.8,P(x,y,10),.48))
   cut(frame,cylinder((x,y,25),(x,y,40),1.25))
 # Four existing cover screw positions retained, with broad printed roots.
 for x in [10,168]:
  for y in [-31,31]:
   union(frame,box(x-5,x+5,y-4,y+4,3,40))
   cut(frame,cylinder((x,-45 if y<0 else 25,38),(x,-25 if y<0 else 45,38),1.25))
 # Integrated vertical power PCB cradle, 70 x 60 x 1.6 mm board, components towards +X.
 for y in [-35,35]:
  union(frame,box(142,151,y-3,y+3,3,71))
  cut(frame,box(145.7,147.9,y-2,y+2,6.7,71.5))
 union(frame,box(142,151,-38,38,3,7))
 # USB converter nest and strain-relief tie slots at the rear.
 union(frame,box(-9,13,-27,27,3,6))
 for y in [-26,26]:union(frame,box(-9,13,y-1.5,y+1.5,5,12))
 for x in [-7,11]:union(frame,box(x-1.3,x+1.3,-26,26,5,10))
 # Battery strap slots and wiring ties; these are in low-stress floor zones.
 for x in [45,106]:
  for y in [-50,50]:cut(frame,box(x-5,x+5,y-1.5,y+1.5,-1,5))
 for x in [0,158]:
  for y in [-19,19]:cut(frame,box(x-3,x+3,y-1,y+1,-1,5))
 # 18 mm long, 14 mm wide reinforced stepped joints. Four example M4 bolts.
 rear=intersect(T.copy(frame),box(-100,64.85,-100,100,-10,100))
 front=intersect(T.copy(frame),box(65.15,260,-100,100,-10,100))
 for y in [-62,62]:
  union(rear,box(60,83,y-7,y+7,0,8))
  cut(front,box(64,83.3,y-7.3,y+7.3,-2,8.3))
  for x in [71,79]:
   cut(rear,cylinder((x,y,-2),(x,y,20),2.15));cut(front,cylinder((x,y,-2),(x,y,20),2.15))
 # Open-center electronics tray leaves access to the battery and underside ports.
 tray=cut(box(33,131,-64,64,36.3,39.5),box(44,120,-48,48,35,40))
 for x in [39,117]:
  for y in [-59,59]:cut(tray,cylinder((x,y,35),(x,y,41),1.65))
 for x in [41,122]:
  for y in [-47,47]:union(tray,cylinder((x,y,39),(x,y,42.8),3.5))
 # Stock NVIDIA base retained; perimeter nesting does not invent device mounting holes.
 for y in [-53.5,53.5]:
  union(tray,box(39,125,y-1.5,y+1.5,39,49.2))
  for x in [45,119]:cut(tray,cylinder((x,y,42),(x,y,51),1.25))
 for x in [34.5,128.7]:union(tray,box(x-1.3,x+1.3,-48,48,39,46))
 # New one-piece cover inherits the sculpted A01 surface and integrates camera supports.
 cover=T.copy(root.occurrences.itemByName('H01 Upper shell:1').bRepBodies.item(0))
 union(cover,box(27,79,-25,25,100,104))
 # Direct lidar mounting on the roof: two M2 clearance holes, no separate pylon.
 for x,y in [(38.9,-14.1),(67.1,14.1)]:cut(cover,cylinder((x,y,98),(x,y,106),1.15))
 cut(cover,box(48,58,-27,-19,99,106))
 # Roof ventilation above the actual stock fan. Ribs between slots support upside-down printing.
 for x in [79,85,91,97,103,109,115]:cut(cover,box(x-1.5,x+1.5,-24,26,98,110))
 # Camera board is 85.2 x 24; optical centers are +/-30.05 and 86 mm high in manufacturer CAD.
 for y in [-40.6,40.6]:
  union(cover,box(162,165.7,y-3,y+3,77,102.5))
  union(cover,box(159,168,y-4,y+4,100,102.6))
  for z in [79.7,93.15]:cut(cover,cylinder((160,y,z),(169,y,z),1.0))
 for y in [-30.05,30.05]:cut(cover,cylinder((167,y,86),(199,y,86),12))
 for y in [-40,40]:union(cover,cylinder((158,y,100),(158,y,104),1.7))
 # Rear external computer connections: two distinct service openings.
 cut(cover,box(-35,8,-24,-9,53,62));cut(cover,box(-35,8,3,23,47,68))
 # Retire A01 manufacturing and proxy components only after all new temporary bodies exist.
 built=[]
 for name,temp,col in [('P01 Rear PLA chassis',rear,(42,47,53)),('P02 Front PLA chassis',front,(42,47,53)),('P03 PLA sensor cover',cover,(222,174,45)),('P04 PLA electronics tray',tray,(42,47,53))]:
  comp=part(root,name,temp,col);built.append({'name':name,'bodies':comp.bRepBodies.count,'bbox':bb(comp.bRepBodies.item(0)),'volume_cm3':sum(b.volume for b in comp.bRepBodies)})
 # Sized references: one custom pack envelope and real U2D2 envelope, not claimed vendor PCB designs.
 refs=[('V03 Custom battery pack 78x94x28',box(37,115,-47,47,4.5,32.5)),('V04 Custom PCB 70x60',box(146,147.6,-35,35,7,67)),('V05 PCB component allowance',box(147.6,163.6,-33,33,9,65)),('V06 U2D2 48x18x14_9',box(-7,11,-24,24,6,20.9))]
 lidar=union(box(33.7,72.3,-19.3,19.3,104,125.15),cylinder((53,0,125.15),(53,0,137.9),18.1))
 for x,y in [(38.9,-14.1),(67.1,14.1)]:cut(lidar,cylinder((x,y,103),(x,y,140),1.25))
 refs.append(('V07 T-mini Plus 38_6x38_6x33_9',lidar))
 for name,temp in refs:
  comp=part(root,name,temp,ref=True)
  for b in comp.bRepBodies:finish_appearance(app,d,b,(35,39,44),'Reference device graphite','PrismMaterial-022')
 # Keep old source geometry available in history, excluded from current visible manufacture.
 for o in root.occurrences:
  tag=o.component.attributes.itemByName('SCONE_HOUSING','owned')
  if tag and tag.value=='A01':o.isLightBulbOn=False;o.component.attributes.add('SCONE_HOUSING','role','retired')
 oldfloor.isLightBulbOn=False
 root.occurrences.itemByName('21700 CELLS:1').isLightBulbOn=False
 for cp in d.allComponents:
  cp.isJointsFolderLightBulbOn=False;cp.isSketchFolderLightBulbOn=False;cp.isConstructionFolderLightBulbOn=False
 (OUT/'p02_build.json').write_text(json.dumps({'parts':built,'material':'PLA nominal density 1.24 g/cc, mass only','critical_relief':{'x_mm':[122.3,138.3],'lower_structure_removed_below_z_mm':24.5},'source_interfaces':'copied root tray motor mounting faces and holes retained; no ARC or motor-center change'},indent=2))
 doc.save('P02 four printed housing parts, raised wheel-clearance bridge, actual hardware and wiring provisions')
 print('P02 BUILT',[(x['name'],x['bodies']) for x in built])
try:run()
except:(OUT/'p02_build_error.txt').write_text(traceback.format_exc(),encoding='utf-8');print(traceback.format_exc())
