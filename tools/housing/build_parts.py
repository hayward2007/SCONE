exec(open('/Users/hayward_kim/Developer/SCONE/tools/housing/build_shell.py').read().split('def run():')[0])
T=f.TemporaryBRepManager.get()
def vec(x,y,z):return c.Vector3D.create(x,y,z)
def box(x0,x1,y0,y1,z0,z1):return T.createBox(c.OrientedBoundingBox3D.create(P((x0+x1)/2,(y0+y1)/2,(z0+z1)/2),vec(1,0,0),vec(0,1,0),(x1-x0)/10,(y1-y0)/10,(z1-z0)/10))
def cylinder(a,b,r):return T.createCylinderOrCone(P(*a),r/10,P(*b),r/10)
def boolean(a,b,op):
 assert T.booleanOperation(a,b,op),'Boolean failed'
 return a
def union(a,b):return boolean(a,b,f.BooleanTypes.UnionBooleanType)
def cut(a,b):return boolean(a,b,f.BooleanTypes.DifferenceBooleanType)
def intersect(a,b):return boolean(a,b,f.BooleanTypes.IntersectionBooleanType)
def compnew(root,name,reference=False):
 assert not root.occurrences.itemByName(name+':1'),'Already built '+name
 o=root.occurrences.addNewComponent(c.Matrix3D.create());o.component.name=name
 o.component.attributes.add('SCONE_HOUSING','owned','A01')
 o.component.attributes.add('SCONE_HOUSING','role','reference' if reference else 'manufactured')
 return o.component

def persist(comp,temps,color=(48,53,61),material='PrismMaterial-417',appearance='Graphite PA12'):
 app=c.Application.get();d=f.Design.cast(app.activeProduct)
 base=comp.features.baseFeatures.add();base.name=comp.name+' geometry';base.startEdit()
 for name,temp in temps:
  b=comp.bRepBodies.add(temp,base);b.name=name
 base.finishEdit()
 for b in comp.bRepBodies:finish_appearance(app,d,b,color,appearance,material)
 return list(comp.bRepBodies)
def cut_native(comp,target,temps,name):
 base=comp.features.baseFeatures.add();base.name=name+' tools';base.startEdit()
 for i,t in enumerate(temps):comp.bRepBodies.add(t,base).name=name+' tool '+str(i)
 base.finishEdit()
 tools=[b for b in comp.bRepBodies if b.name.startswith(name+' tool ')]
 inp=comp.features.combineFeatures.createInput(target,collection(tools));inp.operation=f.FeatureOperations.CutFeatureOperation;inp.isKeepToolBodies=False
 feat=comp.features.combineFeatures.add(inp);feat.name=name
 return feat

def run():
 app,doc,d=guard();r=d.rootComponent;report={}
 d.userParameters.itemByName('housing_roof_z').expression='104 mm'
 # Retained floor footprint, inherited mounting holes, plus two closed-section beams.
 chassis=compnew(r,'H02 Aluminium backbone')
 b=T.copy(r.bRepBodies.itemByName('본체12'));intersect(b,box(-100,250,-100,100,0,2))
 m=c.Matrix3D.create();m.translation=vec(0,0,-.2);T.transform(b,m)
 for y in [-20,20]:
  beam=box(-33,199,y-6,y+6,-10,-2);cut(beam,box(-34,200,y-4.5,y+4.5,-8.5,-3.5));union(b,beam)
 # Cross ties fit inside the original hip mounting zone.
 for x in [-39,205]:union(b,box(x-6,x+6,-35,35,-5,-1))
 persist(chassis,[('2 mm plate with 12 x 8 x 1.5 mm closed beams',b)],(78,86,94),'PrismMaterial-231','Anodized aluminium')
 # Removable lower electronics carrier; it clears the existing eight-cell geometry.
 tray=compnew(r,'H03 Removable Jetson carrier')
 b=box(30,131,-74,74,29.8,31.8);cut(b,box(40,121,-62,62,29,33))
 for y in [-49,49]:union(b,box(30,115,y-4,y+4,29.8,31.8))
 for x in [32,106]:
  for y in [-47,47]:
   union(b,cylinder((x,y,29.8),(x,y,38),3.5));cut(b,cylinder((x,y,29),(x,y,39),1.6))
 for x in [38,128]:
  for y in [-71,71]:
   # Posts are outside the actual cell bounding boxes; no body or joint is moved.
   union(b,cylinder((x,y,3.5),(x,y,31.8),2.2));cut(b,cylinder((x,y,3),(x,y,33),1.15))
 persist(tray,[('Lift-out frame and provisional adjustable board supports',b)],(81,89,98),'PrismMaterial-231','Anodized aluminium')
 pcbtray=compnew(r,'H04 Raised PCB carrier')
 b=box(109,184,-47,47,47.5,49.5);cut(b,box(116,177,-38,38,47,50))
 for x in [117,177]:
  for y in [-41,41]:
   union(b,box(x-4,x+4,y-6,y+6,47.5,49.5));cut(b,box(x-1.6,x+1.6,y-3,y+3,47,50))
 parts=[('PCB slotted frame 70 x 90 mm reserved',b)]
 for x in [142,180]:
  for y in [-30,30]:parts.append(('M3 tubular pillar',cut(cylinder((x,y,3.5),(x,y,47.5),3),cylinder((x,y,3),(x,y,48),1.6))))
 persist(pcbtray,parts,(81,89,98),'PrismMaterial-231','Anodized aluminium')
 # Battery retaining bridge occupies the gap between the two rows and stays above the cells.
 cradle=compnew(r,'H05 Battery retention and lid anchors')
 parts=[]
 for x in [38,128]:parts.append(('Battery end restraint',box(x-1,x+1,-74,74,4,25.5)))
 parts.append(('Removable central battery strap bridge',box(38,128,-3,3,26.5,28.5)))
 for x in [10,168]:
  for side in [-1,1]:
   y=side*33
   post=box(x-4,x+4,y-4,y+4,3.5,40)
   cut(post,cylinder((x,side*26,38),(x,side*44,38),1.5))
   parts.append(('Side-access M3 lid anchor - insert drill to supplier',post))
 persist(cradle,parts)
 # A board envelope is explicit, hidden reference geometry, not a fabricated detailed part model.
 refs=[('R01 Orin Nano RESERVED 79x100x50',(29,108,-50,50,38,88)),('R02 Additional PCB RESERVED 70x90x22',(111,181,-45,45,50,72)),('R03 Power and interface RESERVED',(0,26,-28,28,7,29)),('R04 Rear connector service space',(0,29,-30,30,38,75))]
 for name,bounds in refs:
  cp=compnew(r,name,True);persist(cp,[(name,box(*bounds))],(48,137,121),appearance='Reference teal');cp.attributes.add('SCONE_HOUSING','bounds_mm',json.dumps(bounds));r.occurrences.itemByName(name+':1').isLightBulbOn=False
 # Visible simplified sensors; dimensions remain provisional pending real models.
 lidar=compnew(r,'R05 LiDAR T-mini Plus provisional',True)
 persist(lidar,[('Sensor base',box(53-19.3,53+19.3,-19.3,19.3,112,118)),('Scanner housing',cylinder((53,0,118),(53,0,145.9),19.3))],(27,31,37),appearance='Sensor graphite')
 cam=compnew(r,'R06 Stereo camera provisional',True)
 persist(cam,[('85.2 mm stereo board',box(163,165,-42.6,42.6,76,100))],(31,101,80),appearance='Circuit board green')
 for side in [-1,1]:
  cp=compnew(r,'R06 Lens '+str(side),True);persist(cp,[('Lens barrel',cylinder((165,side*30,88),(187,side*30,88),7))],(24,28,34),appearance='Optical black')
  cp=compnew(r,'R06 Glass '+str(side),True);persist(cp,[('Lens front',cylinder((187,side*30,88),(187.3,side*30,88),5.5))],(33,87,109),appearance='Lens blue')
 mount=compnew(r,'H06 Sensor mounts')
 b=cylinder((53,0,104),(53,0,112),24);cut(b,cylinder((53,0,103),(53,0,113),6))
 for x in [38,68]:
  for y in [-15,15]:cut(b,cylinder((x,y,103),(x,y,113),1.6))
 parts=[('Lidar adapter - hole pattern provisional',b)]
 for y in [-40,40]:
  p=box(157,163,y-3,y+3,49.5,100);cut(p,cylinder((156,y,95),(164,y,95),1.6));parts.append(('Camera riser with mounting slot',p))
 persist(mount,parts)
 # Structural shell remains a native loft/shell feature; ports are separate native cut features.
 shell=r.occurrences.itemByName('H01 Upper shell:1').component;target=shell.bRepBodies.item(0)
 cuts=[cylinder((53,0,100),(53,0,115),6)]
 for side in [-1,1]:cuts.append(cylinder((170,side*30,88),(201,side*30,88),12))
 for x in [10,168]:
  for side in [-1,1]:cuts.append(cylinder((x,side*28,38),(x,side*55,38),1.7))
 # Downstream air openings flank the Jetson. No decorative closed 'vents'.
 for x in [45,54,63,72,81,90,99,108]:
  for side in [-1,1]:cuts.append(box(x,x+4,side*48 if side>0 else -85,85 if side>0 else -48,54,69))
 cut_native(shell,target,cuts,'H01 sensor apertures cable feed and actual vents')
 # Recolor forward shell faces to form a dark sensor mask without an overlapping cosmetic body.
 appearance=d.appearances.itemByName('Housing Sensor graphite')
 for face in shell.bRepBodies.item(0).faces:
  if face.boundingBox.minPoint.x>17.7 and face.boundingBox.maxPoint.z>7.0:face.appearance=appearance
 # Sketches and joint glyphs are hidden in the copy only, so the result is easy to review.
 for cp in d.allComponents:
  cp.isJointsFolderLightBulbOn=False
  cp.isSketchFolderLightBulbOn=False
  cp.isConstructionFolderLightBulbOn=False
 for o in r.occurrences:
  if o.name=='21700 CELLS:1':o.isLightBulbOn=False
 for o in r.occurrences:
  if o.component.attributes.itemByName('SCONE_HOUSING','owned'):
   report[o.name]={'role':o.component.attributes.itemByName('SCONE_HOUSING','role').value if o.component.attributes.itemByName('SCONE_HOUSING','role') else 'manufactured','bodies':[(b.name,b.volume,bb(b)) for b in o.component.bRepBodies]}
 (OUT/'parts_build.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
 doc.save('Housing A01 frame, removable carriers, sensor mounts, ventilation and packaging references')
 print('HOUSING PARTS BUILT AND SAVED')
try:run()
except:(OUT/'parts_error.txt').write_text(traceback.format_exc());print(traceback.format_exc())
