exec(open('/Users/hayward_kim/Developer/SCONE/tools/housing/build_p02.py',encoding='utf-8').read().split('\ndef run():')[0])

def sculpt(root,name,levels,wall,open_top=False):
 comp=compnew(root,name);comp.attributes.add('SCONE_HOUSING','owned','R03')
 inp=comp.features.loftFeatures.createInput(f.FeatureOperations.NewBodyFeatureOperation)
 for i,(z,points,radius) in enumerate(levels):inp.loftSections.add(rounded_profile(comp,name+' section '+str(i),str(z)+' mm',points,radius))
 feat=comp.features.loftFeatures.add(inp);feat.name=name+' smooth loft';b=feat.bodies.item(0);outer=T.copy(b)
 if wall:
  # Explicit nested loft avoids fragile constant-wall shelling at the concave waist.
  def inset(poly,dist):
   lines=[]
   for a,b in zip(poly,poly[1:]+poly[:1]):
    dx=b[0]-a[0];dy=b[1]-a[1];L=math.hypot(dx,dy);lines.append(((a[0]-dy/L*dist,a[1]+dx/L*dist),(dx,dy)))
   out=[]
   for i,(q,v) in enumerate(lines):
    p,u=lines[i-1];det=u[0]*v[1]-u[1]*v[0]
    t=((q[0]-p[0])*v[1]-(q[1]-p[1])*v[0])/det
    out.append((p[0]+t*u[0],p[1]+t*u[1]))
   return out
  inner=comp.features.loftFeatures.createInput(f.FeatureOperations.NewBodyFeatureOperation)
  for i,(z,points,radius) in enumerate(levels):
   iz=z
   if i==0:iz=z+wall if open_top else z-.5
   if i==len(levels)-1:iz=z+.5 if open_top else z-wall
   inner.loftSections.add(rounded_profile(comp,name+' inside '+str(i),str(iz)+' mm',inset(points,wall),max(1.5,radius-wall)))
  ins=comp.features.loftFeatures.add(inner)
  cutin=comp.features.combineFeatures.createInput(comp.bRepBodies.item(0),collection([ins.bodies.item(0)]));cutin.operation=f.FeatureOperations.CutFeatureOperation;comp.features.combineFeatures.add(cutin)
 return comp,outer

def mirrorpts(half):return half+[(x,-y) for x,y in reversed(half)]
def run():
 app,doc,d=guard();r=d.rootComponent
 assert not r.occurrences.itemByName('R01 Rear smooth PLA chassis:1'),'R03 already exists'
 for name in ['R00 Lower surface master:1','R03 Smooth PLA sensor cover:1']:
  old=r.occurrences.itemByName(name)
  if old:old.deleteMe()
 # Back up the current independent copy. The protected user model is never activated.
 if not (OUT/'P02_before_R03.f3d').exists():d.exportManager.execute(d.exportManager.createFusionArchiveExportOptions(str(OUT/'P02_before_R03.f3d')))
 seam=mirrorpts([(-24,-37.5),(28,-37.5),(36,-77),(99,-77),(118,-58),(135,-58),(145,-37.5),(190,-37.5)])
 bottom=mirrorpts([(-24,-33),(25,-33),(40,-71),(97,-71),(118,-53),(135,-53),(145,-33),(190,-33)])
 shoulder=mirrorpts([(-21,-37),(18,-46),(35,-76),(99,-75),(118,-58),(135,-58),(156,-52),(188,-51)])
 roof=mirrorpts([(-10,-30),(16,-42),(39,-62),(99,-62),(118,-56),(135,-56),(155,-49),(180,-47)])
 shell_comp,_=sculpt(r,'R00 Lower surface master',[(-.5,bottom,7),(14.5,mirrorpts([(-24,-35),(27,-35),(38,-75),(99,-75),(118,-56),(135,-56),(145,-35),(190,-35)]),7),(33.5,seam,5)],2.6,True)
 low=T.copy(shell_comp.bRepBodies.item(0))
 cover_comp,cover_outer=sculpt(r,'R03 Smooth PLA sensor cover',[(33.8,seam,5),(53,shoulder,7),(104,roof,9)],2.6,False)
 cv=T.copy(cover_comp.bRepBodies.item(0))
 # Modest radius at roof edge, then shell already produced; retained on curved loft walls.
 # Build functional details into a temporary final cover while preserving the editable loft master.
 for x,y in [(38.9,-14.1),(67.1,14.1)]:cut(cv,cylinder((x,y,98),(x,y,106),1.15))
 cut(cv,box(48,58,-27,-19,99,106))
 for x in [79,85,91,97,103,109,115]:cut(cv,box(x-1.5,x+1.5,-24,26,98,110))
 for y in [-40.6,40.6]:
  union(cv,box(162,165.7,y-3,y+3,77,103));union(cv,box(159,165.7,y-4,y+4,100.8,103))
  for z in [79.7,93.15]:cut(cv,cylinder((159,y,z),(166,y,z),1.0))
 for y in [-30.05,30.05]:cut(cv,cylinder((166,y,86),(201,y,86),12))
 # Lateral ventilation, rear USB and network access. Ends remain within the sculpted surface.
 for side in [-1,1]:
  for x in [-8,1,10,153,162,171,180]:cut(cv,box(x-1.3,x+1.3,side*48-12,side*48+12,59,74))
 cut(cv,box(-35,8,-24,-9,53,62));cut(cv,box(-35,8,3,23,47,68))
 # Cover fastening ears inside the sculpted shell.
 for x in [10,168]:
  for y in [-31,31]:
   union(cv,box(x-4,x+4,-44 if y<0 else 31,-31 if y<0 else 44,35,43))
   cut(cv,cylinder((x,-44 if y<0 else 25,38),(x,-25 if y<0 else 44,38),1.65))
 # Keep the loft body as hidden design history; use a single printable final body.
 cover_comp.bRepBodies.item(0).isLightBulbOn=False
 base=cover_comp.features.baseFeatures.add();base.name='R03 integrated sensor mounts';base.startEdit();b=cover_comp.bRepBodies.add(cv,base);b.name='R03 printable cover';pla(b,(222,174,45));base.finishEdit()
 rear=T.copy(r.occurrences.itemByName('P01 Rear PLA chassis:1').bRepBodies.item(0));front=T.copy(r.occurrences.itemByName('P02 Front PLA chassis:1').bRepBodies.item(0))
 # Retain mounts, internals and reinforced split joint; add matching curved lower skin.
 union(rear,intersect(T.copy(low),box(-100,64.85,-110,110,-10,100)))
 union(front,intersect(T.copy(low),box(65.15,260,-110,110,-10,100)))
 # Superseded straight outer walls will be clipped to the matched outer profile in refinement.
 # Four integral upper motor saddles. Fix to stationary casing holes, never to rotating horn.
 mount=[]
 for xc,target in [(-41.8912974452,rear),(208.1087025548,front)]:
  for side in [-1,1]:
   y0,y1=sorted([side*24,side*43]);cap=box(xc-19.5,xc+19.5,y0,y1,33.3,36.7)
   # Trim relief around the raised center casing; mounting pads retain the CAD hole positions.
   cut(cap,box(xc-12.6,xc+12.6,min(side*31,side*47),max(side*31,side*47),33,36.2))
   # Support posts are inboard of the fixed casing rear face. Center leaves the cable exit accessible.
   for dx in [-17,17]:
    union(target,box(xc+dx-2.5,xc+dx+2.5,min(side*21.5,side*26.1),max(side*21.5,side*26.1),2.5,36.7))
   union(target,cap)
   for dx,ya in [(-15,35.64092583),(15,35.64092583),(-8.5,29.34092583),(8.5,29.34092583)]:
    y=side*ya;cut(target,cylinder((xc+dx,y,32),(xc+dx,y,39),1.15));mount.append([xc+dx,y,33.3])
 # All-angle distal-wheel envelopes at the user requested pose: yaw inward 90, hip180.
 # 20 mm centered-width candidate + 2.1 mm clearance is reserved; original ARC is untouched.
 envs=[]
 for side in [-1,1]:envs.append(cylinder((118.2087026,side*187.64092583,18.5),(142.4087026,side*187.64092583,18.5),124.6))
 # Rear pair's corresponding sweep is outside the body, but validate it explicitly later.
 for target in [rear,front,cv]:
  for env in envs:cut(target,T.copy(env))
 # Avoid intersecting final cover tabs; geometrically exact contact is offset locally at screw pads.
 for target in [rear,front]:
  cut(target,T.copy(cv))
  for x in [10,168]:
   for y in [-31,31]:cut(target,box(x-4.25,x+4.25,-44.25 if y<0 else 30.75,-30.75 if y<0 else 44.25,34.8,43.25))
 # Original tiny motor contacts are preserved for comparison, not blindly enlarged.
 p1=part(r,'R01 Rear smooth PLA chassis',rear);p2=part(r,'R02 Front smooth PLA chassis',front)
 for comp in [p1,p2]:comp.attributes.add('SCONE_HOUSING','owned','R03')
 # Replace cover final with clearance-refined temporary shape.
 base.startEdit();base.bodies.item(0).deleteMe();b=cover_comp.bRepBodies.add(cv,base);b.name='R03 printable cover';pla(b,(222,174,45));base.finishEdit()
 for o in r.occurrences:
  if o.name.startswith(('P01 ','P02 ','P03 ','R00 ')):o.isLightBulbOn=False
 for comp in [p1,p2,cover_comp]:
  comp.isSketchFolderLightBulbOn=False;comp.isConstructionFolderLightBulbOn=False
  for b in comp.bRepBodies:
   if b.name.startswith('R0'):pla(b,(222,174,45) if comp==cover_comp else (42,47,53))
 (OUT/'r03_build.json').write_text(json.dumps({'upper_motor_mount_holes_mm':mount,'example_holes':'M2 clearance 2.3 mm; final screw length after user confirmation','wheel_candidate_reserved_width_mm':20,'actual_ARC_changed':False,'critical_pose':'MX28 inward90, hip XM430180, distal all angles'},indent=2))
 doc.save('R03 smooth matching lower housing, fixed MX28 upper and lower saddles, hip180 wheel clearance')
 print('R03 BUILT')
try:run()
except:(OUT/'r03_build_error.txt').write_text(traceback.format_exc(),encoding='utf-8');print(traceback.format_exc())
