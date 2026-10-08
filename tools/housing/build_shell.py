exec(open('/Users/hayward_kim/Developer/SCONE/tools/housing/common.py',encoding='utf-8').read())
import math

def P(x,y,z=0):return c.Point3D.create(x/10,y/10,z/10)
def V(s):return c.ValueInput.createByString(str(s))
def collection(items):
 a=c.ObjectCollection.create()
 for i in items:a.add(i)
 return a

def rounded_profile(comp,name,z,points,radius):
 pi=comp.constructionPlanes.createInput();pi.setByOffset(comp.xYConstructionPlane,V(z));plane=comp.constructionPlanes.add(pi);plane.name=name+'_level'
 sk=comp.sketches.add(plane);sk.name=name;sk.isComputeDeferred=True
 tangents=[]
 for i,v in enumerate(points):
  before=points[i-1];after=points[(i+1)%len(points)]
  l1=math.dist(v,before);l2=math.dist(v,after)
  u=[(before[q]-v[q])/l1 for q in (0,1)];w=[(after[q]-v[q])/l2 for q in (0,1)]
  th=math.acos(max(-1,min(1,sum(u[q]*w[q] for q in (0,1)))))
  dist=min(radius/math.tan(th/2),l1*.35,l2*.35);rr=dist*math.tan(th/2)
  bis=[u[q]+w[q] for q in (0,1)];bn=math.hypot(*bis)
  cen=[v[q]+bis[q]/bn*rr/math.sin(th/2) for q in (0,1)]
  t1=[v[q]+u[q]*dist for q in (0,1)];t2=[v[q]+w[q]*dist for q in (0,1)]
  mid=[cen[q]+(v[q]-cen[q])/math.dist(v,cen)*rr for q in (0,1)]
  tangents.append((t1,mid,t2))
 for i,(t1,mid,t2) in enumerate(tangents):
  try:a=sk.sketchCurves.sketchArcs.addByThreePoints(P(*t1),P(*mid),P(*t2))
  except:raise RuntimeError(str((name,i,t1,mid,t2)))
  a.isFixed=True
  line=sk.sketchCurves.sketchLines.addByTwoPoints(a.endSketchPoint,P(*tangents[(i+1)%len(points)][0]));line.isFixed=True
 sk.isComputeDeferred=False;sk.isVisible=False;plane.isLightBulbOn=False
 assert sk.profiles.count==1,(name,sk.profiles.count)
 return sk.profiles.item(0)

def finish_appearance(app,d,body,color,name,matid='PrismMaterial-417'):
 lib=app.materialLibraries.itemById('C1EEA57C-3F56-45FC-B8CB-A9EC46A9994C')
 mat=d.materials.itemByName('Housing '+name)
 if not mat:mat=d.materials.addByCopy(lib.materials.itemById(matid),'Housing '+name)
 body.material=mat
 a=d.appearances.itemByName('Housing '+name)
 if not a:a=d.appearances.addByCopy(mat.appearance,'Housing '+name)
 for prop in a.appearanceProperties:
  if prop.id in ('opaque_albedo','surface_albedo'):
   try:prop.value=c.Color.create(*color,255)
   except:pass
 body.appearance=a
 return a

def run():
 app,doc,d=guard();root=d.rootComponent
 old=root.occurrences.itemByName('H01 Upper shell:1')
 if old:
  assert old.component.attributes.itemByName('SCONE_HOUSING','owned') and old.component.bRepBodies.count==0,'Inspect an existing solid before rebuilding'
  old.deleteMe()
 for name,expression,unit,comment in [('housing_skin','2.2 mm','mm','PA12 target wall; validate print process'),('housing_seam_z','33.8 mm','mm','0.3 mm above retained tray'),('housing_shoulder_z','53 mm','mm','Hip relief transition'),('housing_roof_z','96 mm','mm','Roof with Orin cooling allowance')]:
  if not d.userParameters.itemByName(name):d.userParameters.add(name,V(expression),unit,comment)
 occ=root.occurrences.addNewComponent(c.Matrix3D.create());comp=occ.component;comp.name='H01 Upper shell';comp.attributes.add('SCONE_HOUSING','owned','A01')
 pts0=[(-24,-37.5),(28,-37.5),(36,-77),(130,-77),(138,-37.5),(190,-37.5),(190,37.5),(138,37.5),(130,77),(36,77),(28,37.5),(-24,37.5)]
 pts1=[(-21,-37),(18,-46),(35,-76),(132,-76),(156,-52),(188,-51),(188,51),(156,52),(132,76),(35,76),(18,46),(-21,37)]
 pts2=[(-10,-30),(16,-42),(39,-62),(123,-62),(155,-49),(180,-47),(180,47),(155,49),(123,62),(39,62),(16,42),(-10,30)]
 inp=comp.features.loftFeatures.createInput(f.FeatureOperations.NewBodyFeatureOperation)
 for name,z,pts,rad in [('H01 Seam profile','housing_seam_z',pts0,4),('H01 Shoulder profile','housing_shoulder_z',pts1,7),('H01 Roof profile','housing_roof_z',pts2,9)]:
  inp.loftSections.add(rounded_profile(comp,name,z,pts,rad))
 feat=comp.features.loftFeatures.add(inp);feat.name='H01 sculpted hip-relief cover';body=feat.bodies.item(0);body.name='PA12 cover'
 # Round the flat roof boundary before shelling, for a continuous cap transition.
 edges=[e for e in body.edges if abs(e.boundingBox.minPoint.z-9.6)<1e-6 and abs(e.boundingBox.maxPoint.z-9.6)<1e-6]
 fi=comp.features.filletFeatures.createInput();fi.addConstantRadiusEdgeSet(collection(edges),V('4 mm'),False)
 ff=comp.features.filletFeatures.add(fi);ff.name='H01 roof shoulder radius'
 body=comp.bRepBodies.item(0)
 bottom=min([fa for fa in body.faces if abs(fa.boundingBox.minPoint.z-fa.boundingBox.maxPoint.z)<1e-6],key=lambda fa:fa.boundingBox.minPoint.z)
 si=comp.features.shellFeatures.createInput(collection([bottom]),False);si.insideThickness=V('housing_skin')
 sh=comp.features.shellFeatures.add(si);sh.name='H01 constant wall and open underside'
 body=comp.bRepBodies.item(0);finish_appearance(app,d,body,(222,174,45),'Warm ochre PA12')
 # Keep original copied structure; only give the retained tray its housing color.
 finish_appearance(app,d,root.bRepBodies.itemByName('본체12'),(44,49,55),'Graphite PA12')
 (OUT/'shell_build.json').write_text(json.dumps({'volume_cm3':body.volume,'box':bb(body),'skin_mm':2.2,'health':sh.healthState},indent=2))
 doc.save('Housing A01 sculpted upper cover, source preserved')
 print('SHELL BUILT: constant 2.2 mm wall, hip relief, rounded roof')
try:run()
except:(OUT/'shell_error.txt').write_text(traceback.format_exc());print(traceback.format_exc())
