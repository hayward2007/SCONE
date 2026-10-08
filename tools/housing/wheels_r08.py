exec(open('/Users/hayward_kim/Developer/SCONE/tools/housing/build_r05.py',encoding='utf-8').read().split('\ndef run():')[0])
DEST=OUT/'R08_DELIVERY'
def run():
 app,doc,d=guard();r=d.rootComponent;report=[]
 for path,label,sign in [('LEG 1:1+ARC:1','W01',1),('LEG 1(미러):1+ARC(미러):1','W11',-1)]:
  occ=next(o for o in r.allOccurrences if o.fullPathName==path);cp=occ.component
  original=cp.bRepBodies.itemByName('본체1');tire=cp.bRepBodies.itemByName('본체2');assert original and tire
  before={'base':bb(original),'tire':bb(tire)};q=T.copy(original)
  # Add axial material only at the existing tyre seat, preserving every XY contour.
  band=T.copy(q);cut(band,cylinder((0,0,-25),(0,0,25),108.5))
  straight=T.copy(q);intersect(straight,box(8.5,25,-125,125,-25,25));union(band,straight)
  hole_axes=[]
  for face in original.faces:
   cy=c.Cylinder.cast(face.geometry)
   if cy and abs(cy.axis.z)>.99 and cy.radius<.3:hole_axes.append([cy.origin.x*10,cy.origin.y*10,cy.radius*10])
  # Protect the original central mounting pattern from axial thickening.
  for x,y,rad in hole_axes:cut(band,cylinder((x,y,-25),(x,y,25),rad+2.0))
  union(q,move(T.copy(band),z=sign*8));assert q.lumps.count==1
  base=cp.features.baseFeatures.add();base.name='R08 axial tyre-seat extension, original semicircle unchanged';base.startEdit();new=cp.bRepBodies.add(q,base);new.name=label+' PLA rim 18mm seat, original hub';base.finishEdit();original.isLightBulbOn=False;pla(new,(45,48,53))
  width=(tire.boundingBox.maxPoint.z-tire.boundingBox.minPoint.z)*10
  assert abs(width-10)<.001,(cp.name,width)
  inp=cp.features.scaleFeatures.createInput(collection([tire]),cp.originConstructionPoint,c.ValueInput.createByReal(1));inp.setToNonUniform(c.ValueInput.createByReal(1),c.ValueInput.createByReal(1),c.ValueInput.createByReal(2));feat=cp.features.scaleFeatures.add(inp);feat.name='R08 tyre axial width 10 to20 only'
  tire=feat.bodies.item(0);tire.name=('T01' if sign>0 else 'T11')+' TPU tyre 20mm';finish_appearance(app,d,tire,(23,25,28),'TPU mass proxy - unvalidated print stiffness','PrismMaterial-022');tire.material.materialProperties.itemById('structural_Density').value=1200;tire.material.materialProperties.itemById('physmat_Comments').value='TPU nominal density 1200 kg/m3 for mass only; FDM stiffness and infill must be validated.'
  after={'base':bb(new),'tire':bb(tire)}
  for k in ['base','tire']:
   for corner in [0,1]:assert max(abs(before[k][corner][j]-after[k][corner][j]) for j in [0,1])<1e-5
  report.append(dict(component=cp.name,before=before,after=after,base_volume_cm3=new.volume,tire_volume_cm3=tire.volume,unchanged_xy_profile=True,axial_inner_mounting_face_unchanged=True,protected_axial_hole_axes_mm=hole_axes))
 doc.save('R08 20mm TPU tyres, axial-only rim seat extension, original semicircle and mounting axes preserved')
 (DEST/'wheel_width_change.json').write_text(json.dumps(report,ensure_ascii=True,indent=2));print('R08 WHEELS WIDENED')
try:run()
except:(DEST/'wheel_error.txt').write_text(traceback.format_exc(),encoding='utf-8');print(traceback.format_exc())
