import adsk.core as c, adsk.fusion as f, math,json
from pathlib import Path
OUT=Path('/Users/hayward_kim/Developer/SCONE/artifacts/arm/20260929_refinement')
T=f.TemporaryBRepManager.get()
def P(x,y,z):return c.Point3D.create(x/10,y/10,z/10)
def V(x,y,z):return c.Vector3D.create(x,y,z)
def box(x0,x1,y0,y1,z0,z1):return T.createBox(c.OrientedBoundingBox3D.create(P((x0+x1)/2,(y0+y1)/2,(z0+z1)/2),V(1,0,0),V(0,1,0),(x1-x0)/10,(y1-y0)/10,(z1-z0)/10))
def cyl(a,b,r):return T.createCylinderOrCone(P(*a),r/10,P(*b),r/10)
def op(a,b,typ):assert T.booleanOperation(a,b,typ);return a
def join(a,b):return op(a,b,f.BooleanTypes.UnionBooleanType)
def cut(a,b):return op(a,b,f.BooleanTypes.DifferenceBooleanType)
def common(a,b):return op(a,b,f.BooleanTypes.IntersectionBooleanType)
def rounded_x(x0,x1,y0,y1,z0,z1,rad):
 q=box(x0,x1,y0+rad,y1-rad,z0,z1);join(q,box(x0,x1,y0,y1,z0+rad,z1-rad))
 for y in [y0+rad,y1-rad]:
  for z in [z0+rad,z1-rad]:join(q,cyl((x0,y,z),(x1,y,z),rad))
 return q
def rounded_z(x0,x1,y0,y1,z0,z1,rad):
 q=box(x0+rad,x1-rad,y0,y1,z0,z1);join(q,box(x0,x1,y0+rad,y1-rad,z0,z1))
 for x in [x0+rad,x1-rad]:
  for y in [y0+rad,y1-rad]:join(q,cyl((x,y,z0),(x,y,z1),rad))
 return q
def coll(xs):
 q=c.ObjectCollection.create()
 for x in xs:q.add(x)
 return q
def native_op(co,b,temps,name,operation):
 assert co.name.startswith('ARM')
 bf=co.features.baseFeatures.add();bf.name=name+' tools';bf.startEdit()
 for i,q in enumerate(temps):co.bRepBodies.add(q,bf).name=name+' tool '+str(i)
 bf.finishEdit();tools=[v for v in bf.bodies]
 inp=co.features.combineFeatures.createInput(b,coll(tools));inp.operation=operation;inp.isKeepToolBodies=False
 feat=co.features.combineFeatures.add(inp);feat.name=name
 assert feat.healthState==f.FeatureHealthStates.HealthyFeatureHealthState,feat.errorOrWarningMessage
 return feat.bodies.item(0)
def add(co,q,name,appearance,material):
 assert co.name.startswith('ARM')
 bf=co.features.baseFeatures.add();bf.name=name;bf.startEdit();b=co.bRepBodies.add(q,bf);b.name=name;bf.finishEdit();b=bf.bodies.item(0);b.appearance=appearance;b.material=material;return b
def fillet(co,b,predicate,rad,name):
 es=[e for e in b.edges if c.Line3D.cast(e.geometry) and predicate([x*10 for x in e.startVertex.geometry.asArray()],[x*10 for x in e.endVertex.geometry.asArray()])]
 assert es,name+' no edges'
 fi=co.features.filletFeatures.createInput();fi.edgeSetInputs.addConstantRadiusEdgeSet(coll(es),c.ValueInput.createByReal(rad/10),False)
 ft=co.features.filletFeatures.add(fi);ft.name=name
 assert ft.healthState==f.FeatureHealthStates.HealthyFeatureHealthState,ft.errorOrWarningMessage
 return ft.bodies.item(0)
def run(_context):
 app=c.Application.get();d=f.Design.cast(app.activeProduct);assert app.activeDocument.name.startswith('MARC v4 Arm R1 Folded')
 co5=next(co for co in d.allComponents if co.name.startswith('ARM5 '));co6=next(co for co in d.allComponents if co.name.startswith('ARM6 '))
 b5=next(b for b in co5.bRepBodies if b.name.startswith('A5 gripper'));b6=next(b for b in co6.bRepBodies if b.name.startswith('A6 moving jaw ('))
 yellow=b5.appearance;pla=b5.material
 dark=next(b.appearance for o in d.rootComponent.allOccurrences for b in o.bRepBodies if b.isSolid and 'TPU' in b.name and not o.component.name.startswith('ARM'))
 oldp5=next(b for b in co5.bRepBodies if 'pad (TPU)' in b.name);oldp6=next(b for b in co6.bRepBodies if 'pad (TPU)' in b.name);tpu=oldp5.material
 # Existing A5 mounting coordinates stay fixed; only unsupported upper fins are removed.
 b5=native_op(co5,b5,[box(34.8,39.0,-18,18,-4,25),box(-.1,6.1,-18,18,22,30)],'R1 A5 remove unused fins',f.FeatureOperations.CutFeatureOperation)
 b5=fillet(co5,b5,lambda a,b:abs(a[0]-85)<.02 and abs(b[0]-85)<.02 and abs(abs(a[1])-12)<.02 and abs(a[1]-b[1])<.02,3,'R1 A5 rounded jaw nose R3')
 # Taper the moving jaw from 46 mm at the root to 30 mm at the tip.
 wedges=[]
 slope=8/45.75
 for sign in [-1,1]:
  ux,uy=1/math.sqrt(1+slope*slope),-sign*slope/math.sqrt(1+slope*slope)
  nx,ny=sign*-uy,sign*ux
  q=T.createBox(c.OrientedBoundingBox3D.create(P(42+60*nx,sign*(23-(42-19)*slope)+60*ny,-5),V(ux,uy,0),V(nx,ny,0),24,12,8))
  common(q,box(19,70,-90,90,-30,30));wedges.append(q)
 b6=native_op(co6,b6,wedges,'R1 A6 tapered jaw shoulders',f.FeatureOperations.CutFeatureOperation)
 b6=fillet(co6,b6,lambda a,b:abs(a[0]-64.75)<.02 and abs(b[0]-64.75)<.02 and abs(abs(a[1])-15)<.03 and abs(a[1]-b[1])<.02,2.5,'R1 A6 rounded jaw nose R2.5')
 # Recessed TPU pads: same contact planes; 1.2 mm keys into jaw, removable M2 low-head screws.
 for co,b,old,x0,z0,z1,direction in [(co5,b5,oldp5,55,-7.2,-4,1),(co6,b6,oldp6,34.75,-20,-16.8,-1)]:
  pocket=rounded_z(x0-.15,x0+30.15,-12.15,12.15,z0-.02,z1+.02,3.15)
  tools=[pocket]
  for x in [x0+7,x0+23]:
   start=z0 if direction>0 else z1
   end=start-direction*3.8
   tools.append(cyl((x,0,start+direction*.1),(x,0,end),.8))
  b=native_op(co,b,tools,'R1 '+co.name[:4]+' keyed pad seat and M2 pilots',f.FeatureOperations.CutFeatureOperation)
  old.isLightBulbOn=False
  pad=rounded_z(x0,x0+30,-12,12,z0,z1,3)
  for x in [x0+7,x0+23]:
   cut(pad,cyl((x,0,z0-.1),(x,0,z1+.1),1.15))
   if direction>0:cut(pad,cyl((x,0,z1-1.6),(x,0,z1+.1),2.1))
   else:cut(pad,cyl((x,0,z0-.1),(x,0,z0+1.6),2.1))
  # Shallow transverse grooves, avoiding screw seats.
  for x in [x0+3,x0+11,x0+15,x0+19,x0+27]:
   if direction>0:cut(pad,box(x-.35,x+.35,-12.1,12.1,z1-.35,z1+.1))
   else:cut(pad,box(x-.35,x+.35,-12.1,12.1,z0-.1,z0+.35))
  add(co,pad,co.name[:4]+' R1 replaceable TPU pad - 2x M2x5 low-head',dark,tpu)
  if co==co5:b5=b
  else:b6=b
 # Full-depth rear counterbores for the 4 existing J6 M2.5 mounting axes.
 ts=[cyl((-.1,y,z),(2.6,y,z),2.7) for y in [-6,6] for z in [-12,12]]
 b5=native_op(co5,b5,ts,'R1 J6 M2.5 counterbores D5.4 depth2.6',f.FeatureOperations.CutFeatureOperation)
 # Removable camera pod, using the existing 28x28 hole pattern and leaving the lens proud.
 cover=rounded_x(39.2,54.5,-18.5,18.5,-49,-11.5,6)
 cut(cover,rounded_x(39.0,52.0,-16.6,16.6,-46.6,-13.4,2))
 cut(cover,box(39.0,55,-19,19,-12.3,-10))
 for y in [-14,14]:
  for z in [-44,-16]:
   join(cover,cyl((39.2,y,z),(54.5,y,z),3.1))
   cut(cover,cyl((39.1,y,z),(54.6,y,z),1.15))
   cut(cover,cyl((52.3,y,z),(54.6,y,z),2.1))
 cut(cover,cyl((51.9,0,-30),(54.6,0,-30),7.5))
 # Rear cable service opening in the underside of the pod.
 cut(cover,box(39.0,45,-4,4,-49.1,-46.2))
 assert cover.isSolid and cover.lumps.count==1
 pod=add(co5,cover,'A5 R1 camera pod - 4x M2x16 counterbored',yellow,pla)
 for face in pod.faces:
  pl=c.Plane.cast(face.geometry)
  if pl and abs(pl.origin.x*10-54.5)<.02:face.appearance=dark
 (OUT/'gripper_build.json').write_text(json.dumps({'status':'geometry applied','pad_contact_planes_unchanged':True,'pads':'3.2mm keyed TPU, 1.6mm deep D4.2 counterbores, M2x5 low-head candidate','J6_counterbore':'D5.4 x 2.6 deep; 3.4 residual','camera':'existing placeholder retained; 28x28 pattern; cover D4.2 x2.2 CB; M2x16 candidate'},indent=2))
 print('GRIPPER_REFINED',b5.volume,b6.volume,pod.volume)
