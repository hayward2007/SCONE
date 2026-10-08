import adsk.core as c, adsk.fusion as f,math,json
from pathlib import Path
OUT=Path('/Users/hayward_kim/Developer/SCONE/artifacts/arm/20260929_refinement')
def run(_context):
 app=c.Application.get();doc=app.activeDocument;d=f.Design.cast(app.activeProduct)
 assert doc.name.startswith('MARC v4 Arm R1 Folded'),doc.name
 arm=next(x for x in d.allComponents if x.name.startswith('ARM 5DOF'))
 # Angles are relative to the inherited as-built joint frames, not servo zero.
 vals={'J1 base yaw':0,'J2 shoulder':-132,'J3 elbow':-147,'J4 wrist pitch':-13,'J5 wrist roll':-90,'J6 gripper':0}
 for name,v in vals.items():arm.asBuiltJoints.itemByName(name).jointMotion.rotationValue=math.radians(v)
 arm.isJointsFolderLightBulbOn=False
 app.activeViewport.refresh()
 out=[]
 for o in d.rootComponent.allOccurrences:
  if o.component.name.startswith('ARM') and o.bRepBodies.count:
   out.append({'name':o.name,'transform':list(o.transform2.asArray()),'bodies':[{'name':b.name,'box':[[round(x*10,3) for x in p.asArray()] for p in [b.boundingBox.minPoint,b.boundingBox.maxPoint]]} for b in o.bRepBodies]})
 (OUT/'fold_applied.json').write_text(json.dumps({'joint_deg':vals,'parts':out},indent=2))
 cam=app.activeViewport.camera;cam.isSmoothTransition=False;cam.cameraType=c.CameraTypes.OrthographicCameraType;cam.target=c.Point3D.create(8,0,12);cam.eye=c.Point3D.create(50,-80,45);cam.upVector=c.Vector3D.create(0,0,1);cam.viewExtents=36;app.activeViewport.camera=cam
 print('FOLDED',[(v['name'],v['transform'][0],v['transform'][8]) for v in out])
