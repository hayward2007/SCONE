exec(open('/Users/hayward_kim/Developer/SCONE/artifacts/arm/20260929_refinement/inspect_and_backup.py').read().split('def run(_context):')[0])
def run(_context):
 app=c.Application.get();doc=app.activeDocument;d=f.Design.cast(app.activeProduct)
 assert (OUT/'original_before_arm_refinement.f3d').stat().st_size>10000000
 before=json.loads((OUT/'protected_before.json').read_text());after=protected(d)
 assert before==after,'Protected assembly changed before any edit'
 assert doc.dataFile.id=='urn:adsk.wipprod:dm.lineage:cIsNU1hKTEaOfcReFZQlQw'
 assert doc.saveAs('MARC v4 Arm R1 Folded',doc.dataFile.parentFolder,'Arm-only gripper and stow refinement. Body, legs, wheels, electronics and sensors preserved.','')
 arm=next(co for co in d.allComponents if co.name==ARM)
 for j in arm.asBuiltJoints:j.isLightBulbOn=False
 print('WORKING_COPY',doc.name,doc.dataFile.id)
 print('J6_PARENT',arm.asBuiltJoints.itemByName('J6 gripper').geometry.entityOne.parentComponent.name)
 print('camera',c.Camera.setExtents.__doc__)
