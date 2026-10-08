"""Clear Fusion automatic ground-to-parent and persist custom joint axes."""
from fusion_common import *

def run(ctx):
    app,doc,d,folder,p=guard(ctx);root=d.rootComponent
    for o in root.occurrences:
        if o.attributes.itemByName('MARC','instance_id'):
            o.isGroundToParent=False
            o.isGrounded=False
    sk=root.sketches.itemByName('MARC_12_axis_markers')
    results=[]
    for i,(leg_id,sx,sy) in enumerate([('FL',1,1),('FR',1,-1),('RL',-1,1),('RR',-1,-1)]):
        for k,(kind,num) in enumerate([('yaw',i+1),('q1',i+5),('q2',i+9)]):
            j=root.asBuiltJoints.itemByName(f'M{num:02}_{leg_id}_{kind}')
            j.timelineObject.rollTo(True)
            m=fusion.RevoluteJointMotion.cast(j.jointMotion)
            line=sk.sketchCurves.sketchLines.item(i*3+k)
            attr(line,'joint_id',f'M{num:02}')
            assert j.setAsRevoluteJointMotion(fusion.JointDirections.CustomJointDirection, None, line)
            d.timeline.moveToEnd()
            m=fusion.RevoluteJointMotion.cast(j.jointMotion)
            m.rotationValue=0
            results.append({'name':j.name,'axis':m.rotationAxisVector.asArray(),'axis_type':str(m.rotationAxis)})
    for o in list(root.occurrences):
        if o.attributes.itemByName('MARC','instance_id'):
            o.isGroundToParent=False
            o.isGrounded=False
    write(folder/'evidence/joint_axis_configuration.json',results)
    return {'status':'PASS','axes':results}
