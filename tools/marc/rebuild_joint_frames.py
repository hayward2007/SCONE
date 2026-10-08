"""Use explicit circular joint frames; Fusion ignores custom lines on as-built input."""
from fusion_common import *

def run(ctx):
    app,doc,d,folder,p=guard(ctx);root=d.rootComponent
    registry=json.loads((folder/'joints.json').read_text())
    if all(row.get('frame_source') for row in registry['joints']):
        return {'status':'PASS','action':'already configured; rerun G02 for validation'}
    expected_names={x['native_name'] for x in registry['joints']}
    assert {j.name for j in root.asBuiltJoints}==expected_names
    write(folder/'evidence/joints_before_frame_rebuild.json',registry)
    for j in list(root.asBuiltJoints)[::-1]:
        assert j.attributes.itemByName('MARC','joint_id')
        assert j.deleteMe()
    for o in list(root.occurrences):
        if o.attributes.itemByName('MARC','instance_id'):
            o.isGroundToParent=False;o.isGrounded=False
    occ={o.entityToken:o for o in root.occurrences}
    points=root.sketches.itemByName('MARC_12_axis_markers');result=[]
    for row in registry['joints']:
        pos=row['origin_world_mm'];axis=row['axis_world']; name=row['joint_id']+'_oriented_frame'
        if row['kind']=='yaw':
            x=(1,0,0);y=(0,1,0)
        else:
            x=(1,0,0);y=(0,0,-axis[1])
        ps=[]
        for delta in [(0,0,0),tuple(20*a for a in x),tuple(20*a for a in y)]:
            sp=points.sketchPoints.add(point(*(pos[k]+delta[k] for k in range(3))));sp.isFixed=True;ps.append(sp)
        inp=root.constructionPlanes.createInput();assert inp.setByThreePoints(*ps)
        plane=root.constructionPlanes.add(inp);plane.name=name;plane.isLightBulbOn=False
        sk=root.sketches.add(plane);sk.name=name+'_circle';sk.isVisible=False
        local=sk.modelToSketchSpace(point(*pos))
        circle=sk.sketchCurves.sketchCircles.addByCenterRadius(local,1)
        geo=fusion.JointGeometry.createByCurve(circle,fusion.JointKeyPointTypes.CenterKeyPoint)
        av=geo.primaryAxisVector
        assert av.dotProduct(vec(*axis))>.99999,(row['joint_id'],av.asArray(),axis)
        inp=root.asBuiltJoints.createInput(occ[row['child_token']],occ[row['parent_token']],geo)
        assert inp.setAsRevoluteJointMotion(fusion.JointDirections.ZAxisJointDirection)
        j=root.asBuiltJoints.add(inp);j.name=row['native_name'];attr(j,'joint_id',row['joint_id']);attr(j,'logical_reference_rad',row['q_reference_rad']);attr(j,'hardware_mapping','UNCONFIRMED')
        row['token']=j.entityToken;row['frame_source']='Oriented plane and circle; positive primary axis matches docs/30'
        result.append({'joint_id':row['joint_id'],'expected_axis':axis,'geometry_axis':av.asArray(),'native_axis':j.jointMotion.rotationAxisVector.asArray()})
        write(folder/'joints.json',registry)
    for o in list(root.occurrences):
        if o.attributes.itemByName('MARC','instance_id'):o.isGroundToParent=False;o.isGrounded=False
    write(folder/'evidence/joint_frame_rebuild.json',result)
    return {'status':'PASS','frames':result}
