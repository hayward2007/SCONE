"""Check twelve positive joint increments against explicit world-axis rigid transforms."""
from fusion_common import *
import adsk

def run(ctx):
    app,doc,d,folder,p=guard(ctx);root=d.rootComponent
    registry=json.loads((folder/'joints.json').read_text())
    instances=json.loads((folder/'cad/instances.json').read_text())
    actual={o.entityToken:o for o in root.occurrences}
    baseline=[]
    for data in instances:
        o=actual[data['token']];mat=o.transform2.asArray();exp=data['transform_cm']
        baseline.append({'instance_id':data['instance_id'],'max_matrix_error':max(abs(a-b) for a,b in zip(mat,exp))})
    results=[]
    for row in registry['joints']:
        j=root.asBuiltJoints.itemByName(row['native_name']); child=actual[row['child_token']]
        motion=fusion.RevoluteJointMotion.cast(j.jointMotion)
        before=child.transform2.copy();old=motion.rotationValue;eps=math.radians(1)
        affected=[v for v in instances if v['leg']==row['leg'] and (row['kind']=='yaw' or row['kind']=='q1' and v['kind'] in ('link','spin') or row['kind']=='q2' and v['kind']=='spin')]
        chain_before={v['instance_id']:actual[v['token']].transform2.copy() for v in affected}
        rotation=core.Matrix3D.create();rotation.setToRotation(eps,vec(*row['axis_world']),point(*row['origin_world_mm']))
        expected=before.copy();expected.transformBy(rotation)
        try:
            motion.rotationValue=old+eps;adsk.doEvents()
            after=child.transform2
            # Full transformed origin and all three basis axes, not a joint count check.
            ep=point(0,0,0);ep.transformBy(expected);ap=point(0,0,0);ap.transformBy(after)
            error=ep.distanceTo(ap)*10
            dots=[]
            for axis in [(1,0,0),(0,1,0),(0,0,1)]:
                ev=vec(*axis);av=vec(*axis);ev.transformBy(expected);av.transformBy(after);dots.append(ev.dotProduct(av))
            result={'joint_id':row['joint_id'],'increment_deg':1,'origin_error_mm':error,'basis_dot_min':min(dots),'axis_vector_api':motion.rotationAxisVector.asArray(),'status':'PASS' if error<=p['validation']['CAD_axis_distance_mm'] and min(dots)>=p['validation']['axis_dot_abs_min'] else 'FAIL'}
            chain_errors=[]
            for v in affected:
                predicted=chain_before[v['instance_id']].copy();predicted.transformBy(rotation)
                observed=actual[v['token']].transform2
                ep2=point(0,0,0);ap2=point(0,0,0);ep2.transformBy(predicted);ap2.transformBy(observed)
                position_error=ep2.distanceTo(ap2)*10
                dot_min=1.0
                for basis in [(1,0,0),(0,1,0),(0,0,1)]:
                    ev2=vec(*basis);av2=vec(*basis);ev2.transformBy(predicted);av2.transformBy(observed);dot_min=min(dot_min,ev2.dotProduct(av2))
                chain_errors.append({'instance_id':v['instance_id'],'origin_error_mm':position_error,'basis_dot_min':dot_min})
            result['affected_chain']=chain_errors
            if any(x['origin_error_mm']>p['validation']['CAD_axis_distance_mm'] or x['basis_dot_min']<p['validation']['axis_dot_abs_min'] for x in chain_errors):result['status']='FAIL'
            results.append(result)
        finally:
            motion.rotationValue=old;adsk.doEvents()
    reset=[]
    for data in instances:
        mat=actual[data['token']].transform2.asArray();exp=data['transform_cm']
        reset.append({'instance_id':data['instance_id'],'max_matrix_error':max(abs(a-b) for a,b in zip(mat,exp))})
    out={'step_id':'G02-CAD','status':'PASS' if all(r['status']=='PASS' for r in results) and max(x['max_matrix_error'] for x in reset)<1e-5 else 'FAIL','reference_transforms':baseline,'positive_direction_checks':results,'restored_transforms':reset,'hardware_status':'BLOCKED'}
    write(folder/'evidence/G02_kinematics.json',out)
    return out
