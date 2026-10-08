"""Four-leg occurrence transforms, twelve native revolute joints and D0 payload spaces."""
from fusion_common import *

LEGS=[('FL',1,1),('FR',1,-1),('RL',-1,1),('RR',-1,-1)]

def run(ctx):
    app,doc,d,folder,p=guard(ctx);root=d.rootComponent;l=p['leg'];body=p['body']
    if (folder/'evidence/B06_assembly.json').exists() and (folder/'joints.json').exists():
        reg=json.loads((folder/'joints.json').read_text())
        if len(reg['joints'])==12 and all(x.get('frame_source') for x in reg['joints']) and root.asBuiltJoints.count==12:
            return {'step_id':'B06','status':'PASS','action':'Owned assembly already exists; run G02 to revalidate, no duplicate geometry created'}
    owned=json.loads((folder/'cad/owned_components.json').read_text())
    bycomp={o.component.name:o for o in root.occurrences}
    templates={'yaw':bycomp['Yaw_Module_D0'],'link':bycomp['Crank_Link_D0'],'spin':bycomp['Spin_Module_D0']}
    calc=json.loads((folder/'calculations.json').read_text())['derived'];q=math.radians(calc['q1_reference_deg']);cq=math.cos(q);sq=math.sin(q)
    frame=bycomp['Frame_D0']
    instances=[]
    for li,(leg_id,sx,sy) in enumerate(LEGS):
        transforms={
          'yaw':matrix((sx*l['steer_axis_x_mm'],sy*body['track_half_y_mm'],l['steer_axis_z_mm']),(sx,0,0),(0,sx,0),(0,0,1)),
          'link':matrix((sx*l['stage1_axis_x_mm'],sy*body['track_half_y_mm'],0),(sx*cq,0,sq),(-sx*sq,0,cq),(0,-sx,0)),
          'spin':matrix((sx*(l['stage1_axis_x_mm']+calc['arm_mm']),sy*body['track_half_y_mm'],calc['axle_z_mm']),(sx,0,0),(0,0,1),(0,-sx,0))}
        for kind in ('yaw','link','spin'):
            key=leg_id+'_'+kind.upper()
            matches=[o for o in root.occurrences if o.attributes.itemByName('MARC','instance_id') and o.attributes.itemByName('MARC','instance_id').value==key]
            if matches:
                assert len(matches)==1;o=matches[0]
            else:
                o=root.occurrences.addExistingComponent(templates[kind].component,transforms[kind]);attr(o,'instance_id',key)
            instances.append({'instance_id':key,'leg':leg_id,'kind':kind,'token':o.entityToken,'component_token':o.component.entityToken,'transform_cm':o.transform2.asArray()})
    write(folder/'cad/instances.json',instances)
    if 'Payload_ENVELOPES_D0' not in bycomp:
        po=component(root,'Payload_ENVELOPES_D0');attr(po,'exclude_mass','true'); po.isGrounded=True
        owned['payload']={'token':po.component.entityToken,'occurrence_token':po.entityToken}
        entries=[('battery',(0,95,3),(43.1,91.6,23.6),'center',0.204),('U2D2',(0,-60,3),(48,18,14.9),'center',0.009),('SPDB',(0,-100,32),(70,90,22),'base',None),('Jetson',(0,68,32),(100,80,29),'base',0.145),('lidar',(0,-20,70),(38.6,38.6,33.9),'base',0.045)]
        payload=[]
        for name,pos,size,origin,mass in entries:
            x,y,z=pos;wx,wy,wz=size
            bounds=(x-wx/2,x+wx/2,y-wy/2,y+wy/2,z-wz/2 if origin=='center' else z,z+wz/2 if origin=='center' else z+wz)
            b=add_base(po.component,name+'_ENVELOPE',box_temp(bounds));attr(b,'part_id',name);attr(b,'mass_source','DATASHEET');attr(b,'mass_kg',mass);attr(b,'exclude_CAD_mass','true')
            payload.append({'part_id':name,'position_mm':pos,'size_mm':size,'origin':origin,'mass_kg':mass,'geometry_status':'ENVELOPE','inertia_com_local_kgm2':None,'com_local_m':None})
        # The stereo uses optical = origin, local x = width(+y), local y = board up.
        stereo=component(po.component,'Stereo_OPTICAL_ENVELOPE',matrix((70,0,70),(0,1,0),(.5,0,math.sqrt(3)/2),(math.sqrt(3)/2,0,-.5)))
        b=add_base(stereo.component,'IMX219_83_ENVELOPE',box_temp((-42.6,42.6,-12,12,-10.15,10.15)));attr(b,'part_id','stereo');attr(b,'mass_kg',.030);attr(b,'exclude_CAD_mass','true')
        payload.append({'part_id':'stereo','position_mm':[70,0,70],'size_width_height_depth_mm':[85.2,24,20.3],'origin':'optical','axis':[math.sqrt(3)/2,0,-.5],'mass_kg':.03,'geometry_status':'ENVELOPE','inertia_com_local_kgm2':None,'com_local_m':None})
        study=bycomp['_STUDY_D0_UNRESOLVED'].component
        for name,bounds in [('battery_slot_SPACE',(-25,25,45,145,-13,19)),('mast_SPACE',(-30,30,-50,10,30,70))]:
            b=add_base(study,name,box_temp(bounds));attr(b,'exclude_mass','true')
        write(folder/'cad/payload_layout.json',payload)
        write(folder/'cad/owned_components.json',owned)
    # Joint geometry is persistent sketch point / line, with explicit world axes.
    sk=root.sketches.itemByName('MARC_12_axis_markers') or root.sketches.add(root.xYConstructionPlane)
    sk.name='MARC_12_axis_markers';sk.isVisible=False
    joints=[]
    for i,(leg_id,sx,sy) in enumerate(LEGS):
        occ={kind:next(o for o in root.occurrences if o.attributes.itemByName('MARC','instance_id') and o.attributes.itemByName('MARC','instance_id').value==leg_id+'_'+kind.upper()) for kind in ('yaw','link','spin')}
        definitions=[('yaw',i+1,frame,occ['yaw'],(sx*l['steer_axis_x_mm'],sy*body['track_half_y_mm'],l['steer_axis_z_mm']),(0,0,1),0),
                     ('q1',i+5,occ['yaw'],occ['link'],(sx*l['stage1_axis_x_mm'],sy*body['track_half_y_mm'],0),(0,-sx,0),q),
                     ('q2',i+9,occ['link'],occ['spin'],(sx*(l['stage1_axis_x_mm']+calc['arm_mm']),sy*body['track_half_y_mm'],calc['axle_z_mm']),(0,-sx,0),-q)]
        for kind,num,parent,child,pos,axis,zero in definitions:
            name=f'M{num:02}_{leg_id}_{kind}'
            j=root.asBuiltJoints.itemByName(name)
            if j is None:
                start=sk.sketchPoints.add(point(*pos));start.isFixed=True
                end=sk.sketchPoints.add(point(*(pos[k]+20*axis[k] for k in range(3))));end.isFixed=True
                line=sk.sketchCurves.sketchLines.addByTwoPoints(start,end);line.isConstruction=True
                geometry=fusion.JointGeometry.createByPoint(start)
                inp=root.asBuiltJoints.createInput(child,parent,geometry)
                assert inp.setAsRevoluteJointMotion(fusion.JointDirections.CustomJointDirection,line)
                j=root.asBuiltJoints.add(inp);j.name=name
                attr(j,'logical_reference_rad',zero);attr(j,'joint_id',f'M{num:02}');attr(j,'hardware_mapping','UNCONFIRMED')
            joints.append({'joint_id':f'M{num:02}','leg':leg_id,'kind':kind,'native_name':j.name,'token':j.entityToken,'parent_token':parent.entityToken,'child_token':child.entityToken,'origin_world_mm':pos,'axis_world':axis,'q_reference_rad':zero,'fusion_drive_zero_rad':0,'roll_forward_sign':-sx if kind=='q2' else None,'hardware_id':None,'hardware_sign':None,'physical_limits':None})
            write(folder/'joints.json',{'order':[f'M{i:02}' for i in range(1,13)],'joints':sorted(joints,key=lambda x:x['joint_id']),'stage':'D0','hardware_status':'BLOCKED'})
    assert root.asBuiltJoints.count==12
    # The installed Fusion build ignores custom-axis lines on AsBuiltJointInput.
    # Rebuild with explicit circular frames before claiming the assembly step complete.
    current_registry=json.loads((folder/'joints.json').read_text())
    if not all(row.get('frame_source') for row in current_registry['joints']):
        import rebuild_joint_frames
        rebuild_joint_frames.run(ctx)
    # Color by function; these are visual overrides, not material substitutions.
    source=d.materials.itemByName('MARC_v4_PLA_D0').appearance
    colors={'Sector_blue':(40,111,183),'Tire_graphite':(40,43,47),'Structure_gray':(165,177,188),'Link_gray':(88,102,117),'Payload_green':(43,121,91),'Battery_amber':(218,146,44),'Study_amber':(234,153,51)}
    appearances={}
    for name,rgb in colors.items():
        a=d.appearances.itemByName(name) or d.appearances.addByCopy(source,name)
        for pr in a.appearanceProperties:
            if pr.id in ('opaque_albedo','surface_albedo'):
                core.ColorProperty.cast(pr).value=core.Color.create(*rgb,255)
        appearances[name]=a
    for c in d.allComponents:
        for b in c.bRepBodies:
            name=b.name
            color='Sector_blue' if 'PLA_sector' in name else 'Tire_graphite' if 'TPU_tire' in name else 'Link_gray' if 'crank' in name else 'Battery_amber' if 'battery_ENVELOPE' in name else 'Payload_green' if 'ENVELOPE' in name else 'Structure_gray'
            b.appearance=appearances[color]
    app.activeViewport.fit();app.activeViewport.refresh()
    result={'step_id':'B06','status':'PASS','legs':4,'native_revolute_joints':root.asBuiltJoints.count,'motor_geometry':'MISSING: coordinate markers only','reference_angles_rad':{'yaw':0,'q1':q,'q2':-q},'sign_validation':'NOT_RUN'}
    write(folder/'evidence/B06_assembly.json',result)
    return result
