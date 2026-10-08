"""MARC rev4 D0 crank, hollow frame, yokes and clearly marked study envelopes."""
from fusion_common import *

def shell_box(bounds,t):
    return boolean(box_temp(bounds),box_temp(tuple(v+t if i%2==0 else v-t for i,v in enumerate(bounds))),'cut')

def miter_profile(a,b,gamma,h,extend=0):
    ca=math.cos(gamma);sa=math.sin(gamma)
    px=a+b*ca;py=b*sa
    # Intersection of parallel offset lines at the crank bend.
    k=math.tan(gamma/2)
    return [(-extend,-h),(a+h*k,-h),(px+h*sa+extend*ca,py-h*ca+extend*sa),
            (px-h*sa+extend*ca,py+h*ca+extend*sa),(a-h*k,h),(-extend,h)]

def run(ctx):
    app,doc,d,folder,p=guard(ctx);root=d.rootComponent;g=p['geometry'];l=p['leg'];body=p['body']
    assert not any(o.component.name=='Frame_D0' for o in root.occurrences), 'B04 already started; inspect before retry'
    owned=json.loads((folder/'cad/owned_components.json').read_text())
    pla=d.materials.itemByName('MARC_v4_PLA_D0')
    def new(key,name,m=None):
        o=component(root,name,m)
        owned[key]={'token':o.component.entityToken,'occurrence_token':o.entityToken}
        write(folder/'cad/owned_components.json',owned)
        return o
    o=new('frame','Frame_D0'); c=o.component
    hx=body['frame_x_mm']/2;hy=body['track_half_y_mm']+body['frame_end_allowance_mm']/2
    w=body['frame_beam_w_mm'];t=body['frame_beam_wall_mm'];z=body['frame_bottom_z_mm'];h=body['frame_beam_h_mm']
    ring=boolean(box_temp((-hx,hx,-hy,hy,z,z+h)),box_temp((-hx+w,hx-w,-hy+w,hy-w,z-1,z+h+1)),'cut')
    cavity=boolean(box_temp((-hx+t,hx-t,-hy+t,hy-t,z+t,z+h-t)),box_temp((-hx+w-t,hx-w+t,-hy+w-t,hy-w+t,z-1,z+h+1)),'cut')
    boolean(ring,cavity,'cut')
    cr=body['cross_rib_t_mm']
    for y in [-body['track_half_y_mm'],0,body['track_half_y_mm']]: boolean(ring,box_temp((-hx+w,hx-w,y-cr/2,y+cr/2,z,z+h)))
    for x in [-l['steer_axis_x_mm'],l['steer_axis_x_mm']]: boolean(ring,box_temp((x-cr/2,x+cr/2,-hy+w,hy-w,z,z+h)))
    boolean(ring,box_temp((-hx+w,hx-w,-hy+w,hy-w,body['frame_plate_bottom_z_mm'],body['frame_plate_bottom_z_mm']+body['frame_plate_t_mm'])))
    b=add_base(c,'Frame_box_beams_cross_and_long_ribs',ring,pla);attr(b,'part_id','frame_PLA');attr(b,'mass_source','CAD_BULK')
    z=body['deck_bottom_z_mm'];b=add_base(c,'Deck_D0_NO_STANDOFFS',box_temp((-hx,hx,-hy,hy,z,z+body['deck_t_mm'])),pla);attr(b,'part_id','deck_PLA');attr(b,'mass_source','CAD_BULK')
    o.isGrounded=True
    o=new('yaw','Yaw_Module_D0',matrix((l['steer_axis_x_mm'],body['track_half_y_mm'],l['steer_axis_z_mm'])))
    attr(o,'instance_id','FL_YAW');c=o.component;zz=-l['steer_axis_z_mm'];reach=l['stage1_axis_x_mm']-l['steer_axis_x_mm']
    for name,bounds in [('Yoke1_D0',(0,44,-16.5,16.5,zz-13,zz+13)),('Yoke2_D0',(reach-41,reach,-15.5,15.5,zz-12,zz+12))]:
        b=add_base(c,name,shell_box(bounds,l['wall_yoke_mm']),pla);attr(b,'part_id',name);attr(b,'mass_source','CAD_BULK');attr(b,'geometry_status','ENVELOPE_INTERFACE_UNRESOLVED')
    q=math.radians(json.loads((folder/'calculations.json').read_text())['derived']['q1_reference_deg']);cq=math.cos(q);sq=math.sin(q)
    o=new('link','Crank_Link_D0',matrix((l['stage1_axis_x_mm'],body['track_half_y_mm'],0),(cq,0,sq),(-sq,0,cq),(0,-1,0)))
    attr(o,'instance_id','FL_LINK');c=o.component
    gamma=math.radians(l['gamma_crank_deg']);a=l['link_a_mm'];b=l['link_b_mm'];h=l['box_h_mm']/2
    sk=sketch_poly(c,'Crank_outer_miter',miter_profile(a,b,gamma,h))
    outer=extrude(c,sk,'Crank_outer_closed_section',l['box_w_mm']).bodies.item(0)
    sk=sketch_poly(c,'Crank_inner_miter_open_ends',miter_profile(a,b,gamma,h-l['wall_link_mm'],1))
    extrude(c,sk,'Crank_hollow_open_only_at_ends',l['box_w_mm']-2*l['wall_link_mm'],0,fusion.FeatureOperations.CutFeatureOperation,[outer])
    bb=c.bRepBodies.item(0);bb.name='PLA_crank_90_110_25deg';bb.material=pla;attr(bb,'part_id','crank_PLA');attr(bb,'mass_source','CAD_BULK')
    o=new('study','_STUDY_D0_UNRESOLVED');o.isLightBulbOn=False;study=o.component
    # Fork e/f plane is rotated by q1+gamma; local z is -v.
    e=q+gamma;ce=math.cos(e);se=math.sin(e)
    for leg_id,sx,sy in [('FL',1,1),('FR',1,-1),('RL',-1,1),('RR',-1,-1)]:
        arm=json.loads((folder/'calculations.json').read_text())['derived']['arm_mm']
        pos=(sx*(l['stage1_axis_x_mm']+arm),sy*body['track_half_y_mm'],-150.5)
        fo=component(study,'Fork_'+leg_id+'_ENVELOPE',matrix(pos,(sx*ce,0,se),(-sx*se,0,ce),(0,-sx,0)))
        for name,bounds in [('Arm_A',(-22,22,-22,22,12,17)),('Arm_B',(-22,22,-22,22,-17,-12)),('Bridge',(-27,-22,-22,22,-17,17))]:
            b=add_base(fo.component,name,box_temp(bounds));attr(b,'exclude_mass','true');attr(b,'geometry_status','STUDY_ONLY')
    result={'step_id':'B04','status':'PASS','frame_mm':[2*hx,2*hy],'frame_bodies':2,'crank_solid':bb.isSolid,'crank_volume_cm3':bb.physicalProperties.volume,'interfaces':'BLOCKED: motor, yoke overlap, fork/link overlap and deck supports unresolved'}
    write(folder/'evidence/B04_structure.json',result)
    app.activeViewport.fit();app.activeViewport.refresh()
    return result
