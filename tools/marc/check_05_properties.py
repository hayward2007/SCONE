"""Mass, local center-of-mass tensors, and explicit missing physical inputs."""
from fusion_common import *

def run(ctx):
    app,doc,d,folder,p=guard(ctx);root=d.rootComponent
    assert json.loads((folder/'evidence/B05_inertia_units.json').read_text())['status']=='PASS'
    parts=[];summary={};partial_com=[0,0,0];total=0
    for o in root.occurrences:
        if o.component.name not in ['Frame_D0','Yaw_Module_D0','Crank_Link_D0','Spin_Module_D0']:continue
        aid=o.attributes.itemByName('MARC','instance_id');instance=aid.value if aid else 'BODY'
        for b in o.component.bRepBodies:
            ph=b.physicalProperties;mass=ph.mass;vol=ph.volume;com=ph.centerOfMass;cs=com.asArray()
            raw=ph.getXYZMomentsOfInertia();assert raw[0]
            xx,yy,zz,xy,yz,xz=raw[1:];tensor=[[xx,xy,xz],[xy,yy,yz],[xz,yz,zz]]
            inert=[[1e-4*(tensor[i][j]-mass*((sum(v*v for v in cs) if i==j else 0)-cs[i]*cs[j])) for j in range(3)] for i in range(3)]
            density=b.material.materialProperties.itemById('structural_Density').value
            assert abs(mass-vol*density/1e6)<1e-7
            assert all(inert[k][k]>0 for k in range(3))
            world=com.copy();world.transformBy(o.transform2);cw=[x/100 for x in world.asArray()]
            part_id=b.attributes.itemByName('MARC','part_id').value
            parts.append({'part_id':part_id,'instance_id':instance+'/'+part_id,'revision':'rev4-D0','role':'structure_or_tire','mass_source':b.attributes.itemByName('MARC','mass_source').value,'mass_kg':mass,'volume_cm3':vol,'density_kg_m3':density,'com_local_m':[x/100 for x in cs],'com_world_m':cw,'inertia_com_local_kgm2':inert,'geometry_hash':None,'input_sha256':ctx['params_sha256'],'geometry_hash_scope':'Per-body BRep hash not available; complete native archive hash is in cad/document.json','body_token':b.entityToken,'occurrence_token':o.entityToken,'source_file':'cad/MARC-PARAM-D0.f3d','geometry_status':'ENVELOPE_INTERFACE_UNRESOLVED' if 'Yoke' in part_id else 'D0_SOLID'})
            total+=mass
            for k in range(3):partial_com[k]+=mass*cw[k]
            summary.setdefault(part_id,{'count':0,'mass_kg_each':mass,'subtotal_kg':0})
            summary[part_id]['count']+=1;summary[part_id]['subtotal_kg']+=mass
    for leg_id,sx,sy in [('FL',1,1),('FR',1,-1),('RL',-1,1),('RR',-1,-1)]:
        for kind,model in [('yaw','MX-28AT'),('q1','XM430-W350-T'),('q2','XM430-W210-T')]:
            parts.append({'part_id':model,'instance_id':leg_id+'/'+kind+'_motor','revision':None,'role':'actuator','mass_source':'DATASHEET','mass_kg':p['actuator_catalog'][model]['mass_g']/1000,'com_local_m':None,'inertia_com_local_kgm2':None,'geometry_hash':None,'source_file':'evidence/design_plan_snapshot.md section 8.4','geometry_status':'MISSING','blocking_input':'I02'})
    for row in json.loads((folder/'cad/payload_layout.json').read_text()):
        parts.append({'part_id':row['part_id'],'instance_id':'BODY/'+row['part_id'],'revision':None,'role':'payload','mass_source':'DATASHEET','mass_kg':row['mass_kg'],'com_local_m':None,'inertia_com_local_kgm2':None,'geometry_hash':None,'source_file':'evidence/design_plan_snapshot.md section 8.1 and 9.2','geometry_status':'ENVELOPE','blocking_input':'I05' if row['part_id'] not in ('battery','SPDB') else 'I06'})
    missing=['horns','bearings','shafts','deck_standoffs','sensor_mounts','fork_final','fasteners','wiring','converters','power_switches_and_protection','battery_door_and_rails']
    for name in missing:parts.append({'part_id':name,'instance_id':'UNRESOLVED/'+name,'revision':None,'role':'unresolved','mass_source':None,'mass_kg':None,'com_local_m':None,'inertia_com_local_kgm2':None,'geometry_hash':None,'source_file':None,'geometry_status':'MISSING'})
    out={'stage':'D0','status':'BLOCKED','parts':parts,'cad_structure_subtotal_kg':total,'cad_structure_com_world_m':[x/total for x in partial_com],'known_mass_subtotal_kg':sum(x['mass_kg'] for x in parts if x['mass_kg'] is not None),'missing_mass_count':sum(x['mass_kg'] is None for x in parts),'missing_inertia_count':sum(x['inertia_com_local_kgm2'] is None for x in parts),'by_structure_part':summary,'excluded':['study fork envelopes','mast space','battery slot space','payload envelope default CAD density'],'reason':'Partial BOM only; yoke envelopes overlap and physical interfaces are unresolved. Do not treat the subtotal as final robot mass.'}
    write(folder/'parts_manifest.json',out)
    return {k:v for k,v in out.items() if k not in ('parts','by_structure_part')}
