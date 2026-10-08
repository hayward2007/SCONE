from pathlib import Path
import json, numpy as np,mujoco,xml.etree.ElementTree as ET
from scipy.spatial import ConvexHull
from src.simulation.core.model import load_model
from benchmark.model_variants import transform_for_contact_geometry,CLOSED_WHEEL_CENTER
p=Path('archive/ICRA_2027_revision2_20260912/evidence'); payload=json.loads(Path('benchmark/assets/tire_coacd_2mm.json').read_text());parts=payload['parts'];c=np.array(CLOSED_WHEEL_CENTER)
m=load_model(floating_base=True);g=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,'TIRE_1_geom');i=m.geom_dataid[g];a=m.mesh_vertadr[i];n=m.mesh_vertnum[i];R=np.empty(9);mujoco.mju_quat2Mat(R,m.geom_quat[g]);v=m.mesh_vert[a:a+n].astype(float)@R.reshape(3,3).T+m.geom_pos[g]
def probe(groups,point):
 root=ET.Element('mujoco');asset=ET.SubElement(root,'asset');world=ET.SubElement(root,'worldbody')
 for j,points in enumerate(groups):
  ET.SubElement(asset,'mesh',name=f'p{j}',vertex=' '.join(str(x) for q in points for x in q))
  ET.SubElement(world,'geom',type='mesh',mesh=f'p{j}')
 body=ET.SubElement(world,'body',pos=' '.join(str(x) for x in point));ET.SubElement(body,'freejoint');ET.SubElement(body,'geom',type='sphere',size='.001',mass='1')
 model=mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'));data=mujoco.MjData(model);mujoco.mj_forward(model,data)
 return {'contact_count':data.ncon,'minimum_distance_m':min([float(data.contact[j].dist) for j in range(data.ncon)],default=None)}
vv=np.concatenate([x['vertices'] for x in parts]);direction=np.c_[np.zeros(720),np.cos(np.linspace(0,2*np.pi,720)),np.sin(np.linspace(0,2*np.pi,720))]
d=load_model(floating_base=True,xml_transform=transform_for_contact_geometry('decomposed-arc'))
r={'parts':len(parts),'threshold_m':.002,'axis_probe_original':probe([v],c),'axis_probe_decomposed':probe([x['vertices'] for x in parts],c),'mass_identical':bool(np.array_equal(m.body_mass,d.body_mass)),'inertia_identical':bool(np.array_equal(m.body_inertia,d.body_inertia)),'max_planar_support_function_change_m':float(abs((vv@direction.T).max(axis=0)-(v@direction.T).max(axis=0)).max()),'parts_total_volume_m3':float(sum(ConvexHull(x['vertices']).volume for x in parts)),'original_mesh_volume_m3':payload['input_volume_m3']}
(p/'decomposition_validation.json').write_text(json.dumps(r,indent=2));np.savez(p/'collision_vertices.npz',original=v,parts=vv,centre=c);print(json.dumps(r,indent=2))
