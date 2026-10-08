from pathlib import Path
import json,sys,hashlib
import numpy as np,mujoco,coacd,trimesh
from src.simulation.core.model import load_model
out=Path('benchmark/assets');out.mkdir(exist_ok=True)
m=load_model(floating_base=True);g=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,'TIRE_1_geom');mi=int(m.geom_dataid[g]);a=int(m.mesh_vertadr[mi]);n=int(m.mesh_vertnum[mi]);fa=int(m.mesh_faceadr[mi]);fn=int(m.mesh_facenum[mi]);v=m.mesh_vert[a:a+n].astype(float);f=m.mesh_face[fa:fa+fn].copy();rot=np.empty(9);mujoco.mju_quat2Mat(rot,m.geom_quat[g]);v=v@rot.reshape(3,3).T+m.geom_pos[g]
t=trimesh.Trimesh(v,f,process=True);print('mesh',t.is_watertight,t.is_winding_consistent,t.volume,len(t.vertices),len(t.faces),flush=True)
params=dict(threshold=.002,real_metric=True,seed=20260912,preprocess_mode='auto',resolution=2000,mcts_nodes=20,mcts_iterations=150,mcts_max_depth=3,merge=True)
parts=coacd.run_coacd(coacd.Mesh(t.vertices,t.faces),**params)
data={'generator':'CoACD 1.0.14','trimesh':'5.1.0','parameters':params,'input_watertight':bool(t.is_watertight),'input_volume_m3':float(t.volume),'input_vertices_sha256':hashlib.sha256(v.tobytes()+f.tobytes()).hexdigest(),'frame':'TIRE body coordinates, metres','parts':[{'vertices':pv.tolist(),'faces':pf.tolist()} for pv,pf in parts]}
p=out/'tire_coacd_2mm.json';p.write_text(json.dumps(data));print('saved',p,'pieces',len(parts),flush=True)
