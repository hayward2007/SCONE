"""Reproduce the 2026-09-12 internal review; never sends hardware commands.
Run from repository root. Outputs fresh files; refuses to overwrite raw results.
This is a development audit, NOT the locked benchmark.icra evaluation profile.
"""
from pathlib import Path
import sys, json, hashlib, subprocess, platform, time
from dataclasses import asdict
import numpy as np
import mujoco
from scipy.spatial import ConvexHull
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT))
OUT=Path(__file__).resolve().parent
from benchmark.common import SimulationTrial, MetricsRecorder, BenchmarkConfig, Perturbation, source_revision, simulation_provenance
from benchmark.flat import run_flat_trial
from benchmark.model_variants import CLOSED_WHEEL_CENTER, transform_for_contact_geometry
from src.locomotion.scone_gait_v2 import SconeGaitV2,SconeGaitV2Config
from src.locomotion.tripod_gait import TripodGait,GaitConfig,VelocityCommand
from src.simulation.core.cli_bridge import configure_model_gait_controller
from src.simulation.core.model import load_model
from src.simulation.terrain import TerrainType

# Freeze inputs before any numerical outcome is viewed.
paths=sorted([p for folder in ('src','benchmark','tests','packages') for p in (ROOT/folder).rglob('*') if p.is_file() and p.suffix in ('.py','.xml','.stl','.toml') and '__pycache__' not in str(p)])
hashes={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
protocol={'date':'2026-09-12','purpose':'development audit, deterministic initial-phase grid; not independent random trials or held-out evaluation','commands_mps':[0.18,0.45],'phases':[i/8 for i in range(8)],'conditions':{'no_roll':[],'pair_12':[1,2],'pair_56':[5,6],'corners':None,'tripod_equal_budget':'TripodGait with same cadence/duty/stride/command caps'},'settle_s':1.0,'measure_s':8.0,'physics_dt_s':0.002,'control_dt_s':0.02,'gait_config':asdict(SconeGaitV2Config()),'python':platform.python_version(),'numpy':np.__version__,'mujoco':mujoco.__version__,'source_files_sha256':hashes,**source_revision(),**simulation_provenance()}
with (OUT/'audit_protocol.json').open('x') as f:json.dump(protocol,f,indent=2)

model=load_model(floating_base=True)
g=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,'TIRE_1_geom')
mesh=int(model.geom_dataid[g]);a=int(model.mesh_vertadr[mesh]);n=int(model.mesh_vertnum[mesh]);vertices=model.mesh_vert[a:a+n].astype(float)
rot=np.empty(9);mujoco.mju_quat2Mat(rot,model.geom_quat[g]);body_vertices=vertices@rot.reshape(3,3).T+model.geom_pos[g]
hull=ConvexHull(body_vertices)
centre=np.array(CLOSED_WHEEL_CENTER)
max_equation=float(np.max(hull.equations[:,:3]@centre+hull.equations[:,3]))
closed=load_model(floating_base=True,xml_transform=transform_for_contact_geometry('closed-wheel'))
active=[mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_GEOM,i) for i in range(model.ngeom) if model.geom_contype[i] or model.geom_conaffinity[i]]
geom={'nbody_including_world':model.nbody,'nu':model.nu,'nq':model.nq,'nv':model.nv,'mass_kg':float(model.body_mass.sum()),'hinge_count':int(sum(model.jnt_type==mujoco.mjtJoint.mjJNT_HINGE)),'limited_hinges':int(sum(model.jnt_limited[model.jnt_type==mujoco.mjtJoint.mjJNT_HINGE])),'active_collision_geoms':active,'tire_geoms_per_leg':1,'tire_mesh_vertex_count':n,'tire_axis_centre_in_convex_hull':max_equation<0,'axis_centre_max_hull_equation_m':max_equation,'closed_variant_mass_equal':bool(np.array_equal(model.body_mass,closed.body_mass)),'closed_variant_inertia_equal':bool(np.array_equal(model.body_inertia,closed.body_inertia)),'tire_geom_contype':int(model.geom_contype[g]),'tire_geom_conaffinity':int(model.geom_conaffinity[g]),'tire_geom_solref':model.geom_solref[g].tolist(),'interpretation':'One convex mesh collision proxy per tire fills the interior concavity. Missing circular cap remains; this is not a full circle. It cannot establish inner-arc hooking.'}
(OUT/'geometry_audit.json').write_text(json.dumps(geom,indent=2))
np.savez(OUT/'tire_geometry.npz',vertices=body_vertices,centre=centre)
print('GEOMETRY',json.dumps(geom),flush=True)

raw=OUT/'phase_grid.jsonl'
with raw.open('x') as f:
 for vx in protocol['commands_mps']:
  for phase in protocol['phases']:
   for condition,legs in protocol['conditions'].items():
    begin=time.monotonic()
    try:
     with SimulationTrial(terrain=TerrainType.FLAT,perturbation=Perturbation(gait_phase=phase)) as t:
      t.initialize();configure_model_gait_controller(t.controller)
      cfg=SconeGaitV2Config(roll_legs=legs) if condition!='tripod_equal_budget' else GaitConfig(**{k:v for k,v in asdict(SconeGaitV2Config()).items() if k in GaitConfig.__dataclass_fields__})
      gait=(SconeGaitV2 if condition!='tripod_equal_budget' else TripodGait)(t.controller,t.robot.profile,config=cfg)
      gait.reset(phase=phase)
      nominal=gait.nominal_motor_degrees
      t.controller.set_positions({i+1:float(v) for i,v in enumerate(nominal)})
      for _ in range(50):gait.update(VelocityCommand(),dt=.02,send=True);t.advance(.02)
      metrics=MetricsRecorder(t,[vx,0,0]);start=t.data.xpos[t.root_body_id].copy();R=t.data.xmat[t.root_body_id].reshape(3,3).copy();velocity_integral=np.zeros(3);load_min=6
      class Recorder:
       def sample(self,dt):
        nonlocal_dummy=None
        metrics.sample(dt)
        out=np.zeros(6);mujoco.mj_objectVelocity(t.model,t.data,mujoco.mjtObj.mjOBJ_BODY,t.root_body_id,out,0)
        velocity_integral[:]+=out[3:]*dt
      rec=Recorder()
      scheduled_min=6
      for _ in range(400):
       s=gait.update(VelocityCommand(vx=vx),dt=.02,send=True)
       scheduled_min=min(scheduled_min,len(s.stance_legs))
       metrics.record_control(converged=s.converged,stride_clip_fraction=s.stride_clip_fraction,ik_backoff_scale=s.ik_backoff_scale)
       t.advance(.02,rec)
       if metrics.termination_reason:break
      m=asdict(metrics.finalize());end=t.data.xpos[t.root_body_id].copy()
      # Independent velocity integral vs net displacement in the initial frame.
      m.update({'condition':condition,'vx_command':vx,'phase':phase,'scheduled_stance_min':scheduled_min,'velocity_integral_forward_mps':float((R.T@velocity_integral)[0]/m['duration_s']),'position_difference_forward_mps':float((R.T@(end-start))[0]/m['duration_s']),'source_snapshot':'audit_protocol.json','rolling_legs_final':list(gait.rolling_legs) if hasattr(gait,'rolling_legs') else []})
    except Exception as e:m={'condition':condition,'vx_command':vx,'phase':phase,'error':type(e).__name__+': '+str(e)}
    f.write(json.dumps(m)+'\n');f.flush()
    print(condition,vx,phase,round(m.get('mean_vx_mps',0),4),m.get('completed',m.get('error')),round(time.monotonic()-begin,2),flush=True)
# Diagnostic replay of the two older adapter families: retained separately.
legacy=[]
for name in ['articulated-walk','distal-only-roll','full-roll','matched-articulated','matched-distal-only','matched-coordinated']:
 r=run_flat_trial(name,[.18,0,0],config=BenchmarkConfig(settle_seconds=1,measure_seconds=6),perturbation=Perturbation(gait_phase=0));legacy.append(r)
 print('LEGACY',name,r['mean_vx_mps'],flush=True)
(OUT/'legacy_adapter_audit.json').write_text(json.dumps(legacy,indent=2))
(OUT/'source_stability.json').write_text(json.dumps({'changed_during_run':[p for p,h in hashes.items() if hashlib.sha256((ROOT/p).read_bytes()).hexdigest()!=h]},indent=2))
