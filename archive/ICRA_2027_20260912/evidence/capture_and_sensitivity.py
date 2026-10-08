"""Supplemental timestep check and actual rendered replay; simulation only.
Run with mjpython on macOS from the repository root. Existing evidence is retained.
"""
from pathlib import Path
import sys,json,hashlib
from dataclasses import asdict
import numpy as np
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT))
from benchmark.common import SimulationTrial,MetricsRecorder,Perturbation
from benchmark.capture import FrameSink,CaptureConfig
from src.locomotion.scone_gait_v2 import SconeGaitV2,SconeGaitV2Config
from src.locomotion.tripod_gait import VelocityCommand
from src.simulation.core.cli_bridge import configure_model_gait_controller
from src.simulation.terrain import TerrainType
P=Path(__file__).resolve().parents[1]
spec={'purpose':'post-review numerical sensitivity and visual replay; not held-out evaluation','dt_s':[.001,.002,.004],'conditions':{'no_roll':[],'pair_12':[1,2],'corners':None},'phase':0,'vx':.45,'settle_s':1,'measure_s':8,'control_dt_s':.02,'video':{'width':1280,'height':720,'fps':25,'speed':1},'source_manifest':'audit_protocol.json'}
with (P/'evidence/sensitivity_protocol.json').open('x') as f:json.dump(spec,f,indent=2)
protocol=json.loads((P/'evidence/audit_protocol.json').read_text())
assert all(hashlib.sha256((ROOT/p).read_bytes()).hexdigest()==h for p,h in protocol['source_files_sha256'].items())
with (P/'evidence/sensitivity.jsonl').open('x') as f:
 for dt in spec['dt_s']:
  for name,legs in spec['conditions'].items():
   with SimulationTrial(terrain=TerrainType.FLAT,perturbation=Perturbation(gait_phase=0)) as t:
    t.model.opt.timestep=dt
    t.initialize();configure_model_gait_controller(t.controller)
    gait=SconeGaitV2(t.controller,t.robot.profile,config=SconeGaitV2Config(roll_legs=legs));gait.reset(phase=0)
    t.controller.set_positions({i+1:float(v) for i,v in enumerate(gait.nominal_motor_degrees)})
    for _ in range(50):gait.update(VelocityCommand(),dt=.02,send=True);t.advance(.02)
    metrics=MetricsRecorder(t,[.45,0,0]);sink=None;start=float(t.data.time)
    if dt==.002:
     t.model.vis.global_.offwidth=1280;t.model.vis.global_.offheight=720
     sink=FrameSink(t.model,t.data,root_body_id=t.root_body_id,video_path=P/f'media/sim_{name}.mp4',image_path=P/f'figures/sim_{name}.jpg',label='',config=CaptureConfig(1280,720,25,24),camera_distance=1.9,camera_azimuth=140,camera_elevation=-30)
     sink.label=f'SIMULATION | {name} | command 0.45 m/s | 1x';sink.capture(force=True)
     sink.last_frame.save(P/f'figures/sim_{name}_start.png')
    class Recorder:
     def sample(self,step):
      metrics.sample(step)
      if sink and float(t.data.time)-start < 8-1e-7:
       sink.label=f'SIMULATION | {name} | command 0.45 m/s | t={t.data.time-start:.2f}s | 1x'
       sink.capture()
    for k in range(400):
     state=gait.update(VelocityCommand(vx=.45),dt=.02,send=True)
     metrics.record_control(converged=state.converged,stride_clip_fraction=state.stride_clip_fraction,ik_backoff_scale=state.ik_backoff_scale)
     t.advance(.02,Recorder())
     if metrics.termination_reason:break
    if sink:sink.close()
    row={'condition':name,'physics_dt_s':dt,**asdict(metrics.finalize())}
    f.write(json.dumps(row)+'\n');f.flush();print(name,dt,row['mean_vx_mps'],row['completed'],flush=True)
