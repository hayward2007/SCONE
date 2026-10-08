"""Render the same phase-zero trials with a centered camera; compare raw metrics."""
from pathlib import Path
import sys,json
import numpy as np
from PIL import Image,ImageDraw
from dataclasses import asdict
ROOT=Path(__file__).resolve().parents[3];sys.path.insert(0,str(ROOT))
from benchmark.common import SimulationTrial,MetricsRecorder,Perturbation
from benchmark.capture import FrameSink,CaptureConfig
from src.locomotion.scone_gait_v2 import SconeGaitV2,SconeGaitV2Config
from src.locomotion.tripod_gait import VelocityCommand
from src.simulation.core.cli_bridge import configure_model_gait_controller
from src.simulation.terrain import TerrainType
P=Path(__file__).resolve().parents[1]
class CenteredSink(FrameSink):
 def _render_frame(self):
  self.camera.lookat[:]=self.data.subtree_com[self.root_body_id]
  self.renderer.update_scene(self.data,camera=self.camera,scene_option=self.scene_option)
  im=Image.fromarray(self.renderer.render());draw=ImageDraw.Draw(im,'RGBA')
  draw.rectangle((0,0,self.config.width,44),fill=(0,0,0,210));draw.text((14,9),self.label,fill='white',font=self.font)
  return im
rows=[]
for name,legs in {'no_roll':[],'pair_12':[1,2],'corners':None}.items():
 with SimulationTrial(terrain=TerrainType.FLAT,perturbation=Perturbation(gait_phase=0)) as t:
  t.initialize();configure_model_gait_controller(t.controller)
  gait=SconeGaitV2(t.controller,t.robot.profile,config=SconeGaitV2Config(roll_legs=legs));gait.reset(phase=0)
  t.controller.set_positions({i+1:float(v) for i,v in enumerate(gait.nominal_motor_degrees)})
  for _ in range(50):gait.update(VelocityCommand(),dt=.02,send=True);t.advance(.02)
  metrics=MetricsRecorder(t,[.45,0,0]);start=float(t.data.time)
  t.model.vis.global_.offwidth=1280;t.model.vis.global_.offheight=720
  t.model.vis.headlight.ambient[:]=.35;t.model.vis.headlight.diffuse[:]=.8
  sink=CenteredSink(t.model,t.data,root_body_id=t.root_body_id,video_path=P/f'media/sim_{name}.mp4',image_path=P/f'figures/sim_{name}.jpg',label='',config=CaptureConfig(1280,720,25,23),camera_distance=1.5,camera_azimuth=140,camera_elevation=-32)
  sink.label=f'SIMULATION | {name} | command 0.45 m/s | 1x';sink.capture(force=True);sink.last_frame.save(P/f'figures/sim_{name}_start.png')
  class Recorder:
   def sample(self,dt):
    metrics.sample(dt)
    if t.data.time-start <8-1e-7:
     sink.label=f'SIMULATION | {name} | command 0.45 m/s | t={t.data.time-start:.2f}s | 1x';sink.capture()
  for k in range(400):
   state=gait.update(VelocityCommand(vx=.45),dt=.02,send=True)
   metrics.record_control(converged=state.converged,stride_clip_fraction=state.stride_clip_fraction,ik_backoff_scale=state.ik_backoff_scale);t.advance(.02,Recorder())
  sink.close();row={'condition':name,**asdict(metrics.finalize())};rows.append(row)
  ref=next(json.loads(l) for l in (P/'evidence/phase_grid.jsonl').read_text().splitlines() if json.loads(l)['condition']==name and json.loads(l)['vx_command']==.45 and json.loads(l)['phase']==0)
  row['speed_difference_from_original']=row['mean_vx_mps']-ref['mean_vx_mps'];assert abs(row['speed_difference_from_original'])<1e-9
  print(name,row['mean_vx_mps'],'replay matched',flush=True)
(P/'evidence/visual_replay.json').write_text(json.dumps(rows,indent=2))
