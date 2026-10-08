"""Render frozen phase-zero trials; run with mjpython from the frozen source cwd."""
from pathlib import Path
import sys,json
sys.path.insert(0,str(Path.cwd()))
from dataclasses import asdict
from PIL import Image,ImageDraw
from benchmark.common import SimulationTrial,MetricsRecorder,Perturbation
from benchmark.controllers import make_controller
from benchmark.capture import FrameSink,CaptureConfig
from src.locomotion import VelocityCommand
P=Path(__file__).resolve().parents[1]
class CenteredSink(FrameSink):
 def _render_frame(self):
  self.camera.lookat[:]=self.data.subtree_com[self.root_body_id]
  self.renderer.update_scene(self.data,camera=self.camera,scene_option=self.scene_option)
  im=Image.fromarray(self.renderer.render());d=ImageDraw.Draw(im,'RGBA')
  d.rectangle((0,0,self.config.width,44),fill=(0,0,0,215));d.text((14,9),self.label,fill='white',font=self.font)
  return im
rows=[]
for code,name in [('N','role-split-no-roll'),('U','role-split-scone'),('B','rewind-budget-scone')]:
 with SimulationTrial(contact_geometry='decomposed-arc',perturbation=Perturbation(gait_phase=0)) as t:
  t.initialize();a=make_controller(name,t,phase=0);a.prepare(t)
  for _ in range(50):a.update(VelocityCommand(),.02);t.advance(.02)
  rec=MetricsRecorder(t,[.45,0,0]);start=float(t.data.time)
  t.model.vis.global_.offwidth=1280;t.model.vis.global_.offheight=720
  t.model.vis.headlight.ambient[:]=.35;t.model.vis.headlight.diffuse[:]=.8
  sink=CenteredSink(t.model,t.data,root_body_id=t.root_body_id,video_path=P/f'media/sim_{code}.mp4',image_path=P/f'figures/sim_{code}.jpg',label='',config=CaptureConfig(1280,720,25,23),camera_distance=1.5,camera_azimuth=140,camera_elevation=-32)
  sink.label=f'SIMULATION | {code} | command 0.45 m/s | t=0.00s | 1x';sink.capture(force=True);sink.last_frame.save(P/f'figures/replay_{code}_0.png')
  class Recorder:
   def sample(self,dt):
    rec.sample(dt)
    if t.data.time-start<8-1e-7:
     sink.label=f'SIMULATION | {code} | command 0.45 m/s | t={t.data.time-start:.2f}s | 1x';sink.capture()
  for k in range(400):
   sample=a.gait.update(VelocityCommand(vx=.45),dt=.02,send=True)
   rec.record_control(converged=sample.converged,stride_clip_fraction=sample.stride_clip_fraction,ik_backoff_scale=sample.ik_backoff_scale);t.advance(.02,Recorder())
   if k==199:sink.last_frame.save(P/f'figures/replay_{code}_1.png')
  sink.last_frame.save(P/f'figures/replay_{code}_2.png');sink.close()
  row={'code':code,**asdict(rec.finalize())};rows.append(row);print(code,row['mean_vx_mps'],flush=True)
(P/'evidence/rendered_metrics.json').write_text(json.dumps(rows,indent=2))
