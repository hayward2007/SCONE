"""Independent visual replay of two existing trials; no extra performance samples."""
from pathlib import Path
import sys,json
import mujoco
sys.path.insert(0,str(Path.cwd()))
from benchmark import joint_limit_study as study
from benchmark.capture import FrameSink,CaptureConfig
from PIL import Image,ImageDraw
P=Path(__file__).resolve().parents[1]
class CenteredSink(FrameSink):
 def _render_frame(self):
  self.camera.lookat[:]=self.data.subtree_com[self.root_body_id]
  self.renderer.update_scene(self.data,camera=self.camera,scene_option=self.scene_option)
  # Enlarge only the visual scene plane, not model geometry or contact physics.
  for geom in self.renderer.scene.geoms[:self.renderer.scene.ngeom]:
   if geom.type==mujoco.mjtGeom.mjGEOM_PLANE:geom.size[:2]=20
  im=Image.fromarray(self.renderer.render());d=ImageDraw.Draw(im,'RGBA')
  d.rectangle((0,0,self.config.width,44),fill=(0,0,0,215));d.text((14,9),self.label,fill='white',font=self.font)
  return im
original=study.JointMetrics
rendered=[]
for code in 'BP':
 job=next(j for j in study.design() if j['suite']=='main' and j['code']==code and j['phase']==.0625 and j['mode']=='governed' and j['scale']==.8 and j['sequence'][0][1]==.45)
 class VisualMetrics(original):
  def __init__(self,*a,**kw):
   super().__init__(*a,**kw)
   self.model.vis.global_.offwidth=1280;self.model.vis.global_.offheight=720
   self.model.vis.headlight.ambient[:]=.35;self.model.vis.headlight.diffuse[:]=.8
   # The GL context caches the finite display plane at construction. Expand
   # that display mesh, then restore model size before any physics advance.
   floor=mujoco.mj_name2id(self.model,mujoco.mjtObj.mjOBJ_GEOM,'simulation_floor')
   floor_size=self.model.geom_size[floor].copy();self.model.geom_size[floor,:2]=20
   self.sink=CenteredSink(self.model,self.data,root_body_id=self.trial.root_body_id,
      video_path=P/f'media/joint_{code}.mp4',image_path=P/f'figures/joint_{code}.jpg',
      label=f'SIMULATION | {code} | all-joint limits, H 0.8 | t=0.00s | 1x',
      config=CaptureConfig(1280,720,25,23),camera_distance=1.5,camera_azimuth=140,camera_elevation=-32)
   self.model.geom_size[floor]=floor_size
   self.sink.capture(force=True);self.sink.last_frame.save(P/f'figures/joint_replay_{code}_0.png')
  def sample(self,dt):
   super().sample(dt)
   if self.elapsed<20-1e-7:
    self.sink.label=f'SIMULATION | {code} | all-joint limits, H 0.8 | t={self.elapsed:.2f}s | 1x';self.sink.capture()
   if abs(self.elapsed-10)<1e-7:self.sink.last_frame.save(P/f'figures/joint_replay_{code}_1.png')
   if abs(self.elapsed-20)<1e-7:
    self.sink.last_frame.save(P/f'figures/joint_replay_{code}_2.png');self.sink.close()
 study.JointMetrics=VisualMetrics
 row,_=study.run_job(job);rendered.append(row);print(code,row['mean_vx_mps'],flush=True)
(P/'evidence/joint_rendered_metrics.json').write_text(json.dumps(rendered,indent=2))
