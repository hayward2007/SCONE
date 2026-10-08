"""Replay preselected nominal conditions from the frozen experiment."""
from pathlib import Path
import sys,json
import mujoco
from PIL import Image,ImageDraw
sys.path.insert(0,str(Path.cwd()))
from benchmark.stair_simplification import design,run_job
from benchmark.capture import FrameSink,CaptureConfig
P=Path(__file__).resolve().parents[1]
class Sink(FrameSink):
 def _render_frame(self):
  self.camera.lookat[:]=[0.,.60,.05]
  self.renderer.update_scene(self.data,camera=self.camera,scene_option=self.scene_option)
  im=Image.fromarray(self.renderer.render());d=ImageDraw.Draw(im,'RGBA')
  d.rectangle((0,0,1280,44),fill=(0,0,0,210));d.text((12,8),self.label,fill='white',font=self.font)
  return im
rows=[]
for jid in [45,47,75]:
 j=design()[jid];state={}
 def observe(t):
  if not t.measuring:return
  if 'sink' not in state:
   t.model.vis.global_.offwidth=1280;t.model.vis.global_.offheight=720
   t.model.vis.headlight.ambient[:]=.35;t.model.vis.headlight.diffuse[:]=.8
   state['sink']=Sink(t.model,t.data,root_body_id=t.root_id,video_path=P/f'media/stair_{jid}.mp4',image_path=P/f'figures/stair_{jid}.jpg',label='',config=CaptureConfig(1280,720,25,22),camera_distance=2.65,camera_azimuth=160,camera_elevation=-35)
   state['saved']=set()
  s=state['sink'];elapsed=t.elapsed-t.t0
  s.label=f"SIMULATION | {'C-shaped arc' if j['geometry']=='decomposed-arc' else 'Closed wheel'} | {int(j['h']*1000)} mm | t={elapsed:.2f}s | {t.stage} | 1x"
  s.capture()
  for stamp in [0,2,4,6,10,19.9]:
   if elapsed>=stamp and stamp not in state['saved']:
    s.last_frame.save(P/f'figures/stair_{jid}_{stamp:g}.png');state['saved'].add(stamp)
 r=run_job(j,observer=observe)
 if 'sink' in state:state['sink'].close()
 rows.append(r);print(jid,r['ascent_crossed'],r['halt_success'],flush=True)
(P/'evidence/rendered_stair_trials.json').write_text(json.dumps(rows,indent=2))
