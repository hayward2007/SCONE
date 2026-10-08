"""Independent root-origin velocity check on fixed representative trials."""
from pathlib import Path
import sys,json,hashlib
sys.path.insert(0,str(Path.cwd()))
from concurrent.futures import ProcessPoolExecutor
import numpy as np,mujoco
import benchmark.revision_study as study
P=Path(__file__).resolve().parent
Base=study.ObservedMetrics
class OriginMetrics(Base):
 def __init__(self,*args,**kwargs):
  super().__init__(*args,**kwargs);self.origin_integral=np.zeros(3);self.origin_jac=np.zeros((3,self.model.nv));self.rot_jac=np.zeros_like(self.origin_jac)
  global recorder
  recorder=self
 def sample(self,dt):
  super().sample(dt)
  mujoco.mj_jac(self.model,self.data,self.origin_jac,self.rot_jac,self.data.xpos[self.trial.root_body_id],self.trial.root_body_id)
  self.origin_integral+=(self.origin_jac@self.data.qvel)*dt
study.ObservedMetrics=OriginMetrics

def one(job):
 row,_=study.run_job(job)
 vel=float((recorder.start_rotation.T@recorder.origin_integral)[0]/recorder.elapsed)
 return dict(id=job['id'],code=job['code'],command=job['command'],completed=row['completed'],
             displacement_speed=row['mean_vx_mps'],object_velocity_integral=row['velocity_integral_forward_mps'],
             root_origin_velocity_integral=vel,root_origin_difference=vel-row['mean_vx_mps'])
if __name__=='__main__':
 jobs=[j for j in study.design() if j['phase']==0 and ((j['suite']=='main' and j['code'] in 'BNU') or (j['suite']=='domain' and j['code'] in 'BN'))]
 (P/'velocity_validation_protocol.json').write_text(json.dumps({'jobs':jobs,'script_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'source':study.source_revision(),'purpose':'Compare the velocity of the root origin with displacement of that same origin; retain body-object velocity as a different-point diagnostic.'},indent=2))
 with ProcessPoolExecutor(max_workers=4) as pool:rows=list(pool.map(one,jobs))
 reference={x['id']:x for x in (json.loads(l) for l in (P/'confirmation/raw.jsonl').read_text().splitlines())}
 for r in rows:assert abs(r['displacement_speed']-reference[r['id']]['mean_vx_mps'])<1e-9
 report={'trials':rows,'count':len(rows),'maximum_origin_difference_mps':max(abs(r['root_origin_difference']) for r in rows)}
 (P/'velocity_validation.json').write_text(json.dumps(report,indent=2));print(report['count'],report['maximum_origin_difference_mps'],flush=True)
