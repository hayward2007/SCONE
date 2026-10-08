"""Frozen, deterministic mechanism study. Does not estimate hardware uncertainty.
Run on a clean source snapshot: python -m benchmark.revision_study --output DIR.
"""
from pathlib import Path
from dataclasses import asdict
from concurrent.futures import ProcessPoolExecutor
import argparse, json, hashlib, platform
import numpy as np
import mujoco
from .common import SimulationTrial, MetricsRecorder, Perturbation, source_revision, simulation_provenance
from .controllers import make_controller, ROLE_CONFIGS
from src.locomotion import VelocityCommand

NAMES={'U':'role-split-scone','B':'rewind-budget-scone','N':'role-split-no-roll',
       'S':'role-split-superposition','D':'role-split-no-reindex','P':'role-split-periodic',
       'G':'rewind-distance-scone'}

def design():
    jobs=[]
    def add(suite, codes, commands, phases, geometry='decomposed-arc', dt=.002, **stress):
        for cmd in commands:
            for phase in phases:
                for code in codes:
                    jobs.append(dict(id=len(jobs),suite=suite,code=code,command=cmd,phase=phase,
                                     geometry=geometry,physics_dt=dt,stress=stress))
    phases=[k/8 for k in range(8)]; four=phases[::2]
    add('main','UBGNSDP',[[.18,0,0],[.45,0,0]],phases)
    add('domain','BNS',[[.30,0,0],[-.18,0,0],[0,.12,0],[0,-.12,0],
                       [0,0,.45],[0,0,-.45],[.18,0,.35]],four)
    for stress in [dict(mass_scale=1.2),dict(friction_scale=.5),dict(actuator_strength_scale=.8)]:
        add('stress','BNS',[[.45,0,0]],four,**stress)
    add('geometry','BNS',[[.18,0,0],[.45,0,0]],four,geometry='open-arc')
    for dt in [.001,.004]:
        add('timestep','BNS',[[.45,0,0]],[0,.5],dt=dt)
    return jobs

class ObservedMetrics(MetricsRecorder):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self.loaded_min=6;self.under_three=0;self.velocity_integral=np.zeros(3)
    def _contact_metrics(self):
        count,slip,forbidden=super()._contact_metrics()
        self.loaded_min=min(self.loaded_min,count);self.under_three+=int(count<3)
        return count,slip,forbidden
    def sample(self,dt):
        super().sample(dt)
        out=np.zeros(6)
        mujoco.mj_objectVelocity(self.model,self.data,mujoco.mjtObj.mjOBJ_BODY,
                                self.trial.root_body_id,out,0)
        self.velocity_integral+=out[3:]*dt

def run_job(job):
    with SimulationTrial(contact_geometry=job['geometry'],
                         perturbation=Perturbation(gait_phase=job['phase'],**job['stress'])) as t:
        t.model.opt.timestep=job['physics_dt'];t.initialize()
        adapter=make_controller(NAMES[job['code']],t,phase=job['phase']);adapter.prepare(t)
        gait=adapter.gait
        for _ in range(50):adapter.update(VelocityCommand(),.02);t.advance(.02)
        rec=ObservedMetrics(t,job['command']);previous=gait.sector_degrees;previous_motor=None
        sector_peak=0.;joint_peak=0.;max_excursion=0.;rate_violation_frames=0
        trace=[];scheduled_min=6
        command=VelocityCommand.from_array(job['command'])
        for frame in range(400):
            sample=gait.step(command,dt=.02)
            if not sample.converged:
                # The public sender rejects this target. Record a failed
                # trial instead of aborting the entire ablation matrix.
                rec.record_control(converged=False,
                                   stride_clip_fraction=sample.stride_clip_fraction,
                                   ik_backoff_scale=sample.ik_backoff_scale)
                t.advance(.02,rec)
                rec.termination_reason='ik-target-rejected'
                break
            gait.send(sample)
            now=gait.sector_degrees;rate=abs(now-previous)/.02
            sector_peak=max(sector_peak,float(rate.max()));max_excursion=max(max_excursion,float(abs(now).max()))
            rate_violation_frames+=int(rate.max()>gait.config.max_roll_rate_degrees+1e-7)
            if previous_motor is not None:joint_peak=max(joint_peak,float(abs(sample.motor_degrees[12:]-previous_motor).max()/.02))
            previous=now;previous_motor=sample.motor_degrees[12:].copy()
            scheduled_min=min(scheduled_min,len(sample.stance_legs))
            rec.record_control(converged=sample.converged,stride_clip_fraction=sample.stride_clip_fraction,ik_backoff_scale=sample.ik_backoff_scale)
            t.advance(.02,rec)
            if job['suite']=='main' and job['phase']==0 and job['command'][0]==.45:
                trace.append(dict(time_s=(frame+1)*.02,sector_degrees=now.tolist(),sector_rates=rate.tolist(),x=float(t.data.xpos[t.root_body_id][0]),stance_legs=list(sample.stance_legs)))
            if rec.termination_reason:break
        result={**job,**asdict(rec.finalize()),'sector_rate_peak_dps':sector_peak,
                'sector_excursion_peak_degrees':max_excursion,'sector_rate_violation_frames':rate_violation_frames,
                'distal_target_rate_peak_dps':joint_peak,'loaded_leg_min':rec.loaded_min,
                'fraction_physics_samples_below_three_loaded':rec.under_three/rec.sample_count,
                'scheduled_stance_min':scheduled_min,
                'velocity_integral_forward_mps':float((rec.start_rotation.T@rec.velocity_integral)[0]/rec.elapsed)}
        return result,trace

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True);parser.add_argument('--workers',type=int,default=4);args=parser.parse_args()
    revision=source_revision()
    if revision['git_dirty']:raise RuntimeError('Requires a clean frozen source snapshot')
    args.output.mkdir(parents=True,exist_ok=True)
    jobs=design();files={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for folder in ['src','benchmark','packages'] for p in Path(folder).rglob('*') if p.is_file() and '__pycache__' not in str(p)}
    protocol=dict(purpose='prospectively fixed deterministic mechanism study; parameter stress tests are not hardware uncertainty distributions',
                  jobs=jobs,source_files_sha256=files,controller_configs={k:asdict(v) for k,v in ROLE_CONFIGS.items()},
                  python=platform.python_version(),numpy=np.__version__,mujoco=mujoco.__version__,
                  settle_seconds=1,measure_seconds=8,control_dt=.02,**revision,**simulation_provenance())
    with (args.output/'protocol.json').open('x') as f:json.dump(protocol,f,indent=2)
    with (args.output/'raw.jsonl').open('x') as f,ProcessPoolExecutor(max_workers=args.workers) as pool:
        for i,(row,trace) in enumerate(pool.map(run_job,jobs)):
            f.write(json.dumps(row)+'\n');f.flush()
            if trace:(args.output/f"trace_{row['code']}.json").write_text(json.dumps(trace))
            if i%12==0 or i==len(jobs)-1:print(f"{i+1}/{len(jobs)} {row['suite']} {row['code']} {row['mean_vx_mps']:.3f} completed={row['completed']}",flush=True)
    changed=[str(p) for p,h in files.items() if hashlib.sha256(Path(p).read_bytes()).hexdigest()!=h]
    (args.output/'source_stability.json').write_text(json.dumps({'changed':changed,**source_revision()},indent=2))
    if changed:raise RuntimeError('Source changed during execution')
if __name__=='__main__':main()
