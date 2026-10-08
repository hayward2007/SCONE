"""Prospectively fixed all-joint rate stress study; simulation only.

The manufacturer values are 12 V no-load speeds, NOT loaded motor limits.
Run the full design from a clean snapshot, with output outside that checkout.
"""
from pathlib import Path
from dataclasses import asdict
from concurrent.futures import ProcessPoolExecutor
import argparse
import hashlib
import json
import platform
import numpy as np
import mujoco
from .common import SimulationTrial, MetricsRecorder, Perturbation, source_revision, simulation_provenance
from .controllers import make_controller, ROLE_CONFIGS
from .revision_study import NAMES
from .joint_governor import JointTargetGovernor, DEGREES_PER_COUNT, NO_LOAD_DPS, degrees_to_counts
from src.locomotion import VelocityCommand

CONTROL_DT = .02
PHASES = [1/16, 5/16, 9/16, 13/16]
SOURCES = [f'https://emanual.robotis.com/docs/en/dxl/{s}/' for s in
           ['mx/mx-28', 'x/xm430-w350', 'x/xm430-w210']]


def design():
    jobs = []
    def add(suite, codes, sequences, phases, modes, duration, physics_dt=.002):
        for sequence in sequences:
            for phase in phases:
                for code in codes:
                    for mode, scale in modes:
                        jobs.append(dict(id=len(jobs), suite=suite, code=code,
                                         sequence=sequence, phase=phase, mode=mode,
                                         scale=scale, horizon_s=duration, physics_dt=physics_dt))
    constant = [[[0, .18, 0, 0]], [[0, .45, 0, 0]]]
    add('main', 'UBP', constant, PHASES,
        [('unlimited', None), ('governed', 1.), ('governed', .8), ('fixed', .8)], 20.)
    sequences = [[[0,.18,0,0],[6,.45,0,0],[12,0,0,0],[18,-.18,0,0]],
                 [[0,.18,0,0],[6,0,0,.45],[12,.18,0,.35],[18,.18,0,0]]]
    add('transition', 'UBP', sequences, PHASES, [('unlimited',None),('governed',.8)], 24.)
    add('strict', 'UBP', [constant[1]], PHASES, [('governed',.5)], 20.)
    add('long', 'UBP', constant, PHASES[::2], [('governed',.8)], 60.)
    for dt in [.001,.004]:
        add('timestep','BP',[constant[1]],PHASES[::2],[('governed',.8)],20.,dt)
    assert len(jobs) == 176
    return jobs


def command_at(sequence, wall_time):
    return np.array(next(row[1:] for row in reversed(sequence) if wall_time+1e-9 >= row[0]), dtype=float)


class JointMetrics(MetricsRecorder):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.inner_peak = np.zeros(18)
        self.actual_peak = np.zeros(18)
        self.tracking_squared = np.zeros(18)
        self.loaded_min = 6
        self.under_three = 0
        self.root_integral = np.zeros(3)
        self.jp = np.zeros((3,self.model.nv)); self.jr = np.zeros_like(self.jp)

    def _contact_metrics(self):
        count, slip, forbidden = super()._contact_metrics()
        self.loaded_min = min(self.loaded_min, count)
        self.under_three += int(count < 3)
        return count, slip, forbidden

    def sample(self, dt):
        super().sample(dt)
        c = self.controller
        self.inner_peak = np.maximum(self.inner_peak, abs(np.degrees(c._setpoint_velocity[1:])))
        self.actual_peak = np.maximum(self.actual_peak, abs(np.degrees(self.data.qvel[c._dof_addresses[1:]])))
        error = np.degrees(c._target[1:] - self.data.qpos[c._qpos_addresses[1:]])
        self.tracking_squared += error**2 * dt
        mujoco.mj_jac(self.model,self.data,self.jp,self.jr,
                      self.data.xpos[self.trial.root_body_id],self.trial.root_body_id)
        self.root_integral += (self.jp @ self.data.qvel)*dt


def run_job(job):
    with SimulationTrial(contact_geometry='decomposed-arc',
                         perturbation=Perturbation(gait_phase=job['phase'])) as t:
        t.model.opt.timestep=job['physics_dt']; t.initialize()
        adapter=make_controller(NAMES[job['code']],t,phase=job['phase']); adapter.prepare(t)
        gait=adapter.gait
        for _ in range(50):
            adapter.update(VelocityCommand(),CONTROL_DT); t.advance(CONTROL_DT)
        c=t.controller
        previous=np.array([c.radians_to_raw(c._target[i]) for i in range(1,19)])
        governor=None; effective=None
        if job['mode'] != 'unlimited':
            governor=JointTargetGovernor(previous,job['scale'],CONTROL_DT)
            effective=governor.configure_inner_profile(c)
        rec=JointMetrics(t,command_at(job['sequence'],0))
        desired=previous.copy(); previous_desired=previous.copy()
        issued_peak=np.zeros(18); requested_peak=np.zeros(18)
        planner_steps=0; clipped=0; lag_max=0.; violation_frames=0
        sample=None; trace=[]; frames=0
        for frame in range(round(job['horizon_s']/CONTROL_DT)):
            wall_time=frame*CONTROL_DT
            rec.command=command_at(job['sequence'],wall_time)
            should_plan=(job['mode']!='governed' or governor.at_target)
            if should_plan:
                sample=gait.step(VelocityCommand.from_array(rec.command),dt=CONTROL_DT)
                if not sample.converged:
                    rec.record_control(converged=False,stride_clip_fraction=sample.stride_clip_fraction,
                                       ik_backoff_scale=sample.ik_backoff_scale)
                    t.advance(CONTROL_DT,rec)
                    rec.termination_reason='ik-target-rejected'
                    frames=frame+1
                    break
                desired=degrees_to_counts(sample.motor_degrees)
                requested_peak=np.maximum(requested_peak,abs(desired-previous_desired)*DEGREES_PER_COUNT/CONTROL_DT)
                previous_desired=desired.copy(); planner_steps+=1
            issued=desired.copy() if governor is None else governor.advance(desired)
            rate=abs(issued-previous)*DEGREES_PER_COUNT/CONTROL_DT
            issued_peak=np.maximum(issued_peak,rate)
            lag=float(abs(desired-issued).max()*DEGREES_PER_COUNT)
            lag_max=max(lag_max,lag); clipped+=int(lag>0)
            if governor is not None:
                violation_frames+=int(np.any(rate>governor.limits_dps+1e-8))
            c.set_raw_positions({i:int(raw) for i,raw in enumerate(issued,1)})
            previous=issued.copy()
            rec.record_control(converged=True,stride_clip_fraction=sample.stride_clip_fraction,
                               ik_backoff_scale=sample.ik_backoff_scale)
            t.advance(CONTROL_DT,rec); frames=frame+1
            if job['phase']==PHASES[0] and job['code'] in 'BP' and job['suite'] in ['main','transition'] and frame%10==0:
                trace.append(dict(time_s=(frame+1)*CONTROL_DT,command=rec.command.tolist(),
                                  position=t.data.xpos[t.root_body_id].tolist(),
                                  planner_time_s=planner_steps*CONTROL_DT,goal_lag_deg=lag,
                                  issued_rate_dps=rate.tolist(),root_upright=float(t.data.xmat[t.root_body_id].reshape(3,3)[2,2])))
            if rec.termination_reason: break
        result={**job,**asdict(rec.finalize()),
                'planner_steps':planner_steps,'measurement_frames':frames,
                'planner_time_fraction':planner_steps*CONTROL_DT/rec.elapsed,
                'clipped_frame_fraction':clipped/frames,'goal_lag_max_deg':lag_max,
                'issued_target_rate_peak_dps':issued_peak.tolist(),
                'requested_target_rate_per_planner_dt_peak_dps':requested_peak.tolist(),
                'inner_setpoint_rate_peak_dps':rec.inner_peak.tolist(),
                'actual_joint_rate_peak_dps':rec.actual_peak.tolist(),
                'joint_tracking_rms_deg':np.sqrt(rec.tracking_squared/rec.elapsed).tolist(),
                'declared_limit_dps':None if governor is None else governor.limits_dps.tolist(),
                'effective_inner_limit_dps':None if effective is None else effective.tolist(),
                'issued_rate_violation_frames':violation_frames,
                'loaded_leg_min':rec.loaded_min,
                'fraction_physics_samples_below_three_loaded':rec.under_three/rec.sample_count,
                'root_origin_velocity_integral_forward_mps':float((rec.start_rotation.T@rec.root_integral)[0]/rec.elapsed)}
        if governor is not None:
            result['inner_limit_excess_dps']=float(np.max(rec.inner_peak-effective))
        return result,trace


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--workers',type=int,default=4)
    args=parser.parse_args()
    revision=source_revision()
    if revision['git_dirty']:raise RuntimeError('Requires clean frozen source')
    args.output.mkdir(parents=True,exist_ok=True)
    jobs=design()
    files={str(p):hashlib.sha256(p.read_bytes()).hexdigest()
           for folder in ['src','benchmark','packages'] for p in Path(folder).rglob('*')
           if p.is_file() and '__pycache__' not in str(p)}
    protocol=dict(purpose='All-joint command-limit stress test. No-load bounds are not loaded actuator identification.',
                  jobs=jobs,source_files_sha256=files,controller_configs={k:asdict(v) for k,v in ROLE_CONFIGS.items()},
                  python=platform.python_version(),numpy=np.__version__,mujoco=mujoco.__version__,
                  settle_seconds=1,control_dt=CONTROL_DT,no_load_dps=NO_LOAD_DPS.tolist(),datasheet_sources=SOURCES,
                  success_definition='Full horizon without rejected IK, nonfinite state or tilt over 60 degrees; tracking is separate.',
                  governed_clock='Advance gait/filter by 20 ms only after prior quantized target is issued completely; never wait on measured joint convergence.',
                  fixed_clock='Advance gait/filter every 20 ms even if target governor is behind.',
                  scope='No acceleration, contact feasibility, hard-stop or loaded-speed guarantee.',
                  **revision,**simulation_provenance())
    with (args.output/'protocol.json').open('x') as f:json.dump(protocol,f,indent=2)
    with (args.output/'raw.jsonl').open('x') as f,ProcessPoolExecutor(max_workers=args.workers) as pool:
        for i,(row,trace) in enumerate(pool.map(run_job,jobs)):
            f.write(json.dumps(row)+'\n'); f.flush()
            if trace:(args.output/f"trace_{row['id']:03d}.json").write_text(json.dumps(trace))
            if i%8==0 or i==len(jobs)-1:
                print(f"{i+1}/{len(jobs)} {row['suite']} {row['code']} {row['mode']} {row['mean_vx_mps']:.3f} complete={row['completed']}",flush=True)
    changed=[p for p,h in files.items() if hashlib.sha256(Path(p).read_bytes()).hexdigest()!=h]
    (args.output/'source_stability.json').write_text(json.dumps({'changed':changed,**source_revision()},indent=2))
    if changed:raise RuntimeError('Source changed during execution')


if __name__=='__main__':main()
