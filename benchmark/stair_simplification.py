"""Frozen factorial test of stair posture and contact shape; simulation only.

The 195-degree setting was chosen in an earlier nominal pilot. This grid is
sensitivity evidence, not an independent hardware validation or reliability
sample. Existing physical and interactive controllers are left unchanged.
"""
from pathlib import Path
from dataclasses import replace
from unittest.mock import patch
from concurrent.futures import ProcessPoolExecutor, as_completed
import argparse
import json
import math
import numpy as np
import mujoco
from src.simulation import stair_benchmark as sb
from src.simulation.core.model import load_model
from src.simulation.core.stair_climber import SconeStairClimber, SconeStairConfig
from src.simulation.terrain import StairProfile, TerrainType
from src.locomotion import VelocityCommand
from .common import temporary_stair_profile, source_revision, simulation_provenance
from .model_variants import transform_for_contact_geometry, CLOSED_WHEEL_CENTER, CLOSED_WHEEL_RADIUS_M

DT = .02
POSES = [(0., 0.), (-.03, 3.), (.03, -3.)]


def fixed_config(brace=195.):
    return replace(SconeStairConfig(),
        neutral_front_stage1_degrees=brace, medium_front_stage1_degrees=brace,
        tall_front_stage1_degrees=brace, synchronized_phase_degrees=90.,
        tall_synchronized_phase_degrees=90., easy_phase_velocity=200., phase_velocity=200.)


def top_supported(min_center_y, last_riser_y, top_loaded, upright):
    return bool(min_center_y >= last_riser_y + CLOSED_WHEEL_RADIUS_M
                and top_loaded >= 3 and upright >= .5)


def halt_stable(supported, linear_speed, angular_speed):
    return bool(supported and linear_speed < .05 and angular_speed < .2)


def design():
    jobs=[]
    for h in [.10,.15,.20]:
        for d in [.25,.35]:
            for pose in range(len(POSES)):
                for g in ['decomposed-arc','closed-wheel']:
                    for c in ['fixed','neutral']:
                        jobs.append(dict(id=len(jobs),suite='factorial',h=h,d=d,pose=pose,geometry=g,control=c,dt=.002))
                jobs.append(dict(id=len(jobs),suite='lookup',h=h,d=d,pose=pose,geometry='decomposed-arc',control='lookup',dt=.002))
    for h in [.10,.15,.20]:
        for g in ['decomposed-arc','closed-wheel']:
            for dt in [.001,.004]:
                jobs.append(dict(id=len(jobs),suite='timestep',h=h,d=.35,pose=0,geometry=g,control='fixed',dt=dt))
    assert len(jobs)==102
    return jobs


class Trial(sb._Trial):
    def __init__(self, terrain, **kwargs):
        super().__init__(terrain,**kwargs)
        self.measuring=False
        self.trace=[]
        self.observer=None
        self.ids=np.array([mujoco.mj_name2id(self.model,mujoco.mjtObj.mjOBJ_BODY,f'TIRE_{i}') for i in range(1,7)])
        if (self.ids<0).any():raise ValueError('tire bodies missing')
        self.tire_set=set(self.ids.tolist())
        self.dofs=self.model.jnt_dofadr[self.model.actuator_trnid[:,0]]
        self.floor_z=float(self.model.geom_pos[mujoco.mj_name2id(self.model,mujoco.mjtObj.mjOBJ_GEOM,'simulation_floor'),2])
        self.top_z=self.floor_z+self.h*3 if hasattr(self,'h') else math.inf
        self.top_loaded=0; self.loaded=0; self.upright=1.; self.min_center_y=-math.inf
        self.speed=0.;self.angular_speed=0.;self.peak_phase_error=0.
        self.tracking_sq=np.zeros(18);self.actual_rate_peak=np.zeros(18)
        self.jp=np.zeros((3,self.model.nv));self.jr=np.zeros_like(self.jp)
        self.stage='setup'
        self.fail=None

    def advance(self, seconds):
        dt=float(self.model.opt.timestep)
        f=np.zeros(6)
        for _ in range(max(1,round(seconds/dt))):
            self.controller.update(dt);mujoco.mj_step(self.model,self.data)
            self.elapsed+=dt
            self.work+=float(np.sum(abs(self.data.actuator_force*self.data.qvel[self.dofs])))*dt
            self.upright=float(self.data.xmat[self.root_id].reshape(3,3)[2,2])
            self.minimum_upright=min(self.minimum_upright,self.upright)
            if not np.isfinite(self.data.qpos).all() or not np.isfinite(self.data.qvel).all():
                self.fail='nonfinite';return
            if not self.measuring:continue
            loaded=set();top=set()
            for i in range(self.data.ncon):
                con=self.data.contact[i]
                mujoco.mj_contactForce(self.model,self.data,i,f)
                self.peak_force=max(self.peak_force,float(np.linalg.norm(f[:3])))
                b1,b2=self.model.geom_bodyid[[con.geom1,con.geom2]]
                tire=int(b1) if int(b1) in self.tire_set and b2==0 else int(b2) if int(b2) in self.tire_set and b1==0 else None
                if tire is not None and f[0]>1.:
                    loaded.add(tire)
                    if abs(float(con.pos[2])-self.top_z)<.01:top.add(tire)
            self.loaded=len(loaded);self.top_loaded=len(top)
            centers=self.data.xpos[self.ids]+np.einsum('nij,j->ni',self.data.xmat[self.ids].reshape(-1,3,3),np.array(CLOSED_WHEEL_CENTER))
            self.min_center_y=float(centers[:,1].min())
            mujoco.mj_jac(self.model,self.data,self.jp,self.jr,self.data.xpos[self.root_id],self.root_id)
            self.speed=float(np.linalg.norm(self.jp@self.data.qvel));self.angular_speed=float(np.linalg.norm(self.jr@self.data.qvel))
            c=self.controller
            self.tracking_sq+=np.degrees(c._target[1:]-self.data.qpos[c._qpos_addresses[1:]])**2*dt
            self.actual_rate_peak=np.maximum(self.actual_rate_peak,abs(np.degrees(self.data.qvel[c._dof_addresses[1:]])))
            if self.upright<.5:self.fail='tilt-limit'
        if self.measuring:
            self.trace.append(dict(t=round(self.elapsed-self.t0,6),stage=self.stage,y=float(self.data.xpos[self.root_id,1]),z=float(self.data.xpos[self.root_id,2]),upright=self.upright,top_loaded=self.top_loaded,min_center_y=self.min_center_y,speed=self.speed,work=self.work-self.w0))
        if self.observer:self.observer(self)


def run_job(job, observer=None):
    profile=StairProfile(rises=(job['h'],)*3,tread_depths=(job['d'],)*3,widths=(1.,)*3,landing_length=.70)
    def loader(*args,**kwargs):
        kwargs['xml_transform']=transform_for_contact_geometry(job['geometry'])
        m=load_model(*args,**kwargs);m.opt.timestep=job['dt'];return m
    row={**job,**source_revision(), 'ascent_crossed':False,'halt_success':False,'failure':None}
    with temporary_stair_profile(TerrainType.STAIRS_3,profile),patch.object(sb,'load_model',side_effect=loader):
        t=Trial(TerrainType.STAIRS_3)
        try:
            t.observer=observer
            t.prepare_side_on()
            row['side_pose_time_s']=t.elapsed;row['side_pose_work_j']=t.work
            dy,yaw=POSES[job['pose']]
            t.apply_initial_pose(y_m=dy,yaw_degrees=yaw)
            cfg=None if job['control']=='lookup' else fixed_config(195. if job['control']=='fixed' else 180.)
            c=SconeStairClimber(t.controller,terrain=TerrainType.STAIRS_3,config=cfg)
            row.update(brace_deg=c.front_stage1_degrees,initial_phase_deg=c.initial_phase_degrees,phase_rate_dps=c.selected_phase_velocity*1.374)
            for label,method,tolerance in [('brace',c.prepare_front_stage1,c.config.front_stage1_tolerance_raw),('phase',c.prepare,c.config.phase_tolerance_raw)]:
                t.stage=label;targets=method();t.advance(4.)
                err=max(abs(t.controller.get_position(i)-q) for i,q in targets.items())
                row[label+'_acquisition_error_raw']=err
                if err>tolerance:
                    row['failure']=label+'-acquisition';return row
            c.activate();t.stage='ascent';t.t0=t.elapsed;t.w0=t.work;t.measuring=True
            t.top_z=t.floor_z+profile.total_height;t.minimum_upright=t.upright;t.peak_force=0.
            last_y=.35+2*job['d']
            row.update(preparation_time_s=t.elapsed,preparation_work_j=t.work,top_surface_z=t.top_z,last_riser_y=last_y)
            for _ in range(1000):
                c.update(VelocityCommand(vy=c.config.max_vy),DT);t.advance(DT)
                if t.fail:break
                if top_supported(t.min_center_y,last_y,t.top_loaded,t.upright):
                    row.update(ascent_crossed=True,ascent_time_s=t.elapsed-t.t0,ascent_work_j=t.work-t.w0)
                    break
            c.stop();t.stage='hold';hold_start=t.elapsed;stable=0.
            if row['ascent_crossed'] and not t.fail:
                for _ in range(100):
                    t.advance(DT)
                    if t.fail:break
                    ok=halt_stable(top_supported(t.min_center_y,last_y,t.top_loaded,t.upright),t.speed,t.angular_speed)
                    stable=stable+DT if ok else 0.
                    if stable>=.5-1e-9:
                        row['halt_success']=True;break
            elapsed=t.elapsed-t.t0
            row.update(failure=t.fail or (None if row['halt_success'] else 'unstable-halt' if row['ascent_crossed'] else 'ascent-timeout'),
                measurement_time_s=elapsed,total_time_s=t.elapsed,measurement_work_j=t.work-t.w0,total_work_j=t.work,
                minimum_upright=t.minimum_upright,peak_contact_force_n=t.peak_force,final_y=float(t.data.xpos[t.root_id,1]),
                final_z=float(t.data.xpos[t.root_id,2]),hold_time_s=t.elapsed-hold_start,
                tracking_rms_deg=np.sqrt(t.tracking_sq/max(elapsed,1e-9)).tolist(),actual_rate_peak_dps=t.actual_rate_peak.tolist(),
                phase_spread_peak_deg=c.maximum_phase_spread_degrees,trace=t.trace)
            return row
        finally:t.close()


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);p.add_argument('--workers',type=int,default=4);p.add_argument('--pilot',action='store_true');a=p.parse_args()
    revision=source_revision()
    if not a.pilot and revision.get('git_dirty'):raise SystemExit('Use a clean frozen checkout')
    a.output.mkdir(parents=True,exist_ok=True)
    jobs=design()
    if a.pilot:jobs=[j for j in jobs if j['pose']==0 and j['d']==.35 and j['suite']=='factorial' and j['control']=='fixed']
    protocol={'schema':1,'pilot':a.pilot,'jobs':jobs,'poses_dy_m_yaw_deg':POSES,'physics':simulation_provenance(),**revision,
        'development':'195 deg, 90 deg, 200 velocity units selected using 12 earlier nominal exploratory runs; frozen grid is sensitivity evidence.',
        'endpoint':'All six tire centers beyond last riser plus radius, >=3 distinct tire bodies >1 N on top-height contacts; upright >=0.5. Stop phase. Within 2 s achieve 0.5 s of same support with root speed <0.05 m/s and angular speed <0.2 rad/s, sampled at 50 Hz. Tilt below 0.5 at any physics step terminates ascent/hold.',
        'timing':'20 s ascent plus up to 2 s halt; separately include scripted side-pose acquisition and 4+4 s brace/phase acquisition. Initial pose offsets applied before these 8 s; zero initial velocity.'}
    (a.output/'protocol.json').write_text(json.dumps(protocol,indent=2))
    with (a.output/'trials.jsonl').open('w') as f,ProcessPoolExecutor(max_workers=a.workers) as pool:
        tasks={pool.submit(run_job,j):j for j in jobs}
        for future in as_completed(tasks):
            j=tasks[future]
            try:r=future.result()
            except Exception as e:r={**j,**revision,'ascent_crossed':False,'halt_success':False,'failure':'exception','error':repr(e)}
            f.write(json.dumps(r,allow_nan=False)+'\n');f.flush()
            print(j['id'],j['geometry'],j['control'],j['h'],j['d'],j['pose'],r['ascent_crossed'],r['halt_success'],r['failure'],flush=True)
if __name__=='__main__':main()
