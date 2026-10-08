"""Derive manuscript tables and plots from all fixed-protocol rows."""
from pathlib import Path
from collections import Counter
import hashlib,json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
P=Path(__file__).resolve().parent
raw=P/'evidence/joint_limit/raw.jsonl'
rows=[json.loads(s) for s in raw.read_text().splitlines()]
protocol=json.loads((raw.parent/'protocol.json').read_text())
assert len(rows)==len(protocol['jobs'])==176
assert [r['id'] for r in rows]==list(range(176))
for r,j in zip(rows,protocol['jobs']):
 assert all(r[k]==v for k,v in j.items())
 assert abs(r['duration_s']-r['horizon_s'])<1e-7 if r['completed'] else r['termination_reason']
bounded=[r for r in rows if r['mode']!='unlimited']
assert len(bounded)==128
assert sum(r['issued_rate_violation_frames'] for r in bounded)==0
assert max(r['inner_limit_excess_dps'] for r in bounded)<1e-8
for r in bounded:assert np.all(np.array(r['issued_target_rate_peak_dps'])<=np.array(r['declared_limit_dps'])+1e-8)
def pick(**kw):return [r for r in rows if all(r[k]==v for k,v in kw.items())]
def summarize(rs):
 successful=[r for r in rs if r['completed']]
 return dict(n=len(rs),complete=len(successful),speed_mean=float(np.mean([r['mean_vx_mps'] for r in successful])),
  speed_range=[min(r['mean_vx_mps'] for r in successful),max(r['mean_vx_mps'] for r in successful)],
  velocity_rmse_mean=float(np.mean([r['velocity_rmse_mps'] for r in successful])),
  yaw_rmse_mean=float(np.mean([r['yaw_rate_rmse_rps'] for r in successful])),
  worst_joint_rms_max=max(max(r['joint_tracking_rms_deg']) for r in rs),
  requested_goal_lag_max=max(r['goal_lag_max_deg'] for r in rs),
  planner_time_fraction_mean=float(np.mean([r['planner_time_fraction'] for r in rs])))
summary={'n':len(rows),'completed':sum(r['completed'] for r in rows),
 'bounded_n':len(bounded),'issued_violations':0,
 'max_inner_limit_excess_dps':max(r['inner_limit_excess_dps'] for r in bounded),
 'max_actual_rate_over_declared_ratio':max(max(np.array(r['actual_joint_rate_peak_dps'])/np.array(r['declared_limit_dps'])) for r in bounded),
 'max_root_velocity_check_error_mps':max(abs(r['mean_vx_mps']-r['root_origin_velocity_integral_forward_mps']) for r in rows),
 'minimum_loaded_legs':min(r['loaded_leg_min'] for r in bounded),
 'max_fraction_below_three_loaded':max(r['fraction_physics_samples_below_three_loaded'] for r in bounded),
 'raw_sha256':hashlib.sha256(raw.read_bytes()).hexdigest(),'main':{},'transition':{},'long':{},'strict':{}}
for cmd in [.18,.45]:
 summary['main'][str(cmd)]={}
 for mode,scale,label in [('unlimited',None,'unlimited'),('governed',1.,'H1'),('governed',.8,'H08'),('fixed',.8,'F08')]:
  summary['main'][str(cmd)][label]={c:summarize([r for r in pick(suite='main',mode=mode,scale=scale,code=c) if r['sequence'][0][1]==cmd]) for c in 'UBP'}
 for c in 'UBP':summary['long'][f'{cmd}_{c}']=summarize([r for r in pick(suite='long',code=c) if r['sequence'][0][1]==cmd])
for c in 'UBP':summary['strict'][c]=summarize(pick(suite='strict',code=c))
for seq in [0,1]:
 for mode in ['unlimited','governed']:
  summary['transition'][f'{seq}_{mode}']={c:summarize([r for r in pick(suite='transition',mode=mode,code=c) if (r['sequence'][1][1]==.45)==(seq==0)]) for c in 'UBP'}
diffs=[]
for r in pick(suite='timestep'):
 base=next(v for v in pick(suite='main',mode='governed',scale=.8,code=r['code'],phase=r['phase']) if v['sequence'][0][1]==.45)
 diffs.append(r['mean_vx_mps']-base['mean_vx_mps'])
summary['timestep_max_paired_speed_change_mps']=max(abs(v) for v in diffs)
(P/'evidence/joint_summary.json').write_text(json.dumps(summary,indent=2))
# Tables use full window means and full deterministic phase ranges.
lines=[r'\begin{table*}[t]\centering\small',
 r'\caption{All-joint limit study at a 0.45 m/s command: four phases, 20 s per trial}',
 r'\label{tab:joint}\begin{tabular}{ccccccc}\toprule',
 r' & & Speed (m/s) & $e_v$ (m/s) & Joint RMS ($^\circ$) & Goal lag ($^\circ$) & Clock fraction\\',
 r'Clock & Code & Mean [min, max] & Mean & Worst observed & Maximum & Mean\\\midrule']
for mode in ['unlimited','H1','H08','F08']:
 for c in 'UBP':
  s=summary['main']['0.45'][mode][c];lo,hi=s['speed_range'];label={'unlimited':'Unlimited','H1':'H, $\\rho=1.0$','H08':'H, $\\rho=0.8$','F08':'F, $\\rho=0.8$'}[mode]
  lines.append(f"{label} & {c} & {s['speed_mean']:.3f} [{lo:.3f}, {hi:.3f}] & {s['velocity_rmse_mean']:.3f} & {s['worst_joint_rms_max']:.2f} & {s['requested_goal_lag_max']:.1f} & {s['planner_time_fraction_mean']:.3f}\\\\")
 lines.append(r'\midrule' if mode!='F08' else r'\bottomrule')
lines.extend([r'\end{tabular}',r'\par\vspace{5pt}\noindent\parbox{.96\textwidth}{\footnotesize All entries complete 4/4 trials. H holds the gait clock until each goal is issued; F keeps the original gait clock. Joint RMS is the maximum, over trials and joints, of issued-target tracking RMS. Goal lag is requested minus issued target. Clock fraction is gait time divided by wall time. $\rho$ scales the joint-group no-load speeds; it is not a hardware confidence bound.}',r'\end{table*}'])
(P/'joint_table.tex').write_text('\n'.join(lines)+'\n')
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':8,'axes.spines.top':False,'axes.spines.right':False,'pdf.fonttype':42})
fig,ax=plt.subplots(1,3,figsize=(7.05,2.04),layout='constrained')
colors={'U':'#7b8289','B':'#167c80','P':'#bf6a31'}
labels=['Unlimited','H 1.0','H 0.8','F 0.8','H 0.5']
for c in 'UBP':
 ss=[summary['main']['0.45'][m][c] for m in ['unlimited','H1','H08','F08']]+[summary['strict'][c]]
 x=np.arange(5)+{'U':-.14,'B':0,'P':.14}[c]
 y=np.array([v['speed_mean'] for v in ss]);lo=np.array([v['speed_range'][0] for v in ss]);hi=np.array([v['speed_range'][1] for v in ss])
 ax[0].errorbar(x,y,yerr=[y-lo,hi-y],marker='o',markersize=3,lw=1,color=colors[c],label=c,capsize=2)
 ax[1].plot(x,[v['worst_joint_rms_max'] for v in ss],'-o',markersize=3,lw=1,color=colors[c])
 ax[2].plot(x,[v['requested_goal_lag_max'] for v in ss],'-o',markersize=3,lw=1,color=colors[c])
for a in ax:
 a.set_xticks(range(5),labels,rotation=32,ha='right',fontsize=6.8);a.grid(axis='y',alpha=.18)
ax[0].set_ylabel('Net speed (m/s)');ax[0].set_title('(a) Transport');ax[0].legend(ncols=3,frameon=False,fontsize=7,loc='upper left');ax[0].set_ylim(.1,.365)
ax[1].set_ylabel('Worst joint RMS (deg)');ax[1].set_title('(b) Target tracking')
ax[2].set_ylabel('Maximum goal lag (deg)');ax[2].set_title('(c) Requested vs. issued')
fig.savefig(P/'figures/joint_limits.pdf');fig.savefig(P/'figures/joint_limits.png',dpi=220);plt.close(fig)
print(json.dumps({k:v for k,v in summary.items() if k not in ['main','transition','strict','long']},indent=2))
# Preserve the baseline first-figure quantities while using consistent colors.
baseline=[json.loads(s) for s in (P/'evidence/confirmation/raw.jsonl').read_text().splitlines()]
old={c:[r for r in baseline if r['suite']=='main' and r['code']==c and r['command'][0]==.45] for c in 'UBP'}
fig,axs=plt.subplots(2,1,figsize=(3.43,2.9),layout='constrained')
t=np.linspace(.08,.4,200);axs[0].plot(t,np.minimum(100,540*t/1.875),color=colors['B'],lw=1.8)
axs[0].axhline(100,color='#7b8289',ls=':',lw=.8);axs[0].plot(.2,57.6,'o',color=colors['B'],ms=4)
axs[0].annotate('57.6 deg at 0.20 s',xy=(.2,57.6),xytext=(.225,32),fontsize=7,arrowprops={'arrowstyle':'-','lw':.6})
axs[0].text(.09,103,'Geometric limit: 100 deg',fontsize=7,color='#687783')
axs[0].set(xlabel='Available swing time (s)',ylabel='Excursion bound (deg)',ylim=(0,125),title='(a) Time available to rewind limits rolling')
peaks=[max(r['sector_rate_peak_dps'] for r in old[c]) for c in 'UBP']
axs[1].bar(range(3),peaks,color=[colors[c] for c in 'UBP'],width=.65)
for i,v in enumerate(peaks):axs[1].text(i,v+48,f'{v:.0f}',ha='center',fontsize=7)
axs[1].axhline(540,color='black',ls='--',lw=.7);axs[1].annotate('540 limit',xy=(2.3,540),xytext=(2.3,1000),ha='right',fontsize=7,arrowprops={'arrowstyle':'-','lw':.6})
axs[1].set(xticks=range(3),xticklabels=['Original U','Proposed B','Periodic P'],ylabel='Sector peak (deg/s)',ylim=(0,1830),title='(b) Measured at 0.45 m/s command')
axs[1].set_xlabel('Mean speed: '+ ' / '.join(f"{np.mean([r['mean_vx_mps'] for r in old[c]]):.3f}" for c in 'UBP')+' m/s')
for a in axs:a.grid(axis='y',alpha=.15);a.set_axisbelow(True)
fig.savefig(P/'figures/overview.pdf');fig.savefig(P/'figures/overview.png',dpi=220);plt.close(fig)
