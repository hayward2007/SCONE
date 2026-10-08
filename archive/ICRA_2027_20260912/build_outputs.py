"""Derive manuscript numbers/plots only from saved audit records."""
import os,json,csv
from pathlib import Path
from collections import defaultdict
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Wedge,Rectangle,Circle,FancyArrowPatch
from scipy.spatial import ConvexHull
P=Path(__file__).resolve().parent
rows=[json.loads(x) for x in (P/'evidence/phase_grid.jsonl').read_text().splitlines()]
assert len(rows)==80 and all('error' not in r for r in rows)
assert len({(r['vx_command'],r['phase'],r['condition']) for r in rows})==80
ORDER=['no_roll','pair_12','pair_56','corners','tripod_equal_budget']
LABEL={'no_roll':'N','pair_12':'P12','pair_56':'P56','corners':'C','tripod_equal_budget':'T'}
LONG={'no_roll':'No roll','pair_12':'Pair 1,2','pair_56':'Pair 5,6','corners':'Corners','tripod_equal_budget':'Tripod'}
metrics=['mean_vx_mps','velocity_rmse_mps','absolute_mechanical_work_j','mechanical_cost_of_transport','slip_distance_m','minimum_upright','yaw_change_degrees','lateral_drift_m','mean_stride_clip_fraction']
g=defaultdict(list)
for r in rows:g[(r['vx_command'],r['condition'])].append(r)
sumrows=[]
for (vx,c),v in g.items():
 s={'command':vx,'condition':c,'n':len(v),'completed':sum(x['completed'] for x in v),'ik_failure_frames':sum(x['ik_failure_frames'] for x in v)}
 for m in metrics:
  a=np.array([x[m] for x in v]);s[m+'_mean']=float(a.mean());s[m+'_min']=float(a.min());s[m+'_max']=float(a.max())
 sumrows.append(s)
with (P/'evidence/phase_summary.csv').open('w') as f:
 w=csv.DictWriter(f,fieldnames=list(sumrows[0]));w.writeheader();w.writerows(sumrows)
S={(r['command'],r['condition']):r for r in sumrows}
mu=lambda cmd,c,m='mean_vx_mps':S[(cmd,c)][m+'_mean']
vcheck=max(abs(r['velocity_integral_forward_mps']-r['position_difference_forward_mps']) for r in rows)
paired=[]
for vx in [.18,.45]:
 for c in ['pair_12','pair_56','corners']:
  b={r['phase']:r for r in g[vx,'no_roll']};delta=[r['mean_vx_mps']-b[r['phase']]['mean_vx_mps'] for r in g[vx,c]]
  paired.append({'command':vx,'candidate':c,'reference':'no_roll','n':8,'mean_difference_mps':float(np.mean(delta)),'min_difference_mps':min(delta),'max_difference_mps':max(delta)})
(P/'evidence/paired_phase_differences.json').write_text(json.dumps(paired,indent=2))
summary={'n':len(rows),'completed':sum(r['completed'] for r in rows),'ik_failure_frames':sum(r['ik_failure_frames'] for r in rows),'forbidden_collision_steps':sum(r['forbidden_collision_steps'] for r in rows),'minimum_upright':min(r['minimum_upright'] for r in rows),'maximum_abs_yaw_degrees':max(abs(r['yaw_change_degrees']) for r in rows),'maximum_lateral_drift_m':max(r['lateral_drift_m'] for r in rows),'maximum_velocity_crosscheck_difference_mps':vcheck,'scheduled_stance_min':min(r['scheduled_stance_min'] for r in rows),'pair12_to_corners_speed_ratio_high':mu(.45,'pair_12')/mu(.45,'corners'),'corner_vs_n_speed_ratio_high':mu(.45,'corners')/mu(.45,'no_roll')}
(P/'evidence/audit_summary.json').write_text(json.dumps(summary,indent=2))
(P/'numbers.tex').write_text(''.join('\\newcommand{\\'+name+'}{'+value+'}\n' for name,value in {'CornerSpeed':f"${mu(.45,'corners'):.3f}\\,\\mathrm{{m/s}}$",'NoRollSpeed':f"${mu(.45,'no_roll'):.3f}\\,\\mathrm{{m/s}}$",'PairSpeed':f"${mu(.45,'pair_12'):.3f}\\,\\mathrm{{m/s}}$",'MJVersion':json.loads((P/'evidence/audit_protocol.json').read_text())['mujoco']}.items()))
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':8,'axes.spines.top':False,'axes.spines.right':False,'pdf.fonttype':42,'axes.titlesize':9})
colors=['#687582','#23658c','#ca8730','#086b52','#8b769e']
# Conceptual top view, with explicit schematic status.
f,axs=plt.subplots(1,2,figsize=(3.4,2.30),gridspec_kw={'width_ratios':[1,1.15]})
a=axs[0];a.add_patch(Rectangle((-.22,-.43),.44,.86,facecolor='#edf1f4',edgecolor='#3e4b56'))
for i,(x,y) in enumerate([(-.55,-.47),(.55,-.47),(-.65,0),(.65,0),(-.55,.47),(.55,.47)],1):
 roll=i not in [3,4];a.plot([np.sign(x)*.22,x],[y*.65,y],color='#64737c',lw=3);a.add_patch(Circle((x,y),.115,facecolor='#0b775d' if roll else '#89949c'));a.text(x,y,str(i),color='white',ha='center',va='center',weight='bold');a.text(x,y+(-.18 if y<0 else .19),'ROLL' if roll else 'STEP',ha='center',fontsize=6.5)
a.annotate('body +x',xy=(0,.86),xytext=(0,.50),ha='center',fontsize=6.5,arrowprops={'arrowstyle':'->','lw':1.2});a.set_xlim(-.95,.95);a.set_ylim(-.85,.96);a.set_aspect('equal');a.axis('off');a.set_title('(a) Roles',fontsize=7.5)
a=axs[1];a.add_patch(Wedge((0,0),1,67.5,292.5,width=.14,facecolor='#ca8730',edgecolor='#8a6228'));a.plot([-1.40,-.45,0],[.92,.45,0],color='#526574',lw=5)
for p in [(-1.40,.92),(-.45,.45),(0,0)]:a.add_patch(Circle(p,.045,color='#122e45'))
a.annotate('distal axis',xy=(0,0),xytext=(.22,.30),arrowprops={'arrowstyle':'->'},fontsize=7);a.annotate('finite arc',xy=(-.7,-.70),xytext=(.04,-1.03),arrowprops={'arrowstyle':'->'},fontsize=7);a.plot([-1.3,1.1],[-1,-1],color='#687582',lw=1);a.text(.36,.75,'opening',fontsize=7);a.set_xlim(-1.6,1.25);a.set_ylim(-1.30,1.20);a.set_aspect('equal');a.axis('off');a.set_title('(b) Distal arc',fontsize=7.5)
f.tight_layout();f.savefig(P/'figures/concept.pdf',bbox_inches='tight');plt.close(f)
# Actual planar geometry, no fabricated robot imagery.
z=np.load(P/'evidence/tire_geometry.npz');v=z['vertices']-z['centre'];p=v[:,[1,2]]*1000;h=ConvexHull(p)
f,a=plt.subplots(figsize=(3.4,2.70));poly=p[h.vertices];a.fill(poly[:,0],poly[:,1],color='#b6d1df',alpha=.65,label='Collision hull');a.plot(np.r_[poly[:,0],poly[0,0]],np.r_[poly[:,1],poly[0,1]],color='#23658c',lw=1);a.scatter(p[:,0],p[:,1],s=3,color='#ba7522',label='Mesh vertices');a.scatter([0],[0],s=25,marker='x',color='#bd3030',label='1 mm probe');a.set_aspect('equal');a.set_xlabel('Tire local y (mm)');a.set_ylabel('Tire local z (mm)');a.legend(fontsize=6.5,loc='lower center',bbox_to_anchor=(.5,1.04),ncol=1,frameon=False);f.tight_layout();f.savefig(P/'figures/collision.pdf',bbox_inches='tight');plt.close(f)
# Broad comparison: means and full observed phase ranges.
f,axs=plt.subplots(1,3,figsize=(7,2.43))
for ax,metric,title,unit in zip(axs,['mean_vx_mps','absolute_mechanical_work_j','mechanical_cost_of_transport'],['Net forward speed','Absolute mechanical work','Mechanical transport cost'],['m/s','J','dimensionless']):
 x=np.arange(5)
 for k,cmd in enumerate([.18,.45]):
  mean=np.array([mu(cmd,c,metric) for c in ORDER]);lo=np.array([S[cmd,c][metric+'_min'] for c in ORDER]);hi=np.array([S[cmd,c][metric+'_max'] for c in ORDER]);ax.bar(x+(k-.5)*.36,mean,.34,color='#abc1cc' if k==0 else '#176b60',label=f'{cmd:.2f} m/s command');ax.errorbar(x+(k-.5)*.36,mean,yerr=[mean-lo,hi-mean],fmt='none',ecolor='#152e3b',lw=.7,capsize=2)
 ax.set_xticks(x,[LABEL[c] for c in ORDER]);ax.set_title(title);ax.set_ylabel(unit);ax.grid(axis='y',alpha=.18);ax.set_axisbelow(True)
axs[0].legend(frameon=False,fontsize=6);f.tight_layout(w_pad=1.6);f.savefig(P/'figures/results.pdf',bbox_inches='tight');plt.close(f)
# Tables generated from data.
t='''\\subsection{Transport Across the Phase Grid}
Table~\\ref{tab:speed} reports the same eight phases for every condition. At the high command, C reaches %.3f m/s, %.2f times N; P12 reaches %.3f m/s, or %.1f\\%% of C. At the lower command, C reaches %.3f m/s against %.3f m/s for N. These are observed condition means, not a maximum attainable speed. The ordering between the two allowed pairs favors P12 for both tested commands; it need not persist at other commands or geometries.

\\begin{table}[t]
\\caption{Net forward speed: mean [min, max] over eight phases}
\\label{tab:speed}\\centering\\small
\\begin{tabular}{lcc}\\toprule
Condition & $v_x^c=0.18$ m/s & $v_x^c=0.45$ m/s\\\\\\midrule
'''%(mu(.45,'corners'),summary['corner_vs_n_speed_ratio_high'],mu(.45,'pair_12'),100*summary['pair12_to_corners_speed_ratio_high'],mu(.18,'corners'),mu(.18,'no_roll'))
for c in ORDER:
 vals=[f"{mu(cmd,c):.3f} [{S[cmd,c]['mean_vx_mps_min']:.3f}, {S[cmd,c]['mean_vx_mps_max']:.3f}]" for cmd in [.18,.45]]
 t+=LABEL[c]+' & '+' & '.join(vals)+r'\\'+'\n'
t+='''\\bottomrule\\end{tabular}\\end{table}

Pairing each phase with its N counterpart gives the high-command C-minus-N speed difference %.3f m/s, spanning [%.3f, %.3f] m/s over the grid. The analogous P12-minus-N difference is %.3f m/s. These ranges describe sensitivity to the tested starting phases; they are not confidence intervals. The largest discrepancy between the velocity-integral diagnostic and displacement speed over all runs is %.4f m/s. All reported primary speeds are computed directly from net displacement.

\\subsection{Tracking, Work, and Slip}
\\begin{table}[t]
\\caption{High-command metrics, means over eight phases}
\\label{tab:cost}\\centering\\small
\\begin{tabular}{lrrrr}\\toprule
Condition & RMSE (m/s) & $W_{\\rm abs}$ (J) & $C_{\\rm mt}$ & Slip (m)\\\\\\midrule
'''%(paired[-1]['mean_difference_mps'],paired[-1]['min_difference_mps'],paired[-1]['max_difference_mps'],next(r['mean_difference_mps'] for r in paired if r['command']==.45 and r['candidate']=='pair_12'),vcheck)
for c in ORDER:t+=LABEL[c]+' & '+' & '.join(f'{mu(.45,c,m):.3f}' for m in ['velocity_rmse_mps','absolute_mechanical_work_j','mechanical_cost_of_transport','slip_distance_m'])+r'\\'+'\n'
t+='''\\bottomrule\\end{tabular}\\end{table}

The faster configuration is not cheaper over the fixed eight-second window. C uses %.1f J against %.1f J for N, while transporting farther. Its mechanical cost of transport is %.3f against %.3f, a different normalization with a different interpretation. Thus the result supports neither a blanket efficiency claim nor a claim that the speed gain is free. C also accumulates %.3f m of contact-point slip against %.3f m for N. The travel subtraction in the planner does not enforce a no-slip physical constraint.

The high-command tracking RMSE is %.3f m/s for C and %.3f m/s for N. Even when relative tracking improves, the commanded 0.45 m/s is not achieved. C's high-command yaw change ranges from %.2f to %.2f degrees; its largest lateral displacement is %.3f m. Net forward transport, directional accuracy, and mechanical work must therefore be evaluated together.

\\subsection{Execution and Scope}
All %d trials complete their eight-second windows without nonfinite state or the 60-degree tilt termination. There are %d recorded IK-nonconvergent control frames and %d recorded forbidden chassis/ground collision steps. Minimum uprightness across the complete grid is %.5f, and the minimum scheduled stance count is %d. These are finite-horizon model outcomes, not a population success rate or proof that scheduled feet always carry load.

A separate whole-project test run reports 259 passing test cases and one module-import error among 260 executed test entries. The latest candidate test module is not collected successfully through the public package. This packaging failure is distinct from the direct-run simulation data above, and remains a reproducibility issue to resolve before an external release.
'''%(mu(.45,'corners','absolute_mechanical_work_j'),mu(.45,'no_roll','absolute_mechanical_work_j'),mu(.45,'corners','mechanical_cost_of_transport'),mu(.45,'no_roll','mechanical_cost_of_transport'),mu(.45,'corners','slip_distance_m'),mu(.45,'no_roll','slip_distance_m'),mu(.45,'corners','velocity_rmse_mps'),mu(.45,'no_roll','velocity_rmse_mps'),S[.45,'corners']['yaw_change_degrees_min'],S[.45,'corners']['yaw_change_degrees_max'],S[.45,'corners']['lateral_drift_m_max'],summary['n'],summary['ik_failure_frames'],summary['forbidden_collision_steps'],summary['minimum_upright'],summary['scheduled_stance_min'])
(P/'results_text.tex').write_text(t)
md='| 조건 | 0.18 명령 속도 평균 [범위] | 0.45 명령 속도 평균 [범위] |\n|---|---:|---:|\n'
ko={'no_roll':'굴림 off, 동일 scheduler','pair_12':'1·2번만 굴림','pair_56':'5·6번만 굴림','corners':'네 모서리 굴림','tripod_equal_budget':'동일 cadence·stride tripod'}
for c in ORDER:
 md+='| '+ko[c]+' | '+' | '.join(f"{mu(cmd,c):.4f} [{S[cmd,c]['mean_vx_mps_min']:.4f}, {S[cmd,c]['mean_vx_mps_max']:.4f}]" for cmd in [.18,.45])+' |\n'
md+='\n속도 단위는 m/s이며 순전방 변위를 측정 시간으로 나눈 값이다.\n\n| 조건 (0.45 명령) | RMSE (m/s) | 절대 기계일 (J) | 기계 COT | slip (m) |\n|---|---:|---:|---:|---:|\n'
for c in ORDER:md+='| '+ko[c]+' | '+' | '.join(f'{mu(.45,c,m):.3f}' for m in ['velocity_rmse_mps','absolute_mechanical_work_j','mechanical_cost_of_transport','slip_distance_m'])+' |\n'
md+=f"\n총 {summary['n']}개 시행은 모두 8초 구간을 완료했다. IK 실패 프레임 {summary['ik_failure_frames']}개, 기록된 금지 몸체 접촉 {summary['forbidden_collision_steps']}개, 최소 upright {summary['minimum_upright']:.5f}였다. 별도 MuJoCo 속도 적분 진단과 변위 속도의 최대 차이는 {vcheck:.4f} m/s였다. 1·2번 구성은 네 모서리 구성 속도의 {100*summary['pair12_to_corners_speed_ratio_high']:.1f}%지만, 이것이 같은 효율·안정성이나 2모터 로봇을 뜻하지 않는다.\n"
rp=P/'REVIEW_KO.md';rp.write_text(rp.read_text().replace('@@RESULTS@@',md))
print(json.dumps(summary,indent=2))
