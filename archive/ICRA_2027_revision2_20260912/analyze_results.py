from pathlib import Path
from collections import defaultdict
import json,csv
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull
P=Path(__file__).resolve().parent;E=P/'evidence/confirmation'
r=[json.loads(l) for l in (E/'raw.jsonl').read_text().splitlines()]
assert len(r)==268 and len({x['id'] for x in r})==268
assert not json.loads((E/'source_stability.json').read_text())['changed']
MAIN=lambda c,v:[x for x in r if x['suite']=='main' and x['code']==c and x['command'][0]==v]
complete=lambda a:[x for x in a if x['completed']]
mean=lambda a,k:float(np.mean([x[k] for x in a]))
stat=lambda a,k:(mean(a,k),min(x[k] for x in a),max(x[k] for x in a))
fmt=lambda a,k:('%.3f [%.3f, %.3f]'%stat(a,k)) if a else '--'
b=MAIN('B',.45);n=MAIN('N',.45);u=MAIN('U',.45)
paired=[x['mean_vx_mps']-next(y['mean_vx_mps'] for y in n if y['phase']==x['phase']) for x in b]
summary=dict(trials=len(r),completed=sum(x['completed'] for x in r),failure_counts=dict(__import__('collections').Counter(x['termination_reason'] for x in r if not x['completed'])),
    speed_B_high=mean(b,'mean_vx_mps'),speed_N_high=mean(n,'mean_vx_mps'),speed_U_high=mean(u,'mean_vx_mps'),
    speed_ratio_B_N=mean(b,'mean_vx_mps')/mean(n,'mean_vx_mps'),speed_ratio_B_U=mean(b,'mean_vx_mps')/mean(u,'mean_vx_mps'),
    paired_B_N_mean=float(np.mean(paired)),paired_B_N_min=min(paired),paired_B_N_max=max(paired),
    peak_sector_U=max(x['sector_rate_peak_dps'] for x in u),peak_sector_B=max(x['sector_rate_peak_dps'] for x in b),
    max_speed_integral_difference=max(abs(x['velocity_integral_forward_mps']-x['mean_vx_mps']) for x in complete(r)),
    B_total=sum(x['code']=='B' for x in r),B_completed=sum(x['code']=='B' and x['completed'] for x in r),
    B_rate_violation_frames=sum(x['sector_rate_violation_frames'] for x in r if x['code']=='B'),
    B_max_distal_target_rate=max(x['distal_target_rate_peak_dps'] for x in r if x['code']=='B'),
    B_min_loaded_legs=min(x['loaded_leg_min'] for x in r if x['code']=='B'),
    B_main_max_fraction_under_three=max(x['fraction_physics_samples_below_three_loaded'] for x in r if x['code']=='B' and x['suite']=='main'))
(P/'evidence/summary.json').write_text(json.dumps(summary,indent=2))
with (P/'evidence/all_trials.csv').open('w') as f:
 w=csv.DictWriter(f,fieldnames=list(r[0]));w.writeheader();w.writerows(r)
macros={'BudgetSpeed':f"${summary['speed_B_high']:.3f}\\,\\mathrm{{m/s}}$",'WalkSpeed':f"${summary['speed_N_high']:.3f}\\,\\mathrm{{m/s}}$",'AbstractRateResult':f"The maximum sector-command rate decreases from {summary['peak_sector_U']:.0f} to {summary['peak_sector_B']:.0f} degree/s relative to the original allocator, retaining {summary['speed_ratio_B_U']*100:.2f}\\% of its mean forward speed."}
(P/'numbers.tex').write_text(''.join('\\newcommand{\\'+k+'}{'+v+'}\n' for k,v in macros.items()))
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':8,'axes.spines.top':False,'axes.spines.right':False,'pdf.fonttype':42})
colors={'U':'#bc6b2c','B':'#146c65','G':'#b84c55','N':'#6d7f91','P':'#7b68a0'}
f,ax=plt.subplots(1,2,figsize=(7,2.15))
for code in ['U','G','B']:
 t=json.loads((E/f'trace_{code}.json').read_text());x=[a['time_s'] for a in t]
 ax[0].plot(x,[a['sector_degrees'][0] for a in t],label=code,color=colors[code],lw=1)
 ax[1].plot(x,[max(a['sector_rates']) for a in t],label=code,color=colors[code],lw=1)
for sign in [-1,1]:ax[0].axhline(sign*57.6,ls='--',color='black',lw=.7)
ax[0].set(xlabel='Measurement time (s)',ylabel='Leg 1 sector offset (deg)')
ax[1].axhline(540,ls='--',color='black',lw=.7);ax[1].set(xlabel='Measurement time (s)',ylabel='Peak across legs (deg/s)')
for a in ax:a.grid(alpha=.18);a.legend(frameon=False,ncol=3,fontsize=7)
f.tight_layout();f.savefig(P/'figures/rate_budget.pdf',bbox_inches='tight');f.savefig(P/'figures/rate_budget.png',dpi=180,bbox_inches='tight');plt.close(f)
# Genuine mesh cross sections; no generated imagery.
payload=json.loads((P.parents[1]/'benchmark/assets/tire_coacd_2mm.json').read_text())
z=np.load(P/'evidence/collision_vertices.npz');v=(z['original']-z['centre'])[:,1:]*1000
f,axs=plt.subplots(1,2,figsize=(3.45,1.95),sharey=True)
h=ConvexHull(v);axs[0].fill(v[h.vertices,0],v[h.vertices,1],color='#aec4ce',alpha=.9)
for k,part in enumerate(payload['parts']):
 p=(np.array(part['vertices'])-z['centre'])[:,1:]*1000;h=ConvexHull(p)
 axs[1].fill(p[h.vertices,0],p[h.vertices,1],color=plt.cm.tab20(k%20),alpha=.75,lw=.25,edgecolor='white')
for a,title in zip(axs,['Single hull','24 convex pieces']):
 a.scatter([0],[0],marker='x',color='#a32637',s=15);a.set_aspect('equal');a.set_title(title,fontsize=8);a.set_xlabel('Local y (mm)',fontsize=7);a.tick_params(labelsize=6)
axs[0].set_ylabel('Local z (mm)',fontsize=7);f.tight_layout(w_pad=.5);f.savefig(P/'figures/collision.pdf',bbox_inches='tight');f.savefig(P/'figures/collision.png',dpi=180,bbox_inches='tight');plt.close(f)
# Means and full phase ranges for the four fully completed conditions.
f,axs=plt.subplots(1,3,figsize=(7,2.2));codes=['N','U','B','P']
for ax,metric,label in zip(axs,['mean_vx_mps','absolute_mechanical_work_j','slip_distance_m'],['Net forward speed (m/s)','Absolute joint work (J)','Integrated slip (m)']):
 for j,cmd in enumerate([.18,.45]):
  a=[MAIN(c,cmd) for c in codes];mu=np.array([mean(x,metric) for x in a]);low=np.array([min(y[metric] for y in x) for x in a]);high=np.array([max(y[metric] for y in x) for x in a]);pos=np.arange(4)+(j-.5)*.35
  ax.bar(pos,mu,.32,color='#b6c9cc' if j==0 else '#146c65',label=f'{cmd:.2f} m/s command');ax.errorbar(pos,mu,yerr=[mu-low,high-mu],fmt='none',ecolor='#273943',capsize=2,lw=.7)
 ax.set_xticks(range(4),codes);ax.set_ylabel(label);ax.grid(axis='y',alpha=.2);ax.set_axisbelow(True)
f.legend(*axs[0].get_legend_handles_labels(),loc='upper center',ncol=2,fontsize=7,frameon=False);f.tight_layout(rect=(0,0,1,.88));f.savefig(P/'figures/results.pdf',bbox_inches='tight');f.savefig(P/'figures/results.png',dpi=180,bbox_inches='tight');plt.close(f)
text=r'''\subsection{Rewind Feasibility and Forward Transport}
Table~\ref{tab:main} separates completion from full-window transport. B, U, N, and P complete all 16 main trials. At the high command, B reaches %.3f m/s against N's %.3f m/s, a factor of %.2f. The phase-paired B-minus-N gain averages %.3f m/s and spans [%.3f, %.3f] m/s. B retains %.2f\%% of U's mean speed while reducing the maximum sector rate from %.1f to %.1f degree/s. U exceeds the prescribed rate in every high-command trial; B has no violating frames in the main matrix.

\begin{table*}[t]\centering\small
\caption{Main matrix: completion and full-window speed, mean [minimum, maximum]}
\label{tab:main}
\begin{tabular}{cccccc}\toprule
 & \multicolumn{2}{c}{0.18 m/s command} & \multicolumn{2}{c}{0.45 m/s command} & High-command sector\\
Code & Complete & Speed (m/s) & Complete & Speed (m/s) & peak rate (degree/s)\\\midrule
'''%(summary['speed_B_high'],summary['speed_N_high'],summary['speed_ratio_B_N'],summary['paired_B_N_mean'],summary['paired_B_N_min'],summary['paired_B_N_max'],100*summary['speed_ratio_B_U'],summary['peak_sector_U'],summary['peak_sector_B'])
for code in 'UBGNSDP':
 a=MAIN(code,.18);b=MAIN(code,.45);text+=f"{code} & {len(complete(a))}/8 & {fmt(complete(a),'mean_vx_mps')} & {len(complete(b))}/8 & {fmt(complete(b),'mean_vx_mps')} & {max(x['sector_rate_peak_dps'] for x in b):.1f}\\\\\n"
text+=r'''\bottomrule\end{tabular}
\par\vspace{5pt}\noindent\parbox{.96\textwidth}{\footnotesize Peaks for failed conditions cover only the interval before termination. A dash indicates that no trial completed the full 8 s; truncated speeds are not averaged into successful performance.}
\end{table*}

The distance-only restriction G completes the low-command trials but rejects an IK target in all eight high-command trials. Bounding sector excursion without anticipating the next swing can exhaust the rolling budget while the leg remains in stance; the resulting articulated residual becomes unreachable. Adding the lookahead in B restores completion for these trials. The full-superposition restriction S and disabled-reindexing restriction D reject targets in all main trials. These outcomes concern this posture, workspace, and command range; they do not imply that all superposed or nonreindexing controllers must fail.

Periodic scheduling P also completes all main trials and has essentially the same high-command speed as B. At the low command, B has only a small mean advantage. Thus these trials support the need for timely reindexing but do not establish adaptive scheduling as superior to periodic scheduling. The stronger contrast is rate feasibility against U, and reachable residual motion against G and S.

\begin{figure*}[t]\centering\includegraphics[width=.97\textwidth]{results.pdf}
\caption{Four conditions that complete every main trial. Bars are eight-phase means; whiskers are full phase ranges. G, S, and D are reported with their failures in Table~\ref{tab:main}. Faster transport is accompanied by changes in joint work and slip, so speed alone is not an efficiency result.}
\label{fig:results}\end{figure*}
'''
b=MAIN('B',.45);n=MAIN('N',.45)
text+=r'''\subsection{Work, Tracking, and Loaded Support}
At the high command, B uses %.1f J of absolute joint work against N's %.1f J. Integrated slip is %.3f versus %.3f m. Their respective translational RMSE values are %.3f and %.3f m/s. B therefore improves net transport without demonstrating uniformly lower work or slip. Its main trials include a minimum loaded-leg count of %d, and up to %.2f\%% of physics samples have fewer than three. This illustrates why a three-leg stance schedule is not a three-contact load guarantee. The summed distal target reaches %.1f degree/s across the B trials, exceeding the sector-component bound; applying the method to a physical actuator requires a joint-level trajectory constraint.

'''%(mean(b,'absolute_mechanical_work_j'),mean(n,'absolute_mechanical_work_j'),mean(b,'slip_distance_m'),mean(n,'slip_distance_m'),mean(b,'velocity_rmse_mps'),mean(n,'velocity_rmse_mps'),min(x['loaded_leg_min'] for x in r if x['code']=='B' and x['suite']=='main'),100*summary['B_main_max_fraction_under_three'],summary['B_max_distal_target_rate'])
text+=r'''\subsection{Commands Beyond Forward Motion}
Table~\ref{tab:domain} reports all seven additional commands. Translational and yaw errors are separate because a controller can remain upright while tracking poorly. Each comparison uses the same four initial phases. The results describe constant-command operating points, including cases where both B and N fall back to stepping.
\begin{table*}[t]\centering\small
\caption{Additional commands: completion and tracking error over completed trials}
\label{tab:domain}\begin{tabular}{lccccc}\toprule
Command $(v_x,v_y,\omega_z)$ & Complete B/N/S & B $e_v$ & N $e_v$ & B $e_\omega$ & N $e_\omega$\\\midrule
'''
commands=list(dict.fromkeys(tuple(x['command']) for x in r if x['suite']=='domain'))
for cmd in commands:
 a={c:[x for x in r if x['suite']=='domain' and x['code']==c and tuple(x['command'])==cmd] for c in 'BNS'}
 comp='/'.join(str(len(complete(a[c]))) for c in 'BNS');values=[]
 for k,c in [('velocity_rmse_mps','B'),('velocity_rmse_mps','N'),('yaw_rate_rmse_rps','B'),('yaw_rate_rmse_rps','N')]:values.append(f'{mean(complete(a[c]),k):.3f}' if complete(a[c]) else '--')
 text+='('+', '.join(f'{v:g}' for v in cmd)+') & '+comp+' & '+' & '.join(values)+r'\\'+'\n'
text+=r'''\bottomrule\end{tabular}
\par\vspace{5pt}\noindent\parbox{.96\textwidth}{\footnotesize Each completion count is out of four. $e_v$ is translational RMSE (m/s); $e_\omega$ is yaw-rate RMSE (rad/s). Units of the command are m/s, m/s, and rad/s. Failed-trial errors are not mixed with full-window errors.}
\end{table*}

\subsection{Parameter, Geometry, and Timestep Checks}
'''
for key in ['mass_scale','friction_scale','actuator_strength_scale']:
 a={c:[x for x in r if x['suite']=='stress' and x['code']==c and key in x['stress']] for c in 'BN'}
 name={'mass_scale':'1.2-fold mass','friction_scale':'half nominal friction','actuator_strength_scale':'0.8-fold actuator strength'}[key]
 text+=f"With {name}, B completes {len(complete(a['B']))}/4 trials and N completes {len(complete(a['N']))}/4. "
 if complete(a['B']) and complete(a['N']):text+=f"Their mean speeds are {mean(complete(a['B']),'mean_vx_mps'):.3f} and {mean(complete(a['N']),'mean_vx_mps'):.3f} m/s. "
text+='These isolated stress cases are not a population robustness estimate.\n\n'
geometry_delta=[];time_delta=[]
for x in r:
 if x['suite'] not in ['geometry','timestep'] or not x['completed']:continue
 ref=next(y for y in r if y['suite']=='main' and y['code']==x['code'] and y['command']==x['command'] and y['phase']==x['phase'])
 if ref['completed']:
  (geometry_delta if x['suite']=='geometry' else time_delta).append((x['code'],x['mean_vx_mps']-ref['mean_vx_mps']))
summary['max_geometry_speed_change']=max(abs(d) for c,d in geometry_delta)
summary['max_timestep_speed_change']=max(abs(d) for c,d in time_delta)
text+=r'''For paired completed B/N trials, changing from decomposed contacts to the single hull changes net speed by at most %.4f m/s. The 1/2/4 ms comparison changes speed by at most %.4f m/s relative to 2 ms. These are local sensitivity results, not a proof of solver convergence or concave-edge accuracy. Contact-point slip is not compared across the two representations because decomposition changes the contact manifold.

Across the prescribed matrix, %d/%d trials complete; all incomplete trials terminate because an IK target is rejected. B completes %d/%d trials with %d sector-rate violation frames. A separate 20-trial check integrates the velocity at the same root origin used for displacement. Its maximum speed discrepancy is %.5f m/s. Velocity at the body center of mass is retained as a diagnostic at a different point and is not substituted for velocity at the root origin. The three visual replays in Fig.~\ref{fig:replay} reproduce the corresponding nominal records to the verified numerical tolerance.
'''%(summary['max_geometry_speed_change'],summary['max_timestep_speed_change'],summary['completed'],len(r),summary['B_completed'],summary['B_total'],summary['B_rate_violation_frames'],json.loads((P/'evidence/velocity_validation.json').read_text())['maximum_origin_difference_mps'])
(P/'results_text.tex').write_text(text)
(P/'evidence/summary.json').write_text(json.dumps(summary,indent=2))
print(json.dumps(summary,indent=2))
