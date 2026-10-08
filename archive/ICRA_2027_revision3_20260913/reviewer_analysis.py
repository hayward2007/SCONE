"""Post-hoc manuscript audit of the preserved 268-trial confirmation matrix.

This file neither runs simulations nor changes their recorded outcomes.
"""
from pathlib import Path
import hashlib
import json
import statistics as st
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

P = Path(__file__).resolve().parent
raw = P / 'evidence/confirmation/raw.jsonl'
rows = [json.loads(line) for line in raw.read_text().splitlines()]
assert len(rows) == 268 and len({r['id'] for r in rows}) == 268

def main(code, command=.45):
    return sorted([r for r in rows if r['suite'] == 'main' and r['code'] == code
                   and r['command'][0] == command], key=lambda r:r['phase'])

metrics = ['mean_vx_mps', 'sector_rate_peak_dps', 'distal_target_rate_peak_dps',
           'absolute_mechanical_work_j', 'slip_distance_m', 'mechanical_cost_of_transport']
audit = {'analysis_type':'post-hoc reanalysis; no new trials',
         'raw_sha256':hashlib.sha256(raw.read_bytes()).hexdigest(), 'main_high':{}}
for code in 'UBNP':
    a = main(code)
    assert len(a) == 8 and all(r['completed'] for r in a)
    audit['main_high'][code] = {k:{'mean':st.mean(r[k] for r in a),
                                  'min':min(r[k] for r in a),
                                  'max':max(r[k] for r in a)} for k in metrics}
audit['paired_B_minus_U_target_peak_dps'] = [b['distal_target_rate_peak_dps']-u['distal_target_rate_peak_dps'] for b,u in zip(main('B'),main('U'))]
audit['paired_B_minus_P_speed_mps'] = [b['mean_vx_mps']-p['mean_vx_mps'] for b,p in zip(main('B'),main('P'))]
audit['worst_observed_target_peak_reduction_percent'] = 100*(1-audit['main_high']['B']['distal_target_rate_peak_dps']['max']/audit['main_high']['U']['distal_target_rate_peak_dps']['max'])
audit['scope_notes'] = ['Target derivatives exclude the first measurement command, as in the original recorder.',
                        'Maxima compare maxima over eight prescribed phases, not matched individual episodes.',
                        'The 540 degree/s bound applies only to the sector component.',
                        'Periodic P has comparable speed and lower aggregate target peak than B.']
(P/'evidence/reviewer_reanalysis.json').write_text(json.dumps(audit,indent=2))

plt.rcParams.update({'font.family':'DejaVu Sans','font.size':7.2,'pdf.fonttype':42,
                     'axes.spines.top':False,'axes.spines.right':False})
fig = plt.figure(figsize=(3.4,2.95))
ax = fig.add_axes([.16,.63,.79,.26])
t = np.linspace(.08,.40,120)
ax.plot(t,np.minimum(100,(8/15)*540*t),color='#146c65',lw=1.7)
ax.axhline(100,color='#566875',ls=':',lw=.8)
ax.scatter([.2],[57.6],color='#146c65',s=20,zorder=3)
ax.annotate('57.6 deg at 0.20 s',(.2,57.6),xytext=(.235,34),fontsize=7,
            arrowprops={'arrowstyle':'-','lw':.6})
ax.text(.09,104,'Geometric limit: 100 deg',fontsize=6.7,color='#566875')
ax.set(xlim=(.08,.4),ylim=(0,118),xlabel='Available swing time (s)',ylabel='Excursion bound (deg)')
ax.set_title('(a) Time available to rewind limits rolling',loc='left',fontsize=7.4,pad=9)
ax.tick_params(labelsize=6.8);ax.grid(alpha=.15)

ax = fig.add_axes([.16,.12,.79,.28])
codes=['U','B','P'];color=['#bc6b2c','#146c65','#7b68a0']
peaks=[audit['main_high'][c]['sector_rate_peak_dps']['max'] for c in codes]
ax.bar(range(3),peaks,color=color,width=.55)
ax.axhline(540,color='black',ls='--',lw=.7)
for i,(code,peak) in enumerate(zip(codes,peaks)):
    ax.text(i,peak+48,f'{peak:.0f}',ha='center',fontsize=7.2)
ax.set_xticks(range(3),['Original U','Proposed B','Periodic P'])
ax.set(ylim=(0,1890),ylabel='Sector peak (deg/s)')
ax.set_title('(b) Measured at a 0.45 m/s command',loc='left',fontsize=7.4,pad=9)
ax.text(1.83,650,'540 limit',fontsize=6.8,ha='right')
ax.tick_params(labelsize=6.8)
fig.text(.56,.014,'Mean speed: 0.283 / 0.283 / 0.284 m/s',ha='center',fontsize=7)
fig.savefig(P/'figures/overview.pdf');fig.savefig(P/'figures/overview.png',dpi=220);plt.close(fig)
print(json.dumps({'post_hoc_trials':len(rows),'max_target_reduction_percent':audit['worst_observed_target_peak_reduction_percent']},indent=2))
