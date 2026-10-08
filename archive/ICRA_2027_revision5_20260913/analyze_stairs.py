"""Generate manuscript tables and figures from all frozen stair trials."""
from pathlib import Path
import json,hashlib
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from PIL import Image
P=Path(__file__).resolve().parent
rows=[json.loads(x) for x in (P/'evidence/stairs/trials.jsonl').read_text().splitlines()]
assert len(rows)==102 and len({r['id'] for r in rows})==102
assert all(not r['git_dirty'] and r['git_revision']=='14d677ab07969898ee3bee69214a30fac16be6f6' for r in rows)
assert not any(r['failure']=='exception' for r in rows)
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':9,'pdf.fonttype':42,'axes.spines.top':False,'axes.spines.right':False})
configs=[('decomposed-arc','fixed','C195'),('decomposed-arc','neutral','C180'),('decomposed-arc','lookup','CL'),('closed-wheel','fixed','W195'),('closed-wheel','neutral','W180')]
summary=[]
for g,c,label in configs:
 for h in [.1,.15,.2]:
  a=[r for r in rows if r['suite']!='timestep' and r['geometry']==g and r['control']==c and r['h']==h]
  ok=[r for r in a if r['halt_success']]
  summary.append(dict(code=label,h=h,n=len(a),ascent=sum(r['ascent_crossed'] for r in a),halt=len(ok),time=np.mean([r['ascent_time_s'] for r in ok]) if ok else None,time_range=[min(r['ascent_time_s'] for r in ok),max(r['ascent_time_s'] for r in ok)] if ok else None,work=np.mean([r['ascent_work_j'] for r in ok]) if ok else None,total_time=np.mean([r['total_time_s'] for r in ok]) if ok else None))
(P/'evidence/stair_summary.json').write_text(json.dumps(summary,indent=2))
lookup={(r['code'],r['h']):r for r in summary}
tex=r'''\begin{table}[t]\centering\small
\caption{Supported halt after three steps, 2 ms physics step}
\label{tab:stairs}\begin{tabular}{lccc}\toprule
 & 100 mm & 150 mm & 200 mm\\\midrule
'''
for _,_,code in configs:
 tex+=code+' & '+' & '.join(f"{lookup[code,h]['halt']}/6" for h in [.1,.15,.2])+r'\\'+'\n'
tex+=r'''\bottomrule\end{tabular}
\par\vspace{4pt}\noindent\parbox{\columnwidth}{\footnotesize Six conditions per cell: two tread depths and three approach poses. C/W denote decomposed arcs/closed wheels. 195/180 are fixed leading-joint angles; CL uses the height lookup. These are deterministic condition counts.}
\end{table}'''
(P/'stairs_table.tex').write_text(tex)
tex=r'''\begin{table}[t]\centering\small
\caption{Ascent cost among successful arc trials, 2 ms}
\label{tab:cost}\begin{tabular}{ccccc}\toprule
Rise & Code & Ascent (s) & Work (J) & Total (s)\\\midrule
'''
for h in [.1,.15]:
 for code in ['C195','C180','CL']:
  r=lookup[code,h];tex+=f"{h*1000:.0f} & {code} & {r['time']:.2f} & {r['work']:.1f} & {r['total_time']:.2f}"+r'\\'+'\n'
tex+=r'''\bottomrule\end{tabular}
\par\vspace{4pt}\noindent\parbox{\columnwidth}{\footnotesize Means over the same six successful conditions per row. Work is absolute actuator mechanical work during ascent. Total time includes scripted pose acquisition, 8 s of brace/phase acquisition, ascent, and stable halt.}
\end{table}'''
(P/'cost_table.tex').write_text(tex)
fig,axs=plt.subplots(1,2,figsize=(7.1,2.55),gridspec_kw={'width_ratios':[1,1.15]})
xs=np.arange(3)
for i,(code,col) in enumerate([('C195','#087f8c'),('C180','#72a5b0'),('W195','#d66a36')]):
 axs[0].bar(xs+(i-1)*.23,[lookup[code,h]['halt'] for h in [.1,.15,.2]],.22,label=code,color=col)
axs[0].set_xticks(xs,['100','150','200']);axs[0].set_xlabel('Riser height (mm)');axs[0].set_ylabel('Supported halts / 6');axs[0].set_ylim(0,7.2);axs[0].legend(fontsize=8,loc='upper right');axs[0].set_title('(a) Shape matters at 2 ms',loc='left',fontsize=10)
for jid,color,label in [(45,'#087f8c','C195, 150 mm'),(47,'#d66a36','W195, 150 mm'),(75,'#713b73','C195, 200 mm')]:
 r=next(r for r in rows if r['id']==jid);t=r['trace'];axs[1].plot([x['t'] for x in t],[x['min_center_y']-r['last_riser_y'] for x in t],label=label,color=color)
axs[1].axhline(.1225,c='black',lw=.8,ls='--',label='Rear-clearance threshold');axs[1].set_xlabel('Time after acquisition (s)');axs[1].set_ylabel('Rear axle beyond last riser (m)');axs[1].set_title('(b) Body progress is insufficient',loc='left',fontsize=10);axs[1].legend(fontsize=7,loc='lower right')
fig.tight_layout();fig.savefig(P/'figures/stair_overview.pdf');fig.savefig(P/'figures/stair_overview.png',dpi=180);plt.close(fig)
fig,axs=plt.subplots(2,3,figsize=(7.1,3.3))
for row,jid in enumerate([45,47]):
 for col,stamp in enumerate([0,2,4]):
  im=Image.open(P/f'figures/stair_{jid}_{stamp}.png');axs[row,col].imshow(im);axs[row,col].set_xlim(40,1200);axs[row,col].set_ylim(710,170);axs[row,col].axis('off');axs[row,col].set_title(f"{'C195' if row==0 else 'W195'}, t = {[.02,1.98,3.98][col]:.2f} s",fontsize=8,pad=2)
fig.subplots_adjust(left=0,right=1,bottom=0,top=.96,hspace=.16,wspace=.025);fig.savefig(P/'figures/stair_replay.pdf');fig.savefig(P/'figures/stair_replay.png',dpi=180);plt.close(fig)
fig,axs=plt.subplots(1,2,figsize=(3.45,1.65))
for ax,path,label in zip(axs,[P/'figures/stair_45.jpg',P/'figures/stair_75.jpg'],['150 mm: top-supported halt','200 mm: rear bank below top']):
 ax.imshow(Image.open(path));ax.set_xlim(40,1200);ax.set_ylim(710,170);ax.axis('off');ax.set_title(label,fontsize=6.8,pad=2)
fig.subplots_adjust(left=0,right=1,bottom=0,top=.93,wspace=.03);fig.savefig(P/'figures/stair_endpoint.pdf');plt.close(fig)
# A compact schematic describes control modes, not an observed autonomous transition.
fig,ax=plt.subplots(figsize=(3.45,1.8));ax.set_xlim(0,10);ax.set_ylim(0,5);ax.axis('off')
for x,title,text in [(0.1,'Flat mode P','Periodic roll / step\nBounded distal targets\n18 joint target governor'),(5.2,'Stair mode C195','Fixed joint posture\nOne unwrapped phase\n6 position tracking loops')]:
 ax.add_patch(plt.Rectangle((x,.55),4.65,3.35,facecolor='#eaf2f4',edgecolor='#2b697b'))
 ax.text(x+2.325,3.3,title,ha='center',fontsize=10,weight='bold');ax.text(x+2.325,2.0,text,ha='center',va='center',fontsize=8,linespacing=1.6)
ax.text(5,4.6,'One articulated arc mechanism, two control modes',ha='center',fontsize=9)
ax.text(5,.1,'Operator-selected modes; end-to-end transition untested',ha='center',fontsize=7,color='#535b61')
fig.subplots_adjust(0,0,1,1);fig.savefig(P/'figures/hybrid_modes.pdf');plt.close(fig)
print('102 records validated; stair figures and tables written')
