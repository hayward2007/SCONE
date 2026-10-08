from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle, Wedge
P=Path(__file__).resolve().parent
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':8,'pdf.fonttype':42})
# Conceptual top view, with explicit schematic status.
f,axs=plt.subplots(1,2,figsize=(3.4,2.30),gridspec_kw={'width_ratios':[1,1.15]})
a=axs[0];a.add_patch(Rectangle((-.22,-.43),.44,.86,facecolor='#edf1f4',edgecolor='#3e4b56'))
for i,(x,y) in enumerate([(-.55,-.47),(.55,-.47),(-.65,0),(.65,0),(-.55,.47),(.55,.47)],1):
 roll=i not in [3,4];a.plot([np.sign(x)*.22,x],[y*.65,y],color='#64737c',lw=3);a.add_patch(Circle((x,y),.115,facecolor='#0b775d' if roll else '#89949c'));a.text(x,y,str(i),color='white',ha='center',va='center',weight='bold');a.text(x,y+(-.18 if y<0 else .19),'ROLL' if roll else 'STEP',ha='center',fontsize=6.5)
a.annotate('body +x',xy=(0,.86),xytext=(0,.50),ha='center',fontsize=6.5,arrowprops={'arrowstyle':'->','lw':1.2});a.set_xlim(-.95,.95);a.set_ylim(-.85,.96);a.set_aspect('equal');a.axis('off');a.set_title('(a) Roles',fontsize=7.5)
a=axs[1];a.add_patch(Wedge((0,0),1,67.5,292.5,width=.14,facecolor='#ca8730',edgecolor='#8a6228'));a.plot([-1.40,-.45,0],[.92,.45,0],color='#526574',lw=5)
for p in [(-1.40,.92),(-.45,.45),(0,0)]:a.add_patch(Circle(p,.045,color='#122e45'))
a.annotate('distal axis',xy=(0,0),xytext=(.22,.30),arrowprops={'arrowstyle':'->'},fontsize=7);a.annotate('finite arc',xy=(-.7,-.70),xytext=(.12,-1.27),arrowprops={'arrowstyle':'->'},fontsize=7);a.plot([-1.3,1.1],[-1,-1],color='#687582',lw=1);a.text(.36,.75,'opening',fontsize=7);a.set_xlim(-1.6,1.25);a.set_ylim(-1.50,1.20);a.set_aspect('equal');a.axis('off');a.set_title('(b) Distal arc',fontsize=7.5)
f.tight_layout();f.savefig(P/'figures/concept.pdf',bbox_inches='tight');plt.close(f)
