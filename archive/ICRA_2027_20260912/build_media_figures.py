"""Compose scientific panels from direct video frames and simulation renders."""
from pathlib import Path
import subprocess
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from PIL import Image,ImageOps
P=Path(__file__).resolve().parent;F=P/'figures'
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':8,'pdf.fonttype':42})
def picture(ax,name,title):
 im=ImageOps.contain(Image.open(F/name).convert('RGB'),(800,500))
 canvas=Image.new('RGB',(800,500),'white');canvas.paste(im,((800-im.width)//2,(500-im.height)//2))
 ax.imshow(canvas);ax.axis('off');ax.set_title(title,loc='left',fontsize=8,pad=4)
fig,ax=plt.subplots(2,2,figsize=(3.45,2.85),layout='constrained')
picture(ax[0,0],'archive_SCONEv2_03.png','(a) Physical archive, 3 s')
picture(ax[0,1],'archive_SCONEv2_18.png','(b) Physical archive, 18 s')
picture(ax[1,0],'archive_arc_detail.png','(c) Historical arc drawing')
picture(ax[1,1],'sim_corners_start.png','(d) Evaluated simulation')
fig.savefig(F/'platform.pdf',bbox_inches='tight');fig.savefig(F/'platform.png',dpi=240,bbox_inches='tight');plt.close(fig)
fig,ax=plt.subplots(1,4,figsize=(7.05,3.05),layout='constrained')
for a,t in zip(ax,[18,24,30,36]):
 a.imshow(Image.open(F/f'archive_SCONEv2_stairs_{t:02d}.png'));a.axis('off');a.set_title(f'Physical archive | {t} s',fontsize=8)
fig.savefig(F/'hardware_stairs.pdf',bbox_inches='tight');fig.savefig(F/'hardware_stairs.png',dpi=180,bbox_inches='tight');plt.close(fig)
fig,ax=plt.subplots(3,3,figsize=(7.05,4.35),layout='constrained')
for i,(name,label) in enumerate([('no_roll','N: rolling disabled'),('pair_12','P12: IDs 1 and 2'),('corners','C: corner legs')]):
 for j,t in enumerate([0,4,7.96]):
  dest=F/f'replay_{name}_{j}.png'
  subprocess.run(['ffmpeg','-v','error','-ss',str(t),'-i',str(P/f'media/sim_{name}.mp4'),'-frames:v','1','-y',str(dest)],check=True)
  picture(ax[i,j],dest.name,f'{label} | {t:g} s' if j==0 else f'{t:g} s')
fig.savefig(F/'replay.pdf',bbox_inches='tight');fig.savefig(F/'replay.png',dpi=180,bbox_inches='tight');plt.close(fig)
