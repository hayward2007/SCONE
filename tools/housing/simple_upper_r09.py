"""Clean planar covers: preserve interfaces, consolidate vents, omit inherited cavities."""
from pathlib import Path
import json,sys,math
import numpy as np
sys.path.insert(0,str(Path(__file__).parent))
import faceted_r08 as g
from OCP.gp import gp_Pnt,gp_Dir,gp_Ax2
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox,BRepPrimAPI_MakeCylinder
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_FACE
SRC=g.BASE;DEST=SRC.parent/'R09_DELIVERY';DEST.mkdir(exist_ok=True)
def box(xa,xb,ya,yb,za,zb):return BRepPrimAPI_MakeBox(gp_Pnt(xa,ya,za),xb-xa,yb-ya,zb-za).Shape()
def cyl(a,b,rad):
 v=np.subtract(b,a);return BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(*a),gp_Dir(*v)),rad,float(np.linalg.norm(v))).Shape()
def faces(q):
 e=TopExp_Explorer(q,TopAbs_FACE);n=0
 while e.More():n+=1;e.Next()
 return n
def run():
 outer=g.read('outer_master.step');shell=g.read('shell_master.step');cv=g.common(shell,box(-100,260,-120,120,33.8,110));before=g.vol(cv)
 for xc in [-41.8912974452,208.1087025548]:
  for s in [-1,1]:
   cv=g.cut(cv,box(xc-18.01,xc+18.01,26.3 if s>0 else -82,82 if s>0 else -26.3,30,44.6))
   cv=g.cut(cv,box(xc-19.8,xc+19.8,21.2 if s>0 else -43.3,43.3 if s>0 else -21.2,33.7,37))
 for x,ya in [(-14,25),(77,72),(97,69),(180,25)]:
  for s in [-1,1]:
   y=s*ya;boss=g.fuse(cyl((x,y,33.8),(x,y,43),4.8),box(x-4.8,x+4.8,y if s>0 else -80,80 if s>0 else y,35,42));cv=g.fuse(cv,g.common(boss,outer))
   cv=g.cut(cv,cyl((x,y,33.4),(x,y,39.2),2));cv=g.cut(cv,cyl((x,y,33.3),(x,y,42.5),1.4))
 # Functional openings: 3 roof slots replace 7, maintaining 1050 mm2 total open area.
 for y in [-17,0,17]:cv=g.cut(cv,box(75,125,y-3.5,y+3.5,98,110))
 for s in [-1,1]:
  for xa,xb in [(-10,12),(151,182)]:cv=g.cut(cv,box(xa,xb,s*56-5,s*56+5,62,68))
 for x,y in [(38.9,-14.1),(67.1,14.1)]:cv=g.cut(cv,cyl((x,y,98),(x,y,106),1.15))
 cv=g.cut(cv,box(48,58,-27,-19,99,106))
 cv=g.cut(cv,box(-75,8,-24,-9,53,62));cv=g.cut(cv,box(-75,8,3,23,47,68))
 # Four straight circular posts clear SMT parts while keeping the four real mounting axes.
 for y in [-10.5,10.5]:
  for z in [71.7,85.15]:
   post=cyl((209.8,y,z),(227,y,z),2.65)
   cv=g.fuse(cv,g.common(post,outer))
   cv=g.cut(cv,cyl((209.4,y,z),(214,y,z),1.5));cv=g.cut(cv,cyl((209.4,y,z),(219,y,z),1))
 for y in [-30.05,30.05]:cv=g.cut(cv,cyl((137,y,78),(240,y,78),12))
 # One flat tip relief clears the two lower-hole SMT neighborhoods, without imprinting component detail.
 cv=g.cut(cv,box(209.4,211.0,-15,15,73.85,76.1))
 # High panels and ribs. Roof and all mounting bands remain 2.8 mm nominal.
 for s in [-1,1]:
  for xa,xb in [(-52,-14),(-8,24),(30,72),(90,132),(138,180),(186,219)]:cv=g.cut(cv,box(xa,xb,52.9 if s>0 else -53.8,53.8 if s>0 else -52.9,59,93))
  for x in [-11,27,75,87,135,183]:cv=g.fuse(cv,box(x-.9,x+.9,52 if s>0 else -53.5,53.5 if s>0 else -52,58.5,95))
  cv=g.fuse(cv,box(-54,221,s*40-.9,s*40+.9,97.8,101.4))
 # Cut head reserves as featureless cylinders; never imprint tray screw holes into the cover.
 for s in [-1,1]:
  cv=g.cut(cv,cyl((33,s*48,33.7),(33,s*48,36.6),4.3))
  cv=g.cut(cv,box(28.7,43.3,s*48-4.8,s*48+4.8,36.05,39.8))
 for x,ya,z in [(33,48,39.5),(125,50,41)]:
  for s in [-1,1]:cv=g.cut(cv,cyl((x,s*ya,z-.2),(x,s*ya,z+2.35),3.1))
 # Keep the complete wheel rotation envelopes and exact user demonstrated axis.
 saved={o['name']:o for o in json.loads((SRC/'user_pose_before.json').read_text())['occurrences']};m=np.array(saved['LEG 1:5+ARC:1']['transform']).reshape(4,4);m[:3,3]*=10
 a=m[:3,:3]@np.array([0,0,-2.1])+m[:3,3];b=m[:3,:3]@np.array([0,0,22.1])+m[:3,3]
 for s in [-1,1]:
  aa=a.copy();bb=b.copy();aa[1]*=s;bb[1]*=s;cv=g.cut(cv,cyl(aa,bb,124.6))
  cv=g.cut(cv,cyl((113.1087026,s*187.640925829,18.5),(142.5087026,s*187.640925829,18.5),124.8))
 result=[]
 for tag,xa,xb in [('S01',-100,82.85),('S02',83.15,260)]:
  q=g.clean(g.common(cv,box(xa,xb,-120,120,30,110)));old=g.read('PRINT_PACKAGE/'+tag+'.step')
  assert BRepCheck_Analyzer(q).IsValid() and len(g.solids(q))==1,(tag,len(g.solids(q)))
  g.BASE=DEST;g.write(q,tag+'_clean.step');g.BASE=SRC
  result.append(dict(part=tag,old_faces=faces(old),new_faces=faces(q),old_cm3=g.vol(old)/1000,new_cm3=g.vol(q)/1000))
 (DEST/'upper_simplification.json').write_text(json.dumps(dict(parts=result,roof_vent_area_mm2=dict(before=1050,after=1050),side_vent_area_mm2=dict(before=546,after=636),roof_vent_count=dict(before=7,after=3),side_vent_count=dict(before=14,after=4),unchanged=['fore-aft 289mm','lid and sensor screw axes','camera placement','motor bridges','20mm tire swept clearance','FR07']),indent=2));print(result)
if __name__=='__main__':run()
