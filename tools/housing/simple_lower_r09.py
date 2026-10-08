"""Defeature obsolete floor holes and casting imprints; retain functional bolt islands."""
from pathlib import Path
import sys,json
sys.path.insert(0,str(Path(__file__).parent))
from simple_upper_r09 import box,cyl,faces,SRC,DEST
import faceted_r08 as g
from OCP.BRepCheck import BRepCheck_Analyzer
def run():
 outer=g.read('outer_master.step');rear=g.read('PRINT_PACKAGE/R01.step');result=[]
 for tag,xc in [('R01',-41.8912974452),('R02',208.1087025548)]:
  original=g.read('PRINT_PACKAGE/'+tag+'.step');ss=g.solids(original);q=max(ss,key=g.vol);detached=[g.vol(s) for s in ss if g.vol(s)<10]
  # The old motor-floor casting detail is unnecessary away from the four actual screw seats.
  for side in [-1,1]:
   print('motor floor',tag,side,flush=True)
   zone=box(xc-18.2,xc+18.2,26.1 if side>0 else -43.5,43.5 if side>0 else -26.1,1.9,3.51)
   for dx,ya in [(-15,35.64092583),(15,35.64092583),(-8.5,29.34092583),(8.5,29.34092583)]:zone=g.cut(zone,cyl((xc+dx,side*ya,1.8),(xc+dx,side*ya,3.6),3.4))
   q=g.cut(q,zone);slab=g.common(outer,box(xc-18.2,xc+18.2,26.1 if side>0 else -43.5,43.5 if side>0 else -26.1,-.5,1.9));q=g.fuse(q,slab)
   for dx,ya in [(-15,35.64092583),(15,35.64092583),(-8.5,29.34092583),(8.5,29.34092583)]:
    q=g.cut(q,cyl((xc+dx,side*ya,-2),(xc+dx,side*ya,2.1),2.15));q=g.cut(q,cyl((xc+dx,side*ya,2.09),(xc+dx,side*ya,3.55),1.15))
  filled=[]
  if tag=='R02':
   # These are inherited base-plate holes; no current sensor, clip or body screw uses them.
   for x,ya in [(142,30),(168,32),(180,30)]:
    for side in [-1,1]:q=g.fuse(q,cyl((x,side*ya,-.5),(x,side*ya,3.5),2.1));filled.append([x,side*ya])
   for side in [-1,1]:
    plug=g.common(outer,cyl((70,side*69,-.5),(70,side*69,3.2),3.55));plug=g.cut(plug,rear);q=g.fuse(q,plug);filled.append([70,side*69])
  q=g.clean(q);ss=g.solids(q)
  assert .94<g.vol(q)/g.vol(original)<1.08,(tag,'unexpected mass change',g.vol(q),g.vol(original))
  assert len(ss)==1 and BRepCheck_Analyzer(q).IsValid(),(tag,len(ss),BRepCheck_Analyzer(q).IsValid())
  g.BASE=DEST;g.write(q,tag+'_clean.step');g.BASE=SRC
  result.append(dict(part=tag,old_faces=faces(original),new_faces=faces(q),old_cm3=g.vol(original)/1000,new_cm3=g.vol(q)/1000,removed_unattached_fragment_mm3=detached,obsolete_holes_filled=filled,preserved='MX28 screw-seat islands, 16 bottom bolt axes, body/lid/tray screws, strap/tie slots, TTL channels, PCB/U2D2 supports'))
 (DEST/'lower_simplification.json').write_text(json.dumps(result,indent=2));print(result)
if __name__=='__main__':run()
