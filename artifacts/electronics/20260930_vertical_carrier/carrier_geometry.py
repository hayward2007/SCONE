import adsk.core as c, adsk.fusion as f, math, json
from pathlib import Path
OUT=Path('/Users/hayward_kim/Developer/SCONE/artifacts/electronics/20260930_vertical_carrier')
T=f.TemporaryBRepManager.get()
PREFIX='E9 Rear vertical electronics carrier'
def P(x,y,z):return c.Point3D.create(x/10,y/10,z/10)
def V(x,y,z):return c.Vector3D.create(x,y,z)
def box(x0,x1,y0,y1,z0,z1):return T.createBox(c.OrientedBoundingBox3D.create(P((x0+x1)/2,(y0+y1)/2,(z0+z1)/2),V(1,0,0),V(0,1,0),(x1-x0)/10,(y1-y0)/10,(z1-z0)/10))
def cyl(a,b,r):return T.createCylinderOrCone(P(*a),r/10,P(*b),r/10)
def op(a,b,k):assert T.booleanOperation(a,b,k);return a
def join(a,b):return op(a,b,f.BooleanTypes.UnionBooleanType)
def cut(a,b):return op(a,b,f.BooleanTypes.DifferenceBooleanType)
def rx(x0,x1,y0,y1,z0,z1,r):
 q=box(x0,x1,y0+r,y1-r,z0,z1);join(q,box(x0,x1,y0,y1,z0+r,z1-r))
 for y in (y0+r,y1-r):
  for z in (z0+r,z1-r):join(q,cyl((x0,y,z),(x1,y,z),r))
 return q
def ry(x0,x1,y0,y1,z0,z1,r):
 q=box(x0+r,x1-r,y0,y1,z0,z1);join(q,box(x0,x1,y0,y1,z0+r,z1-r))
 for x in (x0+r,x1-r):
  for z in (z0+r,z1-r):join(q,cyl((x,y0,z),(x,y1,z),r))
 return q
def rz(x0,x1,y0,y1,z0,z1,r):
 q=box(x0+r,x1-r,y0,y1,z0,z1);join(q,box(x0,x1,y0+r,y1-r,z0,z1))
 for x in (x0+r,x1-r):
  for y in (y0+r,y1-r):join(q,cyl((x,y,z0),(x,y,z1),r))
 return q
def slotx(x0,x1,y,z0,z1,r):
 q=box(x0,x1,y-r,y+r,z0,z1)
 for z in (z0,z1):join(q,cyl((x0,y,z),(x1,y,z),r))
 return q
def transform(q,tx,ty,tz,axes):
 m=c.Matrix3D.create();m.setWithCoordinateSystem(P(tx,ty,tz),*[V(*v) for v in axes]);assert T.transform(q,m);return q
def bounds(b):return [[round(v*10,6) for v in p.asArray()] for p in (b.boundingBox.minPoint,b.boundingBox.maxPoint)]
def interference(a,b):
 if not a.boundingBox.intersects(b.boundingBox):return 0
 q=T.copy(a);assert T.booleanOperation(q,b,f.BooleanTypes.IntersectionBooleanType)
 return q.volume*1000 if q.isSolid else 0

def generate(app):
 parts=[]
 # One-piece removable cradle: two feet, transverse PCB frame and PHB frame.
 carrier=rx(-20,-14,-38,38,84,140,3)
 cut(carrier,rx(-20.1,-13.9,-23,23,98,129,4))
 for y in (-31,31):
  for lo,hi in ((90,95),(129,134)):
   cut(carrier,slotx(-20.1,-13.9,y,lo,hi,1.7))
   cut(carrier,slotx(-20.1,-16.8,y,lo,hi,3.2))
 # Feet use existing 2.7 mm holes in M6R, without cutting the old bridge.
 for y0,y1 in ((34,54),(-45,-34)):
  join(carrier,rz(-79,-14,y0,y1,84,90.2,2.5))
 # A slide-out PHB cassette: guide channels allow the base bolts to be reached first.
 holes=[]
 cassette=ry(-89,-26,44.5,47.5,90.5,136.2,2)
 cut(cassette,ry(-77,-41,44.4,47.6,101,130,3))
 for lx in (-20.6922,21.3814):
  for lz in (-22.5241,25.5168):
   x=-56.9839-lz;z=114.3807-lx;holes.append((x,z))
   join(cassette,cyl((x,43.15,z),(x,47.5,z),4.3))
   cut(cassette,cyl((x,43.0,z),(x,47.6,z),1.7))
   # Side-loading captive M3 hex nuts, 5.7 across flats and 2.6 thick pocket.
   if x < -60:cut(cassette,box(x-3.4,-75.5,43.95,46.55,z-2.85,z+2.85))
   else:cut(cassette,box(-42.5,x+3.4,43.95,46.55,z-2.85,z+2.85))
 for x0,x1,g0,g1,cx in [(-93,-87.5,-89.25,-87.4,-90.25),(-27.5,-22,-27.6,-25.75,-24.25)]:
  rail=ry(x0,x1,42.8,48.6,86,136.2,1.2)
  cut(rail,box(g0,g1,44.25,47.75,90.3,136.3))
  cut(rail,cyl((cx,46,128),(cx,46,136.3),.8))
  join(carrier,rail)
  clip=rz(x0,x1+.2 if cx < -50 else x1,42.8,48.6,136.2,140.2,1)
  cut(clip,cyl((cx,46,136.1),(cx,46,140.3),1.1))
  cut(clip,cyl((cx,46,138),(cx,46,140.3),2.2))
  parts.append(('P3 removable top clamp '+str(cx)+' - M2x8',clip,'print'))
 # A lower bridge supports both guides, above the original M6R surface.
 join(carrier,rz(-93,-14,40,48.6,86,90.2,2))

 # Bottom feet and diagonal ribs; access remains open above bolt heads.
 for sign in (-1,):
  yy=(34,37) if sign>0 else (-37,-34)
  rib=box(-39,-14,*yy,89.5,114)
  # Sloping top from x=-39,z=90 to x=-14,z=114, defined by a rotated cutter.
  ang=math.atan2(24,25);n=V(-math.sin(ang),0,math.cos(ang));u=V(math.cos(ang),0,math.sin(ang))
  cutter=T.createBox(c.OrientedBoundingBox3D.create(P(-26.5-100*math.sin(ang),(yy[0]+yy[1])/2,102+100*math.cos(ang)),u,V(0,1,0),30,3,20))
  cut(rib,cutter);join(carrier,rib)
 for x in (-70,-50):
  for y in (-40,40):
   cut(carrier,cyl((x,y,83.9),(x,y,90.3),1.4))
   cut(carrier,cyl((x,y,86.9),(x,y,90.3),2.7))
 # Cable-tie slots in the free forward portion of each foot.
 for y in (-39.5,40):cut(carrier,rz(-33,-25,y-1.2,y+1.2,83.9,90.3,1))
 # Clear the protruding lower cassette bosses and U2D2 edge from the foot.
 for x,z in holes:
  if z < 100:cut(carrier,box(x-4.6,x+4.6,42.85,47.8,88.3,90.4))
 cut(carrier,box(-78.7,-59.9,25.8,43.0,89.5,90.4))
 cut(carrier,box(-85.3,-27.7,41.2,43.45,89.5,90.4))
 cut(carrier,box(-76.2,-61.8,25.8,35.7,84.6,90.4))
 assert carrier.isSolid and carrier.lumps.count==1
 parts.append(('P1 frame - 4x M2.5x10 to existing M6R holes',carrier,'print'))
 parts.append(('P2 slide-out U2D2 PHB cassette - captive M3 nuts',cassette,'print'))
 # Custom power PCB stand-offs, 6 mm back clearance.
 for i,(y,z) in enumerate([(y,z) for y in (-31,31) for z in (91,133)]):
  q=cyl((-14,y,z),(-8,y,z),3.8);cut(q,cyl((-14.1,y,z),(-7.9,y,z),1.7))
  parts.append(('P4 power PCB spacer 6mm '+str(i+1),q,'print'))
 board=rx(-8,-6.4,-35,35,87,137,2)
 for y in (-31,31):
  for z in (91,133):cut(board,cyl((-8.1,y,z),(-6.3,y,z),1.6))
 parts.append(('REF power PCB 70x50x1.6 - BMS charger 12V 5V - mechanical envelope only',board,'pcb'))
 # Official reference bodies transformed into the carrier. Nothing in the source is moved.
 for key in ('ROBOTIS_U2D2PHB reference','ROBOTIS_U2D2 reference'):
  doc=next(x for x in app.documents if x.name==key);d=f.Design.cast(doc.products.itemByProductType('DesignProductType'))
  bs=[(o.fullPathName,b) for o in d.rootComponent.allOccurrences for b in o.bRepBodies]
  for i,(path,b) in enumerate(bs):
   q=T.copy(b)
   if 'PHB' in key:transform(q,-56.9839,41.5,114.3807,((0,0,-1),(0,-1,0),(-1,0,0)))
   else:
    # Match the 42 x 12 mm U2D2 mounting pattern to oversized PHB rivet holes.
    transform(q,-56.9839-12.3168,41.5-7.3,114.3807-.3446,((0,0,-1),(0,-1,0),(-1,0,0)))
   parts.append((('ROBOTIS PHB ' if 'PHB' in key else 'ROBOTIS U2D2 ')+path,q,'official'))
 # Pockets for the exact official backside protrusions, with 0.5 mm clearance.
 for name,q,kind in parts:
  if name.startswith('ROBOTIS PHB') and 'PART_1_1' not in name:
   bb=bounds(q)
   if bb[1][1] > 44.0:
    
    for dx,dy,dz in [(0,0,0),(.5,0,0),(-.5,0,0),(0,.5,0),(0,-.5,0),(0,0,.5),(0,0,-.5)]:
     tool=T.copy(q);transform(tool,dx,dy,dz,((1,0,0),(0,1,0),(0,0,1)));cut(cassette,tool)
 return parts
