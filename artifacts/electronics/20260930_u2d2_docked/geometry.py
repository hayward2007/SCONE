exec(open('/Users/hayward_kim/Developer/SCONE/artifacts/electronics/20260930_vertical_carrier/carrier_geometry.py').read().split('def generate(app):')[0])
OUT=Path('/Users/hayward_kim/Developer/SCONE/artifacts/electronics/20260930_u2d2_docked')
IDENT=((1,0,0),(0,1,0),(0,0,1))
RAIL_HOLES=[(x,52) for x in(115,125,225,235)]
def find_parts(d):
 out={}
 for o in d.rootComponent.allOccurrences:
  if o.fullPathName.startswith('E10 ') and o.component.bRepBodies.count:
   if o.component.name in('P1','P2','P3','P4'):out[o.component.name]=o
   elif o.component.name.startswith('R1 '):out['PHB']=o
   elif o.component.name.startswith('R2 '):out['U2D2']=o
   elif o.component.name.startswith('R3 '):out['PCB']=o
 return out

def dock_matrix():
 old=c.Matrix3D.create();old.setWithCoordinateSystem(P(196,33,114.9),V(1,0,0),V(0,0,1),V(0,-1,0));old.invert()
 new=c.Matrix3D.create();new.setWithCoordinateSystem(P(136.5361,37.3,66.6993),V(-1,0,0),V(0,1,0),V(0,0,-1))
 # Exact matrix product new * inverse(old), independent of Matrix3D multiplication convention.
 aa=new.asArray();bb=old.asArray();arr=[sum(aa[4*i+k]*bb[4*k+j] for k in range(4)) for i in range(4) for j in range(4)]
 m=c.Matrix3D.create();m.setWithArray(arr);return m

def proposed(d):
 src=find_parts(d);parts=[]
 fixed=T.copy(src['P1'].bRepBodies.item(0))
 # Remove the motor-top foot, retaining the independent lower frame crossbar.
 cut(fixed,box(187,237,25,45,38.6,44.3))
 for label in('left','right'):
  if label=='left':
   beam=rz(104.1,111,23.5,51,81,87,1.5)
   cut(beam,box(103,106.5,46.5,52,80.9,87.1))
   join(beam,rz(109,130,46,54.6,81,87,2));holes=(115,125);rx0,rx1=104.1,108.1
  else:
   beam=rz(233,239,23.5,54.6,81,87,1.5)
   join(beam,rz(220,240,46,54.6,81,87,2));holes=(225,235);rx0,rx1=235,239
  for x in holes:
   cut(beam,cyl((x,52,80.9),(x,52,87.1),1.4));cut(beam,cyl((x,52,83.9),(x,52,87.1),2.7));cut(beam,box(x-2.7,x+2.7,52,55,83.9,87.1))
  join(fixed,beam)
  rib=box(rx0,rx1,24,46,86.5,110)
  ang=math.atan2(23,22)
  cutter=T.createBox(c.OrientedBoundingBox3D.create(P((rx0+rx1)/2,35+100*math.sin(ang),98.5+100*math.cos(ang)),V(1,0,0),V(0,math.cos(ang),-math.sin(ang)),2,30,20))
  cut(rib,cutter);join(fixed,rib)
 parts.append(('P1 body-rail brace - 4x M2.5x12 plus 2x M3x12',fixed,'P1'))
 plate=T.copy(src['P2'].bRepBodies.item(0))
 # Restore only the two obsolete shelf bores and captive-nut pockets.
 for x in(167.5,227.5):
  join(plate,cyl((x,19.5,111),(x,23.5,111),1.7))
  join(plate,box(x-2.85,x+2.85,19.5,21.9,107.7,114.3))
 parts.append(('P2 lift-out PCB plate - shelf fittings removed',plate,'P2'))
 m=dock_matrix()
 for b in src['U2D2'].bRepBodies:
  q=T.copy(b);assert T.transform(q,m);parts.append((b.name,q,'U2D2'))
 return parts
