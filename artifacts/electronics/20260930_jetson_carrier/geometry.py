exec(open('/Users/hayward_kim/Developer/SCONE/artifacts/electronics/20260930_vertical_carrier/carrier_geometry.py').read().split('def generate(app):')[0])
OUT=Path('/Users/hayward_kim/Developer/SCONE/artifacts/electronics/20260930_jetson_carrier')
PREFIX='E10 Jetson rear electronics brace'
IDENT=((1,0,0),(0,1,0),(0,0,1))
PHB_HOLES=[(136.8807-x,79.0161-z) for x in(-20.6922,21.3814) for z in(-22.5241,25.5168)]
PCB_HOLES=[(x,z) for x in(166.5,228.5) for z in(57,99)]
FOOT_HOLES=[(193.109,35.641),(199.609,29.341),(216.609,29.341),(223.109,35.641)]
def sloty(x,y0,y1,z0,z1,r):
 q=box(x-r,x+r,y0,y1,z0,z1)
 for z in(z0,z1):join(q,cyl((x,y0,z),(x,y1,z),r))
 return q

def official(app):
 d=f.Design.cast(next(x for x in app.documents if x.name.startswith('MARC v4 Body v5 BOX v')).products.itemByProductType('DesignProductType'))
 co=next(o.component for o in d.rootComponent.allOccurrences if o.component.name=='R1 official ROBOTIS U2D2 + PHB')
 out=[]
 for bf in co.features.baseFeatures:
  if not bf.name.startswith('ROBOTIS'):continue
  for b in bf.bodies:
   q=T.copy(b);phb=bf.name.startswith('ROBOTIS PHB')
   mat=c.Matrix3D.create();mat.setWithCoordinateSystem(P(-56.9839 if phb else -69.3007,41.5 if phb else 34.2,114.3807 if phb else 114.0361),V(0,0,-1),V(0,-1,0),V(-1,0,0));mat.invert();T.transform(q,mat)
   if phb:transform(q,136.8807,30,79.0161,((-1,0,0),(0,1,0),(0,0,-1)))
   else:transform(q,196,33,114.9,((1,0,0),(0,0,1),(0,-1,0)))
   out.append((bf.name,q,'official'))
 return out

def generate(app):
 parts=[]
 frame=ry(111.3,232.7,19.5,23.5,48,130.5,2)
 fixed=ry(104.1,239,18.25,25,38.7,47.5,2)
 for x0,x1,cx in[(104.1,111,107.55),(233,239,236)]:
  join(fixed,ry(x0,x1,18.25,25,40,136.5,1.5))
  join(fixed,rz(x0,x1,23.5,34,123,130.5,1))
  cut(fixed,cyl((cx,30,122.9),(cx,30,130.6),1.4))
  cut(fixed,box(cx-2.65,cx+2.65,27.1,34.1,126.4,128.7))
  tab=rz(cx-4,cx+4,25.1,34,130.5,134.5,.8)
  # A neck outside the fixed post joins each fastening tab to the plate.
  if cx<150:join(tab,box(111.3,114.8,23.25,34,128.5,130.6));join(tab,box(108,114.8,25.1,34,130.5,132.5))
  else:join(tab,box(229.2,232.7,23.25,34,128.5,130.6));join(tab,box(229.2,236.5,25.1,34,130.5,132.5))
  cut(tab,cyl((cx,30,130.4),(cx,30,134.6),1.4));cut(tab,cyl((cx,30,131.8),(cx,30,134.6),2.7));join(frame,tab)
 for x in(130,202):
  join(frame,box(x-4,x+4,19.5,23.5,45.5,49))
  cut(fixed,box(x-4.25,x+4.25,19.25,23.75,45.2,47.6))
 for x0,x1,z0,z1 in[(119,149,57,90),(179,221,57,86),(116,156,116,127),(178,226,117,127)]:cut(frame,ry(x0,x1,19.4,23.6,z0,z1,3))
 # Shared top screws through unchanged original clips: M3x12, counterbore 6.4 x 3.2.
 for x in(108.6,221.4):
  arm=rz(x-4.5,x+4.5,5.4,19,131.5,136.5,2)
  cut(arm,cyl((x,10.65,131.4),(x,10.65,136.6),1.7));cut(arm,cyl((x,10.65,133.3),(x,10.65,136.6),3.2));join(fixed,arm)
  if x>200:join(fixed,box(221,239,14,19,131.5,136.5))
 # Use the existing front MX28 top screws, with a shaped wall-side foot.
 foot=rz(187.6,235.7,19.5,39.8,38.7,44.2,2)
 normal=V(0,.8660254,-.5)
 cutter=T.createBox(c.OrientedBoundingBox3D.create(P(210,37.068+.8660254*100,38.7-50),V(1,0,0),V(0,.5,.8660254),30,30,20))
 cut(foot,cutter)
 for x,y in FOOT_HOLES:
  cut(foot,cyl((x,y,38.6),(x,y,44.3),1.45));cut(foot,cyl((x,y,41.5),(x,y,44.3),2.7))
 join(fixed,foot)
 # Permanent PHB support pillars and captive nuts; power terminal end points up.
 for x,z in PHB_HOLES:
  join(frame,cyl((x,23,z),(x,28.35,z),3.8));cut(frame,cyl((x,19.4,z),(x,28.45,z),1.7))
  cut(frame,box(x-2.85,x+2.85,19.4,21.9,z-3.3,z+3.3))
 # The custom 70 x 50 PCB has four adjustable slots and loose insulating spacers.
 for i,(x,z) in enumerate(PCB_HOLES):
  cut(frame,sloty(x,19.4,23.6,z-2,z+2,1.7));cut(frame,sloty(x,19.4,21.1,z-2,z+2,3.1))
  q=cyl((x,23.5,z),(x,28.35,z),3.8);cut(q,cyl((x,23.4,z),(x,28.45,z),1.7))
  parts.append(('P3 PCB spacer 4.85mm '+str(i+1),q,'print'))
 # Independent removable shelf places all three U2D2 communication ports upward.
 shelf=rz(164,231,23.5,44.5,104.5,108,2)
 for x in(167.5,227.5):
  join(shelf,ry(x-3.5,x+3.5,23.5,29.5,108,114,1));cut(shelf,cyl((x,23.4,111),(x,29.6,111),1.7));cut(shelf,cyl((x,26.3,111),(x,29.6,111),3.2))
  cut(frame,cyl((x,19.4,111),(x,23.6,111),1.7));cut(frame,box(x-2.85,x+2.85,19.4,21.9,107.7,114.3))
 for x in(175,217):
  for y in(27,39):
   cut(shelf,cyl((x,y,104.4),(x,y,108.1),1.1));cut(shelf,cyl((x,y,104.4),(x,y,106.6),2.1))
 for x,z in[(161,67),(161,91),(229,121),(116,109)]:cut(frame,ry(x-1.2,x+1.2,19.4,23.6,z-3,z+3,.9))
 parts.insert(0,('P1 body-Jetson brace - 4x M2.5x8 plus 2x M3x12',fixed,'print'))
 parts.insert(1,('P2 lift-out PCB plate - 2x M2.5x8 captive nuts',frame,'print'))
 parts.append(('P4 U2D2 upward port shelf - 4x M2x5',shelf,'print'))
 pcb=ry(162.5,232.5,28.35,29.95,53,103,2)
 for x,z in PCB_HOLES:cut(pcb,cyl((x,28.2,z),(x,30.1,z),1.6))
 parts.append(('REF power PCB 70x50x1.6 - mechanical envelope',pcb,'pcb'))
 parts+=official(app)
 return parts
