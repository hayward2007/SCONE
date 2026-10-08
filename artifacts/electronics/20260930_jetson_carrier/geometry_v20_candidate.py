# Native Fusion temporary BRep geometry; all dimensions below in millimetres.
exec(open('/Users/hayward_kim/Developer/SCONE/artifacts/electronics/20260930_vertical_carrier/carrier_geometry.py').read().split('def generate(app):')[0])
OUT=Path('/Users/hayward_kim/Developer/SCONE/artifacts/electronics/20260930_jetson_carrier')
PREFIX='E10 Jetson rear electronics brace'
IDENT=((1,0,0),(0,1,0),(0,0,1))
def sloty(x,y0,y1,z0,z1,r):
 q=box(x-r,x+r,y0,y1,z0,z1)
 for z in (z0,z1):join(q,cyl((x,y0,z),(x,y1,z),r))
 return q

def generate(app):
 parts=[]
 # Continuous rear frame follows Jetson outline. Open windows provide cooling,
 # wire access, reduced material, and access to the back of the circuit boards.
 frame=ry(104.1,239,16.5,20.5,32,135.5,3)
 cut(frame,ry(116,171.5,16.4,20.6,40,125,3))
 cut(frame,ry(184,220,16.4,20.6,55,109,3))
 cut(frame,ry(181,228,16.4,20.6,122,126,1.5))
 # Upper arms sit on the existing clips, sharing their two M3 screws.
 # Counterbores retain 1.8 mm of material below the socket-head bearing face.
 for x in (108.6,221.4):
  arm=rz(x-4.5,x+4.5,5.4,24.5,131.5,136.5,2)
  cut(arm,cyl((x,10.65,131.4),(x,10.65,136.6),1.7))
  cut(arm,cyl((x,10.65,133.3),(x,10.65,136.6),3.2))
  join(frame,arm)
 # L-shaped feet on the body rail use the four existing 2.6 mm through holes.
 # The lateral beams stay beyond the board outlines, pads extend only near wall.
 for x0,x1,p0,p1,holes in [(107,115.5,109,130,(115,125)),(232,239,220,240,(225,235))]:
  join(frame,rz(x0,x1,20,54.8,81,87,2))
  join(frame,rz(p0,p1,46.5,54.8,81,87,2))
  for x in holes:
   cut(frame,cyl((x,52,80.9),(x,52,87.1),1.4))
   cut(frame,cyl((x,52,83.9),(x,52,87.1),2.7))
  # Vertical ribs stiffen each beam and the upright.
  rib=box(x0,x0+3.5,22.5,46.5,87,103)
  ang=math.atan2(16,24)
  # Cut a diagonal rising toward the frame at y=22.5.
  cutter=T.createBox(c.OrientedBoundingBox3D.create(P(x0+1.75,34.5+100*math.sin(ang),95+100*math.cos(ang)),V(1,0,0),V(0,math.cos(ang),-math.sin(ang)),1,30,20))
  cut(rib,cutter);join(frame,rib)
 # Official PHB standoffs. The exact STEP mounting pattern is asymmetric.
 phbholes=[(146.6193+x,74.9839+z) for x in(-20.6922,21.3814) for z in(-22.5241,25.5168)]
 for x,z in phbholes:
  # Tabs intersect the perimeter and keep PCB rear components clear.
  side=116 if x<145 else 171.5
  join(frame,ry(min(x-4.3,side-1),max(x+4.3,side+1),16.5,20.5,z-4.3,z+4.3,1))
  join(frame,cyl((x,20,z),(x,25.35,z),3.8))
  cut(frame,cyl((x,16.4,z),(x,25.45,z),1.7))
  # M3 nut inserted from the Jetson side before installing the carrier.
  cut(frame,box(x-2.85,x+2.85,16.4,18.9,z-3.3,z+3.3))
 # Adjustable PCB slots: 50 x 70 board, nominal 42 x 62 fixing pattern.
 # Flat spacer faces isolate the PCB 4.85 mm from the supporting frame.
 for i,(x,z) in enumerate([(x,z) for x in(183,225) for z in(51,113)]):
  join(frame,ry(x-5,x+5,16.5,20.5,z-7,z+7,2))
  cut(frame,sloty(x,16.4,20.6,z-2,z+2,1.7))
  cut(frame,sloty(x,16.4,18.1,z-2,z+2,3.1))
  q=cyl((x,20.5,z),(x,25.35,z),3.8);cut(q,cyl((x,20.4,z),(x,25.45,z),1.7))
  parts.append(('P2 power PCB spacer 4.85mm '+str(i+1),q,'print'))
 # Upward-facing U2D2 is bolted through a separate shelf before it is attached.
 # Shelf-to-frame uses two horizontal M3 bolts, so electronics remain removable.
 shelf=rz(110,179,20.5,44.5,107,110,2)
 for x in(114,175):
  join(shelf,ry(x-3.5,x+3.5,20.5,26.5,110,116,1))
  cut(shelf,cyl((x,20.4,113),(x,26.6,113),1.7))
  cut(shelf,cyl((x,23.3,113),(x,26.6,113),3.2))
  join(frame,ry(x-4.5,x+4.5,16.5,20.5,109,118,1.5))
  cut(frame,cyl((x,16.4,113),(x,20.6,113),1.7))
  cut(frame,box(x-2.85,x+2.85,16.4,18.9,109.7,116.3))
 for x in(122.5,164.5):
  for y in(27,39):
   cut(shelf,cyl((x,y,106.9),(x,y,110.1),1.1))
   cut(shelf,cyl((x,y,106.9),(x,y,108.6),2.1))
 parts.append(('P3 U2D2 upward port shelf - 4x M2x6',shelf,'print'))
 # Cable tie openings at the upper/lower frame and near the regulated PCB.
 for x,z in [(177,36),(234,119),(110,120)]:
  cut(frame,ry(x-1.2,x+1.2,16.4,20.6,z-3,z+3,.9))
 parts.insert(0,('P1 body-Jetson brace - 4x M2.5 plus 2x M3',frame,'print'))
 pcb=ry(179,229,25.35,26.95,47,117,2)
 for x in(183,225):
  for z in(51,113):cut(pcb,cyl((x,25.2,z),(x,27.1,z),1.6))
 parts.append(('REF power PCB 50x70x1.6 - mechanical envelope',pcb,'pcb'))
 for key in('ROBOTIS_U2D2PHB reference','ROBOTIS_U2D2 reference'):
  doc=next(x for x in app.documents if x.name==key);d=f.Design.cast(doc.products.itemByProductType('DesignProductType'))
  for o in d.rootComponent.allOccurrences:
   for b in o.bRepBodies:
    q=T.copy(b)
    if 'PHB' in key:transform(q,146.6193,27,74.9839,IDENT)
    else:transform(q,143.5,33,116.9,((1,0,0),(0,0,1),(0,-1,0)))
    parts.append((('ROBOTIS PHB ' if 'PHB' in key else 'ROBOTIS U2D2 ')+o.fullPathName,q,'official'))
 return parts
