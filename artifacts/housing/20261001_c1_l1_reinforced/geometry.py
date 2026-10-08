exec(open('/Users/hayward_kim/Developer/SCONE/artifacts/housing/20260930_c1_l1/geometry.py').read().split('\ndef generate(d):')[0])
OUT=Path('/Users/hayward_kim/Developer/SCONE/artifacts/housing/20261001_c1_l1_reinforced')
PREVIEW='C1 L1 reinforced revision preview'
def foot_regions():
 return [box(x-12,x+12,30 if s>0 else -43,43 if s>0 else -30,153.49,172) for x in(-80,-20) for s in(-1,1)]
def new_foot(x,s):
 # 20 mm width along the rail. Rounded outside corners match the handle feet.
 q=rz(x-10,x+10,29,41,153.5,171,3)
 cut(q,box(x-11,x+11,28,32,153.4,172))
 # Original 45-degree brace: root Z167.5, outer edge Z158.5.
 cut(q,halfspace((x,32,167.5),(0,1,1),(1,0,0)))
 cut(q,cyl((x,35,153.4),(x,35,172),1.7))
 cut(q,cyl((x,35,155.5),(x,35,172),3.2))
 if s<0:
  # Mirror about Y while preserving a proper rotation via X reflection.
  transform(q,2*x,0,0,((-1,0,0),(0,-1,0),(0,0,1)))
 return q

def duct_volume(y0,y1,base,offset):
 segs=[(-12,-3,163.1,163.1),(-3,9,163.1,156.1),(9,62.5,156.1,156.1),(62.5,74.5,156.1,163.1),(74.5,85.5,163.1,163.1)]
 q=None
 for x0,x1,h0,h1 in segs:
  b=profile_roof(x0,x1,y0,y1,base,h0-offset,h1-offset)
  q=join(q,b) if q else b
 return q

def generate(d):
 old=targets(d);l1=T.copy(old['L1 frame'])
 for x in(-80,-20):
  for s in(-1,1):
   cut(l1,box(x-6,x+6,32 if s>0 else -41,41 if s>0 else -32,153.5,168))
   join(l1,new_foot(x,s))
 c1=duct_volume(-15,15,145.5,0);void=duct_volume(-13.4,13.4,140,1.6);join(void,box(-12.1,-8.4,-13.4,13.4,140,165));cut(c1,void)
 join(c1,rz(74.5,85.5,-41,41,153.5,158.5,3));cut(c1,void)
 for y in(-35,35):
  cut(c1,cyl((80,y,153.4),(80,y,158.6),1.7));cut(c1,cyl((80,y,155.5),(80,y,158.6),3.2))
 assert l1.isSolid and l1.lumps.count==1;assert c1.isSolid and c1.lumps.count==1
 return old,{'L1 frame':l1,'C1 USB cable guide':c1}

def fillet(co,edges,r,name):
 coll=c.ObjectCollection.create()
 for e in edges:coll.add(e)
 assert coll.count,name+' no edges selected'
 fi=co.features.filletFeatures.createInput();fi.addConstantRadiusEdgeSet(coll,c.ValueInput.createByReal(r/10),False)
 ft=co.features.filletFeatures.add(fi);ft.name=name
 assert ft.healthState in(0,5),(name,ft.errorOrWarningMessage)
 return ft
