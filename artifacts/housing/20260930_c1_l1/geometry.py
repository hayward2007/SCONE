exec(open('/Users/hayward_kim/Developer/SCONE/artifacts/electronics/20260930_vertical_carrier/carrier_geometry.py').read().split('def generate(app):')[0])
OUT=Path('/Users/hayward_kim/Developer/SCONE/artifacts/housing/20260930_c1_l1')
TARGETS=('C1 USB cable guide','L1 frame')
def source():
 a=c.Application.get();doc=next(x for x in a.documents if x.name.startswith('MARC v4 Body v5 BOX v'));return a,doc,f.Design.cast(doc.products.itemByProductType('DesignProductType'))
def targets(d):return {b.name:b for o in d.rootComponent.allOccurrences for b in o.bRepBodies if b.name in TARGETS}
def halfspace(point,normal,u,size=400):
 n=V(*normal);n.normalize();uu=V(*u);uu.normalize();vv=n.crossProduct(uu);v=n.copy();v.scaleBy(size/20);p=P(*point);p.translateBy(v)
 return T.createBox(c.OrientedBoundingBox3D.create(p,uu,vv,size/10,size/10,size/10))
def profile_roof(x0,x1,y0,y1,base,h0,h1):
 q=box(x0,x1,y0,y1,base,max(h0,h1))
 if h0!=h1:
  slope=(h1-h0)/(x1-x0);cut(q,halfspace((x0,0,h0),(-slope,0,1),(0,1,0)))
 return q
def duct_volume(y0,y1,base,offset):
 segs=[(-12,-3,163.1,163.1),(-3,9,163.1,153.1),(9,62.5,153.1,153.1),(62.5,74.5,153.1,163.1),(74.5,85.5,163.1,163.1)]
 q=None
 for x0,x1,h0,h1 in segs:
  b=profile_roof(x0,x1,y0,y1,base,h0-offset,h1-offset)
  q=join(q,b) if q else b
 return q
def generate(d):
 orig=targets(d);l1=T.copy(orig['L1 frame']);regions=[]
 for x in (-80,-20):
  for s in(-1,1):
   yy=(32,41) if s>0 else(-41,-32)
   region=box(x-6,x+6,*yy,158.5,168)
   cut(l1,region);regions.append(region)
 c1=duct_volume(-15,15,145.5,0)
 void=duct_volume(-13.4,13.4,140,1.6)
 # Fully open entry inside the existing L1 doorway, shortened by 28 mm.
 join(void,box(-12.1,-8.4,-13.4,13.4,140,165))
 cut(c1,void)
 join(c1,rz(74.5,85.5,-41,41,153.5,158.5,3))
 # Restore the continuous tunnel through the new horizontal crossbar.
 cut(c1,void)
 for y in(-35,35):
  cut(c1,cyl((80,y,153.4),(80,y,158.6),1.7))
  cut(c1,cyl((80,y,155.5),(80,y,158.6),3.2))
 assert l1.isSolid and l1.lumps.count==1
 assert c1.isSolid and c1.lumps.count==1
 return orig,{'L1 frame':l1,'C1 USB cable guide':c1},regions
