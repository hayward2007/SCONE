exec(open('/Users/hayward_kim/Developer/SCONE/artifacts/housing/20260930_c1_l1/geometry.py').read())
def run(_context):
 a,src,d=source();pd=next(x for x in a.documents if x.name.startswith('C1 L1 revision preview'));pd.activate();dd=f.Design.cast(pd.products.itemByProductType('DesignProductType'))
 b=next(b for b in dd.rootComponent.bRepBodies if b.name=='C1 USB cable guide');edges=c.ObjectCollection.create()
 for e in b.edges:
  lo,hi=bounds(e)
  if abs(hi[1]-lo[1])<1e-5 and abs(abs(lo[1])-15)<1e-4 and hi[0]-lo[0]>.1 and lo[2]>150 and not(abs(lo[2]-158.5)<1e-4 and abs(hi[2]-158.5)<1e-4):edges.add(e)
 fi=dd.rootComponent.features.filletFeatures.createInput();fi.addConstantRadiusEdgeSet(edges,c.ValueInput.createByReal(.1),False);ff=dd.rootComponent.features.filletFeatures.add(fi);ff.name='C1 cable-guide outer roof edges R1'
 print('Rounded outer roof edges',edges.count,'health',ff.healthState)
 cam=a.activeViewport.camera;cam.isSmoothTransition=False;cam.eye=P(170,255,320);cam.target=P(10,0,167);cam.upVector=V(0,0,1);cam.viewExtents=37;a.activeViewport.camera=cam;a.activeViewport.refresh()
