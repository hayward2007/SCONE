exec(open('/Users/hayward_kim/Developer/SCONE/artifacts/housing/20261001_c1_l1_reinforced/geometry.py').read())
def run(_context):
 a,doc,d=source();old,parts=generate(d);pd=a.documents.add(c.DocumentTypes.FusionDesignDocumentType);pd.name=PREVIEW;dd=f.Design.cast(a.activeProduct);dd.designType=f.DesignTypes.DirectDesignType;co=dd.rootComponent
 new={}
 for name,q in parts.items():
  b=co.bRepBodies.add(q);b.name=name;b.appearance=old[name].appearance;new[name]=b
 # Blend the brace root into the frame wall. The bolt pocket interrupts each root edge.
 b=new['L1 frame'];edges=[]
 for e in b.edges:
  lo,hi=bounds(e)
  if abs(lo[2]-167.5)<1e-5 and abs(hi[2]-167.5)<1e-5 and abs(lo[1]-hi[1])<1e-5 and abs(abs(lo[1])-32)<1e-5 and hi[0]-lo[0]>.5:
   edges.append(e)
 fillet(co,edges,2,'L1 local brace root blends R2')
 b=new['C1 USB cable guide'];edges=[]
 for e in b.edges:
  lo,hi=bounds(e)
  if isinstance(e.geometry,c.Line3D) and hi[0]-lo[0]<1e-5 and hi[2]-lo[2]<1e-5 and hi[1]-lo[1]>20 and any(abs(lo[0]-x)<1e-4 for x in(-3,9,62.5,74.5)):edges.append(e)
 fillet(co,edges,1.2,'C1 taller middle smooth transitions R1.2')
 edges=[]
 for e in b.edges:
  lo,hi=bounds(e)
  if abs(hi[1]-lo[1])<1e-5 and abs(abs(lo[1])-15)<1e-4 and hi[0]-lo[0]>.1 and lo[2]>150 and not(abs(lo[2]-158.5)<1e-4 and abs(hi[2]-158.5)<1e-4):edges.append(e)
 fillet(co,edges,1,'C1 outer roof edges R1')
 for o in d.rootComponent.allOccurrences:
  for ob in o.bRepBodies:
   if ob.name in('L2 lidar plate','H2 handle + arm rest','LID-R','LID-F'):
    cb=co.bRepBodies.add(T.copy(ob));cb.name='CONTEXT '+ob.name;cb.appearance=ob.appearance
 cam=a.activeViewport.camera;cam.isSmoothTransition=False;cam.eye=P(170,255,320);cam.target=P(5,0,165);cam.upVector=V(0,0,1);cam.viewExtents=48;a.activeViewport.camera=cam;a.activeViewport.refresh()
 print(json.dumps({'preview':pd.name,'parts':{k:bounds(v) for k,v in new.items()}},ensure_ascii=False))
