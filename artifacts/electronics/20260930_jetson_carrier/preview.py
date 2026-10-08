exec(open('/Users/hayward_kim/Developer/SCONE/artifacts/electronics/20260930_jetson_carrier/geometry.py').read())
def run(_context):
 a=c.Application.get();src=next(x for x in a.documents if x.name=='MARC v4 Body v5 BOX v21');d=f.Design.cast(src.products.itemByProductType('DesignProductType'))
 parts=generate(a);holder=next(b for o in d.rootComponent.allOccurrences for b in o.bRepBodies if b.name=='M2 holder')
 context=[]
 for o in d.rootComponent.allOccurrences:
  for b in o.bRepBodies:
   if b.name in('M2 holder','M2C clip -x','M2C clip +x'):context.append((b.name,T.copy(b)))
   elif b.name=='Body v5 FRONT':
    q=T.copy(b);cut(q,box(95,275,-70,70,47,150));context.append(('Body front cutaway context',q))
 doc=a.documents.add(c.DocumentTypes.FusionDesignDocumentType);doc.name='E10 Jetson carrier assembly review';dd=f.Design.cast(a.activeProduct);dd.designType=f.DesignTypes.DirectDesignType;r=dd.rootComponent
 colors={}
 for n,rgb in [('print',(224,169,36)),('pcb',(20,103,67)),('dark',(53,59,65)),('context',(157,169,181))]:
  ap=dd.appearances.addByCopy(holder.appearance,'E10 preview '+n);ap.appearanceProperties.itemById('opaque_albedo').value=c.Color.create(*rgb,255);colors[n]=ap
 for n,q,k in parts:
  b=r.bRepBodies.add(q);b.name=n;b.appearance=colors['print' if k=='print' else 'pcb' if k=='pcb' or 'PART_1_1' in n else 'dark']
 for n,q in context:
  b=r.bRepBodies.add(q);b.name='CONTEXT '+n;b.appearance=colors['context']
 cam=a.activeViewport.camera;cam.isFitView=False;cam.isSmoothTransition=False;cam.target=P(173,20,84);cam.eye=P(340,320,230);cam.upVector=V(0,0,1);cam.cameraType=c.CameraTypes.OrthographicCameraType;cam.viewExtents=160;a.activeViewport.camera=cam;a.activeViewport.refresh()
 print('Preview created')
