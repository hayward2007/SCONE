exec(open('/Users/hayward_kim/Developer/SCONE/artifacts/electronics/20260930_vertical_carrier/carrier_geometry.py').read())
def run(_context):
 a=c.Application.get();src=a.activeDocument;d=f.Design.cast(a.activeProduct)
 assert src.name=='MARC v4 Body v5 BOX v19'
 parts=generate(a)
 holder=next(b for o in d.rootComponent.allOccurrences for b in o.bRepBodies if b.name=='M2 holder')
 originals=[(b.name,T.copy(b),b.appearance) for o in d.rootComponent.allOccurrences if any(v in o.name for v in ['M6R Rear','V08 LiDAR']) for b in o.bRepBodies]
 doc=a.documents.add(c.DocumentTypes.FusionDesignDocumentType);doc.name='E9 vertical electronics carrier review'
 dd=f.Design.cast(a.activeProduct);dd.designType=f.DesignTypes.DirectDesignType;r=dd.rootComponent
 gold=dd.appearances.addByCopy(holder.appearance,'Carrier matched yellow')
 green=dd.appearances.addByCopy(holder.appearance,'Reference PCB green')
 prop=green.appearanceProperties.itemById('opaque_albedo');print('COLOR_PROPERTY',bool(prop))
 if prop:prop.value=c.Color.create(22,107,77,255)
 gray=dd.appearances.addByCopy(holder.appearance,'Reference electronics dark')
 prop=gray.appearanceProperties.itemById('opaque_albedo')
 if prop:prop.value=c.Color.create(46,51,58,255)
 for n,q,k in parts:
  b=r.bRepBodies.add(q);b.name=n;b.appearance=gold if k=='print' else green if k=='pcb' or 'PART_1_1' in n else gray
 for n,q,ap in originals:
  b=r.bRepBodies.add(q);b.name='CONTEXT '+n;b.appearance=gold
 cam=a.activeViewport.camera;cam.isFitView=False;cam.isSmoothTransition=False;cam.target=P(-53,8,109);cam.eye=P(130,-220,240);cam.upVector=V(0,0,1);cam.cameraType=c.CameraTypes.OrthographicCameraType;cam.viewExtents=150;a.activeViewport.camera=cam;a.activeViewport.refresh()
 print('PREVIEW_CREATED',doc.name)
