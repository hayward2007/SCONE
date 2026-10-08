import adsk.core as c, adsk.fusion as f

def run(_context: str):
 app=c.Application.get();d=f.Design.cast(app.activeProduct);r=d.rootComponent;b=next(b for b in r.bRepBodies if b.name.startswith('Body42 MX28'))
 lib=app.materialLibraries.itemById('C1EEA57C-3F56-45FC-B8CB-A9EC46A9994C');mat=lib.materials.itemById('PrismMaterial-417')
 appearances=[]
 for name,color in [('Body42 Warm Ochre',(221,171,40)),('Body42 Graphite',(58,63,70))]:
  a=d.appearances.itemByName(name) or d.appearances.addByCopy(mat.appearance,name)
  for prop in a.appearanceProperties:
   if prop.id in ('opaque_albedo','surface_albedo'):prop.value=c.Color.create(*color,255)
  appearances.append(a)
 b.appearance=appearances[0]
 for fc in b.faces:
  if fc.boundingBox.maxPoint.z*10<45.01:fc.appearance=appearances[1]
 for cp in d.allComponents:cp.isJointsFolderLightBulbOn=False;cp.isSketchFolderLightBulbOn=False;cp.isConstructionFolderLightBulbOn=False
 app.activeViewport.fit();print('Warm ochre upper shell with graphite mounting and lower frame')
