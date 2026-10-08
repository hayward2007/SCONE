import adsk.core as c, adsk.fusion as f, json

def run(_context):
 d=f.Design.cast(c.Application.get().activeProduct)
 for typ,attrs in [(f.FilletEdgeSetInputs,['addConstantRadiusEdgeSet']),(f.ExportManager,['createSTEPExportOptions']),(c.Camera,['viewExtents','target','eye']),(f.CombineFeatures,['createInput']),(f.AsBuiltJoint,['geometry']),(f.TemporaryBRepManager,['exportToFile']), (f.ChamferFeatureInput,['edgeSets','chamferEdgeSets'])]:
  for attr in attrs:
   if hasattr(typ,attr):print(typ.__name__,attr,getattr(typ,attr).__doc__)
 arm=next(co for co in d.allComponents if co.name.startswith('ARM 5DOF'))
 for j in arm.asBuiltJoints:print(j.name,'geom',j.geometry.entityOne.objectType if j.geometry and j.geometry.entityOne else None)
 for co in d.allComponents:
  if co.name.startswith(('ARM5','ARM6')):
   for b in co.bRepBodies:
    if not b.name.startswith('A') or 'pad' in b.name:continue
    print('BODY',co.name)
    print('EDGES',json.dumps([{'i':i,'length':round(e.length*10,3),'type':e.geometry.objectType,'p0':[round(x*10,2) for x in e.startVertex.geometry.asArray()],'p1':[round(x*10,2) for x in e.endVertex.geometry.asArray()]} for i,e in enumerate(b.edges) if e.geometry.objectType.endswith('Line3D')]))
