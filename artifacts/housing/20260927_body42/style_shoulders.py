import adsk.core as c, adsk.fusion as f

def run(_context: str):
 app=c.Application.get();d=f.Design.cast(app.activeProduct);r=d.rootComponent;b=next(b for b in r.bRepBodies if b.name.startswith('Body42 MX28'))
 edges=c.ObjectCollection.create()
 for e in b.edges:
  bb=e.boundingBox
  if abs(e.length*10-99.3499810)<.01 and abs(bb.minPoint.z*10-33.5)<.001 and abs(bb.maxPoint.z*10-110.45616)<.01:edges.add(e)
 print('Selected sloped exterior edges',edges.count)
 assert edges.count==4
 inp=r.features.filletFeatures.createInput();inp.edgeSetInputs.addConstantRadiusEdgeSet(edges,c.ValueInput.createByReal(.3),False)
 feat=r.features.filletFeatures.add(inp);feat.name='Body42 R3 sloped exterior transitions'
 assert feat.healthState==f.FeatureHealthStates.HealthyFeatureHealthState
 print('Sloped corner fillets complete')
