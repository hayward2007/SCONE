import adsk.core as c, adsk.fusion as f, json
from pathlib import Path
OUT=Path('/Users/hayward_kim/Developer/SCONE/artifacts/housing/20260927_body42')
def run(_context: str):
 app=c.Application.get();d=f.Design.cast(app.activeProduct);r=d.rootComponent;b=next(b for b in r.bRepBodies if b.name.startswith('Body42 MX28'))
 edges=c.ObjectCollection.create()
 for e in b.edges:
  bb=e.boundingBox
  if abs(bb.minPoint.z*10-113.5)<.001 and abs(bb.maxPoint.z*10-153.5)<.001 and abs(e.length*10-40)<.001 and abs(abs(bb.minPoint.y*10)-86.328947)<.001:edges.add(e)
 assert edges.count==4,edges.count
 inp=r.features.filletFeatures.createInput();inp.edgeSetInputs.addConstantRadiusEdgeSet(edges,c.ValueInput.createByReal(.6),False)
 feat=r.features.filletFeatures.add(inp);feat.name='Spot inspired upper corner radius R6 - cavity preserved'
 assert feat.healthState==f.FeatureHealthStates.HealthyFeatureHealthState
 print('R6 exterior corner fillets created',feat.bodies.item(0).volume)
