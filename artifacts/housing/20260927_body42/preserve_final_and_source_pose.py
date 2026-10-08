import adsk.core as c, adsk.fusion as f, json
from pathlib import Path
OUT=Path('/Users/hayward_kim/Developer/SCONE/artifacts/housing/20260927_body42')
def run(_context: str):
 app=c.Application.get();work=app.activeDocument;d=f.Design.cast(app.activeProduct);b=next(b for b in d.rootComponent.bRepBodies if b.name.startswith('Body42 MX28'))
 assert d.exportManager.execute(d.exportManager.createFusionArchiveExportOptions(str(OUT/'REFINEMENT_WORK.f3d')))
 before={a['path']:a['transform'] for a in json.loads((OUT/'inventory.json').read_text())['occ']}
 source=next(doc for doc in app.documents if doc.name=='MARC Housing PLA R09 Assembled v6');source.activate();sd=f.Design.cast(app.activeProduct)
 delta=[]
 for o in sd.rootComponent.allOccurrences:
  if o.fullPathName in before:
   v=max(abs(a-z) for a,z in zip(before[o.fullPathName],o.transform2.asArray()))
   if v>1e-7:delta.append((o.fullPathName,v))
 assert not delta,delta
 if sd.snapshots.hasPendingSnapshot:sd.snapshots.add().name='Preserve user current pose before Body42 refinement'
 assert sd.exportManager.execute(sd.exportManager.createFusionArchiveExportOptions(str(OUT/'SOURCE_POSE_CAPTURED.f3d')))
 print('Source pose captured without geometry changes; refined geometry backed up')
 work.activate()
