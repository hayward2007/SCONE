import adsk.core as c, adsk.fusion as f, json
from pathlib import Path
OUT=Path('/Users/hayward_kim/Developer/SCONE/artifacts/housing/20260927_body42')
def run(_context: str):
 app=c.Application.get();d=f.Design.cast(app.activeProduct);t=f.TemporaryBRepManager.get();final=next(b for b in d.rootComponent.bRepBodies if b.name.startswith('Body42 MX28'));shape=t.copy(final)
 new=app.importManager.importToNewDocument(app.importManager.createFusionArchiveImportOptions(str(OUT/'SOURCE_POSE_CAPTURED.f3d')));new.name='SCONE Body42 Final - Original Pose'
 nd=f.Design.cast(app.activeProduct);r=nd.rootComponent
 before={a['path']:a['transform'] for a in json.loads((OUT/'inventory.json').read_text())['occ']};changes=[]
 for o in r.allOccurrences:
  if o.fullPathName in before:
   delta=max(abs(a-z) for a,z in zip(before[o.fullPathName],o.transform2.asArray()))
   if delta>1e-7:changes.append((o.fullPathName,delta))
 (OUT/'final_import_pose.json').write_text(json.dumps(changes,ensure_ascii=False,indent=2))
 print('Pose differences after captured source import:',len(changes))
 assert not changes,changes[:3]
 for b in r.bRepBodies:b.isLightBulbOn=False
 base=r.features.baseFeatures.add();base.name='Body42 completed geometry - M2.5 5mm seat 3.5mm web';base.startEdit();r.bRepBodies.add(shape,base);base.finishEdit();b=base.bodies.item(0);b.name='Body42 MX28 M2.5 - 5mm seat 3.5mm web';b.isLightBulbOn=True
 for cp in nd.allComponents:cp.isJointsFolderLightBulbOn=False;cp.isSketchFolderLightBulbOn=False;cp.isConstructionFolderLightBulbOn=False
 app.activeViewport.fit();print('Final model assembled in exact original captured pose')
