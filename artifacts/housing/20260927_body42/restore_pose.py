import adsk.core as c, adsk.fusion as f, json
from pathlib import Path
OUT=Path('/Users/hayward_kim/Developer/SCONE/artifacts/housing/20260927_body42')
def run(_context: str):
 app=c.Application.get();d=f.Design.cast(app.activeProduct);r=d.rootComponent
 data=json.loads((OUT/'inventory.json').read_text());before={a['path']:a['transform'] for a in data['occ']}
 changed=[]
 for o in r.allOccurrences:
  path=o.fullPathName
  if path.startswith('LEG') and path.count('+')==1 and path in before:
   delta=max(abs(a-z) for a,z in zip(before[path],o.transform2.asArray()))
   if delta>1e-9:
    m=c.Matrix3D.create();m.setWithArray(before[path]);o.transform2=m;changed.append(path)
 changes=[]
 for o in r.allOccurrences:
  if o.fullPathName in before:
   delta=max(abs(a-z) for a,z in zip(before[o.fullPathName],o.transform2.asArray()))
   if delta>1e-7:changes.append(dict(path=o.fullPathName,delta=delta))
 assert not changes,changes
 if d.snapshots.hasPendingSnapshot:d.snapshots.add().name='Exact original user leg pose 2026-09-27'
 print('Restored and captured',changed,'all occurrence transforms match original')
 srcdoc=next(doc for doc in app.documents if doc.name=='MARC Housing PLA R09 Assembled v6');sd=f.Design.cast(srcdoc.products.itemByProductType('DesignProductType'))
 print('ORIGINAL BODY42 VOLUME',sd.rootComponent.bRepBodies.item(3).volume)
 (OUT/'pose_restore.json').write_text(json.dumps(dict(changed=changed,remaining_changes=changes,source_volume=sd.rootComponent.bRepBodies.item(3).volume),indent=2))
