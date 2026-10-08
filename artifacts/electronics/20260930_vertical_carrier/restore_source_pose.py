import adsk.core as c,adsk.fusion as f,json
from pathlib import Path
OUT=Path('/Users/hayward_kim/Developer/SCONE/artifacts/electronics/20260930_vertical_carrier')
def run(_context):
 a=c.Application.get();doc=next(x for x in a.documents if x.name=='MARC v4 Body v5 BOX v19');doc.activate();d=f.Design.cast(doc.products.itemByProductType('DesignProductType'));r=d.rootComponent
 before={o['name']:o['transform'] for o in json.loads((OUT/'inventory_before.json').read_text(encoding='utf-8'))['occurrences']}
 changed=[]
 for o in r.allOccurrences:
  path=o.fullPathName
  if path.startswith('LEG') and path.count('+')==1 and path in before:
   delta=max(abs(x-y) for x,y in zip(o.transform2.asArray(),before[path]))
   if delta>1e-8:
    m=c.Matrix3D.create();assert m.setWithArray(before[path]);o.transform2=m;changed.append(path)
 mismatch=[]
 for o in r.allOccurrences:
  if o.fullPathName in before:
   delta=max(abs(x-y) for x,y in zip(o.transform2.asArray(),before[o.fullPathName]))
   if delta>1e-7:mismatch.append([o.fullPathName,delta])
 assert not mismatch,mismatch
 if d.snapshots.hasPendingSnapshot:d.snapshots.add().name='Preserve exact source v19 pose while adding E9 carrier'
 print('RESTORED_ORIGINAL_POSE',len(changed),'parents; all original occurrence transforms match')
 (OUT/'pose_restoration.json').write_text(json.dumps({'restored_parents':changed,'remaining_transform_differences':mismatch},ensure_ascii=False,indent=2),encoding='utf-8')
