import adsk.core as c, adsk.fusion as f, json
from pathlib import Path
OUT=Path('/Users/hayward_kim/Developer/SCONE/artifacts/housing/20260927_body42')
def xyz(p):return [round(v*10,5) for v in p.asArray()]
def body(b):return dict(name=b.name,visible=b.isVisible,solid=b.isSolid,box=[xyz(b.boundingBox.minPoint),xyz(b.boundingBox.maxPoint)],token=b.entityToken)
def run(_context: str):
 app=c.Application.get();doc=app.activeDocument;d=f.Design.cast(app.activeProduct);r=d.rootComponent
 assert doc.name.startswith('MARC Housing PLA R09 Assembled v6'),doc.name
 data=dict(name=doc.name,root=[body(b) for b in r.bRepBodies],occ=[dict(path=o.fullPathName,component=o.component.name,visible=o.isVisible,transform=list(o.transform2.asArray()),bodies=[body(b) for b in o.bRepBodies]) for o in r.allOccurrences])
 (OUT/'inventory.json').write_text(json.dumps(data,ensure_ascii=False,indent=2))
 print(json.dumps(data,ensure_ascii=False))
