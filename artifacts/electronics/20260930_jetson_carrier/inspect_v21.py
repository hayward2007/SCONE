import adsk.core as c, adsk.fusion as f, json
from pathlib import Path
OUT=Path('/Users/hayward_kim/Developer/SCONE/artifacts/electronics/20260930_jetson_carrier')
def bb(b):return [[round(v*10,5) for v in p.asArray()] for p in(b.boundingBox.minPoint,b.boundingBox.maxPoint)]
def run(_context):
 a=c.Application.get();d=f.Design.cast(a.activeProduct)
 out={'document':a.activeDocument.name,'modified':a.activeDocument.isModified,'occurrences':[{'path':o.fullPathName,'name':o.component.name,'visible':o.isVisible,'transform':list(o.transform2.asArray()),'bodies':[{'name':b.name,'box':bb(b),'solid':b.isSolid,'faces':b.faces.count,'visible':b.isVisible} for b in o.bRepBodies],'meshes':[{'name':b.name,'box':bb(b)} for b in o.meshBodies]} for o in d.rootComponent.allOccurrences],'timeline':[{'i':i,'name':d.timeline.item(i).name,'health':d.timeline.item(i).healthState,'error':d.timeline.item(i).errorOrWarningMessage} for i in range(d.timeline.count)]}
 (OUT/'inventory_v21.json').write_text(json.dumps(out,ensure_ascii=False,indent=2),encoding='utf-8')
 print(json.dumps({'document':out['document'],'modified':out['modified'],'top_level':[o['path'] for o in out['occurrences'] if '+' not in o['path']],'warnings':[v for v in out['timeline'] if v['health'] not in(0,5)]},ensure_ascii=False))
