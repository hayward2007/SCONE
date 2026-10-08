import adsk.core as c, adsk.fusion as f, json
from pathlib import Path
OUT=Path('/Users/hayward_kim/Developer/SCONE/artifacts/electronics/20260930_u2d2_docked')
def bb(b):return [[v*10 for v in p.asArray()] for p in(b.boundingBox.minPoint,b.boundingBox.maxPoint)]
def run(_context):
 a=c.Application.get();doc=next(x for x in a.documents if x.name.startswith('MARC v4 Body v5 BOX v'));d=f.Design.cast(doc.products.itemByProductType('DesignProductType'))
 rows=[]
 for o in d.rootComponent.allOccurrences:
  if o.fullPathName.startswith('E10 '):rows.append({'path':o.fullPathName,'component':o.component.name,'transform':list(o.transform2.asArray()),'bodies':[{'name':b.name,'box':bb(b),'faces':b.faces.count} for b in o.bRepBodies],'features':[{'name':bf.name,'source_bodies':len(bf.sourceBodies),'result_bodies':len(bf.bodies)} for bf in o.component.features.baseFeatures]})
 (OUT/'live_e10_before.json').write_text(json.dumps(rows,ensure_ascii=False,indent=2),encoding='utf-8')
 print(json.dumps({'document':doc.name,'modified':doc.isModified,'components':[{'path':x['path'],'transform':x['transform'],'bodies':len(x['bodies'])} for x in rows],'updateBody_doc':f.BaseFeature.updateBody.__doc__},ensure_ascii=False))
