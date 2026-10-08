import adsk.core as c, adsk.fusion as f, json
from pathlib import Path
OUT=Path('/Users/hayward_kim/Developer/SCONE/artifacts/housing/20260930_c1_l1')
def bb(b):return [[round(v*10,6) for v in p.asArray()] for p in(b.boundingBox.minPoint,b.boundingBox.maxPoint)]
def run(_context):
 a=c.Application.get();doc=next(x for x in a.documents if x.name.startswith('MARC v4 Body v5 BOX v'));d=f.Design.cast(doc.products.itemByProductType('DesignProductType'));rows=[]
 for o in d.rootComponent.allOccurrences:
  for b in o.bRepBodies:
   if b.name.startswith(('C1','L1','L2','H2')):
    faces=[]
    for face in b.faces:
     g=face.geometry
     if isinstance(g,c.Plane):faces.append({'kind':'plane','normal':list(g.normal.asArray()),'origin':[v*10 for v in g.origin.asArray()],'box':bb(face),'area_mm2':face.area*100})
     elif isinstance(g,c.Cylinder):faces.append({'kind':'cylinder','radius_mm':g.radius*10,'axis':list(g.axis.asArray()),'origin':[v*10 for v in g.origin.asArray()],'box':bb(face),'area_mm2':face.area*100})
    rows.append({'path':o.fullPathName,'body':b.name,'body_token':b.entityToken,'box':bb(b),'transform':list(o.transform2.asArray()),'faces':faces,'features':[{'index':i,'name':o.component.features.item(i).name,'type':o.component.features.item(i).objectType} for i in range(o.component.features.count)]})
 out={'document':doc.name,'modified':doc.isModified,'targets':rows,'timeline':[{'index':i,'name':d.timeline.item(i).name,'health':d.timeline.item(i).healthState,'error':d.timeline.item(i).errorOrWarningMessage} for i in range(d.timeline.count)]}
 (OUT/'inspection.json').write_text(json.dumps(out,ensure_ascii=False,indent=2),encoding='utf-8');print(json.dumps({'document':doc.name,'modified':doc.isModified,'bodies':[{'path':r['path'],'body':r['body'],'box':r['box']} for r in rows],'last_features':out['timeline'][-25:]},ensure_ascii=False))
