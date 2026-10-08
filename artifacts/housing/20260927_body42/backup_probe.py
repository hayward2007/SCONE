import adsk.core as c, adsk.fusion as f, json
from pathlib import Path
OUT=Path('/Users/hayward_kim/Developer/SCONE/artifacts/housing/20260927_body42')
def xyz(p):return [round(v*10,6) for v in p.asArray()]
def faces(b):
 rows=[]
 for i,fc in enumerate(b.faces):
  g=fc.geometry;row=dict(i=i,type=g.objectType,area=fc.area*100,box=[xyz(fc.boundingBox.minPoint),xyz(fc.boundingBox.maxPoint)])
  for key in ['origin','normal','axis']:
   if hasattr(g,key):row[key]=xyz(getattr(g,key)) if key=='origin' else list(getattr(g,key).asArray())
  if hasattr(g,'radius'):row['radius']=g.radius*10
  rows.append(row)
 return rows
def run(_context: str):
 app=c.Application.get();doc=app.activeDocument;d=f.Design.cast(app.activeProduct);r=d.rootComponent
 assert doc.name=='MARC Housing PLA R09 Assembled v6'
 assert d.exportManager.execute(d.exportManager.createFusionArchiveExportOptions(str(OUT/'SOURCE_UNMODIFIED_v6.f3d')))
 data={b.name:faces(b) for b in r.bRepBodies if b.name in ['본체12','본체42']}
 for o in r.allOccurrences:
  if o.fullPathName.startswith('LEG') and '+' not in o.fullPathName:
   data[o.fullPathName]=[dict(name=b.name,faces=faces(b)) for b in o.bRepBodies]
 (OUT/'faces.json').write_text(json.dumps(data,ensure_ascii=False,indent=2))
 print({k:len(v) for k,v in data.items()})
 new=app.importManager.importToNewDocument(app.importManager.createFusionArchiveImportOptions(str(OUT/'SOURCE_UNMODIFIED_v6.f3d')))
 new.name='SCONE Body42 MX28 M2.5 refinement'
 d=f.Design.cast(app.activeProduct);r=d.rootComponent
 for b in r.bRepBodies:b.isLightBulbOn=b.name=='본체42'
 app.activeViewport.fit();print('Independent work copy active:',new.name)
