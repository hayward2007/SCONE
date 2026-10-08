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
 app=c.Application.get();d=f.Design.cast(app.activeProduct);r=d.rootComponent
 print([(repr(b.name),[ord(x) for x in b.name]) for b in r.bRepBodies])
 data={str(i):dict(name=b.name,faces=faces(b)) for i,b in enumerate(r.bRepBodies)}
 (OUT/'root_faces.json').write_text(json.dumps(data,ensure_ascii=False,indent=2))
 for i,b in enumerate(r.bRepBodies):b.isLightBulbOn=i==3
 app.activeViewport.fit()
 print('root geometry recorded')
