import adsk.core as c, adsk.fusion as f, json
from pathlib import Path
OUT=Path('/Users/hayward_kim/Developer/SCONE/artifacts/housing/20260927_body42')
def xyz(p):return [round(v*10,5) for v in p.asArray()]
def run(_context: str):
 app=c.Application.get();d=f.Design.cast(app.activeProduct);r=d.rootComponent
 for cp in d.allComponents:cp.isJointsFolderLightBulbOn=False
 b=next(b for b in r.bRepBodies if b.name.startswith('Body42 MX28'))
 data={'name':b.name,'volume':b.volume,'solid':b.isSolid,'lumps':b.lumps.count,'edges':[]}
 for i,e in enumerate(b.edges):
  if e.length*10>20:data['edges'].append(dict(i=i,length=e.length*10,box=[xyz(e.boundingBox.minPoint),xyz(e.boundingBox.maxPoint)]))
 (OUT/'mount_inspect.json').write_text(json.dumps(data,indent=2));print(json.dumps(data))
 print('FILLET API',r.features.filletFeatures.createInput.__doc__)
