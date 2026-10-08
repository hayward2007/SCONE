import adsk.core as c, adsk.fusion as f, json
from pathlib import Path
OUT=Path('/Users/hayward_kim/Developer/SCONE/artifacts/housing/20260927_body42')
def xyz(p):return [round(v*10,6) for v in p.asArray()]
def run(_context: str):
 app=c.Application.get();d=f.Design.cast(app.activeProduct);r=d.rootComponent;t=f.TemporaryBRepManager.get();shell=r.bRepBodies.item(3)
 data={'volume':shell.volume,'api':{n:getattr(t,n).__doc__ for n in ['createBox','createCylinderOrCone','booleanOperation']},'intersections':[],'mount_edges':[]}
 for o in r.allOccurrences:
  if not o.fullPathName.startswith('LEG') or not o.isVisible:continue
  for b in o.bRepBodies:
   if not b.isVisible or not b.isSolid:continue
   a=t.copy(shell);ok=t.booleanOperation(a,t.copy(b),f.BooleanTypes.IntersectionBooleanType)
   vol=a.volume if ok else None
   if vol is None or vol>1e-7:data['intersections'].append(dict(path=o.fullPathName,body=b.name,ok=ok,volume_cm3=vol))
   if '+' not in o.fullPathName:
    for e in b.edges:
     g=e.geometry
     if g.objectType.endswith('Circle') and abs(g.center.z*10-3.5)<.01:
      data['mount_edges'].append(dict(path=o.fullPathName,center=xyz(g.center),radius=g.radius*10))
 (OUT/'fit_probe.json').write_text(json.dumps(data,ensure_ascii=False,indent=2));print(json.dumps(data,ensure_ascii=False))
