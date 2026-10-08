import adsk.core as c, adsk.fusion as f, json, hashlib
from pathlib import Path
OUT=Path('/Users/hayward_kim/Developer/SCONE/artifacts/electronics/20260930_jetson_carrier')
def coords(p):return [round(v,7) for v in p.asArray()]
def bbox(b):return [coords(b.boundingBox.minPoint),coords(b.boundingBox.maxPoint)]
def bsig(b):
 verts=sorted(coords(v.geometry) for v in b.vertices)
 return {'name':b.name,'box':bbox(b),'solid':b.isSolid,'faces':b.faces.count,'edges':b.edges.count,'vertices':hashlib.sha256(json.dumps(verts).encode()).hexdigest(),'appearance':b.appearance.name if b.appearance else None,'material':b.material.name if b.material else None,'light':b.isLightBulbOn}
def sig(d):
 r=d.rootComponent
 return {'root':[bsig(b) for b in r.bRepBodies],'occurrences':{o.fullPathName:{'transform':[round(x,7) for x in o.transform2.asArray()],'visible':o.isLightBulbOn,'bodies':[bsig(b) for b in o.component.bRepBodies],'meshes':[{'name':b.name,'box':bbox(b),'light':b.isLightBulbOn} for b in o.component.meshBodies]} for o in r.allOccurrences if not o.fullPathName.startswith(('E9 ','E10 '))},'warnings':[{'i':i,'name':d.timeline.item(i).name,'health':d.timeline.item(i).healthState,'error':d.timeline.item(i).errorOrWarningMessage} for i in range(d.timeline.count) if d.timeline.item(i).healthState not in(0,5)]}
def run(_context):
 a=c.Application.get();d=f.Design.cast(a.activeProduct);assert a.activeDocument.name=='MARC v4 Body v5 BOX v21'
 (OUT/'protected_v21.json').write_text(json.dumps(sig(d),ensure_ascii=False,indent=2),encoding='utf-8')
 print('protected snapshot complete')
