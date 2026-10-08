import adsk.core as c, adsk.fusion as f, json, hashlib
from pathlib import Path
OUT=Path('/Users/hayward_kim/Developer/SCONE/artifacts/electronics/20260930_vertical_carrier')
ARM='ARM 5DOF + gripper (MX28 x2, W350 x2, W210 x2)'
def xyz(p):return [round(v,8) for v in p.asArray()]
def bounds(o):return [xyz(o.boundingBox.minPoint),xyz(o.boundingBox.maxPoint)]
def body_sig(b):
 return {'name':b.name,'solid':b.isSolid,'volume':round(b.volume,8) if b.isSolid else None,'area':round(b.area,8),'box':bounds(b),'faces':b.faces.count,'edges':b.edges.count,'vertices':sorted(xyz(v.geometry) for v in b.vertices),'appearance':b.appearance.name if b.appearance else None,'material':b.material.name if b.material else None,'visible':b.isLightBulbOn}
def protected(d):
 r=d.rootComponent
 return {'root_bodies':[body_sig(b) for b in r.bRepBodies],'occurrences':{o.fullPathName:{'transform':[round(x,8) for x in o.transform2.asArray()],'visible':o.isLightBulbOn,'component':o.component.name,'bodies':[body_sig(b) for b in o.component.bRepBodies]} for o in r.allOccurrences},'root_joints':[{'name':j.name,'type':j.jointMotion.jointType,'suppressed':j.isSuppressed} for j in r.joints]}

def run(_context):
 app=c.Application.get(); d=f.Design.cast(app.activeProduct)
 assert app.activeDocument.name=='MARC v4 Body v5 BOX v19'
 p=protected(d); (OUT/'protected_before.json').write_text(json.dumps(p,ensure_ascii=False,indent=2))
 assert d.exportManager.execute(d.exportManager.createFusionArchiveExportOptions(str(OUT/'SOURCE_v19_before_carrier.f3d')))
 print('Protected snapshot and archive',len(p['occurrences']))
