import adsk.core as c, adsk.fusion as f, json, hashlib
from pathlib import Path
OUT=Path('/Users/hayward_kim/Developer/SCONE/artifacts/arm/20260929_refinement')
ARM='ARM 5DOF + gripper (MX28 x2, W350 x2, W210 x2)'
def xyz(p):return [round(v,8) for v in p.asArray()]
def bounds(o):return [xyz(o.boundingBox.minPoint),xyz(o.boundingBox.maxPoint)]
def body_sig(b):
 return {'name':b.name,'solid':b.isSolid,'volume':round(b.volume,8) if b.isSolid else None,'area':round(b.area,8),'box':bounds(b),'faces':b.faces.count,'edges':b.edges.count,'vertices':sorted(xyz(v.geometry) for v in b.vertices),'appearance':b.appearance.name if b.appearance else None,'material':b.material.name if b.material else None,'visible':b.isLightBulbOn}
def protected(d):
 r=d.rootComponent
 return {'root_bodies':[body_sig(b) for b in r.bRepBodies],'occurrences':{o.fullPathName:{'transform':[round(x,8) for x in o.transform2.asArray()],'visible':o.isLightBulbOn,'component':o.component.name,'bodies':[body_sig(b) for b in o.component.bRepBodies]} for o in r.allOccurrences if not o.fullPathName.startswith(ARM+':')},'root_joints':[{'name':j.name,'type':j.jointMotion.jointType,'suppressed':j.isSuppressed} for j in r.joints]}
def run(_context):
 app=c.Application.get();doc=app.activeDocument;d=f.Design.cast(app.activeProduct)
 assert doc.dataFile.id=='urn:adsk.wipprod:dm.lineage:cIsNU1hKTEaOfcReFZQlQw'
 s=protected(d);(OUT/'protected_before.json').write_text(json.dumps(s,ensure_ascii=False,indent=2))
 assert d.exportManager.execute(d.exportManager.createFusionArchiveExportOptions(str(OUT/'original_before_arm_refinement.f3d')))
 arm=next(x for x in d.allComponents if x.name==ARM)
 print('BACKUP_OK',doc.name,len(s['occurrences']), 'protected occurrences')
 for o in arm.occurrences: print(o.name,'groundParent',o.isGroundToParent,'transform',list(o.transform2.asArray()))
 for typ in [f.FilletEdgeSetInputs,f.ChamferEdgeSetInputs,f.ExportManager,c.Camera,f.BRepBody]:
  for attr in ['addConstantRadiusEdgeSet','addEqualDistanceChamferEdgeSet','createSTEPExportOptions','createSTLExportOptions','saveAsMesh','viewExtents','isFitView','saveAsSAT','revisionId']:
   if hasattr(typ,attr):print(typ.__name__,attr,getattr(typ,attr).__doc__)
