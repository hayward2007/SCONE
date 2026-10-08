import adsk.core as c, adsk.fusion as f
import json, traceback
from pathlib import Path
OUT=Path('/Users/hayward_kim/Developer/SCONE/artifacts/housing/20260915_MARC_v10')
def xyz(p): return [round(v*10,6) for v in p.asArray()]
def bb(b): return [xyz(b.boundingBox.minPoint),xyz(b.boundingBox.maxPoint)]
def safe(obj,key):
 try:
  v=getattr(obj,key)
  if hasattr(v,'asArray'): return list(v.asArray())
  if isinstance(v,(str,int,float,bool)) or v is None:return v
  return str(v)
 except Exception as e:return 'ERROR:'+str(e)
def body(b):
 return dict(name=b.name,box=bb(b),volume_cm3=safe(b,"volume"),faces=b.faces.count,edges=b.edges.count,visible=b.isVisible,token=b.entityToken)
def joint(j):
 m=j.jointMotion
 o=dict(name=j.name,type=m.jointType,occ1=safe(j.occurrenceOne,'fullPathName'),occ2=safe(j.occurrenceTwo,'fullPathName'),suppressed=j.isSuppressed,motion={})
 for k in ['rotationValue','rotationAxisVector','rotationAxis','slideValue','slideDirectionVector']:
  if hasattr(m,k):o['motion'][k]=safe(m,k)
 for k in ['rotationLimits','slideLimits']:
  if hasattr(m,k):
   lim=getattr(m,k);o[k]={q:safe(lim,q) for q in ['isMinimumValueEnabled','minimumValue','isMaximumValueEnabled','maximumValue','isRestValueEnabled','restValue']}
 for k in ['geometryOrOriginOne','geometryOrOriginTwo']:
  try:
   g=getattr(j,k);o[k]={p:safe(g,p) for p in ['origin','primaryAxisVector','secondaryAxisVector']}
  except:pass
 return o
def snapshot(doc):
 d=f.Design.cast(doc.products.itemByProductType('DesignProductType'));r=d.rootComponent
 return dict(name=doc.name,id=doc.dataFile.id if doc.dataFile else None,creationId=doc.creationId,modified=doc.isModified,
  designType=d.designType,timeline=d.timeline.count,root_box=bb(r),root_bodies=[body(b) for b in r.bRepBodies],
  occurrences=[dict(name=o.fullPathName,component=o.component.name,grounded=safe(o,'isGrounded'),groundParent=safe(o,'isGroundToParent'),visible=o.isVisible,referenced=safe(o,'isReferencedComponent'),transform=list(o.transform2.asArray()),box=bb(o),bodies=[body(b) for b in o.bRepBodies]) for o in r.allOccurrences],
  components=[dict(name=k.name,joints=[joint(j) for j in k.joints],asBuiltJoints=[joint(j) for j in k.asBuiltJoints],rigidGroups=k.rigidGroups.count) for k in d.allComponents],
  parameters=[dict(name=p.name,expression=p.expression,unit=p.unit) for p in d.userParameters])
def run():
 app=c.Application.get(); doc=app.activeDocument
 assert doc.name=='MARC v10',doc.name
 data=snapshot(doc)
 data['open_documents']=[{'name':x.name,'id':x.dataFile.id if x.dataFile else None} for x in app.documents]
 data['api_export']=f.ExportManager.createFusionArchiveExportOptions.__doc__
 data['api_import']=c.ImportManager.importToNewDocument.__doc__
 (OUT/'source_inventory.json').write_text(json.dumps(data,ensure_ascii=False,indent=2))
 print('SOURCE INVENTORY SAVED; no document modified; '+str(len(data['occurrences']))+' occurrences')
try:run()
except:(OUT/'inspect_error.txt').write_text(traceback.format_exc());print(traceback.format_exc())
