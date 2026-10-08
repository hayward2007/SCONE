import adsk.core as c, adsk.fusion as f, json, re
from pathlib import Path
OUT=Path('/Users/hayward_kim/Developer/SCONE/artifacts/electronics/20260930_jetson_carrier')
def run(_context):
 a=c.Application.get();doc=next(x for x in a.documents if x.name.startswith('MARC v4 Body v5 BOX v'));doc.activate();d=f.Design.cast(doc.products.itemByProductType('DesignProductType'))
 chk=json.loads((OUT/'preservation_check.json').read_text(encoding='utf-8'));assert chk['unchanged'],chk['differences'][:8]
 parent=next(o for o in d.rootComponent.occurrences if o.component.name=='E10 Jetson rear electronics brace');co=parent.component
 delivery=OUT/'DELIVERY';delivery.mkdir(exist_ok=True);(delivery/'STL').mkdir(exist_ok=True)
 files=[]
 for name,options in [('E10_Jetson_electronics_carrier.f3d',d.exportManager.createFusionArchiveExportOptions(str(delivery/'E10_Jetson_electronics_carrier.f3d'),co)),('E10_Jetson_electronics_carrier.step',d.exportManager.createSTEPExportOptions(str(delivery/'E10_Jetson_electronics_carrier.step'),co))]:
  assert d.exportManager.execute(options);files.append(name)
 for o in co.allOccurrences:
  if not re.match(r'^P[1-4]',o.component.name):continue
  for i,b in enumerate(o.component.bRepBodies):
   name=o.component.name+'_'+str(i+1)+'.stl';opt=d.exportManager.createSTLExportOptions(b,str(delivery/'STL'/name));opt.unitType=f.DistanceUnits.MillimeterDistanceUnits;opt.meshRefinement=f.MeshRefinementSettings.MeshRefinementHigh
   assert d.exportManager.execute(opt);files.append('STL/'+name)
 assert d.exportManager.execute(d.exportManager.createFusionArchiveExportOptions(str(OUT/'MARC_with_E10_verified.f3d')))
 assert doc.save('E10: Jetson rear board cassette, upward U2D2 ports, shared MX28 and Jetson fasteners; existing geometry verified unchanged')
 result={'document':doc.name,'saved':doc.isSaved,'modified':doc.isModified,'files':files,'version':doc.dataFile.versionNumber if doc.dataFile else None}
 (OUT/'delivery_result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8');print(json.dumps(result,ensure_ascii=False))
