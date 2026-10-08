import adsk.core as c,adsk.fusion as f,json,re
from pathlib import Path
OUT=Path('/Users/hayward_kim/Developer/SCONE/artifacts/electronics/20260930_vertical_carrier')
def run(_context):
 a=c.Application.get();doc=next(x for x in a.documents if x.name=='MARC v4 Body v5 BOX v19');doc.activate();d=f.Design.cast(doc.products.itemByProductType('DesignProductType'))
 check=json.loads((OUT/'preservation_check.json').read_text(encoding='utf-8'));assert check['source_geometry_transforms_visibility_material_appearance_unchanged']
 co=next(o.component for o in d.rootComponent.occurrences if o.component.name.startswith('E9 Rear vertical'))
 dest=OUT/'DELIVERY';dest.mkdir(exist_ok=True);(dest/'STL').mkdir(exist_ok=True)
 exports=[]
 for o in co.occurrences:
  if not o.component.name.startswith('P'):continue
  for i,b in enumerate(o.component.bRepBodies):
   path=dest/'STL'/(o.component.name+'_'+str(i+1)+'.stl');opts=d.exportManager.createSTLExportOptions(b,str(path));opts.meshRefinement=f.MeshRefinementSettings.MeshRefinementHigh
   opts.unitType=f.DistanceUnits.MillimeterDistanceUnits
   assert d.exportManager.execute(opts);exports.append(str(path))
 for ext in ['step','f3d']:
  p=dest/('E9_vertical_electronics_carrier.'+ext)
  opts=d.exportManager.createSTEPExportOptions(str(p),co) if ext=='step' else d.exportManager.createFusionArchiveExportOptions(str(p),co)
  assert d.exportManager.execute(opts);exports.append(str(p))
 p=dest/'MARC_v19_with_E9_carrier.f3d';assert d.exportManager.execute(d.exportManager.createFusionArchiveExportOptions(str(p)));exports.append(str(p))
 (OUT/'exports.json').write_text(json.dumps(exports,indent=2),encoding='utf-8');print('EXPORTED',len(exports),'files')
