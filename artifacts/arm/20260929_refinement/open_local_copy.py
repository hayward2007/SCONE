import adsk.core as c, adsk.fusion as f,json
from pathlib import Path
OUT=Path('/Users/hayward_kim/Developer/SCONE/artifacts/arm/20260929_refinement')
def run(_context):
 app=c.Application.get();source=app.activeDocument
 assert source.dataFile.id=='urn:adsk.wipprod:dm.lineage:cIsNU1hKTEaOfcReFZQlQw'
 (OUT/'copy_stage.json').write_text(json.dumps({'stage':'importing local backup','source':source.name}))
 opt=app.importManager.createFusionArchiveImportOptions(str(OUT/'original_before_arm_refinement.f3d'))
 doc=app.importManager.importToNewDocument(opt)
 assert doc
 doc.name='MARC v4 Arm R1 Folded'
 (OUT/'copy_stage.json').write_text(json.dumps({'stage':'copy open','name':doc.name,'source':source.name}))
 print('COPY_OPEN',doc.name)
