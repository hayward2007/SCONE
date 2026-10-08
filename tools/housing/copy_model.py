import adsk.core as c,adsk.fusion as f,json,traceback
from pathlib import Path
OUT=Path('/Users/hayward_kim/Developer/SCONE/artifacts/housing/20260915_MARC_v10')
def run():
 app=c.Application.get(); source=app.activeDocument
 baseline=json.loads((OUT/'source_inventory.json').read_text())
 assert source.dataFile.id==baseline['id'] and source.creationId==baseline['creationId'],'Source changed'
 assert not (OUT/'copy_identity.json').exists(),'Copy already exists; do not duplicate again'
 d=f.Design.cast(app.activeProduct)
 p=OUT/'MARC_v10_user_snapshot.f3d'
 assert not p.exists(),'Snapshot already exists; inspect before retry'
 assert d.exportManager.execute(d.exportManager.createFusionArchiveExportOptions(str(p))), 'Snapshot failed'
 assert p.exists() and p.stat().st_size>10000
 copy=app.importManager.importToNewDocument(app.importManager.createFusionArchiveImportOptions(str(p)))
 assert copy and copy != source
 copy.name='MARC Housing A01'
 copy.attributes.add('SCONE_HOUSING','source_id',baseline['id'])
 copy.attributes.add('SCONE_HOUSING','run_dir',str(OUT))
 (OUT/'copy_identity.json').write_text(json.dumps({'source_id':baseline['id'],'source_creation':source.creationId,'copy_creation':copy.creationId,'copy_name':copy.name,'saved':False},indent=2))
 assert copy.saveAs('MARC Housing A01',source.dataFile.parentFolder,'Independent full copy of MARC v10 including unsaved state; housing concept A01','')
 identity={'source_id':baseline['id'],'source_creation':source.creationId,'source_modified':source.isModified,'copy_creation':copy.creationId,'copy_id':copy.dataFile.id,'copy_name':copy.name,'saved':True}
 (OUT/'copy_identity.json').write_text(json.dumps(identity,indent=2))
 print('FULL COPY CREATED AND SAVED; original remains open and unsaved; '+copy.name)
try:run()
except:(OUT/'copy_error.txt').write_text(traceback.format_exc());print(traceback.format_exc())
