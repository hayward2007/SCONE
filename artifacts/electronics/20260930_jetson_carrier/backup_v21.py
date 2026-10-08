import adsk.core as c, adsk.fusion as f, json
from pathlib import Path
OUT=Path('/Users/hayward_kim/Developer/SCONE/artifacts/electronics/20260930_jetson_carrier')
def run(_context):
 a=c.Application.get();d=f.Design.cast(a.activeProduct)
 assert a.activeDocument.name=='MARC v4 Body v5 BOX v21' and not a.activeDocument.isModified
 assert d.exportManager.execute(d.exportManager.createFusionArchiveExportOptions(str(OUT/'SOURCE_v21_before_PCB_carrier.f3d')))
 print('v21 backup complete')
