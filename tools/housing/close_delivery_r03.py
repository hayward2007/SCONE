import zipfile,json
from pathlib import Path
p=Path('/Users/hayward_kim/Developer/SCONE/artifacts/housing/20260915_MARC_v10/R03_DELIVERY')
try:
 with zipfile.ZipFile(p/'MARC_Housing_PLA_R03.f3d') as z:bad=z.testzip()
 (p/'archive_validation.json').write_text(json.dumps({'CRC_checked':True,'bad_member':bad}));print('F3D CRC',bad)
except Exception as e:
 (p/'archive_validation.json').write_text(json.dumps({'CRC_checked':False,'reason':str(e)}));print(str(e))
app=adsk.core.Application.get();app.activeViewport.fit();app.activeViewport.refresh();palette=app.userInterface.palettes.itemById('TextCommands')
if palette:palette.isVisible=False
