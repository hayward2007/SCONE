import adsk.core as c, adsk.fusion as f, json
from pathlib import Path
OUT=Path('/Users/hayward_kim/Developer/SCONE/artifacts/housing/20260927_body42')
def run(_context: str):
 app=c.Application.get();assembly=app.activeDocument;d=f.Design.cast(app.activeProduct);r=d.rootComponent
 assert assembly.name=='SCONE Body42 Final - Original Pose'
 b=next(b for b in r.bRepBodies if b.name.startswith('Body42 MX28'));shape=f.TemporaryBRepManager.get().copy(b)
 assert d.exportManager.execute(d.exportManager.createFusionArchiveExportOptions(str(OUT/'SCONE_Body42_MX28_Assembly.f3d')))
 bodydoc=app.documents.add(c.DocumentTypes.FusionDesignDocumentType);bodydoc.name='SCONE Body42 - M2.5 body only';bd=f.Design.cast(app.activeProduct);br=bd.rootComponent
 bd.designType=f.DesignTypes.DirectDesignType;part=br.bRepBodies.add(shape);part.name='Body42_MX28_M2p5_5mm_web3p5'
 assert part.isSolid and part.lumps.count==1
 assert bd.exportManager.execute(bd.exportManager.createSTEPExportOptions(str(OUT/'SCONE_Body42_MX28.step'),br))
 assert bd.exportManager.execute(bd.exportManager.createFusionArchiveExportOptions(str(OUT/'SCONE_Body42_MX28_Body.f3d')))
 opts=bd.exportManager.createSTLExportOptions(part,str(OUT/'SCONE_Body42_MX28.stl'));opts.meshRefinement=f.MeshRefinementSettings.MeshRefinementHigh
 assert bd.exportManager.execute(opts)
 report=dict(body_volume_cm3=part.volume,solid=part.isSolid,lumps=part.lumps.count,faces=part.faces.count)
 (OUT/'export_report.json').write_text(json.dumps(report,indent=2));assembly.activate();print('Assembly F3D, single-body STEP, F3D and STL exported',report)
