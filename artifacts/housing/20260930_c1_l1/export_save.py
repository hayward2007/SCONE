exec(open('/Users/hayward_kim/Developer/SCONE/artifacts/housing/20260930_c1_l1/geometry.py').read())
def run(_context):
 a,doc,d=source();doc.activate();live=targets(d);assert len(live)==2
 report=json.loads((OUT/'installation_validation.json').read_text(encoding='utf-8'));assert report['protected_unchanged']
 out=OUT/'delivery';out.mkdir(exist_ok=True)
 copies={k:T.copy(v) for k,v in live.items()}
 assert d.exportManager.execute(d.exportManager.createFusionArchiveExportOptions(str(out/'MARC_C1_L1_updated.f3d')))
 pd=a.documents.add(c.DocumentTypes.FusionDesignDocumentType);pd.name='C1 L1 export only';dd=f.Design.cast(a.activeProduct);dd.designType=f.DesignTypes.DirectDesignType
 export_bodies={}
 for name,q in copies.items():
  export_bodies[name]=dd.rootComponent.bRepBodies.add(q);export_bodies[name].name=name
 assert dd.exportManager.execute(dd.exportManager.createSTEPExportOptions(str(out/'C1_L1_assembly_coordinates.step')))
 for name,b in export_bodies.items():
  code='C1_USB_guide' if name.startswith('C1') else 'L1_frame'
  # Rotation into a practical print orientation, applied only to export copies.
  q=T.copy(b)
  if name.startswith('C1'):transform(q,-145.5,0,85.5,((0,0,-1),(0,1,0),(1,0,0)))
  else:transform(q,97.4,0,185,((1,0,0),(0,-1,0),(0,0,-1)))
  pb=dd.rootComponent.bRepBodies.add(q);pb.name=code+' print orientation'
  opt=dd.exportManager.createSTLExportOptions(pb,str(out/(code+'_print.stl')));opt.unitType=f.DistanceUnits.MillimeterDistanceUnits;opt.meshRefinement=f.MeshRefinementSettings.MeshRefinementHigh
  assert dd.exportManager.execute(opt)
 pd.close(False);doc.activate()
 assert doc.save('C1 guide shortened 28 mm, high-low-high roof for four 5 mm cables, flat M3 rail feet; L1 only four rail feet flattened. Other geometry and poses verified unchanged.')
 (OUT/'save_requested.json').write_text(json.dumps({'document':doc.name,'save_requested':True,'export_files':[p.name for p in out.iterdir()]},ensure_ascii=False,indent=2),encoding='utf-8')
 print('Exports complete; cloud save requested for '+doc.name)
