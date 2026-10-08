exec(open('/Users/hayward_kim/Developer/SCONE/tools/housing/deliver_r03.py',encoding='utf-8').read().split('\ndef run():')[0])
DEST=OUT/'R09_DELIVERY';PKG=DEST/'PRINT_PACKAGE'
def run():
 app,doc,d=guard();r=d.rootComponent;assert doc.name.startswith('MARC Housing PLA R09');PKG.mkdir(exist_ok=True)
 checks=json.loads((DEST/'checks.json').read_text());assert all(not v for k,v in checks.items() if k.endswith('_hits'));assert all(not v for k,v in checks['camera_assembly'].items() if k.endswith('_hits'))
 access=json.loads((DEST/'screw_access.json').read_text());assert access['failed']==0
 parts=[];assembly=[]
 names=['R01 Rear smooth PLA chassis:1','R02 Front smooth PLA chassis:1','S01 Rear fairing lid:1','S02 Front camera fairing lid:1','P04 PLA electronics tray:1','M01 Rear removable motor bridge:1','M02 Front removable motor bridge:1']
 if 'Assembled' in doc.name:
  placed=[];identity_matrix=list(c.Matrix3D.create().asArray())
  for name in names:
   values=list(r.occurrences.itemByName(name).transform2.asArray());assert all(abs(a-b)<1e-8 for a,b in zip(values,identity_matrix)),name;placed.append(name)
  (DEST/'assembled_placement_verification.json').write_text(json.dumps(dict(all_housing_at_verified_assembly_coordinates=True,parts=placed),indent=2))
 for name in names:
  b=current(r.occurrences.itemByName(name).component);parts.append((name[:3],name.split(':')[0],T.copy(b),1,'PLA'));assembly.append((name,T.copy(b)))
 for path,prefix in [('LEG 1:1+LINK:1','L1'),('LEG 1(미러):1+LINK(미러):1','L2')]:
  o=next(o for o in r.allOccurrences if o.fullPathName==path)
  for i,b in enumerate(b for b in o.component.bRepBodies if b.isLightBulbOn):parts.append((prefix+str(i+1),prefix+str(i+1)+' separate 5mm LINK plate',T.copy(b),2,'PLA'))
 for path in ['LEG 1:1+ARC:1','LEG 1(미러):1+ARC(미러):1']:
  o=next(o for o in r.allOccurrences if o.fullPathName==path)
  for b in o.component.bRepBodies:
   if b.isLightBulbOn and b.name.startswith(('W01','W11','T01','T11')):parts.append((b.name[:3],b.name,T.copy(b),2,'TPU' if b.name.startswith('T') else 'PLA'))
 for o in r.allOccurrences:
  if o.fullPathName.startswith('LEG') and ('+LINK' in o.fullPathName or '+ARC' in o.fullPathName):
   for b in o.bRepBodies:
    if b.isLightBulbOn and b.isSolid:assembly.append((o.fullPathName+'/'+b.name,T.copy(b)))
 assert len(parts)==15 and len(assembly)==23,(len(parts),len(assembly))
 meshes=[]
 for tag,name,q,qty,mat in parts:
  assert q.lumps.count==1 and q.isSolid
  calc=q.meshManager.createMeshCalculator();calc.surfaceTolerance=.003;m=calc.calculate();meshes.append(dict(tag=tag,name=name,quantity=qty,material=mat,vertices_mm=[v*10 for v in m.nodeCoordinatesAsDouble],triangles=list(m.nodeIndices),bounds=bb(q),volume_cm3=q.volume,lumps=q.lumps.count))
 with gzip.open(PKG/'manufacturing_meshes.json.gz','wt',encoding='utf-8') as h:json.dump(meshes,h)
 doc.save('R09 CAD checked printable housing, 20mm tyres and axial assembly access')
 d.exportManager.execute(d.exportManager.createFusionArchiveExportOptions(str(PKG/'MARC_Housing_PLA_R09.f3d')))
 final_snapshot=snapshot(doc);(DEST/'final_inventory.json').write_text(json.dumps(final_snapshot,ensure_ascii=True))
 # Geometry and placement of every leg component remain unchanged in this housing-only revision.
 start=json.loads((DEST/'starting_state.json').read_text());before={o['name']:o for o in start['occurrences']};after={o['name']:o for o in final_snapshot['occurrences']}
 def close_values(a,b,tol=1e-5):
  if isinstance(a,(int,float)) and isinstance(b,(int,float)):return abs(a-b)<=tol
  if isinstance(a,(list,tuple)) and isinstance(b,(list,tuple)):return len(a)==len(b) and all(close_values(x,y,tol) for x,y in zip(a,b))
  if isinstance(a,dict) and isinstance(b,dict):return a.keys()==b.keys() and all(close_values(a[k],b[k],tol) for k in a)
  return a==b
 protected=[]
 for name,a in before.items():
  if not name.startswith('LEG'):continue
  b=after[name];okay=close_values(a['transform'],b['transform']) and len(a['bodies'])==len(b['bodies'])
  for ab,bbp in zip(a['bodies'],b['bodies']):
   okay=okay and all(close_values(ab.get(k),bbp.get(k)) for k in ['name','box','volume_cm3','faces','edges'])
  protected.append(dict(path=name,unchanged=okay,FR07='+FR07' in name));assert okay,name
 (PKG/'leg_preservation.json').write_text(json.dumps(dict(all_unchanged=True,checks=protected),ensure_ascii=True,indent=2))
 srcid='urn:adsk.wipprod:dm.lineage:VUu_hd0ATYKQ4csy_mW31g';src=next((x for x in app.documents if x.dataFile and x.dataFile.id==srcid),None)
 preservation={'protected_source_id':srcid,'work_id':doc.dataFile.id,'independent_copy':doc.dataFile.id!=srcid,'source_open':bool(src)}
 if src:
  before=json.loads((DEST/'source_before.json').read_text());after=snapshot(src);(DEST/'source_final_snapshot.json').write_text(json.dumps(after,ensure_ascii=True))
  fields=['timeline','root_box','components','parameters'];preservation['historical_source_fields_equal']={k:close_values(before[k],after[k]) for k in fields}
  def bodykeys(x):return [(o['name'],o['transform'],[(b['name'],b['box'],b['volume_cm3'],b['faces'],b['edges']) for b in o['bodies']]) for o in x['occurrences']]
  preservation['historical_source_geometry_and_poses_equal']=close_values(bodykeys(before),bodykeys(after));preservation['source_modified_on_read']=after['modified']
  # The historical snapshot is from Sep15. Preserve the user's current source state;
  # do not revert poses, save, or require their source document to be clean.
  def intrinsic(x):return [(o['name'],[(b['name'],b['volume_cm3'],b['faces'],b['edges']) for b in o['bodies']]) for o in x['occurrences']]
  preservation['historical_source_body_geometry_equal']=close_values(intrinsic(before),intrinsic(after))
  preservation['source_action']='read only; no activation, editing, saving or pose restoration'
  source_observed=after
  (DEST/'source_observed_before_export.json').write_text(json.dumps(source_observed,ensure_ascii=True))
 (PKG/'preservation_check.json').write_text(json.dumps(preservation,indent=2))
 mdoc=app.documents.add(c.DocumentTypes.FusionDesignDocumentType);md=f.Design.cast(app.activeProduct);md.designType=f.DesignTypes.DirectDesignType
 for tag,name,q,qty,mat in parts:
  o=md.rootComponent.occurrences.addNewComponent(c.Matrix3D.create());o.component.name=tag+' '+name;o.component.bRepBodies.add(q).name=name
  md.exportManager.execute(md.exportManager.createSTEPExportOptions(str(PKG/(tag+'.step')),o.component))
 mdoc.close(False)
 mdoc=app.documents.add(c.DocumentTypes.FusionDesignDocumentType);md=f.Design.cast(app.activeProduct);md.designType=f.DesignTypes.DirectDesignType
 for name,q in assembly:
  o=md.rootComponent.occurrences.addNewComponent(c.Matrix3D.create());o.component.name=name.replace('+',' ');o.component.bRepBodies.add(q)
 md.exportManager.execute(md.exportManager.createSTEPExportOptions(str(PKG/'printed_parts_assembly.step')));mdoc.close(False);doc.activate()
 if src:
  source_end=snapshot(src)
  compare_fields=['modified','timeline','root_box','root_bodies','occurrences','components','parameters']
  preservation['source_current_state_preserved_during_export']=all(close_values(source_observed[k],source_end[k]) for k in compare_fields)
  assert preservation['source_current_state_preserved_during_export']
  (PKG/'preservation_check.json').write_text(json.dumps(preservation,indent=2))
 (PKG/'fusion_identity.json').write_text(json.dumps(dict(name=doc.name,id=doc.dataFile.id,protected_source_id=srcid),indent=2));print('R09 EXPORT COMPLETE',len(parts),len(assembly))
try:run()
except:(DEST/'export_error.txt').write_text(traceback.format_exc(),encoding='utf-8');print(traceback.format_exc())
