exec(open('/Users/hayward_kim/Developer/SCONE/tools/housing/deliver_r03.py',encoding='utf-8').read().split('\ndef run():')[0])
DEST=OUT/'R04_DELIVERY'
def run():
 app,doc,d=guard();r=d.rootComponent;assert doc.name.startswith('MARC Housing PLA R04')
 check=json.loads((DEST/'checks.json').read_text());assert all(not v for k,v in check.items() if k.endswith('_hits'))
 assert all(not v for k,v in check['camera_assembly'].items() if k.endswith('_hits'))
 assert all(x['lumps']==1 for x in check['part_topology'])
 old=json.loads((DEST/'user_pose_before.json').read_text());now=snapshot(doc);no={x['name']:x for x in now['occurrences']}
 assert all(max(abs(a-b) for a,b in zip(o['transform'],no[o['name']]['transform']))<1e-8 for o in old['occurrences'] if o['name'].startswith('LEG')),'User leg pose changed'
 names=['R01 Rear smooth PLA chassis:1','R02 Front smooth PLA chassis:1','R03 Smooth PLA sensor cover:1','P04 PLA electronics tray:1'];parts=[]
 for n in names:
  b=current(r.occurrences.itemByName(n).component);pla(b,(222,174,45) if n.startswith('R03') else (68,73,80));parts.append((n.split(':')[0],T.copy(b)))
 for path,label in [('LEG 1:1+LINK:1','L1'),('LEG 1(미러):1+LINK(미러):1','L2')]:
  o=next(o for o in r.allOccurrences if o.fullPathName==path)
  for i,b in enumerate(b for b in o.bRepBodies if b.isLightBulbOn):
   assert b.lumps.count==1;pla(b,(68,73,80));parts.append((label+str(i+1)+' PLA 5mm plate',T.copy(b)))
 meshes=[]
 for n,q in parts:
  calc=q.meshManager.createMeshCalculator();calc.surfaceTolerance=.003;m=calc.calculate();meshes.append(dict(name=n,vertices_mm=[v*10 for v in m.nodeCoordinatesAsDouble],triangles=list(m.nodeIndices),bounds=bb(q),volume_cm3=q.volume,lumps=q.lumps.count))
 with gzip.open(DEST/'manufacturing_meshes.json.gz','wt',encoding='utf-8') as h:json.dump(meshes,h)
 # Readable native-camera images; all temporary visibility changes are restored.
 vis=[(o,o.isLightBulbOn) for o in r.occurrences];rootvis=[(b,b.isLightBulbOn) for b in r.bRepBodies]
 for cp in d.allComponents:cp.isJointsFolderLightBulbOn=False;cp.isSketchFolderLightBulbOn=False;cp.isConstructionFolderLightBulbOn=False
 shot(app,'R04_DELIVERY/user_pose_fixed.png',(500,-610,300),(70,0,20),True)
 for o in r.occurrences:
  if o.name.startswith('LEG') or o.name.startswith('V'):o.isLightBulbOn=False
 shot(app,'R04_DELIVERY/housing_detail.png',(380,-440,260),(83,0,47),True)
 for o in r.occurrences:o.isLightBulbOn=o.name in ['R03 Smooth PLA sensor cover:1','V02 Waveshare official camera:1']
 for b in r.bRepBodies:b.isLightBulbOn=False
 shot(app,'R04_DELIVERY/camera_installed_inside.png',(-100,-160,-150),(102,0,74),True)
 # Exploded camera is a named reference copy, never a printed body or a joint edit.
 cb=next(o for o in r.allOccurrences if o.fullPathName=='V02 Waveshare official camera:1+0619:1').bRepBodies.item(0)
 cp=compnew(r,'CHECK R04 Camera insertion',True);persist(cp,[('Camera 66mm behind final position',move(T.copy(cb),x=-66))],(60,110,70),'PrismMaterial-022','Camera assembly reference')
 r.occurrences.itemByName('V02 Waveshare official camera:1').isLightBulbOn=False
 shot(app,'R04_DELIVERY/camera_insertion.png',(-115,-150,-125),(102,0,70),True)
 r.occurrences.itemByName(cp.name+':1').isLightBulbOn=False
 for o,v in vis:o.isLightBulbOn=v
 for b,v in rootvis:b.isLightBulbOn=v
 # Isolate the unchanged original motor layout with its revised upper/lower plates.
 for o in r.occurrences:o.isLightBulbOn=o.name=='LEG 1:1'
 shot(app,'R04_DELIVERY/link_plates.png',(390,-410,245),(143,-113,20),True)
 for o,v in vis:o.isLightBulbOn=v
 shot(app,'R04_DELIVERY/user_pose_fixed.png',(500,-610,300),(70,0,20),True)
 doc.save('R04 checked actual user pose, camera insertion and tool access; separate plate links')
 d.exportManager.execute(d.exportManager.createFusionArchiveExportOptions(str(DEST/'MARC_Housing_PLA_R04.f3d')))
 mdoc=app.documents.add(c.DocumentTypes.FusionDesignDocumentType);md=f.Design.cast(app.activeProduct);md.designType=f.DesignTypes.DirectDesignType
 for n,q in parts:
  o=md.rootComponent.occurrences.addNewComponent(c.Matrix3D.create());o.component.name=n;nb=o.component.bRepBodies.add(q);nb.name=n
  md.exportManager.execute(md.exportManager.createSTEPExportOptions(str(DEST/(n[:3]+'.step')),o.component))
 md.exportManager.execute(md.exportManager.createSTEPExportOptions(str(DEST/'printed_parts_assembly.step')))
 mdoc.close(False);doc.activate();app.activeViewport.refresh()
 (DEST/'fusion_identity.json').write_text(json.dumps(dict(name=doc.name,id=doc.dataFile.id,protected_source_id='urn:adsk.wipprod:dm.lineage:VUu_hd0ATYKQ4csy_mW31g'),indent=2))
 print('R04 DELIVERED',len(parts),doc.name)
try:run()
except:(DEST/'delivery_error.txt').write_text(traceback.format_exc(),encoding='utf-8');print(traceback.format_exc())
