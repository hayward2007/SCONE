exec(open('/Users/hayward_kim/Developer/SCONE/tools/housing/build_r05.py',encoding='utf-8').read().split('\ndef run():')[0])
DEST=OUT/'R06_DELIVERY';DEST.mkdir(exist_ok=True)
def run():
 app,doc,d=guard();r=d.rootComponent;assert doc.name.startswith('MARC Housing PLA R05')
 d.exportManager.execute(d.exportManager.createFusionArchiveExportOptions(str(DEST/'R05_before_faceted_revision.f3d')))
 for name in ['source_before.json','user_pose_before.json']:(DEST/name).write_bytes((OUT/'R05_DELIVERY'/name).read_bytes())
 doc.saveAs('MARC Housing PLA R06',doc.dataFile.parentFolder,'Planar facet housing with TTL access and verified assembly routes','')
 items=[]
 for n in ['R01 Rear smooth PLA chassis:1','R02 Front smooth PLA chassis:1','P04 PLA electronics tray:1']:
  items.append((n[:3],T.copy(current(r.occurrences.itemByName(n).component))))
 for i,n in enumerate(['LEG 1:1','LEG 1(미러):1','LEG 1:5','LEG 1(미러):2']):items.append(('MX'+str(i),T.copy(r.occurrences.itemByName(n).bRepBodies.item(0))))
 items.append(('CAM',T.copy(next(o for o in r.allOccurrences if o.fullPathName=='V02 Waveshare official camera:1+0619:1').bRepBodies.item(0))))
 mdoc=app.documents.add(c.DocumentTypes.FusionDesignDocumentType);md=f.Design.cast(app.activeProduct);md.designType=f.DesignTypes.DirectDesignType
 for n,q in items:
  cp=md.rootComponent.occurrences.addNewComponent(c.Matrix3D.create()).component;cp.name=n;cp.bRepBodies.add(q)
  md.exportManager.execute(md.exportManager.createSTEPExportOptions(str(DEST/(n+'_input.step')),cp))
 mdoc.close(False);doc.activate()
 (DEST/'starting_state.json').write_text(json.dumps(snapshot(doc),ensure_ascii=True));print('R06 STARTED AND EXPORTED',doc.name)
try:run()
except:(DEST/'start_error.txt').write_text(traceback.format_exc(),encoding='utf-8');print(traceback.format_exc())
