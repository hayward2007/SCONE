exec(open('/Users/hayward_kim/Developer/SCONE/tools/housing/build_r05.py',encoding='utf-8').read().split('\ndef run():')[0])
DEST=OUT/'R09_DELIVERY'
def run():
 app,doc,d=guard();r=d.rootComponent;assert doc.name.startswith('MARC Housing PLA R09') and 'Assembled' not in doc.name
 before=snapshot(doc);(DEST/'separated_cover_layout_before.json').write_text(json.dumps(before,ensure_ascii=True),encoding='utf-8')
 d.exportManager.execute(d.exportManager.createFusionArchiveExportOptions(str(DEST/'R09_separated_cover_layout.f3d')))
 old_id=doc.dataFile.id;doc.saveAs('MARC Housing PLA R09 Assembled',doc.dataFile.parentFolder,'Assembled delivery copy; preserve separated cover layout in prior R09 file','')
 changes=[]
 for n in ['S01 Rear fairing lid:1','S02 Front camera fairing lid:1']:
  o=r.occurrences.itemByName(n);changes.append(dict(part=n,previous_transform=list(o.transform2.asArray())));o.transform2=c.Matrix3D.create()
 if d.snapshots.hasPendingSnapshot:d.snapshots.add()
 doc.save('R09 Assembled: final cover mounting positions; separated layout retained in R09')
 (DEST/'assembled_copy.json').write_text(json.dumps(dict(source_file_id=old_id,assembled_file_id=doc.dataFile.id,changes=changes,protected_original_untouched=True,FR07_untouched=True),indent=2),encoding='utf-8')
 print('R09 ASSEMBLED COPY',doc.name,doc.dataFile.id)
try:run()
except:(DEST/'assembled_copy_error.txt').write_text(traceback.format_exc(),encoding='utf-8');print(traceback.format_exc())
