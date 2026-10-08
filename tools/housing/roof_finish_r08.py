exec(open('/Users/hayward_kim/Developer/SCONE/tools/housing/build_r05.py',encoding='utf-8').read().split('\ndef run():')[0])
DEST=OUT/'R08_DELIVERY'
def run():
 app,doc,d=guard();r=d.rootComponent;assert doc.name.startswith('MARC Housing PLA R08')
 nd=app.importManager.importToNewDocument(app.importManager.createSTEPImportOptions(str(DEST/'shell_master.step')));dd=f.Design.cast(app.activeProduct);bs=list(dd.rootComponent.bRepBodies)+[b for o in dd.rootComponent.allOccurrences for b in o.bRepBodies];shell=T.copy(bs[0]);nd.close(False);doc.activate()
 cp=r.occurrences.itemByName('S01 Rear fairing lid:1').component;q=T.copy(current(cp));before=q.volume
 intersect(shell,box(24,82.85,-100,100,55.8,110));union(q,shell)
 for s in [-1,1]:cut(q,box(30,72,52.9 if s>0 else -53.8,53.8 if s>0 else -52.9,59,93))
 for x,y in [(38.9,-14.1),(67.1,14.1)]:cut(q,cylinder((x,y,98),(x,y,106),1.15))
 cut(q,box(48,58,-27,-19,99,106))
 for x in [79,85]:cut(q,box(x-1.5,x+1.5,-24,26,98,110))
 assert q.lumps.count==1;replace_final(cp,q,'R08 continuous level roof and full-thickness high side panels');pla(current(cp),(222,174,45));doc.save('R08 unified full-length flat roof, 2.8mm roof and 2.2mm selected side panels')
 (DEST/'roof_finish.json').write_text(json.dumps(dict(before_cm3=before,after_cm3=q.volume,roof_z_mm=104,nominal_roof_thickness_mm=2.8,high_side_panels_mm=2.2,repair='Restored outer upper skin unintentionally clipped by neck core mask above z56'),indent=2));print('ROOF FINISHED')
try:run()
except:(DEST/'roof_finish_error.txt').write_text(traceback.format_exc(),encoding='utf-8');print(traceback.format_exc())
