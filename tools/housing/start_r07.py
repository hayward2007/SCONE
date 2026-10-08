exec(open('/Users/hayward_kim/Developer/SCONE/tools/housing/common.py',encoding='utf-8').read())
DEST=OUT/'R07_DELIVERY'
app,doc,d=guard();assert doc.name.startswith('MARC Housing PLA R06')
doc.saveAs('MARC Housing PLA R07',doc.dataFile.parentFolder,'Simplified long roof matching lower chassis, accessible motor assembly','')
o=d.rootComponent.occurrences.itemByName('V02 Waveshare official camera:1');m=o.transform2;t=m.translation;t.x+=4.2;m.translation=t;o.transform2=m
if d.snapshots.hasPendingSnapshot:d.snapshots.add().name='R07 camera at extended front face'
doc.save('R07 camera moved forward 42mm for long housing front')
(DEST/'starting_state.json').write_text(json.dumps(snapshot(doc),ensure_ascii=True));print('R07 STARTED',doc.name)
