exec(open('/Users/hayward_kim/Developer/SCONE/tools/housing/build_r05.py',encoding='utf-8').read().split('\ndef run():')[0])
DEST=OUT/'R09_DELIVERY'
app,doc,d=guard();assert doc.name.startswith('MARC Housing PLA R09');r=d.rootComponent
cp=r.occurrences.itemByName('R02 Front smooth PLA chassis:1').component;q=T.copy(current(cp));before=q.volume
for side in [-1,1]:
 cut(q,cylinder((180,side*25,-1),(180,side*25,2.5),3.3));cut(q,cylinder((180,side*25,-1),(180,side*25,35),1.65))
assert q.lumps.count==1;replace_final(cp,q,'R09 simplified front chassis, lid counterbores preserved');pla(current(cp),(222,174,45))
(DEST/'counterbore_fix.json').write_text(json.dumps(dict(centers_mm=[[180,-25],[180,25]],removed_cm3=before-q.volume,reason='Obsolete hole closure overlapped nearby functional lid-head reserve; recut original clearance.'),indent=2),encoding='utf-8')
doc.save('R09 preserve functional lid counterbores after closing obsolete adjacent holes');print('R09 HEAD RESERVES CORRECTED')
