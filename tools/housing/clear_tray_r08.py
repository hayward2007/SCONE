exec(open('/Users/hayward_kim/Developer/SCONE/tools/housing/build_r05.py',encoding='utf-8').read().split('\ndef run():')[0])
app,doc,d=guard();assert doc.name.startswith('MARC Housing PLA R08');r=d.rootComponent
cp=r.occurrences.itemByName('S01 Rear fairing lid:1').component;q=T.copy(current(cp));t=current(r.occurrences.itemByName('P04 PLA electronics tray:1').component)
for dz in [0,-.25]:cut(q,move(T.copy(t),z=dz))
assert q.lumps.count==1;replace_final(cp,q,'R08 final tray assembly relief');pla(current(cp),(222,174,45));doc.save('R08 final tray assembly relief')
print('TRAY RELIEF DONE')
