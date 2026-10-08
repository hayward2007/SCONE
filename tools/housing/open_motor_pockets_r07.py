exec(open('/Users/hayward_kim/Developer/SCONE/tools/housing/build_r05.py',encoding='utf-8').read().split('\ndef run():')[0])
DEST=OUT/'R07_DELIVERY'
app,doc,d=guard();r=d.rootComponent
for name,xc in [('R01 Rear smooth PLA chassis:1',-41.8912974452),('R02 Front smooth PLA chassis:1',208.1087025548)]:
 cp=r.occurrences.itemByName(name).component;q=T.copy(current(cp))
 for side in [-1,1]:cut(q,box(xc-18.10,xc+18.10,26.25 if side>0 else -78,78 if side>0 else -26.25,3.49,33.11))
 assert q.lumps.count==1;replace_final(cp,q,'R07 motor top insertion pocket, original lower bolt centers retained');pla(current(cp),(222,174,45))
doc.save('R07 open motor insertion pockets with 0.3mm side clearance');print('MOTOR INSERTION POCKETS OPEN')
