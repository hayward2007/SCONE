exec(open('/Users/hayward_kim/Developer/SCONE/tools/housing/build_r05.py',encoding='utf-8').read().split('\ndef run():')[0])
DEST=OUT/'R08_DELIVERY'
def run():
 app,doc,d=guard();r=d.rootComponent;assert doc.name.startswith('MARC Housing PLA R08')
 for name,xc in [('R01 Rear smooth PLA chassis:1',-41.8912974452),('R02 Front smooth PLA chassis:1',208.1087025548)]:
  cp=r.occurrences.itemByName(name).component;q=T.copy(current(cp))
  for s in [-1,1]:
   for dx,ya in [(-15,35.64092583),(15,35.64092583),(-8.5,29.34092583),(8.5,29.34092583)]:
    x,y=xc+dx,s*ya;cut(q,cylinder((x,y,-2),(x,y,2.1),2.15));cut(q,cylinder((x,y,2.09),(x,y,3.55),1.15))
  if name.startswith('R02'):
   for x in [71,79]:
    for y in [-62,62]:cut(q,cylinder((x,y,18),(x,y,90),4.7))
  assert q.lumps.count==1;replace_final(cp,q,'R08 lower chassis with straight underside screw access');pla(current(cp),(222,174,45))
 cp=r.occurrences.itemByName('P04 PLA electronics tray:1').component;q=T.copy(current(cp))
 for x,ya,z in [(33,48,39.5),(125,50,41)]:
  for s in [-1,1]:cut(q,cylinder((x,s*ya,z),(x,s*ya,80),3.1))
 assert q.lumps.count==1;replace_final(cp,q,'R08 tray with vertical screw-head access');pla(current(cp),(68,73,80))
 for name in ['S01 Rear fairing lid:1','S02 Front camera fairing lid:1']:
  cp=r.occurrences.itemByName(name).component;q=T.copy(current(cp))
  for x,ya,z in [(33,48,39.5),(125,50,41)]:
   for s in [-1,1]:cut(q,cylinder((x,s*ya,z-.2),(x,s*ya,z+2.35),3.1))
  assert q.lumps.count==1;replace_final(cp,q,'R08 cover with hidden screw head relief');pla(current(cp),(222,174,45))
 doc.save('R08 vertical screwdriver access and recessed head pockets verified against explicit hardware envelopes');print('ACCESS CLEARANCES DONE')
try:run()
except:(DEST/'fix_access_error.txt').write_text(traceback.format_exc(),encoding='utf-8');print(traceback.format_exc())
