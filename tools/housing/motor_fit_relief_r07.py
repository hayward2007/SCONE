exec(open('/Users/hayward_kim/Developer/SCONE/tools/housing/build_r05.py',encoding='utf-8').read().split('\ndef run():')[0])
DEST=OUT/'R07_DELIVERY'
def run():
 app,doc,d=guard();r=d.rootComponent;parts=[]
 for name,parents in [('R01 Rear smooth PLA chassis:1',['LEG 1:5','LEG 1(미러):2']),('R02 Front smooth PLA chassis:1',['LEG 1:1','LEG 1(미러):1'])]:
  cp=r.occurrences.itemByName(name).component;q=T.copy(current(cp))
  for n in parents:
   motor=r.occurrences.itemByName(n).bRepBodies.item(0);side=1 if '(미러)' in n else -1
   for dx in [-.12,.12]:cut(q,move(T.copy(motor),x=dx,y=-side*.12,z=-.15))
  assert q.lumps.count==1;parts.append((cp,q))
 for cp,q in parts:replace_final(cp,q,'R07 0.12mm radial casing fit relief');pla(current(cp),(222,174,45))
 doc.save('R07 casing fit relief with motor center and screw axes unchanged');(DEST/'motor_fit_relief.json').write_text(json.dumps({'radial_relief_mm':.12,'vertical_relief_mm':.15,'motor_centers_changed':False,'screw_axes_changed':False}));print('MOTOR FIT RELIEF DONE')
try:run()
except:(DEST/'motor_fit_error.txt').write_text(traceback.format_exc(),encoding='utf-8');print(traceback.format_exc())
