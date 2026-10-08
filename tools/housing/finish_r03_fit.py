exec(open('/Users/hayward_kim/Developer/SCONE/tools/housing/refine_r03.py',encoding='utf-8').read().split('\ndef run():')[0])
def run():
 app,doc,d=guard();r=d.rootComponent
 cp=r.occurrences.itemByName('P04 PLA electronics tray:1').component;q=T.copy(current(cp));cut(q,box(111.7,113.1,-64.1,-62.8,36,40));replace_final(cp,q,'P04 final PLA tray')
 cp=r.occurrences.itemByName('R03 Smooth PLA sensor cover:1').component;q=T.copy(current(cp));cut(q,box(165.5,174,-6.5,6.5,101.1,102));replace_final(cp,q,'R03 final PLA cover')
 # Native reference sources occasionally re-enable their predecessor on recompute; assert exact final visibility.
 for n in ['R01 Rear smooth PLA chassis:1','R02 Front smooth PLA chassis:1','R03 Smooth PLA sensor cover:1','P04 PLA electronics tray:1']:
  cp=r.occurrences.itemByName(n).component
  for i,b in enumerate(cp.bRepBodies):b.isLightBulbOn=(i==cp.bRepBodies.count-1)
 for cp in d.allComponents:cp.isJointsFolderLightBulbOn=False
 doc.save('R03 final camera connector recess and single-solid tray')
 print('R03 FINAL FIT')
try:run()
except:print(traceback.format_exc())
