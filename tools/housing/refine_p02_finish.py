exec(open('/Users/hayward_kim/Developer/SCONE/tools/housing/refine_p02.py',encoding='utf-8').read().split('\ndef run():')[0])
def run():
 app,doc,d=guard();r=d.rootComponent;cover=r.occurrences.itemByName('P03 PLA sensor cover:1').component
 parts=[r.occurrences.itemByName(n+':1').component for n in ['P01 Rear PLA chassis','P02 Front PLA chassis','P04 PLA electronics tray']]
 for comp in parts:
  for base in list(comp.features.baseFeatures):
   if base.name.endswith('cover fitting margin tools'):base.deleteMe()
 skin=T.copy(cover.bRepBodies.item(0));tool=T.copy(skin)
 for x,y in [(.25,0),(-.25,0),(0,.25),(0,-.25)]:union(tool,move(T.copy(skin),x=x,y=y))
 for comp in parts:cut_native(comp,comp.bRepBodies.item(0),[T.copy(tool)],comp.name[:3]+' union cover clearance')
 for comp in parts+[cover]:
  for b in comp.bRepBodies:pla(b,(222,174,45) if comp==cover else (42,47,53))
 doc.save('P02 clearances corrected for PLA printing and continuous distal spin')
 print('P02 FIT COMPLETE')
try:run()
except:(OUT/'p02_finish_error.txt').write_text(traceback.format_exc(),encoding='utf-8');print(traceback.format_exc())
