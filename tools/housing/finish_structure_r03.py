exec(open('/Users/hayward_kim/Developer/SCONE/tools/housing/refine_r03.py',encoding='utf-8').read().split('\ndef run():')[0])
def run():
 app,doc,d=guard();r=d.rootComponent
 cp=r.occurrences.itemByName('R02 Front smooth PLA chassis:1').component;q=T.copy(current(cp))
 for s in [-1,1]:cut(q,cylinder((113.1087026,s*187.64092583,18.5),(142.5087026,s*187.64092583,18.5),124.8))
 replace_final(cp,q,'R02 final PLA front frame')
 records=[]
 for path in ['LEG 1:1+LINK:1','LEG 1(미러):1+LINK(미러):1']:
  o=next(o for o in r.allOccurrences if o.fullPathName==path);cp=o.component
  if cp.attributes.itemByName('SCONE_HOUSING','link_R03'):continue
  before=sum(b.volume for b in o.bRepBodies);q=T.copy(o.bRepBodies.item(0));union(q,T.copy(o.bRepBodies.item(1)));lo,hi=bb(q)
  # Preserve all motor ends, mounting holes, center distances and outside dimensions.
  for y0,y1 in [(lo[1],lo[1]+3),(hi[1]-3,hi[1])]:union(q,box(124,169,y0,y1,4,33))
  y0,y1=lo[1]+7.5,hi[1]-7.5
  win=union(box(130,163,y0,y1,-2,40),box(126,167,y0+4,y1-4,-2,40))
  for x in [130,163]:
   for y in [y0+4,y1-4]:union(win,cylinder((x,y,-2),(x,y,40),4))
  cut(q,win);inv=o.transform2.copy();inv.invert();T.transform(q,inv)
  olds=list(cp.bRepBodies);base=cp.features.baseFeatures.add();base.name='R03 one-piece ribbed PLA link';base.startEdit();b=cp.bRepBodies.add(q,base);b.name='R03 printable cage link';pla(b);base.finishEdit()
  for b in olds:b.isLightBulbOn=False
  cp.attributes.add('SCONE_HOUSING','link_R03','1')
  records.append({'component':cp.name,'original_volume_cm3':before,'new_volume_cm3':q.volume,'reduction_pct':100*(1-q.volume/before),'lumps':q.lumps.count,'original_interface_bbox_mm':[lo,hi]})
 (OUT/'r03_link_optimization.json').write_text(json.dumps(records,ensure_ascii=True,indent=2))
 doc.save('R03 one-piece lightened PLA links; 20mm outward tire clearance allowance')
 print('R03 STRUCTURE COMPLETE',records)
try:run()
except:(OUT/'r03_structure_error.txt').write_text(traceback.format_exc(),encoding='utf-8');print(traceback.format_exc())
