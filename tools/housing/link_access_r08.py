exec(open('/Users/hayward_kim/Developer/SCONE/tools/housing/deliver_r03.py',encoding='utf-8').read().split('\ndef run():')[0])
DEST=OUT/'R08_DELIVERY'
def run():
 app,doc,d=guard();r=d.rootComponent;assert doc.name.startswith('MARC Housing PLA R08');report=[]
 for path in ['LEG 1:1+LINK:1','LEG 1(미러):1+LINK(미러):1']:
  o=next(o for o in r.allOccurrences if o.fullPathName==path);cp=o.component
  for b in [b for b in cp.bRepBodies if b.isLightBulbOn]:
   before=bb(b);v=b.volume;edges=[]
   for e in b.edges:
    lo,hi=bb(e)
    if lo[0]>38.7 and hi[0]<66.9 and lo[1]>-58.6 and hi[1]<-44.3 and hi[2]-lo[2]<.01:edges.append(e)
   assert len(edges)==16,(path,b.name,len(edges))
   fi=cp.features.filletFeatures.createInput();fi.addConstantRadiusEdgeSet(collection(edges),c.ValueInput.createByString('0.5 mm'),False);ft=cp.features.filletFeatures.add(fi);ft.name='R08 cable-window edge relief R0.5; two separate flat plates'
   nb=ft.bodies.item(0);nb.name=b.name.replace('R04','R08');assert max(abs(a-bb(nb)[i][j]) for i,row in enumerate(before) for j,a in enumerate(row))<1e-5
   report.append(dict(part=cp.name+'/'+nb.name,window_middle_mm=[28,14],thickness_mm=5,removed_cm3=v-nb.volume,fillet_radius_mm=.5,outer_bounds_unchanged=True,motor_holes_unchanged=True))
 (DEST/'link_changes.json').write_text(json.dumps(report,indent=2))
 # Record exact existing vertical hole geometry; no inferred screw axes.
 holes=[]
 for o in r.occurrences:
  if not o.name.startswith(('R01 Rear','R02 Front','S01 Rear','S02 Front','P04 PLA','M01 Rear','M02 Front')):continue
  for face in current(o.component).faces:
   cy=c.Cylinder.cast(face.geometry)
   if cy and abs(cy.axis.z)>.99 and cy.radius<=.34:holes.append(dict(part=o.name,axis=xyz(cy.origin),radius_mm=cy.radius*10,bounds=bb(face)))
 (DEST/'vertical_holes.json').write_text(json.dumps(holes,indent=2))
 vis=[(o,o.isLightBulbOn) for o in r.occurrences]
 for o in r.occurrences:o.isLightBulbOn=o.name=='LEG 1:1'
 shot(app,'R08_DELIVERY/link_detail.png',(350,-460,250),(146,-115,18),True)
 for o,v in vis:o.isLightBulbOn=v
 doc.save('R08 minor cable-window edge relief; separate original-dimension flat LINK plates')
 print('R08 LINK AND HOLE PROBE DONE',len(holes))
try:run()
except:(DEST/'link_access_error.txt').write_text(traceback.format_exc(),encoding='utf-8');print(traceback.format_exc())
