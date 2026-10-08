"""Native Fusion integration. Run only after UI access returns; never targets original MARC."""
exec(open('/Users/hayward_kim/Developer/SCONE/tools/housing/build_r05.py',encoding='utf-8').read().split('\ndef run():')[0])
DEST=OUT/'R09_DELIVERY'
def fr07(r):
 result=[];seen=set()
 for o in r.allOccurrences:
  if '+FR07:' not in o.fullPathName or o.component.entityToken in seen:continue
  seen.add(o.component.entityToken)
  result.append(dict(component=o.component.name,bodies=[dict(name=b.name,solid=b.isSolid,volume=b.volume if b.isSolid else None,bounds=bb(b),faces=b.faces.count,edges=b.edges.count) for b in o.component.bRepBodies]))
 return sorted(result,key=lambda x:x['component'])
def run():
 app,doc,d=guard();r=d.rootComponent
 assert doc.name.startswith(('MARC Housing PLA R08','MARC Housing PLA R09')),doc.name
 assert not (DEST/'native_integrated.json').exists(),'R09 already integrated; inspect rather than repeat.'
 before_fr=fr07(r);assert before_fr
 if doc.name.startswith('MARC Housing PLA R08'):
  assert doc.dataFile.id=='urn:adsk.wipprod:dm.lineage:rvCbFXWSSfqHCgwOTRqCOA'
  d.exportManager.execute(d.exportManager.createFusionArchiveExportOptions(str(DEST/'R08_before_simplification.f3d')))
  (DEST/'starting_state.json').write_text(json.dumps(snapshot(doc),ensure_ascii=True),encoding='utf-8')
  for n in ['source_before.json','user_pose_before.json']:(DEST/n).write_bytes((OUT/'R08_DELIVERY'/n).read_bytes())
  doc.saveAs('MARC Housing PLA R09',doc.dataFile.parentFolder,'Simplified functional housing; FR07 preserved','')
 else:assert (DEST/'R08_before_simplification.f3d').exists()
 assert doc.dataFile.id!='urn:adsk.wipprod:dm.lineage:VUu_hd0ATYKQ4csy_mW31g'
 def read_step(n):
  nd=app.importManager.importToNewDocument(app.importManager.createSTEPImportOptions(str(DEST/n)));dd=f.Design.cast(app.activeProduct)
  bs=list(dd.rootComponent.bRepBodies)+[b for o in dd.rootComponent.allOccurrences for b in o.bRepBodies];assert len(bs)==1,n
  q=T.copy(bs[0]);nd.close(False);doc.activate();return q
 # Prepare all candidates before replacing any visible body.
 shapes={};records=[]
 for tag,name in [('S01','S01 Rear fairing lid:1'),('S02','S02 Front camera fairing lid:1')]:
  q=read_step(tag+'_clean.step');old=current(r.occurrences.itemByName(name).component)
  assert q.isSolid and q.lumps.count==1 and .9<q.volume/old.volume<1.05
  shapes[name]=q;records.append(dict(part=tag,before_cm3=old.volume,after_cm3=q.volume))
 for tag,name,xc in [('R01','R01 Rear smooth PLA chassis:1',-41.8912974452),('R02','R02 Front smooth PLA chassis:1',208.1087025548)]:
  old=current(r.occurrences.itemByName(name).component);q=T.copy(old)
  for side in [-1,1]:
   ya,yb=(26.1,43.5) if side>0 else (-43.5,-26.1)
   zone=box(xc-18.2,xc+18.2,ya,yb,1.9,3.51)
   for dx,y in [(-15,35.64092583),(15,35.64092583),(-8.5,29.34092583),(8.5,29.34092583)]:cut(zone,cylinder((xc+dx,side*y,1.8),(xc+dx,side*y,3.6),3.4))
   cut(q,zone)
   slab=box(xc-18.2,xc+18.2,ya,yb,-.5,1.9)
   # Limit slab to the original external envelope; this is not a motor-axis change.
   outer_path=OUT/'R08_DELIVERY'/'outer_master.step'
   nd=app.importManager.importToNewDocument(app.importManager.createSTEPImportOptions(str(outer_path)));dd=f.Design.cast(app.activeProduct)
   bs=list(dd.rootComponent.bRepBodies)+[b for o in dd.rootComponent.allOccurrences for b in o.bRepBodies];assert len(bs)==1
   outer=T.copy(bs[0]);nd.close(False);doc.activate();intersect(slab,outer);union(q,slab)
   for dx,y in [(-15,35.64092583),(15,35.64092583),(-8.5,29.34092583),(8.5,29.34092583)]:
    cut(q,cylinder((xc+dx,side*y,-2),(xc+dx,side*y,2.1),2.15));cut(q,cylinder((xc+dx,side*y,2.09),(xc+dx,side*y,3.55),1.15))
   # One rectangular opening replaces the casing-shaped underside imprint.
   # The casing bulge starts at z0.75, while the four unchanged seats remain above z2.1.
   cut(q,box(xc-12.5,xc+12.5,31.9 if side>0 else -44,44 if side>0 else -31.9,-1,2.05))
  filled=[]
  if tag=='R02':
   for x,y in [(142,30),(168,32),(180,30)]:
    for s in [-1,1]:union(q,cylinder((x,s*y,-.5),(x,s*y,3.5),2.1));filled.append([x,s*y])
   for s in [-1,1]:
    cut(q,cylinder((180,s*25,-1),(180,s*25,2.5),3.3));cut(q,cylinder((180,s*25,-1),(180,s*25,35),1.65))
  # These were intended strain-relief slots, subsequently closed by inherited floor skins.
  # Restore their function instead of treating the trapped cavities as disposable holes.
  x=0 if tag=='R01' else 158
  for s in [-1,1]:cut(q,box(x-3,x+3,s*19-1,s*19+1,-1,5.5))
  assert q.isSolid and q.lumps.count==1 and .94<q.volume/old.volume<1.08,(tag,q.volume,old.volume,q.lumps.count)
  shapes[name]=q;records.append(dict(part=tag,before_cm3=old.volume,after_cm3=q.volume,obsolete_holes_filled=filled,reopened_strain_relief_slot_centers=[[x,-19],[x,19]]))
 # Reject an overlap before changing visibility. Contact is allowed, volumetric overlap is not.
 targets=[(o.name,T.copy(current(o.component))) for o in r.occurrences if o.name.startswith(('P04 PLA','M01 Rear','M02 Front'))]
 targets += [(o.fullPathName+'/'+b.name,T.copy(b)) for o in r.allOccurrences if o.fullPathName.startswith('V') for b in o.bRepBodies if b.isSolid and b.isLightBulbOn]
 targets += [(o.name,T.copy(o.bRepBodies.item(0))) for o in r.occurrences if o.name in ['LEG 1:1','LEG 1(미러):1','LEG 1:5','LEG 1(미러):2']]
 pairs=list(shapes.items());hits=[]
 for i,(name,q) in enumerate(pairs):
  for tn,t in targets+pairs[i+1:]:
   v=intersection_volume(T.copy(q),T.copy(t))
   if v is None or v>.01:
    witness=T.copy(q);intersect(witness,T.copy(t));hits.append(dict(a=name,b=tn,overlap_mm3=v,bounds=bb(witness)))
 (DEST/'native_candidate_hits.json').write_text(json.dumps(hits,indent=2),encoding='utf-8');assert not hits,hits
 for name,q in pairs:
  cp=r.occurrences.itemByName(name).component;replace_final(cp,q,'R09 '+name[:3]+' simplified functional housing');pla(current(cp),(222,174,45))
 assert fr07(r)==before_fr,'FR07 geometry changed unexpectedly'
 (DEST/'native_integrated.json').write_text(json.dumps(dict(parts=records,FR07_unchanged=True,status='Geometry integrated; motion, tool access and exports still require revalidation'),indent=2),encoding='utf-8')
 doc.save('R09 simplified housings; original model and FR07 unchanged; pending full revalidation')
 print('R09 INTEGRATED; RUN ALL R09 CHECKS BEFORE MANUFACTURING EXPORT')
try:run()
except:(DEST/'native_integration_error.txt').write_text(traceback.format_exc(),encoding='utf-8');print(traceback.format_exc())
