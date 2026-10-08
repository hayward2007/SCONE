"""Remove inherited mounting-floor detail completely, keeping four original screw axes."""
exec(open('/Users/hayward_kim/Developer/SCONE/tools/housing/build_r05.py',encoding='utf-8').read().split('\ndef run():')[0])
DEST=OUT/'R09_DELIVERY'
def run():
 app,doc,d=guard();r=d.rootComponent;assert doc.name.startswith('MARC Housing PLA R09')
 nd=app.importManager.importToNewDocument(app.importManager.createSTEPImportOptions(str(OUT/'R08_DELIVERY'/'outer_master.step')));dd=f.Design.cast(app.activeProduct)
 bs=list(dd.rootComponent.bRepBodies)+[b for o in dd.rootComponent.allOccurrences for b in o.bRepBodies];assert len(bs)==1
 outer=T.copy(bs[0]);nd.close(False);doc.activate();candidates=[];report=[]
 for tag,name,xc,case_names in [('R01','R01 Rear smooth PLA chassis:1',-41.8912974452,['LEG 1:5','LEG 1(미러):2']),('R02','R02 Front smooth PLA chassis:1',208.1087025548,['LEG 1:1','LEG 1(미러):1'])]:
  old=current(r.occurrences.itemByName(name).component);q=T.copy(old)
  for side in [-1,1]:
   ya,yb=(24.5,44) if side>0 else (-44,-24.5)
   cut(q,box(xc-20,xc+20,ya,yb,-.6,3.49))
   slab=box(xc-20,xc+20,ya,yb,-.5,3.5);intersect(slab,T.copy(outer));union(q,slab)
   cut(q,box(xc-12.5,xc+12.5,31.9 if side>0 else -44.1,44.1 if side>0 else -31.9,-1,3.6))
   cut(q,box(xc-7,xc+7,side*20-5,side*20+5,-2,7))
   for dx,y in [(-15,35.64092583),(15,35.64092583),(-8.5,29.34092583),(8.5,29.34092583)]:
    cut(q,cylinder((xc+dx,side*y,-2),(xc+dx,side*y,2.1),2.15));cut(q,cylinder((xc+dx,side*y,2.09),(xc+dx,side*y,3.55),1.15))
  hits=[]
  for n in case_names:
   cb=T.copy(r.occurrences.itemByName(n).bRepBodies.item(0));v=intersection_volume(T.copy(q),cb)
   if v is None or v>.01:
    witness=T.copy(q);intersect(witness,cb);hits.append(dict(case=n,mm3=v,bounds=bb(witness)))
  report.append(dict(part=tag,old_faces=old.faces.count,new_faces=q.faces.count,old_cm3=old.volume,new_cm3=q.volume,hits=hits,lumps=q.lumps.count))
  candidates.append((name,q))
 (DEST/'flat_mount_candidates.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
 assert not any(x['hits'] for x in report),report
 assert all(x['lumps']==1 and x['new_faces']<x['old_faces'] for x in report),report
 for name,q in candidates:
  cp=r.occurrences.itemByName(name).component;replace_final(cp,q,'R09 flat motor mounting rails with original bolt axes');pla(current(cp),(222,174,45))
 doc.save('R09 remove inherited motor-floor imprints; simple flat mounting rails, unchanged fastener axes')
 (DEST/'flat_mounts_applied.json').write_text(json.dumps(report,indent=2),encoding='utf-8');print('R09 FLAT MOUNTS APPLIED',report)
 md=app.documents.add(c.DocumentTypes.FusionDesignDocumentType);model=f.Design.cast(app.activeProduct);model.designType=f.DesignTypes.DirectDesignType
 for name,q in candidates:
  oc=model.rootComponent.occurrences.addNewComponent(c.Matrix3D.create());oc.component.bRepBodies.add(q);model.exportManager.execute(model.exportManager.createSTEPExportOptions(str(DEST/(name[:3]+'_native.step')),oc.component))
 md.close(False);doc.activate()
try:run()
except:(DEST/'flat_mount_error.txt').write_text(traceback.format_exc(),encoding='utf-8');print(traceback.format_exc())
