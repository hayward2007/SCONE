exec(open('/Users/hayward_kim/Developer/SCONE/tools/housing/build_r05.py',encoding='utf-8').read().split('\ndef run():')[0])
DEST=OUT/'R07_DELIVERY'
def run():
 app,doc,d=guard();r=d.rootComponent;assert doc.name.startswith('MARC Housing PLA R07')
 nd=app.importManager.importToNewDocument(app.importManager.createSTEPImportOptions(str(DEST/'assembly_clip.step')));dd=f.Design.cast(app.activeProduct);bs=list(dd.rootComponent.bRepBodies)+[b for o in dd.rootComponent.allOccurrences for b in o.bRepBodies];cavity=T.copy(bs[0]);nd.close(False);doc.activate()
 outside=box(-100,260,-100,100,33.51,110);cut(outside,T.copy(cavity))
 report=[]
 for name,label,xc in [('R01 Rear smooth PLA chassis:1','M01 Rear removable motor bridge',-41.8912974452),('R02 Front smooth PLA chassis:1','M02 Front removable motor bridge',208.1087025548)]:
  cp=r.occurrences.itemByName(name).component;q=T.copy(current(cp))
  brace=T.copy(q);intersect(brace,box(xc-20,xc+20,-44,44,33.3,37));union(brace,box(xc-16,xc+16,-24,24,33.3,36.7))
  cut(q,box(xc-20.1,xc+20.1,-44.1,44.1,33.0,37.2))
  for side in [-1,1]:
   y=side*15;union(q,cylinder((xc,y,2),(xc,y,33.3),5));cut(q,cylinder((xc,y,27.8),(xc,y,33.5),2));cut(q,cylinder((xc,y,24),(xc,y,33.6),1.4));cut(brace,cylinder((xc,y,33),(xc,y,38),1.65))
  cut(q,T.copy(outside));replace_final(cp,q,'R07 open-top motor cradle')
  nb=part(r,label,brace,(68,73,80));report.append(dict(part=label,lumps=brace.lumps.count,bounds=bb(brace),volume_cm3=brace.volume))
 tc=r.occurrences.itemByName('P04 PLA electronics tray:1').component;tray=T.copy(current(tc));intersect(tray,T.copy(cavity));replace_final(tc,tray,'R07 tray with 0.3mm shell clearance')
 for name in ['R01 Rear smooth PLA chassis:1','R02 Front smooth PLA chassis:1']:pla(current(r.occurrences.itemByName(name).component),(222,174,45))
 doc.save('R07 removable motor bridges for top insertion; internal seam clearances')
 (DEST/'motor_bridges.json').write_text(json.dumps(dict(parts=report,motor_mount_centers_unchanged=True,example_bridge_screws='4x M3x8, example insert OD4.6 L5 in lower cradles; 16 existing MX top M2 clearances preserved',tray_lumps=tray.lumps.count),indent=2));print('MOTOR BRIDGES BUILT',report)
try:run()
except:(DEST/'assembly_refine_error.txt').write_text(traceback.format_exc(),encoding='utf-8');print(traceback.format_exc())
