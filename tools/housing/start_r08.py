exec(open('/Users/hayward_kim/Developer/SCONE/tools/housing/build_r05.py',encoding='utf-8').read().split('\ndef run():')[0])
DEST=OUT/'R08_DELIVERY';DEST.mkdir(exist_ok=True)
def run():
 app,doc,d=guard();r=d.rootComponent;assert doc.name.startswith('MARC Housing PLA R07')
 d.exportManager.execute(d.exportManager.createFusionArchiveExportOptions(str(DEST/'R07_before_tire_ribs.f3d')))
 for n in ['source_before.json','user_pose_before.json']:(DEST/n).write_bytes((OUT/'R07_DELIVERY'/n).read_bytes())
 doc.saveAs('MARC Housing PLA R08',doc.dataFile.parentFolder,'20mm TPU tire, minimal axial rim changes, ribbed PLA chassis and vertical tool access','')
 info=[]
 for path in ['LEG 1:1+ARC:1','LEG 1:5+ARC:1','LEG 1(미러):1+ARC(미러):1','LEG 1:1+LINK:1','LEG 1(미러):1+LINK(미러):1']:
  o=next((o for o in r.allOccurrences if o.fullPathName==path),None)
  if o:info.append(dict(path=path,component=o.component.name,transform=list(o.transform2.asArray()),bodies=[dict(name=b.name,visible=b.isLightBulbOn,world=bb(b),local=bb(b.nativeObject if b.nativeObject else b),volume_cm3=b.volume,material=b.material.name if b.material else None) for b in o.bRepBodies]))
 for o in r.allOccurrences:
  if '+ARC' in o.fullPathName:info.append(dict(arc_path=o.fullPathName,component=o.component.name))
 (DEST/'component_probe.json').write_text(json.dumps(info,ensure_ascii=True,indent=2));(DEST/'starting_state.json').write_text(json.dumps(snapshot(doc),ensure_ascii=True));print('R08 STARTED',doc.name,info)
try:run()
except:(DEST/'start_error.txt').write_text(traceback.format_exc(),encoding='utf-8');print(traceback.format_exc())
