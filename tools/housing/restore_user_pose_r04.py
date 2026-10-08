exec(open('/Users/hayward_kim/Developer/SCONE/tools/housing/refine_r03.py',encoding='utf-8').read().split('\ndef run():')[0])
DEST=OUT/'R04_DELIVERY'
def run():
 app,doc,d=guard();r=d.rootComponent;old=json.loads((DEST/'user_pose_before.json').read_text());rows={o['name']:o for o in old['occurrences']}
 paths=['LEG 1:5+FR07:1','LEG 1:5+LINK:1','LEG 1:5+XM430:2','LEG 1:5+XM430:1','LEG 1:5+ARC:1']
 for path in paths:
  o=next(x for x in r.allOccurrences if x.fullPathName==path);m=c.Matrix3D.create();m.setWithArray(rows[path]['transform']);o.transform2=m
 now=snapshot(doc);changes=[]
 for o in now['occurrences']:
  if o['name'].startswith('LEG'):
   delta=max(abs(a-b) for a,b in zip(o['transform'],rows[o['name']]['transform']))
   if delta>1e-7:changes.append([o['name'],delta])
 print('POSE RESTORE',len(changes),'PENDING',d.snapshots.hasPendingSnapshot,changes[:5])
 if not changes and d.snapshots.hasPendingSnapshot:
  s=d.snapshots.add();s.name='R04 preserved user collision example pose';doc.save('Preserve the exact user supplied leg pose as a captured position')
 (DEST/'restored_pose_check.json').write_text(json.dumps(dict(differences=changes,snapshot_count=d.snapshots.count),indent=2))
except_placeholder=0
try:run()
except:(DEST/'restore_error.txt').write_text(traceback.format_exc(),encoding='utf-8');print(traceback.format_exc())
