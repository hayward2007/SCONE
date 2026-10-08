exec(open('/Users/hayward_kim/Developer/SCONE/tools/housing/deliver_r03.py',encoding='utf-8').read().split('\ndef run():')[0])
DEST=OUT/'R08_DELIVERY'
app,doc,d=guard();r=d.rootComponent;assert doc.name.startswith('MARC Housing PLA R08')
saved={o['name']:o for o in json.loads((DEST/'user_pose_before.json').read_text())['occurrences']}
cp=compnew(r,'CHECK R08 user rear pose - reference only',True);temps=[]
for o in r.allOccurrences:
 if not o.fullPathName.startswith('LEG 1:5'):continue
 m=c.Matrix3D.create();m.setWithArray(saved[o.fullPathName]['transform'])
 for b in o.bRepBodies:
  if b.isSolid and b.isLightBulbOn:
   q=T.copy(b.nativeObject if b.nativeObject else b);T.transform(q,m);temps.append((o.fullPathName+'/'+b.name,q))
persist(cp,temps,(70,75,82),'PrismMaterial-022','Recorded user pose reference')
co=r.occurrences.itemByName(cp.name+':1');co.isLightBulbOn=False;doc.save('Record user clearance pose as hidden immutable reference; source FR07 geometry unchanged')
vis=[(o,o.isLightBulbOn) for o in r.occurrences]
r.occurrences.itemByName('LEG 1:5').isLightBulbOn=False;co.isLightBulbOn=True
shot(app,'R08_DELIVERY/user_pose_clearance.png',(440,-550,320),(65,0,25),True)
for o,v in vis:o.isLightBulbOn=v
(DEST/'pose_reference.json').write_text(json.dumps(dict(component=cp.name,bodies=len(temps),source='user_pose_before.json',saved_design_pose='Native joints may recompute to their original pose when saving; this hidden solid reference preserves the exact demonstrated pose independently.'),indent=2));print('USER POSE REFERENCE SAVED')
