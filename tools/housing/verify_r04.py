exec(open('/Users/hayward_kim/Developer/SCONE/tools/housing/common.py',encoding='utf-8').read())
DEST=OUT/'R04_DELIVERY'
def run():
 app,doc,d=guard();before=json.loads((DEST/'source_before.json').read_text());src=next(x for x in app.documents if x.dataFile and x.dataFile.id==before['id']);after=snapshot(src);snap=snapshot(doc)
 def sig(b):return (b['name'],b['faces'],b['edges'],round(b['volume_cm3'],8),b['box'])
 def same_occs(a,b):return len(a)==len(b) and all(x['name']==y['name'] and max(abs(k-l) for k,l in zip(x['transform'],y['transform']))<1e-8 and [sig(z) for z in x['bodies']]==[sig(z) for z in y['bodies']] for x,y in zip(a,b))
 report={'protected_source_id':src.dataFile.id,'source_modified':src.isModified,'source_timeline_unchanged':before['timeline']==after['timeline'],'source_components_unchanged':before['components']==after['components'],'source_parameters_unchanged':before['parameters']==after['parameters'],'source_root_bodies_unchanged':[sig(b) for b in before['root_bodies']]==[sig(b) for b in after['root_bodies']],'source_occurrences_unchanged':same_occs(before['occurrences'],after['occurrences'])}
 old=json.loads((DEST/'user_pose_before.json').read_text());current={x['name']:x for x in snap['occurrences']};arc=[];pose=[];interfaces=[]
 for x in old['occurrences']:
  if not x['name'].startswith('LEG'):continue
  y=current[x['name']];delta=max(abs(k-l) for k,l in zip(x['transform'],y['transform']));pose.append([x['name'],delta])
  if '+ARC:' in x['name'] or '+ARC(' in x['name']:arc.append([x['name'],[sig(b) for b in x['bodies']]==[sig(b) for b in y['bodies']]])
  if '+LINK' in x['name']:interfaces.append([x['name'],[sig(b) for b in x['bodies'][:2]]==[sig(b) for b in y['bodies'][:2]]])
 report['arc_geometry_unchanged']=arc;report['original_link_interfaces_retained']=interfaces;report['user_pose_max_transform_delta']=max(x[1] for x in pose);report['joint_health']=[dict(component=cp.name,joint=j.name,health=j.healthState) for cp in d.allComponents for j in cp.joints]
 report['name']=doc.name;report['id']=doc.dataFile.id
 (DEST/'preservation_check.json').write_text(json.dumps(report,ensure_ascii=True,indent=2));(DEST/'final_inventory.json').write_text(json.dumps(snap,ensure_ascii=True));(DEST/'source_after.json').write_text(json.dumps(after,ensure_ascii=True));print('R04 PRESERVATION',report['source_occurrences_unchanged'],report['user_pose_max_transform_delta'],arc)
try:run()
except:(DEST/'verify_error.txt').write_text(traceback.format_exc(),encoding='utf-8');print(traceback.format_exc())
