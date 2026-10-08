exec(open('/Users/hayward_kim/Developer/SCONE/tools/housing/common.py',encoding='utf-8').read())
def run():
 app,doc,d=guard();out=OUT/'R03_DELIVERY';before=json.loads((OUT/'source_before_A02.json').read_text());source=next(x for x in app.documents if x.dataFile and x.dataFile.id==before['id']);after=snapshot(source)
 def shape(b):return (b['name'],b['faces'],b['edges'],b['volume_cm3'],b['box'],b['visible'])
 result={'source_name':source.name,'source_is_modified':source.isModified,'source_id_same':before['id']==after['id'],'source_timeline_unchanged':before['timeline']==after['timeline'],'source_components_joints_unchanged':before['components']==after['components'],'source_parameters_unchanged':before['parameters']==after['parameters'],'source_root_bodies_unchanged':[shape(b) for b in before['root_bodies']]==[shape(b) for b in after['root_bodies']],'source_occurrences_unchanged':all(a['name']==b['name'] and a['transform']==b['transform'] and [shape(x) for x in a['bodies']]==[shape(x) for x in b['bodies']] for a,b in zip(before['occurrences'],after['occurrences'])) and len(before['occurrences'])==len(after['occurrences'])}
 differences=[]
 for a,b in zip(before['occurrences'],after['occurrences']):
  if a['name']!=b['name']:differences.append({'occurrence':a['name'],'different_name':b['name']});continue
  td=max(abs(x-y) for x,y in zip(a['transform'],b['transform']))
  for x,y in zip(a['bodies'],b['bodies']):
   if shape(x)!=shape(y):differences.append({'occurrence':a['name'],'body':x['name'],'face_counts':[x['faces'],y['faces']],'edge_counts':[x['edges'],y['edges']],'volume_values':[x['volume_cm3'],y['volume_cm3']],'bbox_values':[x['box'],y['box']],'visibility':[x['visible'],y['visible']],'transform_max_delta':td})
 result['source_exact_comparison_differences']=differences
 (out/'source_final_snapshot.json').write_text(json.dumps(after,ensure_ascii=True))
 snap=snapshot(doc);baseline=json.loads((OUT/'current_inventory.json').read_text());arc=[]
 for old in baseline['occurrences']:
  if '+ARC' not in old['name']:continue
  now=next(o for o in snap['occurrences'] if o['name']==old['name'])
  def geom(b):return (b['name'],b['faces'],b['edges'],b['volume_cm3'])
  arc.append({'occurrence':old['name'],'shape_unchanged':[geom(b) for b in old['bodies']]==[geom(b) for b in now['bodies']]})
 result['copy_ARC_preservation']=arc;result['joint_health']=[{'component':cp.name,'joint':j.name,'health':j.healthState} for cp in d.allComponents for j in cp.joints]
 (out/'preservation_check.json').write_text(json.dumps(result,ensure_ascii=True,indent=2));(out/'fusion_identity.json').write_text(json.dumps({'name':doc.name,'id':doc.dataFile.id,'protected_source_id':before['id']},ensure_ascii=True,indent=2));(out/'final_inventory.json').write_text(json.dumps(snap,ensure_ascii=True))
 app.activeViewport.refresh()
 print('DELIVERY VERIFIED',doc.name,doc.dataFile.id,result['source_occurrences_unchanged'],arc)
try:run()
except:print(traceback.format_exc())
