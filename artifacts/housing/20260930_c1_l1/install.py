exec(open('/Users/hayward_kim/Developer/SCONE/artifacts/housing/20260930_c1_l1/geometry.py').read())
exec(open('/Users/hayward_kim/Developer/SCONE/artifacts/electronics/20260930_u2d2_docked/capture.py').read().rsplit('def run(_context):',1)[0])
OUT=Path('/Users/hayward_kim/Developer/SCONE/artifacts/housing/20260930_c1_l1')
def protected(s):
 import copy
 q=copy.deepcopy(s)
 for o in q['occurrences'].values():o['bodies']=[b for b in o['bodies'] if b['name'] not in TARGETS]
 q['root']=[b for b in q['root'] if b['name'] not in TARGETS]
 return q

_signature_cache={}
def bsig(b):
 key=b.entityToken
 if key in _signature_cache:return _signature_cache[key]
 verts=b.vertices;vs=sorted(coords(verts.item(i).geometry) for i in range(verts.count))
 q={'name':b.name,'box':bbox(b),'solid':b.isSolid,'faces':b.faces.count,'edges':b.edges.count,'vertices':hashlib.sha256(json.dumps(vs).encode()).hexdigest(),'appearance':b.appearance.name if b.appearance else None,'material':b.material.name if b.material else None,'light':b.isLightBulbOn}
 _signature_cache[key]=q;return q
def check_equal(a,b,path=''):
 if isinstance(a,(int,float)) and isinstance(b,(int,float)):return [] if abs(a-b)<2e-7 else [(path,a,b)]
 if type(a)!=type(b):return [(path,a,b)]
 if isinstance(a,dict):return [v for k in a.keys()|b.keys() for v in check_equal(a.get(k),b.get(k),path+'/'+str(k))]
 if isinstance(a,list):return [(path,'length '+str(len(a)),'length '+str(len(b)))] if len(a)!=len(b) else [v for i,(x,y) in enumerate(zip(a,b)) for v in check_equal(x,y,path+'/'+str(i))]
 return [] if a==b else [(path,a,b)]

def run(_context):
 a,doc,d=source();assert doc.name=='MARC v4 Body v5 BOX v36' and not doc.isModified
 baseline=json.loads((OUT/'baseline.json').read_text(encoding='utf-8'))
 (OUT/'install_progress.txt').write_text('Source still saved/unmodified v36. Starting target-only edits.',encoding='utf-8')
 pd=next(x for x in a.documents if x.name.startswith('C1 L1 revision preview'));dd=f.Design.cast(pd.products.itemByProductType('DesignProductType'))
 finished={b.name:T.copy(b) for b in dd.rootComponent.bRepBodies if b.name in TARGETS};assert len(finished)==2
 old,calc,regions=generate(d);before_l1=T.copy(old['L1 frame']);attrs={k:(b.appearance,b.material,b.isLightBulbOn) for k,b in old.items()}
 doc.activate();d.activateRootComponent()
 # Cut only the four external foot wedges, as a new end-of-timeline feature.
 b=old['L1 frame'].nativeObject;co=b.parentComponent;bf=co.features.baseFeatures.add();bf.name='L1 four rail foot flattening tools'
 assert bf.startEdit();tools=c.ObjectCollection.create()
 for i,q in enumerate(regions):
  tool=co.bRepBodies.add(q,bf);tool.name='L1 local foot cutter '+str(i+1);tools.add(tool)
 assert bf.finishEdit()
 tools=c.ObjectCollection.create()
 for tool in bf.bodies:tools.add(tool)
 inp=co.features.combineFeatures.createInput(b,tools);inp.operation=f.FeatureOperations.CutFeatureOperation;inp.isKeepToolBodies=False
 ft=co.features.combineFeatures.add(inp);ft.name='L1 flatten only four rail feet to Z158.5'
 (OUT/'install_progress.txt').write_text('L1 four-foot cut applied.',encoding='utf-8')
 # C1 is replaced independently; its sibling H2 and its fillets remain intact.
 b=old['C1 USB cable guide'].nativeObject;co=b.parentComponent
 rem=co.features.removeFeatures.add(b);rem.name='C1 superseded long constant-height guide'
 bf=co.features.baseFeatures.add();bf.name='C1 shorter high-low-high guide with flat rail feet'
 assert bf.startEdit();new=co.bRepBodies.add(finished['C1 USB cable guide'],bf);new.name='C1 USB cable guide';assert bf.finishEdit();new=bf.bodies.item(0)
 new.appearance=attrs['C1 USB cable guide'][0];new.material=attrs['C1 USB cable guide'][1];new.isLightBulbOn=attrs['C1 USB cable guide'][2]
 (OUT/'install_progress.txt').write_text('C1 applied. Comparing all protected geometry and poses.',encoding='utf-8')
 after=all_sig(d);(OUT/'after_install.json').write_text(json.dumps(after,ensure_ascii=False,indent=2),encoding='utf-8')
 changes=check_equal(protected(baseline),protected(after))
 (OUT/'protected_differences.json').write_text(json.dumps(changes,ensure_ascii=False,indent=2),encoding='utf-8')
 assert not changes,'Protected geometry/pose or health changed: '+str(changes)[:1500]
 live=targets(d);exact={}
 for k,q in finished.items():
  aa=T.copy(live[k]);cut(aa,q);bb=T.copy(q);cut(bb,live[k]);exact[k]=(aa.volume*1000 if aa.isSolid else 0)+(bb.volume*1000 if bb.isSolid else 0)
  assert exact[k]<1e-4
 rem=T.copy(before_l1);cut(rem,live['L1 frame'])
 for region in regions:cut(rem,region)
 outside=rem.volume*1000 if rem.isSolid else 0;assert outside<1e-5
 report={'protected_unchanged':True,'occurrence_count':len(after['occurrences']),'candidate_symmetric_difference_mm3':exact,'L1_removed_outside_four_feet_mm3':outside,'warnings':after['warnings'],'timeline_count':d.timeline.count}
 (OUT/'installation_validation.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
 print(json.dumps(report,ensure_ascii=False))
 a.activeViewport.refresh()
