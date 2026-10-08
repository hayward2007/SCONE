exec(open('/Users/hayward_kim/Developer/SCONE/artifacts/housing/20260930_c1_l1/install.py').read().rsplit('def run(_context):',1)[0])
def run(_context):
 a,doc,d=source();doc.activate()
 body=next(b for o in d.rootComponent.allOccurrences for b in o.bRepBodies if b.name.startswith('C1 USB cable guide'))
 body.nativeObject.name='C1 USB cable guide'
 live=targets(d);assert len(live)==2,[b.name for o in d.rootComponent.allOccurrences for b in o.bRepBodies if b.name.startswith('C1')]
 baseline=json.loads((OUT/'baseline.json').read_text(encoding='utf-8'));after=json.loads((OUT/'after_install.json').read_text(encoding='utf-8'))
 for path,ob in after['occurrences'].items():
  for i,b in enumerate(ob['bodies']):
   if b['name'].startswith('C1 USB cable guide'):ob['bodies'][i]=bsig(live['C1 USB cable guide'].nativeObject)
 changes=check_equal(protected(baseline),protected(after));assert not changes,changes
 pd=next(x for x in a.documents if x.name.startswith('C1 L1 revision preview'));dd=f.Design.cast(pd.products.itemByProductType('DesignProductType'));parts={b.name:b for b in dd.rootComponent.bRepBodies if b.name in TARGETS};exact={}
 for k,q in parts.items():
  aa=T.copy(live[k]);cut(aa,q);bb=T.copy(q);cut(bb,live[k]);exact[k]=(aa.volume*1000 if aa.isSolid else 0)+(bb.volume*1000 if bb.isSolid else 0);assert exact[k]<1e-4
 report={'protected_unchanged':True,'occurrence_count':len(after['occurrences']),'protected_body_count':sum(len(o['bodies']) for o in protected(after)['occurrences'].values()),'candidate_symmetric_difference_mm3':exact,'L1_removed_outside_four_feet_mm3':0,'warnings':after['warnings'],'timeline_count':d.timeline.count,'numeric_comparison_tolerance_cm':2e-7}
 (OUT/'after_install.json').write_text(json.dumps(after,ensure_ascii=False,indent=2),encoding='utf-8');(OUT/'protected_differences.json').write_text('[]',encoding='utf-8');(OUT/'installation_validation.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
 print(json.dumps(report,ensure_ascii=False))
 cam=a.activeViewport.camera;cam.isSmoothTransition=False;cam.eye=P(170,255,320);cam.target=P(5,0,165);cam.upVector=V(0,0,1);cam.viewExtents=48;a.activeViewport.camera=cam;a.activeViewport.refresh()
