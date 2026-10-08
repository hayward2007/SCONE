exec(open('/Users/hayward_kim/Developer/SCONE/artifacts/housing/20260930_c1_l1/fast_compare.py').read().rsplit('def run(_context):',1)[0])
def run(_context):
 global bsig
 a,doc,d=source();doc.activate();bsig=fastbsig
 current=all_sig(d);expected=json.loads((OUT/'after_install.json').read_text(encoding='utf-8'))
 for o in expected['occurrences'].values():
  for b in o['bodies']:b.pop('vertices',None)
 for b in expected['root']:b.pop('vertices',None)
 changes=check_equal(expected,current);assert not changes,changes
 out={'document':doc.name,'version':doc.dataFile.versionNumber,'modified':doc.isModified,'saved':doc.isSaved,'after_save_differences':changes,'protected_body_count':266,'occurrences':len(current['occurrences']),'warnings':current['warnings']}
 (OUT/'postsave_validation.json').write_text(json.dumps(out,ensure_ascii=False,indent=2),encoding='utf-8');print(json.dumps(out,ensure_ascii=False))
 # Keep the current model visible, close only this task's disposable preview.
 for pd in list(a.documents):
  if pd.name.startswith('C1 L1 revision preview'):pd.close(False)
 doc.activate();cam=a.activeViewport.camera;cam.isSmoothTransition=False;cam.eye=P(170,255,320);cam.target=P(5,0,165);cam.upVector=V(0,0,1);cam.viewExtents=48;a.activeViewport.camera=cam;a.activeViewport.refresh()
