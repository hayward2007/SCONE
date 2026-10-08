exec(open('/Users/hayward_kim/Developer/SCONE/artifacts/arm/20260929_refinement/inspect_and_backup.py').read().split('def run(_context):')[0])
def run(_context):
 app=c.Application.get();d=f.Design.cast(app.activeProduct);r=d.rootComponent
 assert app.activeDocument.name.startswith('MARC v4 Arm R1 Folded')
 before=json.loads((OUT/'protected_before.json').read_text());cache={};actual={};mismatch=[]
 for o in r.allOccurrences:
  if o.fullPathName.startswith(ARM+':'):continue
  key=o.component.entityToken
  if key not in cache:cache[key]=[body_sig(b) for b in o.component.bRepBodies]
  actual[o.fullPathName]={'transform':[round(x,8) for x in o.transform2.asArray()],'visible':o.isLightBulbOn,'component':o.component.name,'bodies':cache[key]}
  old=before['occurrences'].get(o.fullPathName)
  if old!=actual[o.fullPathName]:
   change={k:True for k in actual[o.fullPathName] if old is None or old[k]!=actual[o.fullPathName][k]}
   mismatch.append({'name':o.fullPathName,'changes':change,'old_transform':old['transform'] if old else None,'new_transform':actual[o.fullPathName]['transform']})
 root=[body_sig(b) for b in r.bRepBodies]
 out={'protected_occurrences_before':len(before['occurrences']),'after':len(actual),'unique_components':len(cache),'missing':sorted(set(before['occurrences'])-set(actual)),'root_bodies_equal':root==before['root_bodies'],'mismatch':mismatch}
 (OUT/'protected_comparison.json').write_text(json.dumps(out,ensure_ascii=False,indent=2));(OUT/'protected_after.json').write_text(json.dumps({'occurrences':actual,'root_bodies':root},ensure_ascii=False,indent=2))
 print(json.dumps(out,ensure_ascii=False))
