exec(open('/Users/hayward_kim/Developer/SCONE/artifacts/housing/20260930_c1_l1/install.py').read().rsplit('def run(_context):',1)[0])
OUT=Path('/Users/hayward_kim/Developer/SCONE/artifacts/housing/20261001_c1_l1_reinforced')
def run(_context):
 a,doc,d=source();assert doc.name=='MARC v4 Body v5 BOX v37';doc.activate()
 (OUT/'source_status.json').write_text(json.dumps({'document':doc.name,'modified_at_start':doc.isModified}),encoding='utf-8')
 baseline=all_sig(d);(OUT/'baseline.json').write_text(json.dumps(baseline,ensure_ascii=False,indent=2),encoding='utf-8')
 assert d.exportManager.execute(d.exportManager.createFusionArchiveExportOptions(str(OUT/'SOURCE_v37_before_reinforcement.f3d')))
 rows=[]
 for o in d.rootComponent.allOccurrences:
  for b in o.bRepBodies:
   if b.name in ('C1 USB cable guide','L1 frame','H2 handle + arm rest'):
    rows.append({'body':b.name,'bounds':bounds(b),'edges':[{'box':bounds(e),'kind':e.geometry.objectType,'length_mm':e.length*10} for e in b.edges]})
 (OUT/'inspection.json').write_text(json.dumps(rows,ensure_ascii=False,indent=2),encoding='utf-8')
 print(json.dumps({'source':doc.name,'protected_occurrences':len(baseline['occurrences']),'bodies':sum(len(o['bodies']) for o in baseline['occurrences'].values()),'warnings':baseline['warnings']},ensure_ascii=False))
