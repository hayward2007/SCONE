exec(open('/Users/hayward_kim/Developer/SCONE/tools/housing/build_r05.py',encoding='utf-8').read().split('\ndef run():')[0])
DEST=OUT/'R08_DELIVERY'
app,doc,d=guard();assert doc.name.startswith('MARC Housing PLA R08');r=d.rootComponent
cp=r.occurrences.itemByName('R02 Front smooth PLA chassis:1').component;q=T.copy(current(cp));before=q.volume
routes=json.loads((DEST/'wiring_clearance.json').read_text())['routes']
for row in routes:
 if row['name'].startswith('front TTL'):
  for a,b in zip(row['centerline_mm'],row['centerline_mm'][1:]):cut(q,cylinder(a,b,2.15))
assert q.lumps.count==1;replace_final(cp,q,'R08 front chassis with non-tangent 4.3mm wire passages');pla(current(cp),(222,174,45));doc.save('R08 remove zero-width tangent edges at cable passage for manifold print mesh')
(DEST/'print_topology_fix.json').write_text(json.dumps(dict(part='R02',removed_cm3=before-q.volume,cable_channel_diameter_mm=4.3),indent=2));print('PRINT TOPOLOGY FIXED')
