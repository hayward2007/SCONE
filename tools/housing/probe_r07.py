exec(open('/Users/hayward_kim/Developer/SCONE/tools/housing/check_r07.py',encoding='utf-8').read().split('\ndef run():')[0])
app,doc,d=guard();r=d.rootComponent
parts={o.name:current(o.component) for o in r.occurrences if o.name.startswith(('R01 Rear','R02 Front','S01 Rear','S02 Front','P04 PLA'))}
res=[]
for hit in json.loads((DEST/'checks.json').read_text())['printed_hits']:
 a,b=hit['a'],hit['b'];q=T.copy(parts[a]);intersect(q,T.copy(parts[b]));res.append(dict(a=a,b=b,bounds=bb(q),volume_mm3=q.volume*1000))
(DEST/'printed_intersection_bounds.json').write_text(json.dumps(res,indent=2));print(res)
