exec(open('tools/housing/rom_mesh.py').read().split('def check():')[0])
all3=json.load(gzip.open(OUT/'r03_meshes.json.gz','rt'))
leg=[Shape(v,(250,0,0)) for v in data if v['name'].startswith('LEG 1:5+')]
arc=[s for s in leg if '+ARC:' in s.name];link=[s for s in leg if '+ARC:' not in s.name]
other=[Shape(v) for v in all3 if v['role']=='leg' and not v['name'].startswith('LEG 1:1+')]
body=[Shape(v) for v in all3 if v['role']!='leg']
ym=matrix(-90,'z',(208.1087025548,-65.140925829,39));pm=matrix(180,'y',(208.1087025548,-95.140925829,18.5));base=ym@pm
for s in link:s.set(ym if '+FR07:' in s.name else base)
report={}
for variant in ['original10','tire20_centered','all20_centered','tire20_outward']:
 variants=[]
 for original in [v for v in data if v['name'].startswith('LEG 1:5+ARC:')]:
  v=dict(original);a=np.array(v['vertices_mm']).reshape(-1,3).copy()
  if variant=='all20_centered' or ('tire20' in variant and v['name'].endswith('/본체2')):
   a[:,1]=(a[:,1]-a[:,1].mean())*2+a[:,1].mean() # replaced below with bbox center to avoid tessellation bias
   orig=np.array(v['vertices_mm']).reshape(-1,3);cy=(orig[:,1].min()+orig[:,1].max())/2;a[:,1]=(orig[:,1]-cy)*2+cy
   if variant=='tire20_outward':a[:,1]-=5
  v['vertices_mm']=a.ravel().tolist();variants.append(Shape(v,(250,0,0)))
 pairs={};angles=[]
 for spin in range(0,360,2):
  sm=matrix(spin,'y',(85.6087025548,-138.140925829,18.5))
  for s in variants:s.set(base@sm)
  hits=[]
  for category,targets in [('self',link),('other',other),('housing',body)]:
   for s in variants:
    for t in targets:
     if s.hits(t):
      k=(category,s.name,t.name);pairs[k]=pairs.get(k,0)+1;hits.append(k)
  if hits:angles.append(spin)
 report[variant]={'pairs':[{'category':k[0],'wheel':k[1],'target':k[2],'angle_count':v} for k,v in pairs.items()],'angles_with_surface_contacts':angles}
 print(variant,len(pairs),flush=True)
(OUT/'r03_wheel_width_screen.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
