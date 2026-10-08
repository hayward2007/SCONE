exec(open('/Users/hayward_kim/Developer/SCONE/tools/housing/build_r05.py',encoding='utf-8').read().split('\ndef run():')[0])
import gzip

def run():
 app,doc,d=guard();r=d.rootComponent
 parts=[(o.name,current(o.component)) for o in r.occurrences if o.name.startswith(('R01 Rear','R02 Front','S01 Rear','S02 Front','P04 PLA'))]
 hw=[(o.fullPathName+'/'+b.name,b) for o in r.allOccurrences if o.fullPathName.startswith('V') for b in o.bRepBodies if b.isSolid and b.isLightBulbOn]
 saved={o['name']:o for o in json.loads((DEST/'user_pose_before.json').read_text())['occurrences']}
 leg=[]
 for o in r.allOccurrences:
  if not o.fullPathName.startswith('LEG'):continue
  m=c.Matrix3D.create();m.setWithArray(saved[o.fullPathName]['transform'])
  for b in o.bRepBodies:
   if b.isSolid and b.isLightBulbOn:
    q=T.copy(b.nativeObject if b.nativeObject else b);T.transform(q,m);leg.append((o.fullPathName+'/'+b.name,q))
 report={'pose_source':'saved user_pose_before.json transforms applied explicitly to current component bodies'}
 def test(items,targets):
  hits=[]
  for n,q in items:
   for tn,t in targets:
    v=intersection_volume(T.copy(q),T.copy(t))
    if v is None or v>.01:
     qb=T.copy(q);intersect(qb,T.copy(t));hits.append(dict(a=n,b=tn,overlap_mm3=v,bounds=bb(qb)))
  return hits
 report['current_user_pose_hits']=test(leg,parts)
 (DEST/'check_progress.json').write_text(json.dumps(report,ensure_ascii=True))
 report['hardware_hits']=test(hw,parts)
 report['printed_hits']=sum([test([p],parts[i+1:]) for i,p in enumerate(parts)],[])
 # Existing rear wheel axis follows the actual user pose, including their slight hip angle.
 arc=next(o for o in r.allOccurrences if o.fullPathName=='LEG 1:5+ARC:1')
 env=cylinder((0,0,-2.1),(0,0,12.1),124.6);em=c.Matrix3D.create();em.setWithArray(saved[arc.fullPathName]['transform']);T.transform(env,em)
 report['user_pose_continuous_spin_hits']=test([('actual user rear wheel 360 envelope +2.1mm',env)],parts+hw)
 mirror=T.copy(env);mt=c.Matrix3D.create();mt.setWithArray([1,0,0,0,0,-1,0,0,0,0,1,0,0,0,0,1]);T.transform(mirror,mt)
 report['mirrored_user_pose_continuous_spin_hits']=test([('opposite rear wheel 360 envelope +2.1mm',mirror)],parts+hw)
 # Preserve the previously requested front inward90 / hip180 body clearance.
 ce=[('front '+str(side),cylinder((113.2087026,side*187.640925829,18.5),(142.4087026,side*187.640925829,18.5),124.6)) for side in [-1,1]]
 report['front_90_180_continuous_spin_hits']=test(ce,parts+hw)
 cv=next(b for n,b in parts if n.startswith('S02'))
 cb=next(b for n,b in hw if n.startswith('V02'))
 # Assembly: cover removed from robot. Raise module from open bottom at x shifted -66,
 # then slide forward 66mm, then fit screws from the rear. Final camera is 8mm lower than R03.
 entry=[]
 for dz in range(-80,1,2):entry += test([('lift at dx -66 dz '+str(dz),move(T.copy(cb),x=-66,z=dz))],[('cover',cv)])
 slide=[]
 for dx in range(-66,1):slide += test([('slide dx '+str(dx),move(T.copy(cb),x=dx))],[('cover',cv)])
 drivers=[]
 for y in [-10.5,10.5]:
  for z in [71.7,85.15]:drivers+=test([('driver '+str([y,z]),cylinder((100,y,z),(165.8,y,z),2.0))],[('cover',cv)])
 (DEST/'check_progress.json').write_text(json.dumps(report,ensure_ascii=True))
 report['camera_assembly']={'cover_removed':True,'entry_lift_step_mm':2,'entry_hits':entry,'forward_slide_step_mm':1,'forward_slide_hits':slide,'driver_diameter_mm':4,'driver_shaft_hits':drivers,'camera_shift_z_mm':-8}
 report['camera_assembly']['ffc_reserve_cover_overlap_mm3']=intersection_volume(box(145,176,-18,18,94,100.2),T.copy(cv))
 report['part_topology']=[dict(name=n,lumps=b.lumps.count,volume_cm3=b.volume,bounds=bb(b)) for n,b in parts]
 (DEST/'checks.json').write_text(json.dumps(report,ensure_ascii=True,indent=2))
 meshes=[]
 for n,b in parts+hw+leg:
  calc=b.meshManager.createMeshCalculator();calc.surfaceTolerance=.003;m=calc.calculate();meshes.append(dict(name=n,vertices_mm=[v*10 for v in m.nodeCoordinatesAsDouble],triangles=list(m.nodeIndices),bounds=bb(b),volume_cm3=b.volume,lumps=b.lumps.count))
 with gzip.open(DEST/'validation_meshes.json.gz','wt',encoding='utf-8') as h:json.dump(meshes,h)
 print('R05 CHECK',[(k,len(v) if isinstance(v,list) else {a:len(b) for a,b in v.items() if isinstance(b,list)} if isinstance(v,dict) else v) for k,v in report.items()])
try:run()
except:(DEST/'check_error.txt').write_text(traceback.format_exc(),encoding='utf-8');print(traceback.format_exc())
