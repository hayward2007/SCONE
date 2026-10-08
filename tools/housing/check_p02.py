exec(open('/Users/hayward_kim/Developer/SCONE/tools/housing/check_geometry.py',encoding='utf-8').read().split('\ndef run():')[0])
import gzip

def run():
 app,doc,d=guard();r=d.rootComponent;printed=[];refs=[];legs=[];meshes=[];parts=[]
 for o in r.allOccurrences:
  role='printed' if o.fullPathName.startswith(('P01 ','P02 ','P03 ','P04 ')) else 'hardware' if o.fullPathName.startswith('V') else 'leg' if o.fullPathName.startswith('LEG') else None
  if not role:continue
  for i,b in enumerate(o.bRepBodies):
   if not b.isSolid:continue
   name=o.fullPathName+'/'+b.name+'/'+str(i);q=T.copy(b)
   (printed if role=='printed' else refs if role=='hardware' else legs).append((name,q))
   calc=q.meshManager.createMeshCalculator();calc.surfaceTolerance=.005;m=calc.calculate()
   meshes.append({'name':name,'role':role,'bbox':bb(q),'vertices_mm':[v*10 for v in m.nodeCoordinatesAsDouble],'triangles':list(m.nodeIndices)})
   if role=='printed':parts.append({'name':name,'bounds':bb(q),'lumps':q.lumps.count,'volume_cm3':q.volume,'solid_PLA_mass_g':q.volume*1.24,'material':b.material.name})
 report={'printed_parts':parts,'printed_intersections':[],'hardware_intersections':[],'current_pose_leg_intersections':[],'continuous_distal_envelope':[]}
 for i,(name,q) in enumerate(printed):
  for n,b in printed[i+1:]:
   v=intersection_volume(q,b)
   if v is None or v>.01:report['printed_intersections'].append([name,n,v])
  for n,b in refs:
   v=intersection_volume(q,b)
   if v is None or v>.01:report['hardware_intersections'].append([name,n,v])
  for n,b in legs:
   v=intersection_volume(q,b)
   if v is None or v>.01:report['current_pose_leg_intersections'].append([name,n,v])
 # Exact BRep envelope bounds ALL distal spins, including between sampled angles.
 # Front-right, yaw world -90 (inward user +90), hip -90. 2.1 mm radial/axial margin.
 envelope=cylinder((123.2087,-65.14092583,-104),(137.4087,-65.14092583,-104),124.6)
 for name,b in printed+refs:
  v=intersection_volume(envelope,b)
  if v is None or v>.01:report['continuous_distal_envelope'].append([name,v])
 (OUT/'p02_check.json').write_text(json.dumps(report,indent=2,ensure_ascii=True))
 with gzip.open(OUT/'p02_meshes.json.gz','wt',encoding='utf-8') as file:json.dump(meshes,file)
 (OUT/'p02_inventory.json').write_text(json.dumps(snapshot(doc),ensure_ascii=True))
 cam=app.activeViewport.camera;cam.isSmoothTransition=False;cam.cameraType=c.CameraTypes.OrthographicCameraType;cam.eye=P(600,-650,430);cam.target=P(70,0,-15);cam.upVector=vec(0,0,1);cam.isFitView=True;app.activeViewport.camera=cam;app.activeViewport.visualStyle=c.VisualStyles.ShadedVisualStyle;app.activeViewport.refresh();app.activeViewport.saveAsImageFile(str(OUT/'p02_assembly.png'),1800,1200)
 print('P02 CHECK',[(k,len(v)) for k,v in report.items() if isinstance(v,list)])
try:run()
except:(OUT/'p02_check_error.txt').write_text(traceback.format_exc(),encoding='utf-8');print(traceback.format_exc())
