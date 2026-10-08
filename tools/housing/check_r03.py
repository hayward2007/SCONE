exec(open('/Users/hayward_kim/Developer/SCONE/tools/housing/probe_clearance.py',encoding='utf-8').read().split('\ndef run():')[0])
import gzip

def run():
 app,doc,d=guard();r=d.rootComponent;printed=[];refs=[];legs=[];parts=[];meshes=[]
 for o in r.allOccurrences:
  role='printed' if o.fullPathName.startswith(('R01 Rear smooth','R02 Front smooth','R03 Smooth','P04 PLA')) else 'hardware' if o.fullPathName.startswith('V') else 'leg' if o.fullPathName.startswith('LEG') else None
  if not role:continue
  for i,b in enumerate(o.bRepBodies):
   if not b.isSolid or not b.isLightBulbOn:continue
   name=o.fullPathName+'/'+b.name+'/'+str(i);q=T.copy(b)
   (printed if role=='printed' else refs if role=='hardware' else legs).append((name,q))
   calc=q.meshManager.createMeshCalculator();calc.surfaceTolerance=.005;m=calc.calculate();meshes.append({'name':name,'role':role,'vertices_mm':[v*10 for v in m.nodeCoordinatesAsDouble],'triangles':list(m.nodeIndices)})
   if role=='printed':parts.append({'name':name,'bounds':bb(q),'lumps':q.lumps.count,'volume_cm3':q.volume,'solid_PLA_mass_g':q.volume*1.24})
 report={'parts':parts,'printed_hits':[],'hardware_hits':[],'default_leg_hits':[],'critical_wheel_envelope_20mm':[],'critical_front_leg_hits':[]}
 for i,(name,q) in enumerate(printed):
  for key,targets in [('printed_hits',printed[i+1:]),('hardware_hits',refs),('default_leg_hits',legs)]:
   for n,b in targets:
    v=intersection_volume(q,b)
    if v is None or v>.01:report[key].append([name,n,v])
 for side in [-1,1]:
  envelope=cylinder((113.2087026,side*187.64092583,18.5),(142.4087026,side*187.64092583,18.5),124.6)
  for name,b in printed+refs:
   v=intersection_volume(envelope,b)
   if v is None or v>.01:report['critical_wheel_envelope_20mm'].append([side,name,v])
 # Reconstruct a zero-pose front-right leg from the preserved rear occurrence, not a user display pose.
 posed=[]
 for o in r.allOccurrences:
  if not o.fullPathName.startswith('LEG 1:5+'):continue
  for b in o.bRepBodies:
   if not b.isSolid or not b.isLightBulbOn:continue
   q=T.copy(b);m=c.Matrix3D.create();m.translation=vec(25,0,0);T.transform(q,m)
   if 'FR07' not in o.fullPathName:rotate(q,180,(0,1,0),(208.1087025548,-95.140925829,18.5))
   rotate(q,-90,(0,0,1),(208.1087025548,-65.140925829,39));posed.append((o.fullPathName+'/'+b.name,q))
 for name,q in posed:
  if 'ARC' in name:continue
  for n,b in printed+refs:
   v=intersection_volume(q,b)
   if v is None or v>.01:report['critical_front_leg_hits'].append([name,n,v])
 report['critical_components']=[{'name':n,'bounds':bb(b)} for n,b in posed]
 (OUT/'r03_check.json').write_text(json.dumps(report,ensure_ascii=True,indent=2))
 with gzip.open(OUT/'r03_meshes.json.gz','wt',encoding='utf-8') as file:json.dump(meshes,file)
 (OUT/'r03_inventory.json').write_text(json.dumps(snapshot(doc),ensure_ascii=True))
 for doc2 in app.documents:
  if doc2.dataFile and doc2.dataFile.id=='urn:adsk.wipprod:dm.lineage:VUu_hd0ATYKQ4csy_mW31g':(OUT/'source_after_R03.json').write_text(json.dumps(snapshot(doc2),ensure_ascii=True))
 cam=app.activeViewport.camera;cam.isSmoothTransition=False;cam.cameraType=c.CameraTypes.OrthographicCameraType;cam.eye=P(490,-590,340);cam.target=P(70,0,20);cam.upVector=vec(0,0,1);cam.isFitView=True;app.activeViewport.camera=cam;app.activeViewport.refresh();app.activeViewport.saveAsImageFile(str(OUT/'r03_assembly.png'),1800,1200)
 print('R03 CHECK',[(k,len(v)) for k,v in report.items()])
try:run()
except:(OUT/'r03_check_error.txt').write_text(traceback.format_exc(),encoding='utf-8');print(traceback.format_exc())
