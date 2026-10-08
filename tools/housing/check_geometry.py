exec(open('/Users/hayward_kim/Developer/SCONE/tools/housing/geometry.py',encoding='utf-8').read())
def overlaps(a,b):
 aa=a.boundingBox;bb=b.boundingBox
 return all(getattr(aa.maxPoint,k)>getattr(bb.minPoint,k)+1e-6 and getattr(bb.maxPoint,k)>getattr(aa.minPoint,k)+1e-6 for k in ['x','y','z'])
def intersection_volume(a,b):
 if not overlaps(a,b):return 0.0
 q=T.copy(a)
 if not T.booleanOperation(q,b,f.BooleanTypes.IntersectionBooleanType):return None
 return q.volume*1000 if q.isSolid else 0.0

def run():
 app,doc,d=guard();r=d.rootComponent
 new=[];legs=[];refs=[];errors=[];bounds=[];masses=[]
 for o in r.allOccurrences:
  for b in o.bRepBodies:
   if not b.isSolid:continue
   if o.component.attributes.itemByName('SCONE_HOUSING','owned'):
    role=o.component.attributes.itemByName('SCONE_HOUSING','role')
    category=refs if role and role.value=='reference' else new
   elif o.fullPathName.startswith('LEG'):category=legs
   else:continue
   try:
    temp=T.copy(b)
    category.append((o.fullPathName+'/'+b.name,temp))
    if len(bounds)<4:bounds.append({'name':o.fullPathName,'proxy':bb(b),'temp':bb(temp)})
   except Exception as e:errors.append([o.fullPathName,b.name,str(e)])
 for o in r.occurrences:
  if o.component.attributes.itemByName('SCONE_HOUSING','owned') and not o.name.startswith('R'):
   masses.append({'name':o.name,'mass_kg':sum(b.physicalProperties.mass for b in o.component.bRepBodies),'materials':list(set(b.material.name for b in o.component.bRepBodies))})
 report={'nominal_leg_intersections':[],'internal_intersections':[],'payload_intersections':[],'errors':errors,'proxy_copy_check':bounds,'masses':masses}
 for name,a in new:
  for lname,b in legs:
   vol=intersection_volume(a,b)
   if vol is None or vol>.01:report['nominal_leg_intersections'].append([name,lname,vol])
 for i,(name,a) in enumerate(new):
  for lname,b in new[i+1:]:
   vol=intersection_volume(a,b)
   if vol is None or vol>.01:report['internal_intersections'].append([name,lname,vol])
  for lname,b in refs:
   vol=intersection_volume(a,b)
   if vol is None or vol>.01:report['payload_intersections'].append([name,lname,vol])
 # Existing battery cells are preserved and checked separately despite being hidden.
 cells=r.occurrences.itemByName('21700 CELLS:1')
 report['battery_intersections']=[]
 for name,a in new:
  for b in cells.bRepBodies:
   vol=intersection_volume(a,T.copy(b))
   if vol is None or vol>.01:report['battery_intersections'].append([name,b.name,vol])
 report['joint_health']=[{'component':cp.name,'name':j.name,'health':safe(j,'healthState'),'message':safe(j,'errorOrWarningMessage')} for cp in d.allComponents for j in list(cp.joints)+list(cp.asBuiltJoints)]
 (OUT/'geometry_check.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
 (OUT/'current_inventory.json').write_text(json.dumps(snapshot(doc),ensure_ascii=False,indent=2))
 source=next(x for x in app.documents if x.dataFile and x.dataFile.id==json.loads((OUT/'copy_identity.json').read_text())['source_id'])
 (OUT/'source_latest.json').write_text(json.dumps(snapshot(source),ensure_ascii=False,indent=2))
 cam=app.activeViewport.camera;cam.isSmoothTransition=False;cam.cameraType=c.CameraTypes.OrthographicCameraType;cam.eye=P(620,-640,470);cam.target=P(60,0,-20);cam.upVector=vec(0,0,1);cam.isFitView=True;app.activeViewport.camera=cam
 app.activeViewport.visualStyle=c.VisualStyles.ShadedVisualStyle
 app.activeViewport.refresh();app.activeViewport.saveAsImageFile(str(OUT/'housing_assembly.png'),1800,1200)
 print('CHECK COMPLETE',len(report['nominal_leg_intersections']),len(report['internal_intersections']),len(report['payload_intersections']),len(report['battery_intersections']))
try:run()
except:(OUT/'check_error.txt').write_text(traceback.format_exc());print(traceback.format_exc())
