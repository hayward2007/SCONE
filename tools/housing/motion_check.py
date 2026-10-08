exec(open('/Users/hayward_kim/Developer/SCONE/tools/housing/check_geometry.py').read().split('\ndef run():')[0])
import math,time

def rotation(angle,axis,origin):
 m=c.Matrix3D.create();m.setToRotation(math.radians(angle),vec(*axis),P(*origin));return m

def run():
 app,doc,d=guard();root=d.rootComponent
 added=[];baseline=[T.copy(root.bRepBodies.itemByName('본체12'))]
 leg_groups={}
 for o in root.allOccurrences:
  if o.fullPathName.startswith('LEG'):
   top=o.fullPathName.split('+')[0]
   for b in o.bRepBodies:
    if not b.isSolid:continue
    if '+' not in o.fullPathName:continue
    leg_groups.setdefault(top,[]).append((o.fullPathName+'/'+b.name,T.copy(b),'ARC' in o.fullPathName,'FR07' in o.fullPathName))
  elif o.component.attributes.itemByName('SCONE_HOUSING','owned') and not o.fullPathName.startswith(('R01','R02','R03','R04')):
   for b in o.bRepBodies:
    if b.isSolid:added.append((o.fullPathName+'/'+b.name,T.copy(b)))
 report={'method':'Temporary geometry only; 3-axis reconstruction from CAD joint pivots; original and copy joint definitions never edited','sample_units':'degrees; yaw signed outward by side; pitch and spin about world +Y before yaw','tested':0,'baseline_colliding':0,'baseline_free':0,'added_collision_poses':[],'unresolved':[],'source_pose_intersections':[],'families':{},'added_body_count':len(added)}
 # Direct original unsaved pose as an independent, unmodified reference.
 source=next(x for x in app.documents if x.dataFile and x.dataFile.id==json.loads((OUT/'copy_identity.json').read_text())['source_id'])
 sr=f.Design.cast(source.products.itemByProductType('DesignProductType')).rootComponent
 for o in sr.allOccurrences:
  if not o.fullPathName.startswith('LEG'):continue
  for b in o.bRepBodies:
   if not b.isSolid:continue
   temp=T.copy(b)
   for name,a in added:
    vol=intersection_volume(a,temp)
    if vol is None or vol>.01:report['source_pose_intersections'].append([name,o.fullPathName+'/'+b.name,vol])
 # Main operational candidate: steer +/-30, pitch -105..-45, and complete distal revolutions.
 poses=[(yaw,pitch,spin,'operating_candidate') for yaw in [-30,-15,0,15,30] for pitch in [-105,-90,-75,-60,-45] for spin in range(0,360,30)]
 # Extended exploratory limits, deliberately separate from the operating candidate.
 poses += [(yaw,-75,200,'extended_yaw') for yaw in range(-180,180,15)]
 poses += [(15,pitch,200,'extended_pitch') for pitch in range(-180,180,15)]
 for leg,items in leg_groups.items():
  side=1 if '미러' in leg else -1
  hx=-41.8912974452213 if leg.endswith(':5') or leg.endswith(':2') else 208.1087025547787
  for yaw,pitch,spin,family in poses:
   yawm=rotation(side*yaw,(0,0,1),(hx,side*65.14092582896613,39))
   pm=rotation(pitch,(0,1,0),(hx,side*95.14092582896613,18.5))
   sm=rotation(spin,(0,1,0),(hx-122.5,side*138.14092582896532,18.5))
   moving=[]
   for name,body,isarc,isbracket in items:
    b=T.copy(body)
    if isarc:T.transform(b,sm)
    if not isbracket:T.transform(b,pm)
    T.transform(b,yawm);moving.append((name,b))
   basebad=False
   for _,b in moving:
    for floor in baseline:
     vol=intersection_volume(b,floor)
     if vol is None:report['unresolved'].append([leg,yaw,pitch,spin,'baseline Boolean']);basebad=True;break
     if vol>.01:basebad=True;break
    if basebad:break
   report['tested']+=1;report['families'][family]=report['families'].get(family,0)+1
   if basebad:report['baseline_colliding']+=1;continue
   report['baseline_free']+=1;hit=None
   for lname,b in moving:
    for name,a in added:
     vol=intersection_volume(a,b)
     if vol is None or vol>.01:hit={'leg':leg,'yaw':yaw,'pitch':pitch,'spin':spin,'family':family,'housing':name,'leg_body':lname,'intersection_mm3':vol};break
    if hit:break
   if hit:report['added_collision_poses'].append(hit)
   if report['tested']%25==0:(OUT/'motion_progress.json').write_text(json.dumps({'tested':report['tested'],'free':report['baseline_free'],'collisions':len(report['added_collision_poses'])}))
  (OUT/'motion_check.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
 print('MOTION CHECK',report['tested'],report['baseline_free'],len(report['added_collision_poses']),len(report['source_pose_intersections']))
try:run()
except:(OUT/'motion_error.txt').write_text(traceback.format_exc());print(traceback.format_exc())
