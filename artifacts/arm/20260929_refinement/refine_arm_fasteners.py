exec(open('/Users/hayward_kim/Developer/SCONE/artifacts/arm/20260929_refinement/refine_gripper.py').read().split('def run(_context):')[0])
def run(_context):
 app=c.Application.get();d=f.Design.cast(app.activeProduct);assert app.activeDocument.name.startswith('MARC v4 Arm R1 Folded')
 report=[]
 for idx in [0,1,2,3,4]:
  co=next(co for co in d.allComponents if co.name.startswith('ARM'+str(idx)+' '));b=next(b for b in co.bRepBodies if b.name.startswith('A'+str(idx)+' '))
  if idx in [2,3]:
   b=fillet(co,b,lambda a,b:abs(abs(a[1])-23)<.02 and abs(a[1]-b[1])<.02 and abs(a[0]-b[0])>20,.7,'R1 A'+str(idx)+' softened link rails R0.7')
  adds=[];cuts=[]
  if idx==0:
   for x in [220,240]:
    for y in [-35,35]:
     adds.append(cyl((x,y,157.3),(x,y,159.6),5))
     cuts.extend([cyl((x,y,153.4),(x,y,159.8),1.7),cyl((x,y,156.4),(x,y,159.8),3.1)])
   info='4x M3 D6.2 counterbores, 3.2 deep, residual2.9'
  elif idx==1:
   for sign in [-1,1]:
    for y in [-6,6]:
     adds.append(cyl((sign*17.4,y,13.25),(sign*20.15,y,13.25),3.8))
     cuts.extend([cyl((sign*14.5,y,13.25),(sign*20.3,y,13.25),1.5),cyl((sign*17.55,y,13.25),(sign*20.3,y,13.25),2.7)])
   info='4x M2.5 D5.4 counterbores, 2.6 deep, residual2.9'
  elif idx in [2,3]:
   for sign in [-1,1]:
    for z in [-11,11]:
     adds.append(cyl((58,sign*22.5,z),(58,sign*25,z),3.8))
     cuts.extend([cyl((58,sign*16.9,z),(58,sign*25.2,z),1.5),cyl((58,sign*22.4,z),(58,sign*25.2,z),2.7)])
   info='4x M2.5 D5.4 counterbores, 2.6 deep, residual3.2 to existing relief'
  else:
   for y in [-29.5,-12.5,4.5]:
    for z in [-15,15]:cuts.append(cyl((23.9,y,z),(26.6,y,z),2.7))
   info='6x M2.5 D5.4 counterbores deepened to2.6, residual3.4'
  if adds:b=native_op(co,b,adds,'R1 A'+str(idx)+' reinforced screw seats',f.FeatureOperations.JoinFeatureOperation)
  b=native_op(co,b,cuts,'R1 A'+str(idx)+' recessed socket screw heads',f.FeatureOperations.CutFeatureOperation)
  report.append({'part':'A'+str(idx),'fastening':info,'volume_cm3':b.volume})
 co5=next(co for co in d.allComponents if co.name.startswith('ARM5 '));co6=next(co for co in d.allComponents if co.name.startswith('ARM6 '))
 b5=next(b for b in co5.bRepBodies if b.name.startswith('A5 gripper'));b6=next(b for b in co6.bRepBodies if b.name.startswith('A6 moving jaw ('))
 b5=fillet(co5,b5,lambda a,b:abs(a[2]-22)<.02 and abs(b[2]-22)<.02 and abs(abs(a[1])-17.5)<.02 and abs(a[1]-b[1])<.02,3,'R1 A5 radiused rear bracket corners R3')
 b6=fillet(co6,b6,lambda a,b:abs(a[2]+6)<.02 and abs(b[2]+6)<.02 and abs(a[0]-b[0])>25,.65,'R1 A6 softened finger edges R0.65')
 for co in [co5,co6]:
  old=next(b for b in co.bRepBodies if b.name.endswith('pad (TPU)'))
  ft=co.features.removeFeatures.add(old);ft.name='R1 retire original glued pad'
 pod=next(b for b in co5.bRepBodies if b.name.startswith('A5 R1 camera pod'))
 # Clear the unchanged placeholder PCB at the four fastener columns.
 qs=[cyl((39.1,y,z),(43.8,y,z),3.2) for y in [-14,14] for z in [-44,-16]]
 pod=native_op(co5,pod,qs,'R1 camera PCB clearance at screw columns',f.FeatureOperations.CutFeatureOperation)
 (OUT/'fastener_build.json').write_text(json.dumps(report,indent=2));print('FASTENERS_AND_EDGES_COMPLETE',report)
