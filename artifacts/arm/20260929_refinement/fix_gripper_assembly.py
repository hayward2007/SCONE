exec(open('/Users/hayward_kim/Developer/SCONE/artifacts/arm/20260929_refinement/refine_gripper.py').read().split('def run(_context):')[0])
def run(_context):
 app=c.Application.get();d=f.Design.cast(app.activeProduct);assert app.activeDocument.name.startswith('MARC v4 Arm R1 Folded')
 co=next(x for x in d.allComponents if x.name.startswith('ARM5 '));b=next(b for b in co.bRepBodies if b.name.startswith('A5 gripper'))
 adds=[]
 for y in [-6,6]:
  for z in [-12,12]:
   q=cyl((0,y,z),(2.61,y,z),2.72);join(q,cyl((2.5,y,z),(6,y,z),1.52));adds.append(q)
 for sign in [-1,1]:
  ya,yb=sorted([sign*16.8,sign*24.8]);adds.append(box(4,36,ya,yb,-24,-10))
 b=native_op(co,b,adds,'R1 serviceable J6 side mounts and closed unused rear holes',f.FeatureOperations.JoinFeatureOperation)
 b=fillet(co,b,lambda a,b:abs(a[0]-b[0])<.01 and (abs(a[0]-4)<.02 or abs(a[0]-36)<.02) and abs(a[2]+10)<.02 and abs(b[2]+10)<.02 and abs(a[1]-b[1])>3,2,'R1 J6 side cheek corners R2')
 cuts=[box(5.6,34.85,-17.3,17.3,-19.6,29)]
 for sign in [-1,1]:
  for x in [9.25,31.25]:
   cuts.extend([cyl((x,sign*16.8,-16),(x,sign*25,-16),1.5),cyl((x,sign*16.8,-16),(x,sign*19.2,-16),3),cyl((x,sign*22.2,-16),(x,sign*25,-16),2.7)])
 b=native_op(co,b,cuts,'R1 J6 side-access M2.5 counterbores D5.4 depth2.6',f.FeatureOperations.CutFeatureOperation)
 b.name='A5 R1 gripper cradle - wrist 8x M2; J6 side 4x M2.5x8'
 print('J6_SIDE_MOUNTS_DONE',b.volume,b.lumps.count)
 (OUT/'assembly_revision.json').write_text(json.dumps({'J6_mounting':'4 side M2.5x8 socket screws at x=9.25/31.25, z=-16, y=+/-24.8; D5.4 x2.6 CB; 3.0mm web to relief','assembly_order':['mount A5 empty cradle to J5 horn from the open +X side','lower J6 from +Z into cradle, install 4 side screws','install moving jaw on J6 horn, then TPU pads and camera cover'],'obsolete_rear_J6_holes':'filled; no inaccessible rear fasteners'},indent=2))
