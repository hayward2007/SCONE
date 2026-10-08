import adsk.core as c, adsk.fusion as f, json, math
from pathlib import Path
OUT=Path('/Users/hayward_kim/Developer/SCONE/artifacts/housing/20260927_body42')
def pt(x,y,z):return c.Point3D.create(x/10,y/10,z/10)
def box(t,x0,x1,y0,y1,z0,z1):return t.createBox(c.OrientedBoundingBox3D.create(pt((x0+x1)/2,(y0+y1)/2,(z0+z1)/2),c.Vector3D.create(1,0,0),c.Vector3D.create(0,1,0),(x1-x0)/10,(y1-y0)/10,(z1-z0)/10))
def cyl(t,x,y,z0,z1,rad):return t.createCylinderOrCone(pt(x,y,z0),rad/10,pt(x,y,z1),rad/10)
def op(t,a,b,kind):assert t.booleanOperation(a,b,kind),'Boolean failed'
def run(_context: str):
 app=c.Application.get();doc=app.activeDocument;assert doc.name.startswith('SCONE Body42')
 d=f.Design.cast(app.activeProduct);r=d.rootComponent;t=f.TemporaryBRepManager.get();source=r.bRepBodies.item(3);body=t.copy(source)
 ref=r.bRepBodies.item(0);holes=[]
 for fc in ref.faces:
  g=fc.geometry
  if isinstance(g,c.Cylinder) and abs(g.radius*10-1.405)<.001:holes.append([g.origin.x*10,g.origin.y*10])
 assert len(holes)==16
 report={'source_volume_cm3':source.volume,'holes_mm':holes,'stages':[]}
 for x in [-41.8912973869,208.1087026131]:
  for sign in [-1,1]:
   # Motor body occupies the lower wall. Preserve the reference z=3.5 seating plane.
   ys=sorted([sign*26.240926,sign*90])
   op(t,body,box(t,x-18.1,x+18.1,*ys,3.5,45.0),f.BooleanTypes.DifferenceBooleanType)
   ys=sorted([sign*21.5,sign*40.140926])
   plate=box(t,x-21,x+21,*ys,-1.5,3.5)
   # Clearance for the lower motor case, while retaining four screw lands.
   yn=sorted([sign*33.25,sign*44])
   op(t,plate,box(t,x-11.05,x+11.05,*yn,-2,3.6),f.BooleanTypes.DifferenceBooleanType)
   op(t,body,plate,f.BooleanTypes.UnionBooleanType)
   # Two short 3 mm ribs support the inner ledge; keep all screw axes accessible.
   for dx in [-18.5,18.5]:
    yr=sorted([sign*24,sign*36.0])
    op(t,body,box(t,x+dx-1.5,x+dx+1.5,*yr,-9.5,-1.4),f.BooleanTypes.UnionBooleanType)
 report['stages'].append({'stage':'seats_and_pockets','volume':body.volume})
 # Ensure the actual motor underside does not intersect the seats.
 for o in r.allOccurrences:
  if o.fullPathName.startswith('LEG') and '+' not in o.fullPathName and o.isVisible:
   for b in o.bRepBodies:op(t,body,t.copy(b),f.BooleanTypes.DifferenceBooleanType)
 for x,y in holes:
  op(t,body,cyl(t,x,y,-1.6,3.6,1.405),f.BooleanTypes.DifferenceBooleanType)
  op(t,body,cyl(t,x,y,-1.51,0,2.5),f.BooleanTypes.DifferenceBooleanType)
 # Driver access through the deep lower hull, coaxial with all 16 underside screws.
 for x,y in holes:op(t,body,cyl(t,x,y,-55,-1.51,3.2),f.BooleanTypes.DifferenceBooleanType)
 report['stages'].append({'stage':'holes_counterbores_driver_access','volume':body.volume})
 assert body.isSolid and body.lumps.count==1
 base=r.features.baseFeatures.add();base.name='Body42 - exact MX28 seats and 16 M2.5 counterbores';base.startEdit();new=r.bRepBodies.add(body,base);base.finishEdit()
 new=base.bodies.item(0);new.name='Body42 MX28 M2.5 - 5mm seat 3.5mm web';new.isLightBulbOn=True;source.isLightBulbOn=False
 new.attributes.add('Body42','dimensions','16x dia2.81; counterbore dia5.0 x 1.5; seat 5.0; web 3.5 mm')
 for comp in d.allComponents:
  comp.isJointsFolderLightBulbOn=False
 report['final_volume_cm3']=new.volume;report['faces']=new.faces.count;report['lumps']=new.lumps.count
 (OUT/'build_report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report))
 print('created',new.name)
