exec(open('/Users/hayward_kim/Developer/SCONE/artifacts/electronics/20260930_vertical_carrier/carrier_geometry.py').read())
def run(_context):
 a=c.Application.get();doc=next(x for x in a.documents if x.name=='MARC v4 Body v5 BOX v19')
 assert not doc.isModified,'Source has changed while the carrier was prepared; inspect before installing.'
 doc.activate();d=f.Design.cast(a.activeProduct);r=d.rootComponent
 assert not any(o.component.name.startswith(PREFIX) for o in r.occurrences)
 candidate=json.loads((OUT/'candidate_check.json').read_text(encoding='utf-8'));service=json.loads((OUT/'service_check.json').read_text(encoding='utf-8'))
 assert not candidate['existing_collisions'] and not candidate['internal_collisions']
 assert not service['cassette_lift_collisions'] and not service['base_driver_collisions'] and not service['envelope_collisions']
 assert all(n==1 for _,n in service['part_lumps'])
 parts=generate(a)
 holder=next(b for o in r.allOccurrences for b in o.bRepBodies if b.name=='M2 holder')
 gold=d.appearances.addByCopy(holder.appearance,'E9 carrier warm yellow')
 gold.appearanceProperties.itemById('opaque_albedo').value=c.Color.create(220,168,45,255)
 green=d.appearances.addByCopy(holder.appearance,'E9 reference PCB green')
 green.appearanceProperties.itemById('opaque_albedo').value=c.Color.create(22,107,77,255)
 dark=d.appearances.addByCopy(holder.appearance,'E9 official electronics dark')
 dark.appearanceProperties.itemById('opaque_albedo').value=c.Color.create(46,51,58,255)
 parent=r.occurrences.addNewComponent(c.Matrix3D.create());parent.component.name=PREFIX+' - additive only';parent.isGroundToParent=True
 groups={};added=[]
 for n,q,kind in parts:
  group=n.split(' ')[0] if kind=='print' else 'R1 official ROBOTIS U2D2 + PHB' if kind=='official' else 'R2 power PCB envelope 70x50'
  if group not in groups:
   o=parent.component.occurrences.addNewComponent(c.Matrix3D.create());o.component.name=group;o.isGroundToParent=True;groups[group]=o.component
  co=groups[group];bf=co.features.baseFeatures.add();bf.name=n;bf.startEdit();b=co.bRepBodies.add(q,bf);b.name=n;bf.finishEdit();b=bf.bodies.item(0)
  b.appearance=gold if kind=='print' else green if kind=='pcb' or 'PART_1_1' in n else dark
  assert b.isSolid and b.lumps.count==1,n
  added.append({'name':n,'component':co.name,'kind':kind,'box':bounds(b),'volume_mm3':b.volume*1000})
 parent.component.description='Additive rear electronics carrier. Existing M6R 2.7 mm holes at x=-70/-50, y=+/-40. 70x50 power PCB, 62x42 nominal hole pattern. U2D2 USB-C and PHB from official ROBOTIS STEP. No existing part geometry or placement changed.'
 (OUT/'installed_parts.json').write_text(json.dumps(added,ensure_ascii=False,indent=2),encoding='utf-8')
 print('INSTALLED',parent.component.name,len(added),'bodies; original parts were not selected for modification')
