exec(open('/Users/hayward_kim/Developer/SCONE/artifacts/electronics/20260930_jetson_carrier/geometry.py').read())
def run(_context):
 a=c.Application.get();doc=next(x for x in a.documents if x.name=='MARC v4 Body v5 BOX v21');assert not doc.isModified,'Source changed since v21 baseline'
 doc.activate();d=f.Design.cast(doc.products.itemByProductType('DesignProductType'));r=d.rootComponent
 chk=json.loads((OUT/'candidate_check.json').read_text(encoding='utf-8'));svc=json.loads((OUT/'service_check.json').read_text(encoding='utf-8'))
 assert not chk['existing_collisions'] and not chk['internal_collisions']
 assert not svc['lift_collisions'] and not svc['driver_collisions'] and not svc['envelope_collisions']
 assert all(n==1 for _,n in svc['part_lumps'])
 assert not any(o.component.name.startswith(PREFIX) for o in r.occurrences)
 parts=generate(a);holder=next(b for o in r.allOccurrences for b in o.bRepBodies if b.name=='M2 holder')
 colors={}
 for key,rgb in [('print',(224,169,36)),('pcb',(20,103,67)),('dark',(53,59,65))]:
  ap=d.appearances.addByCopy(holder.appearance,'E10 '+key);ap.appearanceProperties.itemById('opaque_albedo').value=c.Color.create(*rgb,255);colors[key]=ap
 parent=r.occurrences.addNewComponent(c.Matrix3D.create());parent.component.name=PREFIX;parent.isGroundToParent=True
 groups={};added=[]
 for n,q,kind in parts:
  group=n.split(' ')[0] if kind=='print' else 'R1 PHB power terminals upward' if n.startswith('ROBOTIS PHB') else 'R2 U2D2 communication ports upward' if n.startswith('ROBOTIS U2D2') else 'R3 Power PCB 70x50 mechanical reference'
  if group not in groups:
   occ=parent.component.occurrences.addNewComponent(c.Matrix3D.create());occ.component.name=group;occ.isGroundToParent=True;groups[group]=occ.component
  co=groups[group];bf=co.features.baseFeatures.add();bf.name=n;bf.startEdit();b=co.bRepBodies.add(q,bf);b.name=n;bf.finishEdit();b=bf.bodies.item(0);b.name=n
  b.appearance=colors['print' if kind=='print' else 'pcb' if kind=='pcb' or 'PART_1_1' in n else 'dark']
  assert b.isSolid and b.lumps.count==1,n
  added.append({'name':n,'component':group,'kind':kind,'box':bounds(b),'volume_mm3':b.volume*1000})
 old=next(o for o in r.occurrences if o.component.name.startswith('E9 Rear vertical'))
 rem=r.features.removeFeatures.add(old);rem.name='E10 retire superseded rear PCB references - preserve feature history'
 parent.component.description='Front MX28 upper M2.5x8 x4 and existing Jetson upper M3x12 x2. Removable PCB cassette with 2x M2.5x8. U2D2 ports up; PHB power terminals up. 70x50 custom power PCB envelope. Existing source geometry untouched.'
 (OUT/'installed_parts.json').write_text(json.dumps(added,ensure_ascii=False,indent=2),encoding='utf-8')
 print('Installed E10 with '+str(len(added))+' bodies; superseded E9 removed at the timeline end')
