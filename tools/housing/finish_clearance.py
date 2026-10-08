exec(open('/Users/hayward_kim/Developer/SCONE/tools/housing/geometry.py').read())
def run():
 app,doc,d=guard();r=d.rootComponent
 # Narrow only the excess outside edge of the PCB carrier.
 cp=r.occurrences.itemByName('H04 Raised PCB carrier:1').component;b=next(b for b in cp.bRepBodies if b.name.startswith('PCB'))
 cut_native(cp,b,[box(181,185,-50,50,47,50),box(108,185,45.5,49,47,50),box(108,185,-49,-45.5,47,50)],'H04 perimeter clearance')
 cp=r.occurrences.itemByName('H05 Battery retention and lid anchors:1').component
 for b in list(cp.bRepBodies):
  if b.name.startswith('Side-access'):
   cut_native(cp,b,[box(0,180,34,40,37,41),box(0,180,-40,-34,37,41)],'H05 shoulder relief '+b.name[-3:])
 # Raised immutable reference envelope. No occurrence transform that can reset during recompute.
 o=r.occurrences.itemByName('R01 Orin Nano RESERVED 79x100x50:1');o.deleteMe()
 cp=compnew(r,'R01 Orin Nano RESERVED 79x100x50',True);persist(cp,[('Orin and fan provisional envelope',box(29,108,-50,50,41,91))],(48,137,121),appearance='Reference teal');cp.attributes.add('SCONE_HOUSING','bounds_mm',json.dumps([29,108,-50,50,41,91]));r.occurrences.itemByName(cp.name+':1').isLightBulbOn=False
 # Raise the four provisional board pads to the new base plane.
 cp=r.occurrences.itemByName('H03 Removable Jetson carrier:1').component
 added=[]
 for x in [40,106]:
  for y in [-47,47]:added.append(('3 mm removable board spacer',cut(cylinder((x,y,38),(x,y,41),3.5),cylinder((x,y,37),(x,y,42),1.6))))
 persist(cp,added,(81,89,98),'PrismMaterial-231','Anodized aluminium')
 # Apply materials to both source bodies of base features and result bodies; later recompute must retain them.
 for o in r.occurrences:
  cp=o.component
  if not cp.attributes.itemByName('SCONE_HOUSING','owned'):continue
  if cp.name.startswith(('H02','H03','H04')):matid='PrismMaterial-231';color=(81,89,98);name='Anodized aluminium'
  elif cp.name.startswith('H01'):matid='PrismMaterial-417';color=(222,174,45);name='Warm ochre PA12'
  elif cp.name.startswith('R06 Glass'):matid='PrismMaterial-022';color=(31,84,110);name='Lens blue'
  elif cp.name.startswith('R06 Stereo'):matid='PrismMaterial-022';color=(31,101,80);name='Circuit board green'
  elif cp.name.startswith('R0'):matid='PrismMaterial-022';color=(25,29,34);name='Sensor graphite'
  else:matid='PrismMaterial-417';color=(44,49,55);name='Graphite PA12'
  for base in cp.features.baseFeatures:
   base.startEdit()
   for b in base.bodies:finish_appearance(app,d,b,color,name,matid)
   base.finishEdit()
  for b in cp.bRepBodies:finish_appearance(app,d,b,color,name,matid)
 # Recess adapter screws below the sensor base so they do not occupy the sensor volume.
 cp=r.occurrences.itemByName('H06 Sensor mounts:1').component
 adapter=next(b for b in cp.bRepBodies if b.name.startswith('Lidar'))
 cut_native(cp,adapter,[cylinder((x,y,109.8),(x,y,112.5),2.9) for x in [38,68] for y in [-15,15]],'H06 recessed adapter screw heads')
 # Match roof fasteners with visible removable screws.
 cp=compnew(r,'H08 Roof fasteners')
 parts=[]
 for x,y in [(38,-15),(38,15),(68,-15),(68,15),(158,-40),(158,40)]:
  z=110 if x in [38,68] else 104
  parts.append(('M3 low profile head - schematic',cylinder((x,y,z),(x,y,z+1.5),2.7)))
 persist(cp,parts,(35,40,46),'PrismMaterial-231','Black fastener')
 for cp in d.allComponents:cp.isSketchFolderLightBulbOn=False;cp.isConstructionFolderLightBulbOn=False;cp.isJointsFolderLightBulbOn=False
 doc.save('Housing A01 final packaging clearances and persistent materials')
 print('CLEARANCES REFINED AND MATERIALS STORED')
try:run()
except:(OUT/'finish_error.txt').write_text(traceback.format_exc());print(traceback.format_exc())
