exec(open('/Users/hayward_kim/Developer/SCONE/tools/housing/geometry.py').read())
def run():
 app,doc,d=guard();root=d.rootComponent
 if not doc.attributes.itemByName('SCONE_HOUSING','revision_P02'):
  doc.saveAs('MARC Housing P02 PLA',doc.dataFile.parentFolder,'PLA printable housing, source and A01 checkpoint preserved','')
  doc.attributes.add('SCONE_HOUSING','revision_P02','1')
 records=[]
 for name,path in [('V01 NVIDIA official mechanical model','references/orin_mechanical.stp'),('V02 Waveshare official camera','references/camera_3d/IMX219-83-Stereo-Camera-3D-Drawing/0619.stp')]:
  existing=root.occurrences.itemByName(name+':1')
  if existing:comp=existing.component
  else:
   comp=compnew(root,name,True);comp.attributes.add('SCONE_HOUSING','owned','P02')
   opt=app.importManager.createSTEPImportOptions(str(OUT/path));app.importManager.importToTarget(opt,comp)
  bodies=[b for b in comp.bRepBodies]+[b for o in comp.allOccurrences for b in o.bRepBodies]
  rec={'name':name,'bodies':len(bodies),'bounds':[bb(b) for b in bodies],'cylinders':[]}
  if name.startswith('V02'):
   for b in bodies:
    for face in b.faces:
     g=c.Cylinder.cast(face.geometry)
     if g:rec['cylinders'].append({'radius_mm':g.radius*10,'origin':xyz(g.origin),'axis':list(g.axis.asArray())})
  records.append(rec)
 lib=app.materialLibraries.itemById('C1EEA57C-3F56-45FC-B8CB-A9EC46A9994C');mat=lib.materials.itemById('PrismMaterial-022')
 records.append({'material_properties':[{'id':p.id,'name':p.name,'value':safe(p,'value'),'unit':safe(p,'units')} for p in mat.materialProperties]})
 (OUT/'hardware_import.json').write_text(json.dumps(records,ensure_ascii=True,indent=2))
 doc.save('Imported manufacturer mechanical geometry for printed housing')
 print('HARDWARE READY',[(q.get('name'),q.get('bodies')) for q in records])
try:run()
except:(OUT/'hardware_import_error.txt').write_text(traceback.format_exc());print(traceback.format_exc())
