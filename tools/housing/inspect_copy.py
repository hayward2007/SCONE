exec(open('/Users/hayward_kim/Developer/SCONE/tools/housing/common.py').read())
def run():
 app,doc,d=guard();r=d.rootComponent
 data=snapshot(doc)
 (OUT/'copy_inventory.json').write_text(json.dumps(data,ensure_ascii=False,indent=2))
 source=next(x for x in app.documents if x.dataFile and x.dataFile.id==json.loads((OUT/'copy_identity.json').read_text())['source_id'])
 (OUT/'source_after_copy.json').write_text(json.dumps(snapshot(source),ensure_ascii=False,indent=2))
 detailed={'root_bodies':[],'legs':[],'joint_health':[],'materials':[],'apis':{}}
 for b in r.bRepBodies:
  detailed['root_bodies'].append({'name':b.name,'vertices':[xyz(v.geometry) for v in b.vertices],'faces':[{'type':fa.geometry.objectType,'area':fa.area,'box':bb(fa)} for fa in b.faces]})
 for o in r.occurrences:
  detailed['legs'].append({'name':o.name,'bodies':[body(b) for b in o.bRepBodies]})
 for comp in d.allComponents:
  for j in list(comp.joints)+list(comp.asBuiltJoints):
   detailed['joint_health'].append({'component':comp.name,'name':j.name,'health':safe(j,'healthState'),'message':safe(j,'errorOrWarningMessage')})
 for lib in app.materialLibraries:
  detailed['materials'].append({'id':lib.id,'name':lib.name,'examples':[{'id':m.id,'name':m.name} for m in lib.materials if any(s in m.name.lower() for s in ['6061','abs plastic','abs 플라스틱','nylon','나일론','aluminum'])][:12]})
 for name in ['createCylinderOrCone','createWireFromCurves','createFaceFromPlanarWires','createRuledSurface','createBox']:
  detailed['apis'][name]=getattr(f.TemporaryBRepManager,name).__doc__
 (OUT/'geometry_details.json').write_text(json.dumps(detailed,ensure_ascii=False,indent=2))
 print('COPY INSPECTED, original source rechecked without changes')
try:run()
except:(OUT/'inspect_copy_error.txt').write_text(traceback.format_exc());print(traceback.format_exc())
