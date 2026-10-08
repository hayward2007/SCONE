exec(open('/Users/hayward_kim/Developer/SCONE/tools/housing/geometry.py').read())
import gzip,time
try:
 app,doc,d=guard();r=d.rootComponent;data=[]
 for name,b in [('ROOT/'+b.name,b) for b in r.bRepBodies if b.name=='본체12']+[(o.fullPathName+'/'+b.name,b) for o in r.allOccurrences if o.fullPathName.startswith('LEG') or o.component.attributes.itemByName('SCONE_HOUSING','owned') for b in o.bRepBodies]:
  if not b.isSolid:continue
  temp=T.copy(b);calc=temp.meshManager.createMeshCalculator();calc.surfaceTolerance=.005;mesh=calc.calculate()
  assert mesh,name
  data.append({'name':name,'bbox':bb(temp),'vertices_mm':[v*10 for v in mesh.nodeCoordinatesAsDouble],'triangles':list(mesh.nodeIndices)})
 with gzip.open(OUT/'analysis_meshes.json.gz','wt') as file:json.dump(data,file)
 source=next(x for x in app.documents if x.dataFile and x.dataFile.id==json.loads((OUT/'copy_identity.json').read_text())['source_id'])
 (OUT/'source_before_A02.json').write_text(json.dumps(snapshot(source),ensure_ascii=False,indent=2))
 print('ANALYSIS EXPORTED',len(data),'bodies')
except:(OUT/'analysis_export_error.txt').write_text(traceback.format_exc());print(traceback.format_exc())
