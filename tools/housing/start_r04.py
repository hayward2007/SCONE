exec(open('/Users/hayward_kim/Developer/SCONE/tools/housing/refine_r03.py',encoding='utf-8').read().split('\ndef run():')[0])
import gzip
DEST=OUT/'R04_DELIVERY';DEST.mkdir(exist_ok=True)
def run():
 app,doc,d=guard();r=d.rootComponent
 assert doc.name.startswith('MARC Housing PLA R03'),doc.name
 snap=snapshot(doc);(DEST/'user_pose_before.json').write_text(json.dumps(snap,ensure_ascii=True))
 src=next(x for x in app.documents if x.dataFile and x.dataFile.id=='urn:adsk.wipprod:dm.lineage:VUu_hd0ATYKQ4csy_mW31g')
 (DEST/'source_before.json').write_text(json.dumps(snapshot(src),ensure_ascii=True))
 d.exportManager.execute(d.exportManager.createFusionArchiveExportOptions(str(DEST/'R03_user_pose_backup.f3d')))
 records=[]
 for o in r.allOccurrences:
  if not (o.fullPathName.startswith('LEG') or o.name.startswith(('R01 Rear','R02 Front','R03 Smooth','P04 PLA','V'))):continue
  for i,b in enumerate(o.bRepBodies):
   if not b.isSolid:continue
   calc=b.meshManager.createMeshCalculator();calc.surfaceTolerance=.005;m=calc.calculate()
   records.append(dict(path=o.fullPathName,index=i,name=b.name,visible=b.isVisible,light=b.isLightBulbOn,vertices_mm=[v*10 for v in m.nodeCoordinatesAsDouble],triangles=list(m.nodeIndices),bounds=bb(b),volume_cm3=b.volume,transform=list(o.transform2.asArray())))
 with gzip.open(DEST/'user_pose_meshes.json.gz','wt',encoding='utf-8') as h:json.dump(records,h)
 doc.saveAs('MARC Housing PLA R04',doc.dataFile.parentFolder,'User posed collision correction, serviceable camera and plate links','')
 print('R04 COPY AND USER POSE CAPTURED',len(records),doc.name)
try:run()
except:(DEST/'start_error.txt').write_text(traceback.format_exc(),encoding='utf-8');print(traceback.format_exc())
