exec(open('/Users/hayward_kim/Developer/SCONE/artifacts/electronics/20260930_vertical_carrier/capture_source.py').read())
OUT=Path('/Users/hayward_kim/Developer/SCONE/artifacts/electronics/20260930_jetson_carrier')
def run(_context):
 a=c.Application.get();doc=a.activeDocument;d=f.Design.cast(a.activeProduct)
 assert doc.name=='MARC v4 Body v5 BOX v20',doc.name
 p=protected(d)
 p['timeline']=[{'index':i,'name':d.timeline.item(i).name,'health':d.timeline.item(i).healthState,'error':d.timeline.item(i).errorOrWarningMessage} for i in range(d.timeline.count)]
 p['joint_values']=[{'path':o.fullPathName,'joints':[{'name':j.name,'type':j.jointMotion.jointType,'angle':getattr(j.jointMotion,'rotationValue',None)} for j in o.component.joints]} for o in d.rootComponent.allOccurrences if o.component.joints.count]
 (OUT/'baseline_v20.json').write_text(json.dumps(p,ensure_ascii=False,indent=2),encoding='utf-8')
 (OUT/'world_inventory.json').write_text(json.dumps([{'path':o.fullPathName,'transform':list(o.transform2.asArray()),'bodies':[{'name':b.name,'box':[[v*10 for v in pt.asArray()] for pt in (b.boundingBox.minPoint,b.boundingBox.maxPoint)]} for b in o.bRepBodies]} for o in d.rootComponent.allOccurrences],ensure_ascii=False,indent=2),encoding='utf-8')
 assert d.exportManager.execute(d.exportManager.createFusionArchiveExportOptions(str(OUT/'SOURCE_v20_before_jetson_carrier.f3d')))
 print(json.dumps({'document':doc.name,'modified':doc.isModified,'occurrences':len(p['occurrences']),'timeline':len(p['timeline']),'warnings':[x for x in p['timeline'] if x['health']!=0]},ensure_ascii=False))
