exec(open('/Users/hayward_kim/Developer/SCONE/artifacts/electronics/20260930_jetson_carrier/protect_v21.py').read())
OUT=Path('/Users/hayward_kim/Developer/SCONE/artifacts/electronics/20260930_u2d2_docked')
def all_sig(d):
 r=d.rootComponent
 return {'root':[bsig(b) for b in r.bRepBodies],'occurrences':{o.fullPathName:{'transform':[round(x,7) for x in o.transform2.asArray()],'visible':o.isLightBulbOn,'bodies':[bsig(b) for b in o.component.bRepBodies],'meshes':[{'name':b.name,'box':bbox(b),'light':b.isLightBulbOn} for b in o.component.meshBodies]} for o in r.allOccurrences},'warnings':[{'i':i,'name':d.timeline.item(i).name,'health':d.timeline.item(i).healthState,'error':d.timeline.item(i).errorOrWarningMessage} for i in range(d.timeline.count) if d.timeline.item(i).healthState not in(0,5)]}
def run(_context):
 a=c.Application.get();doc=next(x for x in a.documents if x.name.startswith('MARC v4 Body v5 BOX v'));d=f.Design.cast(doc.products.itemByProductType('DesignProductType'))
 (OUT/'baseline.json').write_text(json.dumps(all_sig(d),ensure_ascii=False,indent=2),encoding='utf-8')
 assert d.exportManager.execute(d.exportManager.createFusionArchiveExportOptions(str(OUT/'SOURCE_live_v22_before_docking.f3d')))
 print('Current unsaved v22 state backed up; baseline includes all existing components')
