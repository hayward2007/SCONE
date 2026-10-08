exec(open('/Users/hayward_kim/Developer/SCONE/tools/housing/common.py').read())
app=c.Application.get()
try:
 docs=[{'name':v.name,'id':v.dataFile.id if v.dataFile else None,'modified':v.isModified} for v in app.documents]
 (OUT/'session_documents.json').write_text(json.dumps(docs,ensure_ascii=False,indent=2))
 app,doc,d=guard()
 (OUT/'session_inventory.json').write_text(json.dumps(snapshot(doc),ensure_ascii=False,indent=2))
 assert d.exportManager.execute(d.exportManager.createFusionArchiveExportOptions(str(OUT/'A01_before_print_redesign.f3d')))
 print('SESSION READY',doc.name,len(docs),d.timeline.count)
except:(OUT/'session_error.txt').write_text(traceback.format_exc());print(traceback.format_exc())
