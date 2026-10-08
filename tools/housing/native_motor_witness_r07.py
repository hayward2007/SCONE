exec(open('/Users/hayward_kim/Developer/SCONE/tools/housing/build_r05.py',encoding='utf-8').read().split('\ndef run():')[0])
DEST=OUT/'R07_DELIVERY'
app,doc,d=guard();r=d.rootComponent;report=[]
for n,frame in [('LEG 1:1','R02 Front smooth PLA chassis:1'),('LEG 1(미러):1','R02 Front smooth PLA chassis:1'),('LEG 1:5','R01 Rear smooth PLA chassis:1'),('LEG 1(미러):2','R01 Rear smooth PLA chassis:1')]:
 try:
  o=r.occurrences.itemByName(frame);a=r.occurrences.itemByName(n).bRepBodies.item(0);b=current(o.component).createForAssemblyContext(o)
  inp=d.createInterferenceInput(collection([a,b]));inp.areCoincidentFacesIncluded=False;res=d.analyzeInterference(inp)
  report.append(dict(motor=n,frame=frame,result_count=res.count,volumes_mm3=[x.interferenceBody.volume*1000 for x in res]))
 except Exception as e:report.append(dict(motor=n,error=str(e)))
(DEST/'native_motor_witness.json').write_text(json.dumps(report,ensure_ascii=True,indent=2));print(report)
