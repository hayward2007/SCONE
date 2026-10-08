exec(open('/Users/hayward_kim/Developer/SCONE/tools/housing/refine_r03.py',encoding='utf-8').read().split('\ndef run():')[0])
def run():
 app,doc,d=guard();r=d.rootComponent;cv=T.copy(current(r.occurrences.itemByName('R03 Smooth PLA sensor cover:1').component));cam=T.copy(next(o for o in r.allOccurrences if o.fullPathName=='V02 Waveshare official camera:1+0619:1').bRepBodies.item(0));intersect(cv,cam)
 report={'camera_intersection_box':bb(cv),'camera_intersection_mm3':cv.volume*1000}
 cp=r.occurrences.itemByName('P04 PLA electronics tray:1').component;b=current(cp);report['tray_lumps']=[]
 for lump in b.lumps:
  points=[xyz(v.geometry) for fa in lump.faces for e in fa.edges for v in [e.startVertex,e.endVertex]];report['tray_lumps'].append([[min(p[a] for p in points) for a in range(3)],[max(p[a] for p in points) for a in range(3)]])
 (OUT/'r03_fit_probe.json').write_text(json.dumps(report,indent=2));print(report)
try:run()
except:print(traceback.format_exc())
