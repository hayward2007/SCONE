exec(open('/Users/hayward_kim/Developer/SCONE/tools/housing/build_r05.py',encoding='utf-8').read().split('\ndef run():')[0])
DEST=OUT/'R08_DELIVERY'
NAMES={'R01':'R01 Rear smooth PLA chassis:1','R02':'R02 Front smooth PLA chassis:1','S01':'S01 Rear fairing lid:1','P04':'P04 PLA electronics tray:1'}
def run():
 app,doc,d=guard();r=d.rootComponent;assert doc.name.startswith('MARC Housing PLA R08')
 def read(n):
  nd=app.importManager.importToNewDocument(app.importManager.createSTEPImportOptions(str(DEST/n)));dd=f.Design.cast(app.activeProduct);bs=list(dd.rootComponent.bRepBodies)+[b for o in dd.rootComponent.allOccurrences for b in o.bRepBodies];assert len(bs)==1
  q=T.copy(bs[0]);nd.close(False);doc.activate();return q
 def progress(s):
  (DEST/'neck_progress.json').write_text(json.dumps({'stage':s}));print(s,flush=True)
 outer=read('outer_master.step');skin=read('shell_master.step');clip=read('internal_clip.step');assembly=read('assembly_clip.step')
 keep=T.copy(clip)
 for b in [box(-100,24,-120,120,-2,120),box(84,260,-120,120,-2,120),box(-100,260,-120,120,-2,3.2)]:union(keep,b)
 src={k:T.copy(current(r.occurrences.itemByName(n).component)) for k,n in NAMES.items()}
 result={};report=[]
 for k in ['R01','R02','S01']:
  q=T.copy(src[k])
  if k=='R01':
   for s in [-1,1]:
    cut(q,cylinder((57,s*59,3.1),(57,s*59,38),5.25));cut(q,cylinder((60,s*69,-1),(60,s*69,35),5.1))
  if k=='S01':
   for s in [-1,1]:cut(q,box(54.8,65.2,63.8 if s>0 else -83,83 if s>0 else -63.8,33.6,43.2))
  intersect(q,T.copy(keep));patch=T.copy(skin)
  intersect(patch,box(24 if k!='R02' else 65.15,64.85 if k=='R01' else (82.85 if k=='S01' else 84),-100,100,33.8 if k=='S01' else 3,56 if k=='S01' else 33.5))
  union(q,patch)
  # Floor outline follows the new neck; keep the rear-to-front keyed overlap intact.
  if k!='S01':
   floortrim=box(24,64.85,-100,100,-1,3.2);cut(floortrim,T.copy(outer));cut(q,floortrim)
  result[k]=q;progress(k+' neck rebuilt')
 for k in ['R01','R02']:
  q=result[k];xa,xb=(31,64.85) if k=='R01' else (65.15,99)
  for s in [-1,1]:
   union(q,box(xa,xb,s*50-1.5,s*50+1.5,2.8,18))
   if k=='R01':
    union(q,cylinder((33,s*48,2.8),(33,s*48,36.3),4))
    cut(q,cylinder((33,s*48,27.3),(33,s*48,38),1.25))
  cut(q,box(36.7,115.3,-47.3,47.3,4.2,32.8))
  progress(k+' ribs and tray supports')
 for s in [-1,1]:
  x,y=77,s*72;q=result['R02'];union(q,cylinder((x,y,0),(x,y,33.5),4.8));cut(q,cylinder((x,y,-2),(x,y,35),1.65));cut(q,cylinder((x,y,-2),(x,y,2.5),3.3))
  boss=cylinder((x,y,33.8),(x,y,43),4.8);union(boss,box(x-4.8,x+4.8,y if s>0 else -80,80 if s>0 else y,35,42));intersect(boss,T.copy(outer));union(result['S01'],boss)
  cut(result['S01'],cylinder((x,y,33.5),(x,y,39.2),2));cut(result['S01'],cylinder((x,y,33.4),(x,y,42.5),1.4))
 overlap=T.copy(result['R01']);intersect(overlap,box(64.9,84,-80,80,-2,12));cut(result['R02'],overlap)
 tray=src['P04']
 for s in [-1,1]:
  # Add mounting ears before clipping to preserve a continuous flat tray.
  union(tray,box(29,43,s*48-4.5,s*48+4.5,36.3,39.5));cut(tray,cylinder((33,s*48,35),(33,s*48,43),1.65))
 intersect(tray,assembly);result['P04']=tray
 progress('supports relocated')
 # Continuous cylinder includes the widened tire plus 2.1 mm running allowance.
 saved={o['name']:o for o in json.loads((DEST/'user_pose_before.json').read_text())['occurrences']}
 env=cylinder((0,0,-2.1),(0,0,22.1),124.6);m=c.Matrix3D.create();m.setWithArray(saved['LEG 1:5+ARC:1']['transform']);T.transform(env,m)
 mirror=T.copy(env);mt=c.Matrix3D.create();mt.setWithArray([1,0,0,0,0,-1,0,0,0,0,1,0,0,0,0,1]);T.transform(mirror,mt)
 for k,q in result.items():
  for e in [env,mirror]:cut(q,T.copy(e))
  assert q.lumps.count==1,(k,q.lumps.count)
  cp=r.occurrences.itemByName(NAMES[k]).component;replace_final(cp,q,'R08 '+k+' 20mm wheel neck and ribbed supports');pla(current(cp),(68,73,80) if k=='P04' else (222,174,45))
  report.append(dict(part=k,lumps=q.lumps.count,bounds=bb(q),before_cm3=src[k].volume,after_cm3=q.volume))
  progress(k+' committed')
 doc.save('R08 narrowed rear waist for 20mm tires, relocated vertical fasteners and ribbed tray supports')
 (DEST/'neck_change.json').write_text(json.dumps(dict(parts=report,lid_station_from=[60,69],lid_station_to=[77,72],tray_station_from=[57,59],tray_station_to=[33,48]),indent=2));print('R08 NECK DONE')
try:run()
except:(DEST/'neck_error.txt').write_text(traceback.format_exc(),encoding='utf-8');print(traceback.format_exc())
