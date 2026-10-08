exec(open('/Users/hayward_kim/Developer/SCONE/tools/housing/build_r05.py',encoding='utf-8').read().split('\ndef run():')[0])
DEST=OUT/'R07_DELIVERY'
_native_cut=cut;_native_union=union;_native_common=intersect;_native_move=move
def move(a,x=0,y=0,z=0):return _native_move(T.copy(a),x,y,z)
# Functional wrappers retain the cached masters and fixed reference bodies.
def cut(a,b):return _native_cut(T.copy(a),T.copy(b))
def fuse(a,b):return _native_union(T.copy(a),T.copy(b))
def common(a,b):return _native_common(T.copy(a),T.copy(b))
def cyl(a,b,rad):return cylinder(a,b,rad)
_cache={}
def read(n):
 if n in _cache:return T.copy(_cache[n])
 if n[:3] in ['R01','R02']:
  o=r.occurrences.itemByName({'R01':'R01 Rear smooth PLA chassis:1','R02':'R02 Front smooth PLA chassis:1'}[n[:3]]);q=T.copy(current(o.component))
 elif n.startswith('MX'):
  names=['LEG 1:1','LEG 1(미러):1','LEG 1:5','LEG 1(미러):2'];q=T.copy(r.occurrences.itemByName(names[int(n[2])]).bRepBodies.item(0))
 elif n.startswith('CAM'):
  q=T.copy(next(o for o in r.allOccurrences if o.fullPathName=='V02 Waveshare official camera:1+0619:1').bRepBodies.item(0))
 else:
  nd=app.importManager.importToNewDocument(app.importManager.createSTEPImportOptions(str(DEST/n)));dd=f.Design.cast(app.activeProduct)
  bs=list(dd.rootComponent.bRepBodies)+[b for o in dd.rootComponent.allOccurrences for b in o.bRepBodies]
  assert len(bs)==1,(n,len(bs));q=T.copy(bs[0]);nd.close(False);doc.activate()
 _cache[n]=q;return T.copy(q)
def report(q,n):
 assert q.isSolid and q.volume>1,(n,q.volume);print(n,q.lumps.count,q.volume)
 cp=r.occurrences.itemByName({'R01':'R01 Rear smooth PLA chassis:1','R02':'R02 Front smooth PLA chassis:1','S01':'S01 Rear fairing lid:1','S02':'S02 Front camera fairing lid:1','P04':'P04 PLA electronics tray:1'}[n]).component;replace_final(cp,q,'R07 '+n+' planar printable part');pla(current(cp),(222,174,45) if n!='P04' else (68,73,80))
 (DEST/(n+'_built.json')).write_text(json.dumps(dict(volume_cm3=q.volume,lumps=q.lumps.count,bounds=bb(q))))
def build():
 global app,doc,d,r
 app,doc,d=guard();r=d.rootComponent;assert doc.name.startswith('MARC Housing PLA R07')
 outer=read('outer_master.step');shell=read('shell_master.step');clip=read('internal_clip.step');clip=fuse(clip,box(-100,260,-100,100,-2,3.2))
 cv=common(shell,box(-100,260,-120,120,33.8,110));low=common(shell,box(-100,260,-120,120,3,33.5))
 lower=[]
 for n,x0,x1 in [('R01',-80,64.85),('R02',65.15,250)]:
  core=common(read(n+'_input.step'),clip);patch=common(low,box(x0,x1,-100,100,2,34))
  xc=-41.8912974452 if n=='R01' else 208.1087025548
  for side in [-1,1]:
   clearance=box(xc-18.01,xc+18.01,26.3 if side>0 else -82,82 if side>0 else -26.3,-1,44.6);patch=cut(patch,clearance);cv=cut(cv,clearance)
  q=fuse(core,patch);q=common(q,outer)
  q=cut(q,box(36.7,115.3,-47.3,47.3,4.2,32.8))
  # Remove R05 fixing station that crossed the chassis lap joint.
  if n=='R02':
   for side in [-1,1]:q=cut(q,cyl((70,side*69,8),(70,side*69,35),5.05))
  xc=-41.8912974452 if n=='R01' else 208.1087025548
  for side in [-1,1]:
   q=cut(q,box(xc-12,xc+12,18 if side>0 else -29,29 if side>0 else -18,9,22))
   q=cut(q,box(xc-7,xc+7,side*20-5,side*20+5,-2,7))
  lower.append(q);(DEST/'build_progress.json').write_text(json.dumps({'stage':'lower '+n,'volume_cm3':q.volume}))
 for x,ya in [(-14,25),(60,69),(97,69),(180,25)]:
  li=0 if x<65 else 1
  for side in [-1,1]:
   y=side*ya
   lower[li]=fuse(lower[li],cyl((x,y,0),(x,y,33.5),4.8));lower[li]=cut(lower[li],cyl((x,y,-2),(x,y,35),1.65));lower[li]=cut(lower[li],cyl((x,y,-2),(x,y,2.5),3.3))
   boss=fuse(cyl((x,y,33.8),(x,y,43),4.8),box(x-4.8,x+4.8,-80 if side<0 else y,y if side<0 else 80,35,42));cv=fuse(cv,common(boss,outer))
   cv=cut(cv,cyl((x,y,33.5),(x,y,39.2),2));cv=cut(cv,cyl((x,y,33.4),(x,y,42.5),1.4))
 # Remove the changed station's interference with the pre-existing keyed lap.
 lower[1]=cut(lower[1],common(lower[0],box(64.9,84,-80,80,-2,12)))
 tray=common(read('P04.step'),read('internal_clip.step'))
 for side in [-1,1]:
  # Trim obsolete fragments at the narrow rear waist without severing the relocated mounting bosses.
  tray=cut(tray,box(30,51.9,58.5 if side>0 else -68,68 if side>0 else -58.5,36,43))
 (DEST/'build_progress.json').write_text(json.dumps({'stage':'fasteners done'}))
 # Sensor roof and fixed port access.
 for x,y in [(38.9,-14.1),(67.1,14.1)]:cv=cut(cv,cyl((x,y,98),(x,y,106),1.15))
 cv=cut(cv,box(48,58,-27,-19,99,106))
 for x in [79,85,91,97,103,109,115]:cv=cut(cv,box(x-1.5,x+1.5,-24,26,98,110))
 for side in [-1,1]:
  for x in [-8,1,10,153,162,171,180]:cv=cut(cv,box(x-1.3,x+1.3,side*48-12,side*48+12,59,74))
 cv=cut(cv,box(-75,8,-24,-9,53,62));cv=cut(cv,box(-75,8,3,23,47,68))
 # Rear camera insertion corridor and four front-side mounting posts.
 cv=cut(cv,box(137,209.79,-42.9,42.9,69.35,94.05))
 for y in [-10.5,10.5]:
  for z in [71.7,85.15]:
   post=T.createCylinderOrCone(P(209.8,y,z),.34,P(232,y,z),.55);cv=fuse(cv,common(post,outer))
   cv=cut(cv,cyl((209.4,y,z),(214,y,z),1.5));cv=cut(cv,cyl((209.4,y,z),(219,y,z),1))
 for y in [-30.05,30.05]:cv=cut(cv,cyl((137,y,78),(240,y,78),12))
 camera=read('CAM_input.step')
 for dx,dy,dz in [(0,0,0),(-.2,0,0),(0,-.2,0),(0,.2,0),(0,0,-.2),(0,0,.2)]:cv=cut(cv,move(camera,dx,dy,dz))
 # Clearance around the PCB envelope and four unchanged MX top screw heads.
 cv=cut(cv,box(145.7,163.9,-30.3,30.3,6.7,77.3))
 for xc in [-41.8912974452,208.1087025548]:
  for side in [-1,1]:
   for dx,ya in [(-15,35.64092583),(15,35.64092583),(-8.5,29.34092583),(8.5,29.34092583)]:cv=cut(cv,cyl((xc+dx,side*ya,36.5),(xc+dx,side*ya,40),2.5))
 (DEST/'build_progress.json').write_text(json.dumps({'stage':'camera done'}))
 # Clear the unchanged upper saddles using explicit service envelopes.
 for xc in [-41.8912974452,208.1087025548]:
  for side in [-1,1]:cv=cut(cv,box(xc-19.8,xc+19.8,21.2 if side>0 else -43.3,43.3 if side>0 else -21.2,33.7,37.0))
 cv=cut(cv,move(tray,z=.25))
 # Both front continuous wheel cylinders, exact unchanged axis and 20 mm candidate reserve.
 for side in [-1,1]:
  env=cyl((113.1087026,side*187.640925829,18.5),(142.5087026,side*187.640925829,18.5),124.8)
  cv=cut(cv,env);lower=[cut(q,env) for q in lower]
 back=common(cv,box(-100,82.85,-120,120,30,160));front=common(cv,box(83.15,260,-120,120,30,160))
 for n,q in zip(['R01','R02','S01','S02','P04'],lower+[back,front,tray]):report(q,n)
 doc.save('R07 explicit planar housing and assembly clearances')
 print('R07 PARTS BUILT',flush=True)

try:build()
except:(DEST/'build_error.txt').write_text(traceback.format_exc(),encoding='utf-8');print(traceback.format_exc())
