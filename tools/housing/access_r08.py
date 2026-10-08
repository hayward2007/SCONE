exec(open('/Users/hayward_kim/Developer/SCONE/tools/housing/build_r05.py',encoding='utf-8').read().split('\ndef run():')[0])
DEST=OUT/'R08_DELIVERY'
def run():
 app,doc,d=guard();r=d.rootComponent;assert doc.name.startswith('MARC Housing PLA R08')
 parts=[(o.name,T.copy(current(o.component))) for o in r.occurrences if o.name.startswith(('R01 Rear','R02 Front','S01 Rear','S02 Front','P04 PLA','M01 Rear','M02 Front'))]
 hw=[(o.fullPathName+'/'+b.name,T.copy(b)) for o in r.allOccurrences if o.fullPathName.startswith('V') for b in o.bRepBodies if b.isSolid and b.isLightBulbOn]
 fixed_hw=[p for p in hw if not p[0].startswith(('V02','V07'))]
 lower=[p for p in parts if p[0].startswith(('R01','R02'))];openbody=[p for p in parts if not p[0].startswith(('S01','S02'))];results=[];heads=[]
 def check(label,tool,targets,details):
  hits=[]
  for n,b in targets:
   try:v=intersection_volume(T.copy(tool),T.copy(b))
   except Exception as e:_=app.activeDocument.name;hits.append([n,None,str(e)]);continue
   if v is None or v>.01:hits.append([n,v])
  results.append(dict(name=label,hits=hits,**details))
 for x,ya in [(-14,25),(77,72),(97,69),(180,25)]:
  for s in [-1,1]:
   y=s*ya;check('lid M3 underside '+str([x,y]),cylinder((x,y,-85),(x,y,2.45),2.4),parts+hw,dict(driver_diameter_mm=4.8,approach='-Z',screw='M3x35 low head D6 H2.5'))
   heads.append(('lid '+str([x,y]),cylinder((x,y,0),(x,y,2.4),3)))
 for xc in [-41.8912974452,208.1087025548]:
  for s in [-1,1]:
   for dx,ya in [(-15,35.64092583),(15,35.64092583),(-8.5,29.34092583),(8.5,29.34092583)]:
    x,y=xc+dx,s*ya
    check('MX top M2 '+str([x,y]),cylinder((x,y,36.75),(x,y,130),1.75),openbody+fixed_hw,dict(driver_diameter_mm=3.5,approach='+Z',screw='M2x6, 4 per motor',stage='sensor lids removed together with camera/lidar, leg subassembly not installed'))
    check('MX bottom M2 '+str([x,y]),cylinder((x,y,-85),(x,y,2.05),1.75),openbody+fixed_hw,dict(driver_diameter_mm=3.5,approach='-Z',screw='M2x4, 4 per motor',stage='before legs/wheels'))
    heads.append(('MX top '+str([x,y]),cylinder((x,y,36.72),(x,y,38.3),1.9)))
    heads.append(('MX bottom '+str([x,y]),cylinder((x,y,.55),(x,y,2.05),1.9)))
   y=s*15;check('bridge M3 '+str([xc,y]),cylinder((xc,y,36.75),(xc,y,130),2.4),openbody+fixed_hw,dict(driver_diameter_mm=4.8,approach='+Z',screw='M3x8 low head D5.5 H2'))
   heads.append(('bridge '+str([xc,y]),cylinder((xc,y,36.72),(xc,y,38.7),2.75)))
 for x in [71,79]:
  for y in [-62,62]:
   check('chassis M4 bottom '+str([x,y]),cylinder((x,y,-85),(x,y,-5.05),3),lower,dict(driver_diameter_mm=6,approach='-Z',screw='M4x25 with nut and washers',stage='tray/lids removed'))
   check('chassis M4 nut '+str([x,y]),cylinder((x,y,18.05),(x,y,85),4.5),lower,dict(driver_diameter_mm=9,approach='+Z',stage='tray/lids removed'))
   heads.append(('M4 head '+str([x,y]),cylinder((x,y,-5),(x,y,-1),3.5)))
   heads.append(('M4 nut '+str([x,y]),cylinder((x,y,18.05),(x,y,21.4),4)))
 for x,ya,top in [(33,48,39.5),(125,50,41)]:
  for s in [-1,1]:
   y=s*ya;check('tray M3 '+str([x,y]),cylinder((x,y,top+.05),(x,y,130),2.4),openbody,dict(driver_diameter_mm=4.8,approach='+Z',screw='M3x10 low head D5.5 H2',stage='Jetson/lids removed'))
   heads.append(('tray '+str([x,y]),cylinder((x,y,top+.02),(x,y,top+2),2.75)))
 for x,ya in [(45,53.5),(119,53.5)]:
  for s in [-1,1]:check('Jetson retaining clip '+str([x,s*ya]),cylinder((x,s*ya,49.25),(x,s*ya,130),1.75),openbody+fixed_hw,dict(driver_diameter_mm=3.5,approach='+Z',screw='M3x6 plus nonconductive retaining tab',required_slim_shaft_diameter_mm=3.5,stage='lids removed'))
 for x,y in [(38.9,-14.1),(67.1,14.1)]:check('lidar M2 '+str([x,y]),cylinder((x,y,65),(x,y,101.15),1.75),[p for p in parts if p[0].startswith('S01')],dict(driver_diameter_mm=3.5,approach='-Z',screw='M2x5 example',stage='rear cover detached'))
 for label,head in heads:check('HEAD '+label,head,parts+hw,dict(kind='fastener head clearance'))
 (DEST/'screw_access.json').write_text(json.dumps(dict(tests=results,total=len(results),failed=sum(bool(t['hits']) for t in results),note='Subassemblies are oriented so the listed axial tool direction is vertical on the assembly table. Legs/wheels installed after body motor bolts. Tool handle not included.'),indent=2))
 print('ACCESS',len(results),'FAIL',[(x['name'],x['hits']) for x in results if x['hits']])
try:run()
except:(DEST/'access_error.txt').write_text(traceback.format_exc(),encoding='utf-8');print(traceback.format_exc())
