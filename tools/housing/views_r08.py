exec(open('/Users/hayward_kim/Developer/SCONE/tools/housing/deliver_r03.py',encoding='utf-8').read().split('\ndef run():')[0])
DEST=OUT/'R08_DELIVERY'
app,doc,d=guard();r=d.rootComponent
names=['R01 Rear smooth PLA chassis:1','R02 Front smooth PLA chassis:1','S01 Rear fairing lid:1','S02 Front camera fairing lid:1','P04 PLA electronics tray:1','M01 Rear removable motor bridge:1','M02 Front removable motor bridge:1']
for cp in d.allComponents:cp.isJointsFolderLightBulbOn=False;cp.isSketchFolderLightBulbOn=False;cp.isConstructionFolderLightBulbOn=False
vis=[(o,o.isLightBulbOn) for o in r.occurrences];rv=[(b,b.isLightBulbOn) for b in r.bRepBodies]
for b in r.bRepBodies:b.isLightBulbOn=False
for o in r.occurrences:o.isLightBulbOn=o.name in names
shot(app,'R08_DELIVERY/housing_detail.png',(420,-450,280),(83,0,50),True)
for o in r.occurrences:o.isLightBulbOn=o.name in names+['V01 NVIDIA official assembly:1'] or o.name.startswith('V')
r.occurrences.itemByName('S01 Rear fairing lid:1').isLightBulbOn=False;r.occurrences.itemByName('S02 Front camera fairing lid:1').isLightBulbOn=False
shot(app,'R08_DELIVERY/internals.png',(440,-480,500),(83,0,35),True)
for o,v in vis:o.isLightBulbOn=v
for b,v in rv:b.isLightBulbOn=v
shot(app,'R08_DELIVERY/assembly.png',(500,-630,320),(70,0,20),True)
print('R08 VIEWS SAVED')
