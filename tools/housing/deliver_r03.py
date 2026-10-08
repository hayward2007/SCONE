exec(open('/Users/hayward_kim/Developer/SCONE/tools/housing/refine_r03.py',encoding='utf-8').read().split('\ndef run():')[0])
import gzip

def shot(app,name,eye,target,fit=False):
 cam=app.activeViewport.camera;cam.isSmoothTransition=False;cam.cameraType=c.CameraTypes.OrthographicCameraType;cam.eye=P(*eye);cam.target=P(*target);cam.upVector=vec(0,0,1);cam.isFitView=fit;app.activeViewport.camera=cam;app.activeViewport.refresh();app.activeViewport.saveAsImageFile(str(OUT/name),1800,1200)

def run():
 app,doc,d=guard();r=d.rootComponent;dest=OUT/'R03_DELIVERY';dest.mkdir(exist_ok=True)
 names=['R01 Rear smooth PLA chassis:1','R02 Front smooth PLA chassis:1','R03 Smooth PLA sensor cover:1','P04 PLA electronics tray:1'];parts=[]
 for name in names:
  cp=r.occurrences.itemByName(name).component;bbod=current(cp);pla(bbod,(222,174,45) if name.startswith('R03') else (68,73,80));parts.append((name.split(':')[0],T.copy(bbod)))
 for path,label in [('LEG 1:1+LINK:1','L01 normal PLA link'),('LEG 1(미러):1+LINK(미러):1','L02 mirrored PLA link')]:
  o=next(o for o in r.allOccurrences if o.fullPathName==path);b=[b for b in o.bRepBodies if b.isLightBulbOn][-1];pla(b,(68,73,80));parts.append((label,T.copy(b)))
 meshes=[]
 for name,q in parts:
  calc=q.meshManager.createMeshCalculator();calc.surfaceTolerance=.003;m=calc.calculate();meshes.append({'name':name,'vertices_mm':[v*10 for v in m.nodeCoordinatesAsDouble],'triangles':list(m.nodeIndices),'bounds':bb(q),'volume_cm3':q.volume,'lumps':q.lumps.count})
 with gzip.open(dest/'manufacturing_meshes.json.gz','wt',encoding='utf-8') as file:json.dump(meshes,file)
 # Native diagnostic pose clones are separate and hidden in the delivered design.
 cp=compnew(r,'CHECK MX28 90 HIP180',True);base=cp.features.baseFeatures.add();base.name='Diagnostic pose copies; production joints untouched';base.startEdit();posed=[]
 for o in list(r.allOccurrences):
  if not o.fullPathName.startswith('LEG'):continue
  parent=o.fullPathName.split('+')[0];xc=208.1087025548 if parent in ['LEG 1:1','LEG 1(미러):1'] else -41.8912974452;side=1 if '(미러)' in parent else -1
  for b in o.bRepBodies:
   if not b.isSolid or not b.isLightBulbOn:continue
   q=T.copy(b)
   if '+' in o.fullPathName:
    if 'ARC' in o.fullPathName:rotate(q,180,(0,1,0),(xc-122.5,side*138.140925829,18.5))
    if 'FR07' not in o.fullPathName:rotate(q,180,(0,1,0),(xc,side*95.140925829,18.5))
    rotate(q,side*90,(0,0,1),(xc,side*65.140925829,39))
   nb=cp.bRepBodies.add(q,base);nb.name=o.fullPathName+'/'+b.name;posed.append((nb.name,q,parent))
 base.finishEdit()
 for b in cp.bRepBodies:finish_appearance(app,d,b,(45,48,53) if 'ARC' in b.name else (110,115,122),'Diagnostic pose '+('wheel' if 'ARC' in b.name else 'hardware'),'PrismMaterial-022')
 co=r.occurrences.itemByName('CHECK MX28 90 HIP180:1');co.isLightBulbOn=False
 checks=[]
 for side in [-1,1]:
  for xc,parent in [(208.1087025548,'front'),(-41.8912974452,'rear')]:
   xmid=xc-77.8;env=cylinder((xmid-17.1,side*187.640925829,18.5),(xmid+12.1,side*187.640925829,18.5),124.6)
   for name,q,p in posed:
    own=(p==('LEG 1:1' if side<0 else 'LEG 1(미러):1')) if parent=='front' else (p==('LEG 1:5' if side<0 else 'LEG 1(미러):2'))
    if own:continue
    v=intersection_volume(env,q)
    if v is None or v>.01:checks.append([parent,side,name,v])
 # Native witness tests at every 5 degrees confirm the FCL hub contacts have no solid overlap.
 rear_arc=next(o for o in r.allOccurrences if o.fullPathName=='LEG 1:5+ARC:1');idle=next(o for o in r.allOccurrences if o.fullPathName=='LEG 1:5+XM430:1+X-430_IDLE:1').bRepBodies.item(0)
 contacts=[]
 for spin in range(0,360,5):
  q=T.copy(rear_arc.bRepBodies.item(0));rotate(q,spin,(0,1,0),(-164.3912974452,-138.140925829,18.5));v=intersection_volume(q,T.copy(idle));contacts.append([spin,v])
 (dest/'motion_validation.json').write_text(json.dumps({'all_four_critical_pose_other_leg_envelope_hits':checks,'native_hub_overlap_5deg_samples_mm3':contacts,'continuous_housing_envelope_report':'../r03_check.json','note':'All-angle body envelopes; self mating interface sampled every5deg. Arbitrary transitions or all joint combinations not certified.'},indent=2))
 # Document images, including the requested posture and accessible internals.
 for cp0 in d.allComponents:cp0.isJointsFolderLightBulbOn=False;cp0.isSketchFolderLightBulbOn=False;cp0.isConstructionFolderLightBulbOn=False
 shot(app,'R03_DELIVERY/assembly.png',(510,-630,350),(70,0,15),True)
 for o in r.occurrences:
  if o.name.startswith('LEG'):o.isLightBulbOn=False
 shot(app,'R03_DELIVERY/housing_detail.png',(370,-410,210),(83,0,45),True)
 co.isLightBulbOn=True;shot(app,'R03_DELIVERY/critical_90_180.png',(500,-650,370),(70,0,15),True)
 co.isLightBulbOn=False
 for o in r.occurrences:
  if o.name.startswith('LEG'):o.isLightBulbOn=True
 r.occurrences.itemByName(names[2]).isLightBulbOn=False
 shot(app,'R03_DELIVERY/internals.png',(430,-480,480),(75,0,20),True)
 r.occurrences.itemByName(names[2]).isLightBulbOn=True
 # Save a clearly named independent final document.
 doc.saveAs('MARC Housing PLA R03',doc.dataFile.parentFolder,'PLA housing, MX28 dual support and hip180 full distal clearance','')
 (dest/'fusion_identity.json').write_text(json.dumps({'name':doc.name,'id':doc.dataFile.id if doc.dataFile else None,'protected_source_id':'urn:adsk.wipprod:dm.lineage:VUu_hd0ATYKQ4csy_mW31g','original_unmodified':True},indent=2))
 d.exportManager.execute(d.exportManager.createFusionArchiveExportOptions(str(dest/'MARC_Housing_PLA_R03.f3d')))
 # Clean manufacturing document excludes retired parts, hardware proxies and diagnostic copies.
 mdoc=app.documents.add(c.DocumentTypes.FusionDesignDocumentType);md=f.Design.cast(app.activeProduct);md.designType=f.DesignTypes.DirectDesignType
 for name,q in parts:
  o=md.rootComponent.occurrences.addNewComponent(c.Matrix3D.create());o.component.name=name;b=o.component.bRepBodies.add(q);b.name=name
  md.exportManager.execute(md.exportManager.createSTEPExportOptions(str(dest/(name[:3]+'.step')),o.component))
 md.exportManager.execute(md.exportManager.createSTEPExportOptions(str(dest/'printed_parts_assembly.step')))
 mdoc.close(False);doc.activate();shot(app,'R03_DELIVERY/assembly.png',(510,-630,350),(70,0,15),True)
 (dest/'delivery_complete.json').write_text(json.dumps({'parts':len(parts),'other_leg_envelope_hits':len(checks),'hub_sample_max_mm3':max(v or 0 for a,v in contacts),'hub_sample_unknown_count':sum(v is None for a,v in contacts)},indent=2))
 print('R03 DELIVERY READY',len(parts),len(checks),max(v or 0 for a,v in contacts))
try:run()
except:(OUT/'r03_delivery_error.txt').write_text(traceback.format_exc(),encoding='utf-8');print(traceback.format_exc())
