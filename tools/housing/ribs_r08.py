exec(open('/Users/hayward_kim/Developer/SCONE/tools/housing/build_r05.py',encoding='utf-8').read().split('\ndef run():')[0])
DEST=OUT/'R08_DELIVERY'
def run():
 app,doc,d=guard();r=d.rootComponent;assert doc.name.startswith('MARC Housing PLA R08');report=[]
 for name,xlo,xhi,bays,stations in [('S01 Rear fairing lid:1',-54,82.85,[(-52,-14),(-8,24),(30,72)],[-11,27,75]),('S02 Front camera fairing lid:1',83.15,221,[(90,132),(138,180),(186,219)],[87,135,183])]:
  cp=r.occurrences.itemByName(name).component;old=current(cp);before=old.volume;q=T.copy(old)
  for s in [-1,1]:
   # Thin only clear high-wall panels; leave 6 mm bands as rib roots.
   for xa,xb in bays:cut(q,box(xa,xb,52.9 if s>0 else -53.8,53.8 if s>0 else -52.9,59,93))
   for x in stations:union(q,box(x-.9,x+.9,52.0 if s>0 else -53.5,53.5 if s>0 else -52.0,58.5,95))
   union(q,box(xlo,xhi,s*40-.9,s*40+.9,97.8,101.4))
  assert q.lumps.count==1;replace_final(cp,q,'R08 ribbed 2.2mm upper panels, 2.8mm mounting perimeter');pla(current(cp),(222,174,45))
  # Clear relocated tray ears and cradle tops inside the new neck.
  for lower_name in ['R01 Rear smooth PLA chassis:1','R02 Front smooth PLA chassis:1','P04 PLA electronics tray:1']:
   target=current(r.occurrences.itemByName(lower_name).component)
   for dx,dy,dz in [(0,0,.25),(-.2,0,.25),(.2,0,.25),(0,-.2,.25),(0,.2,.25)]:cut(q,move(T.copy(target),x=dx,y=dy,z=dz))
  replace_final(cp,q,'R08 ribbed cover with relocated assembly clearance');pla(current(cp),(222,174,45))
  # Small radius only on accessible tall exterior corners; keep all planar styling.
  b=current(cp);edges=[]
  for e in b.edges:
   pts=[v.geometry for v in [e.startVertex,e.endVertex] if v]
   if len(pts)!=2:continue
   aa,bbp=pts
   if abs(aa.x-bbp.x)<1e-6 and abs(aa.y-bbp.y)<1e-6 and abs(abs(aa.y)*10-56)<.02 and abs(abs(aa.z-bbp.z)*10-40)<.05 and min(abs(aa.x*10+61.4),abs(aa.x*10-227.6))<.02:edges.append(e)
  fillet={'candidates':len(edges),'radius_mm':1.0,'applied':False}
  if edges:
   try:
    fi=cp.features.filletFeatures.createInput();fi.addConstantRadiusEdgeSet(collection(edges),c.ValueInput.createByString('1 mm'),False);ft=cp.features.filletFeatures.add(fi);ft.name='R08 small exterior corner R1';fillet['applied']=True
   except Exception as er:fillet['error']=str(er);_=app.activeDocument.name
  report.append(dict(part=name,before_cm3=before,after_cm3=current(cp).volume,fillet=fillet))
 for name,xc in [('M01 Rear removable motor bridge:1',-41.8912974452),('M02 Front removable motor bridge:1',208.1087025548)]:
  cp=r.occurrences.itemByName(name).component;b=current(cp);before=b.volume;q=T.copy(b)
  for dx in [-10,10]:union(q,box(xc+dx-1,xc+dx+1,-24,24,29,33.5))
  union(q,box(xc-10,xc+10,-1,1,29,33.5));assert q.lumps.count==1
  replace_final(cp,q,'R08 removable MX28 bridge with underside ladder ribs');pla(current(cp),(68,73,80));report.append(dict(part=name,before_cm3=before,after_cm3=q.volume,rib_thickness_mm=2,rib_depth_mm=4.3))
 doc.save('R08 ribs, selected 2.2mm upper panels and small exterior fillets; unchanged motor mounting axes')
 (DEST/'rib_changes.json').write_text(json.dumps(report,indent=2));print('R08 RIBS DONE',report)
try:run()
except:(DEST/'ribs_error.txt').write_text(traceback.format_exc(),encoding='utf-8');print(traceback.format_exc())
