exec(open('/Users/hayward_kim/Developer/SCONE/artifacts/housing/20260930_c1_l1/geometry.py').read())
def vol(b):return b.volume*1000 if b.isSolid else 0

def run(_context):
 a,src,d=source();pd=next(x for x in a.documents if x.name.startswith('C1 L1 revision preview'));dd=f.Design.cast(pd.products.itemByProductType('DesignProductType'))
 orig=targets(d);parts={b.name:b for b in dd.rootComponent.bRepBodies if b.name in TARGETS};report={'source':src.name,'modified':src.isModified,'interferences':[],'bolt_checks':[],'cable_checks':[],'protected_checks':{}}
 for name,b in parts.items():
  for o in d.rootComponent.allOccurrences:
   for other in o.bRepBodies:
    if not other.isVisible or other.name==name:continue
    q=parts.get(other.name,other)
    if not b.boundingBox.intersects(q.boundingBox):continue
    v=interference(b,q)
    if v>1e-4:
     old=interference(orig[name],other)
     report['interferences'].append({'part':name,'other':o.fullPathName+'/'+other.name,'before_mm3':old,'after_mm3':v,'increase_mm3':v-old})
 for x in(-80,-20,80):
  for y in(-35,35):
   b=parts['C1 USB cable guide' if x==80 else 'L1 frame']
   shank=cyl((x,y,153.5),(x,y,185),1.5)
   head=cyl((x,y,155.5),(x,y,157.15),2.85)
   driver=cyl((x,y,157.15),(x,y,180),1.3)
   report['bolt_checks'].append({'xy_mm':[x,y],'shank_mm3':interference(b,shank),'head_mm3':interference(b,head),'driver_mm3':interference(b,driver),'retained_web_mm':2,'counterbore_depth_mm':3})
 for y in(-9,-3,3,9):
  cable=cyl((9,y,148.5),(62.5,y,148.5),2.5)
  report['cable_checks'].append({'diameter_mm':5,'center_y_mm':y,'solid_overlap_mm3':interference(parts['C1 USB cable guide'],cable)})
 _,calc,regions=generate(d);removed=T.copy(orig['L1 frame']);cut(removed,parts['L1 frame'])
 for region in regions:cut(removed,region)
 added=T.copy(parts['L1 frame']);cut(added,orig['L1 frame'])
 report['protected_checks']['L1_removed_outside_feet_mm3']=vol(removed);report['protected_checks']['L1_added_mm3']=vol(added)
 report['bodies']={name:{'bounds_mm':bounds(b),'solid':b.isSolid,'lumps':b.lumps.count,'volume_mm3':vol(b)} for name,b in parts.items()}
 (OUT/'candidate_validation.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8');print(json.dumps(report,ensure_ascii=False))
