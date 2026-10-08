exec(open('/Users/hayward_kim/Developer/SCONE/artifacts/electronics/20260930_u2d2_docked/capture.py').read().split('def run(_context):')[0])
OUT=Path('/Users/hayward_kim/Developer/SCONE/artifacts/housing/20260930_c1_l1')
def run(_context):
 a=c.Application.get();doc=next(x for x in a.documents if x.name=='MARC v4 Body v5 BOX v36');d=f.Design.cast(doc.products.itemByProductType('DesignProductType'))
 assert not doc.isModified
 (OUT/'baseline.json').write_text(json.dumps(all_sig(d),ensure_ascii=False,indent=2),encoding='utf-8')
 assert d.exportManager.execute(d.exportManager.createFusionArchiveExportOptions(str(OUT/'SOURCE_v36_before_C1_L1.f3d')))
 print('v36 backup and complete geometry/pose baseline captured')
