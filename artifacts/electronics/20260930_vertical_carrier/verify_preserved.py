exec(open('/Users/hayward_kim/Developer/SCONE/artifacts/electronics/20260930_vertical_carrier/capture_source.py').read().split('def run(_context):')[0])
def run(_context):
 app=c.Application.get();doc=next(x for x in app.documents if x.name=='MARC v4 Body v5 BOX v19');d=f.Design.cast(doc.products.itemByProductType('DesignProductType'))
 old=json.loads((OUT/'protected_before.json').read_text(encoding='utf-8'));new=protected(d)
 diff=[];numerical=[]
 def compare(a,b,path):
  if isinstance(a,dict):
   for k in a:
    if k not in b:diff.append([path+'/'+k,'missing'])
    else:compare(a[k],b[k],path+'/'+k)
  elif isinstance(a,list):
   if len(a)!=len(b):diff.append([path,'length',len(a),len(b)])
   else:
    for i,(x,y) in enumerate(zip(a,b)):compare(x,y,path+'/'+str(i))
  elif isinstance(a,(int,float)) and not isinstance(a,bool) and isinstance(b,(int,float)):
   delta=abs(a-b)
   if delta>2e-7:diff.append([path,a,b,delta])
   elif delta:numerical.append([path,delta])
  elif a!=b:diff.append([path,a,b])
 compare(old,new,'source')
 additions=sorted(set(new['occurrences'])-set(old['occurrences']))
 assert all(p.startswith('E9 Rear vertical electronics carrier') for p in additions)
 out={'protected_occurrences':len(old['occurrences']),'added_occurrences':additions,'differences':diff,'numeric_rounding_only':numerical,'source_geometry_transforms_visibility_material_appearance_unchanged':not diff}
 (OUT/'protected_after.json').write_text(json.dumps(new,ensure_ascii=False,indent=2),encoding='utf-8');(OUT/'preservation_check.json').write_text(json.dumps(out,ensure_ascii=False,indent=2),encoding='utf-8')
 print(json.dumps({'protected_occurrences':out['protected_occurrences'],'differences':diff,'rounding_fields':len(numerical),'added_occurrences':len(additions),'preserved':not diff},ensure_ascii=False))
