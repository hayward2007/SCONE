exec(open('/Users/hayward_kim/Developer/SCONE/artifacts/electronics/20260930_jetson_carrier/protect_v21.py').read())
def differences(a,b,path=''):
 if isinstance(a,dict) and isinstance(b,dict):
  out=[]
  for k in a.keys()|b.keys():
   if k not in a or k not in b:out.append([path+'/'+k,'missing'])
   else:out+=differences(a[k],b[k],path+'/'+k)
  return out
 if isinstance(a,list) and isinstance(b,list):
  if len(a)!=len(b):return [[path,'length',len(a),len(b)]]
  return [v for i,(x,y) in enumerate(zip(a,b)) for v in differences(x,y,path+'/'+str(i))]
 if isinstance(a,(int,float)) and isinstance(b,(int,float)):
  return [] if abs(a-b)<1e-6 else [[path,a,b]]
 return [] if a==b else [[path,a,b]]
def run(_context):
 a=c.Application.get();doc=next(x for x in a.documents if x.name.startswith('MARC v4 Body v5 BOX v'));d=f.Design.cast(doc.products.itemByProductType('DesignProductType'))
 before=json.loads((OUT/'protected_v21.json').read_text(encoding='utf-8'));after=sig(d);diff=differences(before,after)
 out={'document':doc.name,'protected_occurrences':len(before['occurrences']),'unchanged':not diff,'differences':diff,'warnings':after['warnings'],'new_bodies':[{'component':o.fullPathName,'name':b.name,'solid':b.isSolid,'lumps':b.lumps.count} for o in d.rootComponent.allOccurrences if o.fullPathName.startswith('E10 ') for b in o.bRepBodies]}
 (OUT/'preservation_check.json').write_text(json.dumps(out,ensure_ascii=False,indent=2),encoding='utf-8');print(json.dumps({'unchanged':not diff,'protected_occurrences':out['protected_occurrences'],'differences':diff[:30],'new_bodies':len(out['new_bodies']),'warnings':out['warnings']},ensure_ascii=False))
