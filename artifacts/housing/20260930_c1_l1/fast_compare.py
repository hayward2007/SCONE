exec(open('/Users/hayward_kim/Developer/SCONE/artifacts/housing/20260930_c1_l1/install.py').read().rsplit('def run(_context):',1)[0])
from functools import lru_cache
def fastbsig(b):return {'name':b.name,'box':bbox(b),'solid':b.isSolid,'faces':b.faces.count,'edges':b.edges.count,'appearance':b.appearance.name if b.appearance else None,'material':b.material.name if b.material else None,'light':b.isLightBulbOn}
def diff(a,b,path=''):
 if type(a)!=type(b):return [(path,a,b)]
 if isinstance(a,dict):return [v for k in a.keys()|b.keys() for v in diff(a.get(k),b.get(k),path+'/'+str(k))]
 if isinstance(a,list):return [(path,'length '+str(len(a)),'length '+str(len(b)))] if len(a)!=len(b) else [v for i,(x,y) in enumerate(zip(a,b)) for v in diff(x,y,path+'/'+str(i))]
 return [] if a==b else [(path,a,b)]
def run(_context):
 global bsig
 a,doc,d=source();bsig=fastbsig
 current=all_sig(d);baseline=json.loads((OUT/'baseline.json').read_text(encoding='utf-8'))
 for o in baseline['occurrences'].values():
  for b in o['bodies']:b.pop('vertices',None)
 for b in baseline['root']:b.pop('vertices',None)
 changes=diff(baseline,current)
 (OUT/'fast_comparison.json').write_text(json.dumps({'changes':changes,'current':current},ensure_ascii=False,indent=2),encoding='utf-8')
 print(json.dumps({'modified':doc.isModified,'changes':changes},ensure_ascii=False))
