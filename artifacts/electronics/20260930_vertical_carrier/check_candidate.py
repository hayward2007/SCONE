exec(open('/Users/hayward_kim/Developer/SCONE/artifacts/electronics/20260930_vertical_carrier/carrier_geometry.py').read())
def run(_context):
 a=c.Application.get();doc=next(x for x in a.documents if x.name=='MARC v4 Body v5 BOX v19');d=f.Design.cast(doc.products.itemByProductType('DesignProductType'))
 parts=generate(a)
 old=[(o.fullPathName+'/'+b.name,T.copy(b)) for o in d.rootComponent.allOccurrences if o.isVisible for b in o.bRepBodies if b.isVisible and b.isSolid and b.boundingBox.intersects(box(-97,16,-49,55,83,141).boundingBox)]
 old += [('ROOT/'+b.name,T.copy(b)) for b in d.rootComponent.bRepBodies if b.isVisible and b.isSolid and b.boundingBox.intersects(box(-97,16,-49,55,83,141).boundingBox)]
 collisions=[]
 for name,q,kind in parts:
  for on,ob in old:
   vol=interference(q,ob)
   if vol>.001:collisions.append({'new':name,'old':on,'mm3':round(vol,5)})
 internal=[]
 for i,(n,q,k) in enumerate(parts):
  for n2,q2,k2 in parts[i+1:]:
   if k==k2=='official' and (n.startswith('ROBOTIS PHB')==n2.startswith('ROBOTIS PHB')):continue
   vol=interference(q,q2)
   if vol>.001:internal.append({'a':n,'b':n2,'mm3':round(vol,5)})
 out={'existing_collisions':collisions,'internal_collisions':internal,'parts':[{'name':n,'box':bounds(q),'volume_mm3':q.volume*1000,'kind':k,'lumps':q.lumps.count} for n,q,k in parts]}
 (OUT/'candidate_check.json').write_text(json.dumps(out,ensure_ascii=False,indent=2),encoding='utf-8')
 print(json.dumps({'existing_collisions':collisions,'internal_collisions':internal,'parts':len(parts)},ensure_ascii=False))
