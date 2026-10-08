exec(open('/Users/hayward_kim/Developer/SCONE/artifacts/electronics/20260930_jetson_carrier/geometry.py').read())
def run(_context):
 a=c.Application.get();d=f.Design.cast(next(x for x in a.documents if x.name=='MARC v4 Body v5 BOX v20').products.itemByProductType('DesignProductType'))
 body=next(b for o in d.rootComponent.allOccurrences for b in o.bRepBodies if b.name=='Body v5 FRONT')
 out=[]
 for n,q,k in generate(a):
  if q.boundingBox.intersects(body.boundingBox):
   j=T.copy(q);op(j,T.copy(body),f.BooleanTypes.IntersectionBooleanType)
   if j.isSolid and j.volume>1e-6:out.append({'part':n,'box':bounds(j),'lumps':[bounds(l) for l in j.lumps]})
 print(json.dumps(out))
