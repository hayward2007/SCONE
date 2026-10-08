exec(open('/Users/hayward_kim/Developer/SCONE/artifacts/electronics/20260930_u2d2_docked/geometry.py').read())
def run(_context):
 a=c.Application.get();doc=next(x for x in a.documents if x.name.startswith('MARC v4 Body v5 BOX v'));d=f.Design.cast(doc.products.itemByProductType('DesignProductType'));src=find_parts(d)
 parts=proposed(d);excluded={src[k].fullPathName for k in('P1','P2','P4','U2D2')}
 old=[(o.fullPathName+'/'+b.name,T.copy(b)) for o in d.rootComponent.allOccurrences if o.fullPathName not in excluded for b in o.bRepBodies if b.isVisible and b.isSolid and b.boundingBox.intersects(box(100,244,0,56,30,145).boundingBox)]
 collisions=[]
 for n,q,k in parts:
  for on,ob in old:
   v=interference(q,ob)
   if v>.001:
    j=T.copy(q);op(j,ob,f.BooleanTypes.IntersectionBooleanType);collisions.append([n,on,round(v,6),bounds(j)])
 internal=[]
 for i,(n,q,k) in enumerate(parts):
  for n2,q2,k2 in parts[i+1:]:
   if k==k2=='U2D2':continue
   v=interference(q,q2)
   if v>.001:internal.append([n,n2,round(v,6)])
 out={'existing_collisions':collisions,'internal_collisions':internal,'parts':[{'name':n,'kind':k,'box':bounds(q),'lumps':q.lumps.count,'volume_mm3':q.volume*1000} for n,q,k in parts],'matrix':list(dock_matrix().asArray()),'api_move':f.MoveFeatures.createInput2.__doc__,'api_free':f.MoveFeatureInput.defineAsFreeMove.__doc__}
 (OUT/'candidate_check.json').write_text(json.dumps(out,ensure_ascii=False,indent=2),encoding='utf-8');print(json.dumps(out,ensure_ascii=False))
