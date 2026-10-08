exec(open('/Users/hayward_kim/Developer/SCONE/tools/housing/build_shell.py',encoding='utf-8').read().split('def run():')[0])
try:
 T=f.TemporaryBRepManager.get()
except RuntimeError:
 # Fusion can surface the previous failed ASM operation once on the next API call.
 T=f.TemporaryBRepManager.get()
def vec(x,y,z):return c.Vector3D.create(x,y,z)
def box(x0,x1,y0,y1,z0,z1):return T.createBox(c.OrientedBoundingBox3D.create(P((x0+x1)/2,(y0+y1)/2,(z0+z1)/2),vec(1,0,0),vec(0,1,0),(x1-x0)/10,(y1-y0)/10,(z1-z0)/10))
def cylinder(a,b,r):return T.createCylinderOrCone(P(*a),r/10,P(*b),r/10)
def boolean(a,b,op):
 assert T.booleanOperation(a,b,op),'Boolean failed'
 return a
def union(a,b):return boolean(a,b,f.BooleanTypes.UnionBooleanType)
def cut(a,b):return boolean(a,b,f.BooleanTypes.DifferenceBooleanType)
def intersect(a,b):return boolean(a,b,f.BooleanTypes.IntersectionBooleanType)
def compnew(root,name,reference=False):
 assert not root.occurrences.itemByName(name+':1'),'Already built '+name
 o=root.occurrences.addNewComponent(c.Matrix3D.create());o.component.name=name
 o.component.attributes.add('SCONE_HOUSING','owned','A01')
 o.component.attributes.add('SCONE_HOUSING','role','reference' if reference else 'manufactured')
 return o.component

def persist(comp,temps,color=(48,53,61),material='PrismMaterial-417',appearance='Graphite PA12'):
 app=c.Application.get();d=f.Design.cast(app.activeProduct)
 base=comp.features.baseFeatures.add();base.name=comp.name+' geometry';base.startEdit()
 for name,temp in temps:
  b=comp.bRepBodies.add(temp,base);b.name=name
 base.finishEdit()
 for b in comp.bRepBodies:finish_appearance(app,d,b,color,appearance,material)
 return list(comp.bRepBodies)
def cut_native(comp,target,temps,name):
 base=comp.features.baseFeatures.add();base.name=name+' tools';base.startEdit()
 for i,t in enumerate(temps):comp.bRepBodies.add(t,base).name=name+' tool '+str(i)
 base.finishEdit()
 tools=[b for b in comp.bRepBodies if b.name.startswith(name+' tool ')]
 inp=comp.features.combineFeatures.createInput(target,collection(tools));inp.operation=f.FeatureOperations.CutFeatureOperation;inp.isKeepToolBodies=False
 feat=comp.features.combineFeatures.add(inp);feat.name=name
 return feat
