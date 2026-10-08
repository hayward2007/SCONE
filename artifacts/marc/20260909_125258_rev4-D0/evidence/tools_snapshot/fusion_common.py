"""Shared Fusion D0 helpers. mm in inputs, cm only at API boundaries."""
from pathlib import Path
import hashlib, json, math
import adsk.core as core
import adsk.fusion as fusion

def write(path, data):
    p = Path(path)
    t = p.with_suffix(p.suffix + '.part')
    t.write_text(json.dumps(data, ensure_ascii=False, indent=2)+'\n')
    t.replace(p)

def load(ctx):
    folder=Path(ctx['run_dir'])
    assert hashlib.sha256((Path(ctx['repo_root'])/'docs/30-scone-v3-design-plan.md').read_bytes()).hexdigest() == ctx['source_sha256'], 'Document changed; prepare a new run'
    assert hashlib.sha256((folder/'params.json').read_bytes()).hexdigest() == ctx['params_sha256'], 'Parameters changed; prepare a new run'
    return folder, json.loads((folder/'params.json').read_text())

def guard(ctx):
    folder,p=load(ctx)
    app=core.Application.get()
    doc=app.activeDocument
    if ctx.get('document_id'):
        if ctx['document_id'].startswith('/') and doc.dataFile.id.startswith('urn:'):
            assert doc.attributes.itemByName('MARC','run_dir').value == str(folder)
            assert doc.attributes.itemByName('MARC','source_sha256').value == ctx['source_sha256']
            ctx['initial_save_id']=ctx['document_id']
            ctx['document_id']=doc.dataFile.id
            ctx['creation_id']=doc.creationId
            write(folder/'context.json',ctx)
        assert doc.dataFile.id == ctx['document_id'], 'Document ID mismatch'
    else:
        assert doc.creationId == ctx['creation_id'], 'Unsaved document ID mismatch'
    assert doc.attributes.itemByName('MARC','run_dir').value == str(folder)
    return app,doc,fusion.Design.cast(app.activeProduct),folder,p

def point(x,y,z=0): return core.Point3D.create(x/10,y/10,z/10)
def vec(x,y,z): return core.Vector3D.create(x,y,z)
def value(s): return core.ValueInput.createByString(str(s))

def matrix(origin=(0,0,0), x=(1,0,0), y=(0,1,0), z=(0,0,1)):
    m=core.Matrix3D.create()
    m.setWithCoordinateSystem(point(*origin),vec(*x),vec(*y),vec(*z))
    assert abs(m.determinant-1)<1e-8
    return m

def attr(obj,key,value): obj.attributes.add('MARC',key,str(value))

def component(root,name,transform=None):
    assert not any(o.component.name == name for o in root.occurrences), 'Already created: '+name
    occ=root.occurrences.addNewComponent(transform or matrix())
    occ.component.name=name
    attr(occ.component,'owner','MARC_rev4_D0')
    return occ

def sketch_poly(comp,name,pts,plane=None):
    sk=comp.sketches.add(plane or comp.xYConstructionPlane)
    sk.name=name
    sk.isComputeDeferred=True
    first=None; prev=None
    for x,y in pts:
        p=point(x,y)
        if prev is None:
            first=p; prev=p; continue
        line=sk.sketchCurves.sketchLines.addByTwoPoints(prev,p)
        line.isFixed=True
        prev=line.endSketchPoint
    line=sk.sketchCurves.sketchLines.addByTwoPoints(prev,first)
    line.isFixed=True
    sk.isComputeDeferred=False
    assert sk.profiles.count == 1, (name, sk.profiles.count)
    sk.isVisible=False
    return sk

def extrude(comp,sk,name,thickness,center=0,operation=None,targets=None):
    op=operation if operation is not None else fusion.FeatureOperations.NewBodyFeatureOperation
    inp=comp.features.extrudeFeatures.createInput(sk.profiles.item(0),op)
    inp.setSymmetricExtent(value(str(thickness)+' mm'),True)
    if center:
        inp.startExtent=fusion.OffsetStartDefinition.create(value(str(center)+' mm'))
    if targets: inp.participantBodies=targets
    f=comp.features.extrudeFeatures.add(inp); f.name=name
    return f

def rect(x0,y0,x1,y1): return [(x0,y0),(x1,y0),(x1,y1),(x0,y1)]

def box_temp(bounds):
    x0,x1,y0,y1,z0,z1=bounds
    bb=core.OrientedBoundingBox3D.create(point((x0+x1)/2,(y0+y1)/2,(z0+z1)/2),vec(1,0,0),vec(0,1,0),(x1-x0)/10,(y1-y0)/10,(z1-z0)/10)
    return fusion.TemporaryBRepManager.get().createBox(bb)

def boolean(target,tool,op='union'):
    kind={'union':fusion.BooleanTypes.UnionBooleanType,'cut':fusion.BooleanTypes.DifferenceBooleanType,'intersect':fusion.BooleanTypes.IntersectionBooleanType}[op]
    assert fusion.TemporaryBRepManager.get().booleanOperation(target,tool,kind), 'Boolean failed: '+op
    return target

def add_base(comp,name,body,material=None):
    f=comp.features.baseFeatures.add(); f.name=name
    assert f.startEdit()
    try: b=comp.bRepBodies.add(body,f)
    finally: f.finishEdit()
    b=comp.bRepBodies.item(comp.bRepBodies.count-1)
    b.name=name
    if material: b.material=material
    attr(b,'geometry_method','Exact D0 solid rebuilt from params; base feature does not update automatically')
    return b
