import adsk.core, adsk.fusion, json
from pathlib import Path

OUT = Path('/Users/hayward_kim/Developer/SCONE/artifacts/electronics/20261007_power_review')

def box(mgr, size):
    l,w,h = size
    return mgr.createBox(adsk.core.OrientedBoundingBox3D.create(
        adsk.core.Point3D.create(0.07,0,(5.81+h/2)/10),
        adsk.core.Vector3D.create(1,0,0), adsk.core.Vector3D.create(0,1,0), l/10,w/10,h/10))

def overlap(a,b):
    return all(getattr(a.maxPoint,k)>getattr(b.minPoint,k)+1e-7 and
               getattr(b.maxPoint,k)>getattr(a.minPoint,k)+1e-7 for k in ('x','y','z'))

def run(_context):
    app=adsk.core.Application.get()
    root=adsk.fusion.Design.cast(app.activeProduct).rootComponent
    assert app.activeDocument.dataFile.versionNumber == 63
    mgr=adsk.fusion.TemporaryBRepManager.get()
    existing=[]
    for o in root.allOccurrences:
        for b in o.bRepBodies:
            if b.isVisible and b.isSolid and 'V03 battery' not in o.fullPathName:
                existing.append((o.fullPathName,b))
    cases=[('CNHL16000',[185,45,77],False),('NE10000_rotated',[175,42,49],False),
           ('EP10000_rotated',[160,44,46],False),('CNHL10000_rotated',[177,45,49],False),
           ('current_geometric_limit',[185.98,45.98,77.48],False),
           ('modified_bay_186x90x100',[185.98,90,100],True),
           ('modified_bay_186x100x110',[185.98,100,110],True)]
    results=[]
    for name,size,modified in cases:
        probe=box(mgr,size)
        collisions=[]
        for path,b in existing:
            if modified and ('BATTERY GUIDE' in path or 'POWER PCB' in path):
                continue
            if not overlap(probe.boundingBox,b.boundingBox):
                continue
            result=mgr.copy(probe)
            assert mgr.booleanOperation(result,mgr.copy(b),adsk.fusion.BooleanTypes.IntersectionBooleanType)
            if result.faces.count and result.isSolid and result.volume>1e-7:
                collisions.append({'path':path,'body':b.name,'intersection_mm3':round(result.volume*1000,6)})
        results.append({'name':name,'size_xyz_mm':size,'replacement_of_guides_and_PCB_required':modified,
                        'collisions':collisions})
    data={'document':app.activeDocument.name,'version':63,'modified_after':app.activeDocument.isModified,
          'method':'temporary BRep intersections only; static installed battery boxes; excludes original battery',
          'results':results}
    (OUT/'fit_results.json').write_text(json.dumps(data,ensure_ascii=False,indent=2))
    print(json.dumps(data,ensure_ascii=False))
