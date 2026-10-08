import adsk.core, adsk.fusion, json
from pathlib import Path

OUT = Path('/Users/hayward_kim/Developer/SCONE/artifacts/electronics/20261007_power_review')

def pnt(p):
    return [round(v * 10, 6) for v in (p.x, p.y, p.z)]

def bbox(b):
    return {'min': pnt(b.minPoint), 'max': pnt(b.maxPoint)}

def run(_context):
    app = adsk.core.Application.get()
    doc = app.activeDocument
    d = adsk.fusion.Design.cast(app.activeProduct)
    root = d.rootComponent
    data = {'document': doc.name, 'id': doc.dataFile.id if doc.dataFile else None,
            'version': doc.dataFile.versionNumber if doc.dataFile else None,
            'modified': doc.isModified, 'root': root.name,
            'mass_g': root.physicalProperties.mass * 1000, 'occurrences': [], 'root_bodies': []}
    for b in root.bRepBodies:
        data['root_bodies'].append({'name': b.name, 'bounds': bbox(b.boundingBox)})
    selected = []
    for o in root.allOccurrences:
        row = {'path': o.fullPathName, 'component': o.component.name,
               'visible': o.isVisible, 'transform': o.transform2.asArray(), 'bodies': []}
        interesting = any(s in o.fullPathName.lower() for s in ['batt', 'power', 'pcb', 'tray', 'base_link', 'lid', 'box', 'carrier'])
        for b in o.bRepBodies:
            br = {'name': b.name, 'bounds': bbox(b.boundingBox), 'solid': b.isSolid,
                  'material': b.material.name if b.material else None}
            if interesting:
                br['planes'] = [{'bounds': bbox(f.boundingBox), 'point': pnt(f.pointOnFace),
                                 'normal': list(f.geometry.normal.asArray()), 'area_mm2': f.area * 100}
                                for f in b.faces if f.geometry.objectType == adsk.core.Plane.classType()]
            row['bodies'].append(br)
        data['occurrences'].append(row)
        if interesting and row['bodies']:
            selected.append({'path': row['path'], 'visible': row['visible'],
                             'bodies': [{k:v for k,v in b.items() if k != 'planes'} for b in row['bodies']]})
    (OUT / 'fusion_inventory.json').write_text(json.dumps(data, ensure_ascii=False, indent=2))
    print(json.dumps({k:v for k,v in data.items() if k not in ['occurrences', 'root_bodies']}, ensure_ascii=False))
    print(json.dumps(selected, ensure_ascii=False))
