import adsk.core, adsk.fusion, json
from pathlib import Path

def run(_context):
    app=adsk.core.Application.get()
    root=adsk.fusion.Design.cast(app.activeProduct).rootComponent
    mgr=adsk.fusion.TemporaryBRepManager.get()
    obs=[]
    for o in root.allOccurrences:
        if any(t in o.fullPathName for t in ['V03 battery','BATTERY GUIDE','POWER PCB']): continue
        for b in o.bRepBodies:
            if b.isVisible and b.isSolid: obs.append((o.fullPathName,b))
    def collision(size):
        l,w,h=size
        probe=mgr.createBox(adsk.core.OrientedBoundingBox3D.create(adsk.core.Point3D.create(.07,0,(5.81+h/2)/10),adsk.core.Vector3D.create(1,0,0),adsk.core.Vector3D.create(0,1,0),l/10,w/10,h/10))
        hits=[]
        for path,b in obs:
            a=probe.boundingBox; bb=b.boundingBox
            if not all(getattr(a.maxPoint,k)>getattr(bb.minPoint,k)+1e-7 and getattr(bb.maxPoint,k)>getattr(a.minPoint,k)+1e-7 for k in ('x','y','z')):continue
            cp=mgr.copy(probe)
            assert mgr.booleanOperation(cp,mgr.copy(b),adsk.fusion.BooleanTypes.IntersectionBooleanType)
            if cp.faces.count and cp.isSolid and cp.volume>1e-7: hits.append({'path':path,'body':b.name})
        return hits
    cases=[]
    for h in [80,90,100,110]:
        lo,hi=40.,90.
        for i in range(10):
            mid=(lo+hi)/2
            if collision([185.98,mid,h]):hi=mid
            else:lo=mid
        cases.append({'height_mm':h,'length_mm':185.98,'max_width_static_mm':round(lo,2),'blocking_bodies':collision([185.98,hi,h])})
    results={'version':app.activeDocument.dataFile.versionNumber,'modified':app.activeDocument.isModified,
             'assumptions':'original battery, B3/B4 guides, P1/P2 removed for clearance calculation only; other visible geometry retained; no CAD edits',
             'cases':cases,'checks_46mm':[{'size':[185.98,46,h],'collisions':collision([185.98,46,h])} for h in [90,100,110]]}
    Path('/Users/hayward_kim/Developer/SCONE/artifacts/electronics/20261007_power_review/expansion_results.json').write_text(json.dumps(results,ensure_ascii=False,indent=2))
    print(json.dumps(results,ensure_ascii=False))
