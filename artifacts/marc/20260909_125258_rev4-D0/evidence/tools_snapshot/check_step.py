"""STEP readback of visible D0 solids; no joints or study volumes are expected."""
from fusion_common import *

def run(ctx):
    app,source,d,folder,p=guard(ctx)
    options=app.importManager.createSTEPImportOptions(str(folder/'cad/MARC-PARAM-D0.step'))
    test=app.importManager.importToNewDocument(options);assert test
    try:
        td=fusion.Design.cast(app.activeProduct);r=td.rootComponent
        bodies=list(r.bRepBodies)
        for o in r.allOccurrences:bodies.extend(list(o.bRepBodies))
        invalid=[b.name for b in bodies if not b.isSolid or b.volume<=0]
        result={'status':'PASS' if len(bodies)==28 and not invalid else 'FAIL','visible_solid_instances':len(bodies),'invalid_bodies':invalid,'study_geometry_included':False,'expected_visible_count':28,'method':'Fusion STEP import to a new unsaved document; native model retains the hidden 14 study solids separately'}
        write(folder/'evidence/STEP_roundtrip.json',result)
        return result
    finally:test.close(False);source.activate()
