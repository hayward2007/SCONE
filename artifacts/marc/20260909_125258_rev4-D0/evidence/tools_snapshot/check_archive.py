"""Open the exported F3D in Fusion, check native content, then close only that temporary copy."""
from fusion_common import *

def run(ctx):
    app,source,d,folder,p=guard(ctx)
    opts=app.importManager.createFusionArchiveImportOptions(str(folder/'cad/MARC-PARAM-D0.f3d'))
    test=app.importManager.importToNewDocument(opts)
    assert test and test!=source
    try:
        td=fusion.Design.cast(app.activeProduct);root=td.rootComponent
        reg=json.loads((folder/'joints.json').read_text());axes=[]
        for row in reg['joints']:
            j=root.asBuiltJoints.itemByName(row['native_name']);assert j
            axis=j.jointMotion.rotationAxisVector
            dot=axis.dotProduct(vec(*row['axis_world']))
            axes.append({'name':j.name,'axis_dot_expected':dot});assert dot>.9999
        solid_count=0;bad=[]
        for o in root.allOccurrences:
            for b in o.component.bRepBodies:
                solid_count+=1
                if not b.isSolid or b.volume<=0:bad.append(o.fullPathName+'/'+b.name)
        expected_names={v['name'] for v in json.loads((folder/'cad/parameter_manifest.json').read_text())}
        assert {u.name for u in td.userParameters}==expected_names
        result={'status':'PASS' if not bad and root.asBuiltJoints.count==12 else 'FAIL','method':'Fusion ImportManager.importToNewDocument on exported native archive; temporary document closed without saving','parameters':td.userParameters.count,'native_revolute_joints':root.asBuiltJoints.count,'solid_instances_including_study':solid_count,'invalid_bodies':bad,'joint_axes':axes}
        write(folder/'evidence/F3D_roundtrip.json',result)
        return result
    finally:
        test.close(False)
        source.activate()
