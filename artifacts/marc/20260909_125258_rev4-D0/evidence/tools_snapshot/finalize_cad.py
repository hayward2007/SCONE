"""Bind extrusion widths, verify parameter registry, save cloud and local CAD files."""
from fusion_common import *

def run(ctx):
    app,doc,d,folder,p=guard(ctx);root=d.rootComponent
    binding={'Rim':'geometry_b_sector_mm','Lip_minus':'geometry_t_lip_mm','Lip_plus':'geometry_t_lip_mm','Web':'geometry_t_web_mm','Hub_no_interface':'geometry_hub_boss_h_mm','TPU_effective_envelope':'geometry_b_sector_mm','Crank_outer_closed_section':'leg_box_w_mm','Crank_hollow_open_only_at_ends':'leg_box_w_mm - 2 * leg_wall_link_mm'}
    report=[]
    for prm in list(d.allParameters):
        if prm.objectType!='adsk::fusion::ModelParameter':continue
        name=prm.createdBy.name
        expr=None
        if prm.role=='AlongDistance':
            expr=binding.get(name)
            if name.startswith('Rib_'):expr='geometry_b_sector_mm - 2 * geometry_t_lip_mm'
        elif prm.role=='ProfileOffset' and name in ('Lip_minus','Lip_plus'):
            expr=('(-1)' if name=='Lip_minus' else '1')+' * (geometry_b_sector_mm - geometry_t_lip_mm) / 2'
        if expr:
            old=prm.value;prm.expression=expr
            assert abs(prm.value-old)<1e-9,(name,old,prm.value)
            report.append({'feature':name,'role':prm.role,'expression':prm.expression,'value_internal':prm.value})
    write(folder/'cad/feature_bindings.json',{'bindings':report,'limitation':'Polyline coordinates, planar crank shape, frame base features and assembly transforms are rebuilt from source-locked params. Only the listed extrusion widths and lip offsets update associatively.'})
    expected=json.loads((folder/'cad/parameter_manifest.json').read_text())
    required={row['name'] for row in expected};actual={u.name for u in d.userParameters};assert required==actual
    verify=[]
    for row in expected:
        u=d.userParameters.itemByName(row['name']);actual_unit=u.unit
        internal=row['input']/10 if row['unit']=='mm' else math.radians(row['input']) if row['unit']=='deg' else row['input']
        assert abs(u.value-internal)<1e-9
        assert actual_unit==row['unit'],(row['name'],actual_unit,row['unit'])
        verify.append({'name':u.name,'unit':u.unit,'expression':u.expression,'value_internal':u.value,'status':'PASS'})
    write(folder/'evidence/G00_parameter_match.json',{'status':'PASS','missing_keys':[],'unexpected_keys':[],'parameters':verify})
    for c in d.allComponents:
        for s in c.sketches:s.isVisible=False
        for plane in c.constructionPlanes:plane.isLightBulbOn=False
    for j in root.asBuiltJoints:j.isLightBulbOn=False
    for o in root.occurrences:
        if o.component.name.startswith('_STUDY'):o.isLightBulbOn=False
    app.activeViewport.visualStyle=core.VisualStyles.ShadedWithVisibleEdgesOnlyVisualStyle
    cam=app.activeViewport.camera
    cam.cameraType=core.CameraTypes.OrthographicCameraType
    cam.eye=point(750,-1000,650);cam.target=point(0,0,-75);cam.upVector=vec(0,0,1);cam.isFitView=True
    app.activeViewport.camera=cam;app.activeViewport.refresh()
    attr(doc,'geometry_status','D0 reference assembly; G02 CAD 12/12 PASS; P00 intersections unresolved; not fabrication-ready')
    attr(doc,'parametric_scope','Associative widths / offsets; profile, frame and layout regeneration requires source-locked script run')
    assert doc.save('MARC rev4 D0 geometry and twelve verified revolute joints. Unresolved P00 overlaps retained for review.')
    outputs=[]
    for kind,filename in [('f3d','MARC-PARAM-D0.f3d'),('step','MARC-PARAM-D0.step')]:
        path=folder/'cad'/filename
        opts=d.exportManager.createFusionArchiveExportOptions(str(path)) if kind=='f3d' else d.exportManager.createSTEPExportOptions(str(path))
        assert d.exportManager.execute(opts),kind+' export failed'
        assert path.exists() and path.stat().st_size>0
        outputs.append({'format':kind,'path':str(path),'bytes':path.stat().st_size,'sha256':hashlib.sha256(path.read_bytes()).hexdigest(),'scope':'Entire D0 document; STEP contains the visible 28 solids; native F3D also contains hidden study geometry. No production approval.'})
    document={'name':doc.name,'id':doc.dataFile.id,'creation_id':doc.creationId,'is_saved':doc.isSaved,'is_modified':doc.isModified,'folder':doc.dataFile.parentFolder.name,'exports':outputs}
    write(folder/'cad/document.json',document)
    ctx['creation_id']=doc.creationId;write(folder/'context.json',ctx)
    return {'step_id':'CAD-save','status':'PASS','document':document,'binding_count':len(report)}
