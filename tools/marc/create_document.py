"""Create the new, source-locked MARC Fusion document and parameter registry."""
from pathlib import Path
import json, math
from fusion_common import *

def run(ctx):
    folder,p=load(ctx)
    assert ctx['document_id'] is None, 'Run already owns a saved document'
    app=core.Application.get()
    if ctx.get('creation_id'):
        doc=app.activeDocument
        assert doc.creationId == ctx['creation_id']
        assert doc.attributes.itemByName('MARC','run_dir').value == str(folder)
        source=next(x for x in app.documents if x.isSaved and x.name=='SCONEv3 v8')
        source_id=source.dataFile.id
        destination=source.dataFile.parentFolder
    else:
        source=app.activeDocument
        source_id=source.dataFile.id
        destination=source.dataFile.parentFolder
        doc=app.documents.add(core.DocumentTypes.FusionDesignDocumentType)
        doc.name='MARC-PARAM-D0'
        ctx['creation_id']=doc.creationId
        attr(doc,'run_dir',folder)
        attr(doc,'source_sha256',ctx['source_sha256'])
        attr(doc,'design_revision',p['design_revision'])
        attr(doc,'status','D0 CANDIDATE - interfaces incomplete')
        write(folder/'context.json',ctx)
    d=fusion.Design.cast(app.activeProduct)
    d.designType=fusion.DesignTypes.ParametricDesignType
    d.unitsManager.distanceDisplayUnits= fusion.DistanceUnits.MillimeterDistanceUnits
    expected=[]
    for group in ('geometry','leg','body'):
        for key,val in p[group].items():
            if isinstance(val,(float,int)):
                unit='mm' if key.endswith('_mm') else 'deg' if key.endswith('_deg') else ''
                name=group+'_'+key
                expression=str(val)+(' '+unit if unit else '')
                u=d.userParameters.itemByName(name) or d.userParameters.add(name,value(expression),unit,group+'.'+key+'; docs/30 rev4-D0 CANDIDATE')
                expected.append({'path':group+'.'+key,'name':name,'unit':unit,'expression':u.expression,'value_internal':u.value,'input':val})
    write(folder/'cad/parameter_manifest.json',expected)
    lib=app.materialLibraries.itemById('C1EEA57C-3F56-45FC-B8CB-A9EC46A9994C')
    source_mat=lib.materials.itemById('PrismMaterial-022')
    materials=[]
    for name,density in [('MARC_v4_PLA_D0',1180),('MARC_v4_TPU_envelope_D0',494)]:
        material=d.materials.itemByName(name) or d.materials.addByCopy(source_mat,name)
        props=[]
        for prop in material.materialProperties:
            item={'id':prop.id,'name':prop.name,'type':prop.objectType}
            if isinstance(prop,core.FloatProperty):
                item.update(units=prop.units,value=prop.value)
                if prop.id=='structural_Density':
                    assert prop.units=='KilogramPerCubicMeter', prop.units
                    prop.value=density
                    item['value_set']=prop.value
            props.append(item)
        material.description='D0 preliminary material. TPU density is effective envelope mass only; no validated stiffness.'
        materials.append({'name':name,'id':material.id,'properties':props})
    write(folder/'cad/materials.json',materials)
    assert doc.saveAs('MARC-PARAM-D0',destination,'MARC v4 rev4-D0 from docs/30. Candidate geometry, interfaces unverified. Source SHA256 '+ctx['source_sha256'],'')
    ctx['document_id']=doc.dataFile.id
    write(folder/'context.json',ctx)
    write(folder/'cad/document.json',{'name':doc.name,'id':doc.dataFile.id,'creation_id':doc.creationId,'source_document_id':source_id,'folder':destination.name,'run_dir':str(folder)})
    return {'step_id':'B03-create','status':'PASS','document':doc.name,'document_id':doc.dataFile.id,'parameters':len(expected)}
