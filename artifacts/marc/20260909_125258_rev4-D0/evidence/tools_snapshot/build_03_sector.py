"""D0 sector with native sketches/extrusions; exact 0.5-degree polyline."""
from fusion_common import *

def clip(poly, nx,ny,offset):
    out=[]
    for a,b in zip(poly,poly[1:]+poly[:1]):
        da=nx*a[0]+ny*a[1]-offset; db=nx*b[0]+ny*b[1]-offset
        if da <= 1e-9: out.append(a)
        if (da < -1e-9 and db > 1e-9) or (da > 1e-9 and db < -1e-9):
            t=da/(da-db); out.append((a[0]+t*(b[0]-a[0]),a[1]+t*(b[1]-a[1])))
    return out

def run(ctx):
    app,doc,d,folder,p=guard(ctx); g=p['geometry']
    assert not d.rootComponent.occurrences.count, 'B03 already started; inspect existing owned geometry before retry'
    for name,density in [('MARC_v4_PLA_D0',1180),('MARC_v4_TPU_envelope_D0',494)]:
        m=d.materials.itemByName(name)
        prop=core.FloatProperty.cast(m.materialProperties.itemById('structural_Density'))
        assert prop.units=='KilogramPerCubicMeter'
        prop.value=density
        assert abs(prop.value-density)<1e-6
        if name=='MARC_v4_PLA_D0':
            for pr in m.materialProperties:
                if pr.id.startswith('structural_Young_modulus'): core.FloatProperty.cast(pr).value=3200000
                if pr.id.startswith('structural_Shear_modulus'): core.FloatProperty.cast(pr).value=1230000
                if pr.id=='structural_Minimum_tensile_strength': core.FloatProperty.cast(pr).value=48000
        m.description='D0 preliminary density only accepted for mass. Inherited unspecified material fields MUST NOT be used for FEA.'
    pla=d.materials.itemByName('MARC_v4_PLA_D0'); tpu=d.materials.itemByName('MARC_v4_TPU_envelope_D0')
    derived=json.loads((folder/'calculations.json').read_text())['derived']
    o=component(d.rootComponent,'Spin_Module_D0',matrix((p['leg']['stage1_axis_x_mm']+derived['arm_mm'],p['body']['track_half_y_mm'],derived['axle_z_mm']), (1,0,0),(0,0,1),(0,-1,0)))
    attr(o,'instance_id','FL_SPIN'); c=o.component
    write(folder/'cad/owned_components.json',{'spin':{'token':c.entityToken,'occurrence_token':o.entityToken}})
    half=g['th_tread_deg']/2; end=half+g['d_lobe_deg']; step=g['profile_step_deg']
    angles=sorted(set([-end+i*step for i in range(round(2*end/step)+1)]+[-half,half]))
    def radius(a): return g['R_tread_mm']-(g['R_tread_mm']-g['r_lobe_tip_mm'])*max(abs(a)-half,0)/g['d_lobe_deg']
    def curve(offset=0,r=None):
        return [((radius(a)-offset if r is None else r)*math.cos(math.radians(a)),(radius(a)-offset if r is None else r)*math.sin(math.radians(a))) for a in angles]
    def band(outer,inner): return outer+list(reversed(inner))
    rs=curve(g['t_tire_mm']); rr=curve(g['t_tire_mm']+g['t_rim_mm']); rl=curve(g['t_tire_mm']+g['t_rim_mm']+g['h_lip_mm'])
    width=g['b_sector_mm']
    specs=[('Rim',band(rs,rr),width,0),('Lip_minus',band(rr,rl),g['t_lip_mm'],-(width-g['t_lip_mm'])/2),('Lip_plus',band(rr,rl),g['t_lip_mm'],(width-g['t_lip_mm'])/2),('Web',band(rl,curve(r=g['r_hub_mm'])),g['t_web_mm'],0)]
    for name,poly,w,center in specs:
        sk=sketch_poly(c,name+'_profile',poly)
        extrude(c,sk,name,w,center)
        adsk_events()
    sk=c.sketches.add(c.xYConstructionPlane); sk.name='Hub_solid_placeholder'
    sk.sketchCurves.sketchCircles.addByCenterRadius(point(0,0),g['r_hub_mm']/10)
    sk.isVisible=False
    extrude(c,sk,'Hub_no_interface',g['hub_boss_h_mm'])
    for a in g['rib_angles_deg']:
        ca=math.cos(math.radians(a)); sa=math.sin(math.radians(a))
        poly=[(0,0)]+rs
        for nx,ny,offset in [(ca,sa,g['R_tread_mm']),(-ca,-sa,-g['r_hub_mm']+1),(-sa,ca,g['t_rib_mm']/2),(sa,-ca,g['t_rib_mm']/2)]: poly=clip(poly,nx,ny,offset)
        sk=sketch_poly(c,'Rib_'+str(a)+'_profile',poly)
        extrude(c,sk,'Rib_'+str(a),width-2*g['t_lip_mm'])
    bodies=[b for b in c.bRepBodies]
    coll=core.ObjectCollection.create()
    for b in bodies[1:]: coll.add(b)
    inp=c.features.combineFeatures.createInput(bodies[0],coll)
    inp.operation=fusion.FeatureOperations.JoinFeatureOperation
    f=c.features.combineFeatures.add(inp); f.name='Union_PLA_rim_lips_web_hub_7ribs'
    assert c.bRepBodies.count==1 and c.bRepBodies.item(0).isSolid
    b=c.bRepBodies.item(0); b.name='PLA_sector_D0_SOLID_HUB'; b.material=pla
    attr(b,'part_id','sector_PLA');attr(b,'mass_source','CAD_BULK')
    sk=sketch_poly(c,'TPU_contact_profile',band(curve(),rs))
    extrude(c,sk,'TPU_effective_envelope',width)
    b=c.bRepBodies.item(1); b.name='TPU_tire_D0_EFFECTIVE_ENVELOPE'; b.material=tpu
    attr(b,'part_id','tire_TPU');attr(b,'mass_source','ENVELOPE_EFFECTIVE')
    attr(c,'profile_rebuild','Profile points are fixed native sketch entities. Rebuild from a new source-locked params run after radius/angle changes.')
    result={'step_id':'B03','status':'PASS','profile_vertices':len(angles),'bodies':[{'name':b.name,'solid':b.isSolid,'volume_cm3':b.physicalProperties.volume,'mass_kg':b.physicalProperties.mass,'density_kg_m3':b.material.materialProperties.itemById('structural_Density').value} for b in c.bRepBodies]}
    assert all(b['solid'] and b['volume_cm3']>0 for b in result['bodies'])
    write(folder/'evidence/B03_sector.json',result)
    app.activeViewport.fit(); app.activeViewport.refresh()
    return result

def adsk_events():
    import adsk
    adsk.doEvents()
