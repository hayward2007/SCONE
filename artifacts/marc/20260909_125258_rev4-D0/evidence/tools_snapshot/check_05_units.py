"""Independent translated, rotated box verifies Fusion inertia signs and reference point."""
from fusion_common import *

def run(ctx):
    app,source,d,folder,p=guard(ctx)
    material=d.materials.itemByName('MARC_v4_PLA_D0')
    test=app.documents.add(core.DocumentTypes.FusionDesignDocumentType)
    test.name='MARC_UNITS_TEMP_NOT_A_ROBOT_PART'
    try:
        td=fusion.Design.cast(app.activeProduct)
        mat=td.materials.addByCopy(material,'MARC_units_PLA_1180')
        temp=box_temp((-50,50,-100,100,-150,150));angle=math.radians(30)
        tr=matrix((40,50,60),(math.cos(angle),math.sin(angle),0),(-math.sin(angle),math.cos(angle),0),(0,0,1))
        mgr=fusion.TemporaryBRepManager.get();assert mgr.transform(temp,tr)
        b=add_base(td.rootComponent,'known_box',temp,mat);ph=b.physicalProperties
        mass=7.08;diag=[mass*(20**2+30**2)/12,mass*(10**2+30**2)/12,mass*(10**2+20**2)/12]
        co=math.cos(angle);si=math.sin(angle)
        ic=[[co*co*diag[0]+si*si*diag[1],co*si*(diag[0]-diag[1]),0],[co*si*(diag[0]-diag[1]),si*si*diag[0]+co*co*diag[1],0],[0,0,diag[2]]]
        cc=[4,5,6];i0=[[ic[i][j]+mass*((sum(v*v for v in cc) if i==j else 0)-cc[i]*cc[j]) for j in range(3)] for i in range(3)]
        raw=ph.getXYZMomentsOfInertia();assert raw[0]
        vals=list(raw[1:]);expected=[i0[0][0],i0[1][1],i0[2][2],i0[0][1],i0[1][2],i0[0][2]]
        # Installed Fusion returns signed tensor off-diagonals, verified against this fixture.
        error=max(abs(a-b) for a,b in zip(vals,expected))
        out={'step_id':'B05-units','status':'PASS' if error<1e-6 and abs(ph.mass-mass)<1e-7 else 'FAIL','raw_kg_cm2':vals,'expected_tensor_about_origin_kg_cm2':expected,'max_error':error,'mass_kg':ph.mass,'expected_mass_kg':mass,'com_cm':ph.centerOfMass.asArray(),'convention':'XYZ values are signed tensor entries about component origin. Subtract the parallel-axis term and multiply all entries by 1e-4.'}
        write(folder/'evidence/B05_inertia_units.json',out)
        return out
    finally:
        test.close(False)
        source.activate()
