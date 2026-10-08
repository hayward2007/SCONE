"""P00 exact solid intersections at the reference pose; this is not path acceptance."""
from fusion_common import *
import itertools

def run(ctx):
    app,doc,d,folder,p=guard(ctx);root=d.rootComponent;mgr=fusion.TemporaryBRepManager.get()
    bodies=[]
    for o in root.allOccurrences:
        path=o.fullPathName
        if '_STUDY' in path:continue
        ai=o.attributes.itemByName('MARC','instance_id');inst=ai.value if ai else path
        # Native bodies transformed once using the root-context occurrence transform.
        for b in o.component.bRepBodies:
            tb=mgr.copy(b);assert mgr.transform(tb,o.transform2)
            bodies.append({'id':inst+'/'+b.name,'body':tb,'box':tb.boundingBox,'is_envelope':'ENVELOPE' in b.name or 'Yoke' in b.name,'is_tire':'TPU_tire' in b.name,'instance':inst})
    intersections=[];errors=[];allowed=[];tested=0
    for a,b in itertools.combinations(bodies,2):
        if a['instance']==b['instance'] and {a['is_tire'],b['is_tire']}=={True,False} and 'SPIN' in a['instance']:
            allowed.append({'pair':[a['id'],b['id']],'mode':'PLA outer surface to TPU inner surface, coincident zero-volume interface','pose':'P00 reference'});continue
        ba,bb=a['box'],b['box']
        overlaps=[min(ba.maxPoint.asArray()[k],bb.maxPoint.asArray()[k])-max(ba.minPoint.asArray()[k],bb.minPoint.asArray()[k]) for k in range(3)]
        if min(overlaps)<=1e-7:continue
        tested+=1
        try:
            tb=mgr.copy(a['body'])
            if not mgr.booleanOperation(tb,b['body'],fusion.BooleanTypes.IntersectionBooleanType):
                errors.append({'pair':[a['id'],b['id']],'error':'Fusion intersection operation returned false'});continue
            volume=tb.volume
            if volume>1e-7:
                intersections.append({'pair':[a['id'],b['id']],'intersection_cm3':volume,'classification':'ENVELOPE_OVERLAP' if a['is_envelope'] or b['is_envelope'] else 'SOLID_INTERSECTION','pose':'P00','status':'FAIL'})
        except Exception as exc:errors.append({'pair':[a['id'],b['id']],'error':str(exc)})
    ground=json.loads((folder/'calculations.json').read_text())['derived']['ground_z_mm']
    support=[{'id':x['id'],'min_z_mm':x['box'].minPoint.z*10,'gap_to_reference_ground_mm':x['box'].minPoint.z*10-ground} for x in bodies if x['is_tire']]
    for x in support:allowed.append({'pair':[x['id'],'reference_ground_z_'+str(ground)],'mode':'Tire tangent contact; no penetration','pose':'P00'})
    # Front/rear conservative radial envelopes, straight reference and beta lower bound.
    derived=json.loads((folder/'calculations.json').read_text())['derived'];L=derived['L_link_mm']
    gaps=[{'beta_deg':beta,'cylinder_envelope_gap_mm':2*(p['leg']['stage1_axis_x_mm']+L*math.cos(math.radians(beta)))-2*p['geometry']['R_tread_mm']} for beta in [derived['beta_reference_deg'],-80.0]]
    out={'step_id':'B09-P00','status':'FAIL' if intersections else 'BLOCKED','body_count':len(bodies),'tested_positive_bbox_pairs':tested,'intersections':intersections,'errors':errors,'tire_ground':support,'front_rear_envelope_checks':gaps,'paths_status':'NOT_RUN: P00 has unresolved intersections; no diagnostic motion paths accepted','excluded_from_collision':'Missing motor geometries; hidden study fork and reserved spaces reported separately; native exact solids and payload/yoke envelopes included','blocking_inputs':['I02','I03','I05','I08']}
    write(folder/'evidence/G03_P00_intersections.json',out)
    write(folder/'allowed_contacts.json',{'stage':'D0','pairs':allowed,'note':'No yoke/link/hub overlaps are whitelisted. Frame rib-to-motor contact requires actual motor geometry.'})
    return out
