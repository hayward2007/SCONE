"""B01 independent polygon support convergence, no Fusion imports."""
from pathlib import Path
import argparse, hashlib, json, math

def run(ctx):
    f=Path(ctx['run_dir']);p=json.loads((f/'params.json').read_text());g=p['geometry']
    assert hashlib.sha256((Path(ctx['repo_root'])/'docs/30-scone-v3-design-plan.md').read_bytes()).hexdigest()==ctx['source_sha256']
    end=g['th_tread_deg']/2+g['d_lobe_deg'];half=g['th_tread_deg']/2
    def pts(step):
        aa=sorted(set([-end+i*step for i in range(round(2*end/step)+1)]+[-half,half]))
        out=[]
        for a in aa:
            r=g['R_tread_mm']-(g['R_tread_mm']-g['r_lobe_tip_mm'])*max(abs(a)-half,0)/g['d_lobe_deg']
            out.append((r*math.cos(math.radians(a)),r*math.sin(math.radians(a))))
        return out
    coarse=pts(g['profile_step_deg']);fine=pts(g['profile_step_deg']/2)
    def supports(pp):
        result=[]
        for i in range(1440):
            a=math.radians(i/4);s=math.sin(a);c=math.cos(a)
            result.append(max(-x*s-z*c for x,z in pp))
        return result
    hc=supports(coarse);hf=supports(fine)
    diff=max(abs(a-b) for a,b in zip(hc,hf));heave=max(hf)-min(hf)
    chord=2*g['r_lobe_tip_mm']*math.sin(math.radians(180-end))
    status='PASS' if diff<p['validation']['profile_convergence_mm'] and abs(heave-18.56355259248)<.1 and abs(chord-86.10377228215)<.1 else 'FAIL'
    out={'step_id':'B01','status':status,'source_sha256':ctx['source_sha256'],'params_sha256':ctx['params_sha256'],'max_support_difference_mm':diff,'heave_mm':heave,'tip_chord_mm':chord,'alpha_step_deg':.25,'coarse_vertices':len(coarse),'fine_vertices':len(fine),'method':'Compare both polylines at identical 0.25 deg absolute angles; linear-segment minimum occurs at a vertex.'}
    (f/'evidence/B01_profile.json').write_text(json.dumps(out,indent=2)+'\n')
    return out

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--run-dir',required=True);args=parser.parse_args();ctx=json.loads((Path(args.run_dir)/'context.json').read_text());print(json.dumps(run(ctx),indent=2))
