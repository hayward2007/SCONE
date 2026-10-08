import json, math, os, sys
from pathlib import Path
snap = json.loads(Path(os.path.expanduser("~/mnt/SCONE/tmp/marc-watch/review_snapshot.json")).read_text())
com = snap['root']['com_mm']; M = snap['root']['mass_g']/1000.0
GROUND = -273.0; FOOT_X = 232.47748464411413; T2 = 160.0; STEER_X = 50.0
REACH = 108.0 - STEER_X + 124.47748464411413
G = 9.81; SEPMIN = 2*122.5 + 10.0

def margin(poly, c):
    n=len(poly); a2=sum(poly[i][0]*poly[(i+1)%n][1]-poly[(i+1)%n][0]*poly[i][1] for i in range(n))
    sg=1.0 if a2>=0 else -1.0; best=1e18; inside=True
    for i in range(n):
        x1,y1=poly[i]; x2,y2=poly[(i+1)%n]; ex,ey=x2-x1,y2-y1; ln=math.hypot(ex,ey)
        cr=sg*(ex*(c[1]-y1)-ey*(c[0]-x1))/ln
        if cr<0: inside=False
        best=min(best,abs(cr))
    return best if inside else -best

F = {"FL": (FOOT_X, T2), "FR": (FOOT_X, -T2), "RL": (-FOOT_X, T2), "RR": (-FOOT_X, -T2)}
STEER = {"FL": (STEER_X, T2), "FR": (STEER_X, -T2), "RL": (-STEER_X, T2), "RR": (-STEER_X, -T2)}
SGN = {"FL": 1, "FR": 1, "RL": -1, "RR": -1}
def foot_yaw(k, psi):
    s = SGN[k]; a = math.radians(psi)
    return (STEER[k][0]+s*REACH*math.cos(a), STEER[k][1]+s*REACH*math.sin(a))

c = (com[0], com[1])
h_com = com[2] - GROUND
res = {
 "measured": {"mass_kg_lower_bound": round(M,6), "com_mm": com, "com_height_above_ground_mm": round(h_com,3)},
 "doc_assumption": {"com_z_candidate_mm": 12.0, "com_height_mm": 285.0,
                    "tipover_roll_deg": 29.310007, "tipover_pitch_deg": 39.204486,
                    "margin_4pt_mm": 160.0, "margin_3pt_yaw_mm": 110.968},
}
res["tipover"] = {
  "roll_deg_min": round(math.degrees(math.atan2(T2-abs(com[1]), h_com)),3),
  "roll_deg_max": round(math.degrees(math.atan2(T2+abs(com[1]), h_com)),3),
  "pitch_deg_min": round(math.degrees(math.atan2(FOOT_X-abs(com[0]), h_com)),3),
  "pitch_deg_max": round(math.degrees(math.atan2(FOOT_X+abs(com[0]), h_com)),3),
}
poly4 = [F["FL"], F["FR"], F["RR"], F["RL"]]
res["margin_4pt_mm"] = round(margin(poly4, c), 4)
tri = {}
for lift in ("FL","FR","RL","RR"):
    keys=[k for k in ("FL","FR","RR","RL") if k!=lift]
    tri[lift]=round(margin([F[k] for k in keys], c), 4)
res["margin_3pt_fixed_yaw_mm"] = tri
crawl={}
for lift in ("FL","FR","RL","RR"):
    keys=[k for k in ("FL","FR","RR","RL") if k!=lift]
    best=None
    for a in range(-60,61,5):
        f0=foot_yaw(keys[0],a)
        for b in range(-60,61,5):
            f1=foot_yaw(keys[1],b)
            if math.dist(f0,f1)<SEPMIN: continue
            for d in range(-60,61,5):
                f2=foot_yaw(keys[2],d)
                if math.dist(f1,f2)<SEPMIN or math.dist(f0,f2)<SEPMIN: continue
                m=margin([f0,f1,f2], c)
                if best is None or m>best[0]: best=(m,a,b,d)
    crawl[lift]={"legs":keys,"yaw_deg":list(best[1:]),"margin_mm":round(best[0],4)}
res["margin_3pt_yaw_replaced_mm"] = crawl
res["worst_3pt_yaw_replaced_mm"] = round(min(v["margin_mm"] for v in crawl.values()),4)
res["target_static_margin_mm"] = 80.0
res["verdict"] = {
 "margin_4pt_pass": res["margin_4pt_mm"] >= 80.0,
 "margin_3pt_pass": res["worst_3pt_yaw_replaced_mm"] >= 80.0,
 "note": "CoM from the measured CAD subtotal; missing parts (SPDB, forks, horns, bearings, fasteners, wiring) are not included",
}
print(json.dumps(res, ensure_ascii=False, indent=1))
