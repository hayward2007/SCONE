from pathlib import Path
import hashlib, json, math, itertools

root = Path.cwd()
doc = root / "docs/30-scone-v3-design-plan.md"
source = doc.read_text(encoding="utf-8")
block = source.split("<!-- marc-params:start -->", 1)[1]
block = block.split("<!-- marc-params:end -->", 1)[0]
p = json.loads(block.split("```json", 1)[1].split("```", 1)[0])
g, leg, body = p["geometry"], p["leg"], p["body"]
val, mat = p["validation"], p["material_preliminary"]
rad, deg = math.radians, math.degrees

# ---- 1. crank -> effective link
A = leg["link_a_mm"] + leg["link_b_mm"]*math.cos(rad(leg["gamma_crank_deg"]))
B = leg["link_b_mm"]*math.sin(rad(leg["gamma_crank_deg"]))
L = math.hypot(A, B); phi = math.atan2(B, A)
ground = body["frame_bottom_z_mm"] - leg["H_reference_mm"]
axle_z = ground + g["R_tread_mm"]
beta0 = math.asin((axle_z-leg["stage1_axis_z_mm"])/L)
q1_0 = beta0 - phi
q2_0 = rad(leg["alpha_abs_reference_deg"]) - q1_0
arm0 = L*math.cos(beta0)

# ---- 2. profile / support function
half_tread = g["th_tread_deg"]/2
half_material = half_tread + g["d_lobe_deg"]
assert half_material < 180 and g["d_lobe_deg"] > 0
def rho(c):
    e = max(abs(c)-half_tread, 0.0)
    return g["R_tread_mm"] - (g["R_tread_mm"]-g["r_lobe_tip_mm"])*e/g["d_lobe_deg"]
def scan(step):
    n = round(2*half_material/step)
    chi = sorted(set([-half_material+2*half_material*i/n for i in range(n+1)]+[-half_tread, half_tread]))
    pts = [(rho(c)*math.cos(rad(c)), rho(c)*math.sin(rad(c))) for c in chi]
    m = round(360/step); hs = []
    for j in range(m):
        a = 2*math.pi*j/m; ca, sa = math.cos(a), math.sin(a)
        hs.append(-min(x*sa+z*ca for x, z in pts))
    return hs
def summ(hs):
    m = len(hs)
    return {"heave_mm": max(hs)-min(hs), "h_max_mm": max(hs), "h_min_mm": min(hs),
            "ideal_distance_integral_mm": sum(hs)*2*math.pi/m}
coarse, fine = scan(g["profile_step_deg"]), scan(g["profile_step_deg"]/2)
prof_c, prof_f = summ(coarse), summ(fine)
tip_chord = 2*g["r_lobe_tip_mm"]*math.sin(rad(180-half_material))
roll_only = 2*math.pi*g["R_tread_mm"]*(g["th_tread_deg"]/360.0)

# ---- 3. loads
G = val["g_m_s2"]; m_tot = p["mass_candidate"]["total_kg"]; Wt = m_tot*G
W = {"4pt": Wt/4, "3pt": Wt/3, "trot2": Wt/2}
kf = val["design_factor"]/val["drive_efficiency"]

# ---- 4. posture table
def posture(bdeg):
    b = rad(bdeg); az = L*math.sin(b); gr = az-g["R_tread_mm"]
    a = L*math.cos(b)
    return {"beta_deg": bdeg, "q1_deg": deg(b-phi), "H_mm": body["frame_bottom_z_mm"]-gr,
            "arm_mm": a, "wheelbase_mm": 2*(leg["stage1_axis_x_mm"]+a),
            "foot_x_mm": leg["stage1_axis_x_mm"]+a, "ground_z_mm": gr,
            "tau_3pt_Nm": kf*W["3pt"]*a/1000, "tau_4pt_Nm": kf*W["4pt"]*a/1000,
            "tau_trot_Nm": kf*W["trot2"]*a/1000,
            "front_rear_gap_mm": 2*(leg["stage1_axis_x_mm"]+a)-2*g["R_tread_mm"]}
betas = sorted(set(leg["beta_mode_deg"]+[deg(beta0)]))
postures = [posture(bd) for bd in betas]
stall1 = p["actuator_catalog"]["XM430-W350-T"]["stall_Nm"]
for s in postures:
    for k in ("3pt","4pt","trot"):
        s["pct_"+k] = 100*s["tau_%s_Nm"%k]/stall1

# ---- 5. stride / speed
def stride(bdeg, sweep_deg=15.0):
    b = rad(bdeg); return {"beta_deg": bdeg, "dx_dbeta_mm_per_rad": L*abs(math.sin(b)),
                           "stride_mm": 2*L*abs(math.sin(b))*rad(sweep_deg)}
strides = [stride(x) for x in (-50.406090, -70.0, -80.0)]
w210 = p["actuator_catalog"]["XM430-W210-T"]
dist_rev = prof_f["ideal_distance_integral_mm"]
roll_speed_max = dist_rev/1000*(w210["no_load_rad_s"]/(2*math.pi))

# ---- 6. sector out-of-plane section (rim + two lips), b = axial width
b_s, t_r, t_l, h_l = g["b_sector_mm"], g["t_rim_mm"], g["t_lip_mm"], g["h_lip_mm"]
I_sec = (t_r*b_s**3)/12 + 2*(h_l*t_l**3/12 + h_l*t_l*((b_s-t_l)/2)**2)
Z_sec = I_sec/(b_s/2)
Lm = g["R_tread_mm"]-g["r_hub_mm"]          # moment arm, load at tread
Ld = g["r_lobe_tip_mm"]-g["r_hub_mm"]       # plate cantilever length
sigma5 = 5*W["3pt"]*Lm/Z_sec
SF5 = mat["PLA_strength_MPa"]/sigma5
defl3 = 3*W["3pt"]*Ld**3/(3*mat["PLA_E_MPa"]*I_sec)

# ---- 7. feet, support polygon, static margins
T2 = body["track_half_y_mm"]
def feet_at(a):
    return {"FL": (leg["stage1_axis_x_mm"]+a,  T2), "FR": (leg["stage1_axis_x_mm"]+a, -T2),
            "RL": (-(leg["stage1_axis_x_mm"]+a), T2), "RR": (-(leg["stage1_axis_x_mm"]+a), -T2)}
def margin(poly, com=(0.0, 0.0)):
    n = len(poly); best = float("inf"); inside = True
    cx, cy = com
    area2 = sum(poly[i][0]*poly[(i+1) % n][1]-poly[(i+1) % n][0]*poly[i][1] for i in range(n))
    sgn = 1.0 if area2 >= 0 else -1.0
    for i in range(n):
        x1, y1 = poly[i]; x2, y2 = poly[(i+1) % n]
        ex, ey = x2-x1, y2-y1; ln = math.hypot(ex, ey)
        cr = sgn*(ex*(cy-y1)-ey*(cx-x1))/ln
        if cr < 0: inside = False
        best = min(best, abs(cr))
    return best if inside else -best
F = feet_at(arm0)
poly4 = [F["FL"], F["FR"], F["RR"], F["RL"]]          # CCW-ish rectangle
m4 = margin(poly4)
tri = {}
for lift in ("FL", "FR", "RL", "RR"):
    keys = [k for k in ("FL", "FR", "RR", "RL") if k != lift]
    tri[lift] = margin([F[k] for k in keys])
h_com = body["com_z_candidate_mm"]-ground
tip_roll = deg(math.atan2(T2, h_com))
tip_pitch = deg(math.atan2(leg["stage1_axis_x_mm"]+arm0, h_com))
def roll_for_margin(mm): return deg(math.atan2(mm, h_com))


# ---- 7b. static crawl: re-yaw the three support legs
reach = leg["stage1_axis_x_mm"]-leg["steer_axis_x_mm"]+arm0
STEER = {"FL": (leg["steer_axis_x_mm"], T2), "FR": (leg["steer_axis_x_mm"], -T2),
         "RL": (-leg["steer_axis_x_mm"], T2), "RR": (-leg["steer_axis_x_mm"], -T2)}
SGN = {"FL": 1, "FR": 1, "RL": -1, "RR": -1}
def foot_yaw(k, psi_deg):
    s = SGN[k]; a = rad(psi_deg)
    return (STEER[k][0]+s*reach*math.cos(a), STEER[k][1]+s*reach*math.sin(a))
sep_min = 2*g["R_tread_mm"]+val["moving_leg_clearance_mm"]
same_side_yaw_limit = None
for a in range(0, 91):
    if math.dist(foot_yaw("FL", a), foot_yaw("RL", -a)) < sep_min:
        same_side_yaw_limit = a-1; break
lim = int(leg["yaw_search_deg"][1])
crawl = {}
for lift in ("FL", "FR", "RL", "RR"):
    keys = [k for k in ("FL", "FR", "RR", "RL") if k != lift]
    best = None
    for a in range(-lim, lim+1, 5):
        f0 = foot_yaw(keys[0], a)
        for b_ in range(-lim, lim+1, 5):
            f1 = foot_yaw(keys[1], b_)
            if math.dist(f0, f1) < sep_min: continue
            for c_ in range(-lim, lim+1, 5):
                f2 = foot_yaw(keys[2], c_)
                if math.dist(f1, f2) < sep_min or math.dist(f0, f2) < sep_min: continue
                mg = margin([f0, f1, f2])
                if best is None or mg > best[0]: best = (mg, a, b_, c_)
    crawl[lift] = {"legs": keys, "yaw_deg": list(best[1:]), "margin_mm": best[0]}

# ---- 8. rolling phase study: rigid body resting on 4 support functions
n_h = len(fine); step = 360.0/n_h
def h_at(deg_q): return fine[int(round((deg_q % 360)/step)) % n_h]
def rest(hs, pts):
    # minimise plane height at origin subject to plane >= h_i at each foot
    best = None
    idx = range(4)
    for c in itertools.combinations(idx, 3):
        M = [[1.0, pts[i][0], pts[i][1]] for i in c]; rhs = [hs[i] for i in c]
        det = (M[0][0]*(M[1][1]*M[2][2]-M[1][2]*M[2][1])
               -M[0][1]*(M[1][0]*M[2][2]-M[1][2]*M[2][0])
               +M[0][2]*(M[1][0]*M[2][1]-M[1][1]*M[2][0]))
        if abs(det) < 1e-9: continue
        def solve(col):
            N = [row[:] for row in M]
            for r in range(3): N[r][col] = rhs[r]
            return (N[0][0]*(N[1][1]*N[2][2]-N[1][2]*N[2][1])
                    -N[0][1]*(N[1][0]*N[2][2]-N[1][2]*N[2][0])
                    +N[0][2]*(N[1][0]*N[2][1]-N[1][1]*N[2][0]))/det
        z0, a, bb = solve(0), solve(1), solve(2)
        ok = all(z0+a*pts[i][0]+bb*pts[i][1] >= hs[i]-1e-7 for i in idx)
        if ok and (best is None or z0 < best[0]-1e-9):
            n_contact = sum(1 for i in idx if abs(z0+a*pts[i][0]+bb*pts[i][1]-hs[i]) < 1e-6)
            best = (z0, a, bb, n_contact)
    return best
pts4 = [F["FL"], F["FR"], F["RL"], F["RR"]]
def phase_study(offsets, name):
    z0s, pitch, roll, nc = [], [], [], []
    for k in range(0, 720, 2):
        q = k/2.0
        hs = [h_at(q+o) for o in offsets]
        z0, a, bb, n = rest(hs, pts4)
        z0s.append(z0); pitch.append(deg(math.atan(a))); roll.append(deg(math.atan(bb))); nc.append(n)
    return {"name": name, "offsets_deg": offsets,
            "heave_mm": max(z0s)-min(z0s),
            "pitch_abs_max_deg": max(abs(x) for x in pitch),
            "roll_abs_max_deg": max(abs(x) for x in roll),
            "contacts_min": min(nc), "contacts_max": max(nc)}
# order of pts4: FL, FR, RL, RR
phases = [phase_study([0,0,0,0], "all in phase"),
          phase_study([0,0,180,180], "front pair / rear pair 180"),
          phase_study([0,180,0,180], "left pair / right pair 180"),
          phase_study([0,180,180,0], "diagonal pairs 180"),
          phase_study([0,90,180,270], "90 deg spacing")]

# ---- 9. stair hook torque, steering torque
tau_stair = W["3pt"]*g["R_tread_mm"]/1000
w210_stall = w210["stall_Nm"]
trail = leg["stage1_axis_x_mm"]-leg["steer_axis_x_mm"]+arm0
tau_steer_loaded = 1.0*W["3pt"]*trail/1000
tau_steer_swing = 1.0*0.15*W["3pt"]*trail/1000

# ---- 10. regressions
assert abs(prof_c["heave_mm"]-prof_f["heave_mm"]) < val["profile_convergence_mm"]
assert abs(prof_f["heave_mm"]-18.56355259248) < 0.1
assert abs(tip_chord-86.10377228215) < 0.1
assert abs(A*math.sin(q1_0)+B*math.cos(q1_0)+leg["stage1_axis_z_mm"]-axle_z) < 1e-9
assert abs(L-195.307179) < 1e-4 and abs(deg(phi)-13.770009) < 1e-4
assert abs(deg(beta0)+50.406090) < 1e-4 and abs(arm0-124.477485) < 1e-4
assert abs(m4-T2) < 1e-9
assert all(abs(v) < 1e-6 for v in tri.values())
assert min(s["front_rear_gap_mm"] for s in postures) > val["moving_leg_clearance_mm"]
assert min(v["margin_mm"] for v in crawl.values()) >= val["static_margin_3pt_target_mm"]
assert abs(phases[3]["heave_mm"]) < 1e-6 and phases[3]["contacts_min"] >= 3

out = {
 "document_sha256": hashlib.sha256(doc.read_bytes()).hexdigest(),
 "scope": "D0 mathematical regression for the 4-leg platform; not fabrication acceptance",
 "derived": {"L_link_mm": L, "phi_off_deg": deg(phi), "beta_reference_deg": deg(beta0),
             "q1_reference_deg": deg(q1_0), "q2_reference_deg": deg(q2_0), "arm_mm": arm0,
             "ground_z_mm": ground, "axle_z_mm": axle_z,
             "frame_y_mm": 2*T2+body["frame_end_allowance_mm"],
             "wheelbase_mm": 2*(leg["stage1_axis_x_mm"]+arm0), "track_mm": 2*T2},
 "profile": {"coarse": prof_c, "fine": prof_f, "tip_chord_mm": tip_chord,
             "arc_only_distance_mm": roll_only,
             "distance_per_rev_mm": dist_rev,
             "tip_pivot_share_pct": 100*tip_chord/dist_rev,
             "lobe_roll_mm": dist_rev-roll_only-tip_chord},
 "loads_N": W, "torque_factor": kf,
 "postures": postures, "strides": strides,
 "roll_speed_max_m_s": roll_speed_max,
 "sector_section": {"I_mm4": I_sec, "Z_mm3": Z_sec, "moment_arm_mm": Lm,
                    "cantilever_mm": Ld, "sigma_5W_MPa": sigma5, "SF_5W": SF5,
                    "deflection_3W_mm": defl3},
 "support": {"feet_mm": F, "margin_4pt_mm": m4, "margin_3pt_mm": tri,
             "com_height_mm": h_com, "tipover_roll_deg": tip_roll, "tipover_pitch_deg": tip_pitch,
             "body_roll_for_80mm_deg": roll_for_margin(80.0),
             "same_side_yaw_limit_deg": same_side_yaw_limit,
             "crawl_with_yaw": crawl},
 "rolling_phase": phases,
 "stair_hook_torque_Nm": tau_stair, "stair_hook_pct_W210": 100*tau_stair/w210_stall,
 "steer": {"trail_mm": trail, "tau_loaded_Nm": tau_steer_loaded, "tau_swing_Nm": tau_steer_swing,
           "MX28_stall_Nm": p["actuator_catalog"]["MX-28AT"]["stall_Nm"]},
 "regression_status": "PASS"}
print(json.dumps(out, ensure_ascii=False, indent=1))
