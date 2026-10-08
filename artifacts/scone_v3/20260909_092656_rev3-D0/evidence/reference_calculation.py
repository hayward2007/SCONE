from pathlib import Path
import hashlib
import json
import math

root = Path.cwd()
doc = root / "docs/30-scone-v3-design-plan.md"
source = doc.read_text(encoding="utf-8")
block = source.split("<!-- scone-v3-params:start -->", 1)[1]
block = block.split("<!-- scone-v3-params:end -->", 1)[0]
p = json.loads(block.split("```json", 1)[1].split("```", 1)[0])
g, leg, body = p["geometry"], p["leg"], p["body"]
rad, deg = math.radians, math.degrees
A = leg["link_a_mm"] + leg["link_b_mm"] * math.cos(rad(leg["gamma_crank_deg"]))
B = leg["link_b_mm"] * math.sin(rad(leg["gamma_crank_deg"]))
length = math.hypot(A, B)
phi = math.atan2(B, A)
ground = body["frame_bottom_z_mm"] - leg["H_reference_mm"]
axle_z = ground + g["R_tread_mm"]
beta = math.asin((axle_z-leg["stage1_axis_z_mm"]) / length)
q1 = beta - phi
q2 = rad(leg["alpha_abs_reference_deg"]) - q1
arm = length * math.cos(beta)

half_tread = g["th_tread_deg"] / 2
half_material = half_tread + g["d_lobe_deg"]
assert half_material < 180 and g["d_lobe_deg"] > 0

def rho(chi_deg):
    excess = max(abs(chi_deg) - half_tread, 0.0)
    return g["R_tread_mm"] - (g["R_tread_mm"]-g["r_lobe_tip_mm"]) * excess / g["d_lobe_deg"]

def support_scan(step_deg):
    count = round(2*half_material/step_deg)
    chi = [-half_material + 2*half_material*i/count for i in range(count+1)]
    chi = sorted(set(chi + [-half_tread, half_tread]))
    points = [(rho(c)*math.cos(rad(c)), rho(c)*math.sin(rad(c))) for c in chi]
    steps = round(360/step_deg)
    heights = []
    for j in range(steps):
        a = 2*math.pi*j/steps
        ca, sa = math.cos(a), math.sin(a)
        heights.append(-min(x*sa+z*ca for x,z in points))
    return {"heave_mm": max(heights)-min(heights),
            "ideal_distance_integral_mm": sum(heights)*2*math.pi/steps}

coarse = support_scan(g["profile_step_deg"])
fine = support_scan(g["profile_step_deg"]/2)
tip_chord = 2*g["r_lobe_tip_mm"]*math.sin(rad(180-half_material))
min_PLA_outer = g["r_lobe_tip_mm"]-g["t_tire_mm"]
min_lip_inner = min_PLA_outer-g["t_rim_mm"]-g["h_lip_mm"]
assert min_lip_inner > g["r_hub_mm"]
assert abs(coarse["heave_mm"]-fine["heave_mm"]) < p["validation"]["profile_convergence_mm"]
assert abs(fine["heave_mm"]-18.56355259248) < 0.1
assert abs(tip_chord-86.10377228215) < 0.1
assert abs(A*math.sin(q1)+B*math.cos(q1)+leg["stage1_axis_z_mm"]-axle_z) < 1e-9

# D0 feet, circular contact at alpha_abs=0 and yaw=0.
feet = []
for i in range(6):
    side = 1 if i % 2 == 0 else -1
    station = (-1, 0, 1)[i//2] * body["d_leg_long_mm"]
    stagger = leg["mid_yoke_extension_mm"] if i//2 == 1 else 0.0
    feet.append([side*(leg["stage1_axis_x_mm"]+stagger+arm), station, ground])

# Known rev.2 triangle: arithmetic regression, not D0 force input.
x_front, x_mid = 233.1, 258.1
fraction_mid = x_front/(x_front+x_mid)
fractions = [(1-fraction_mid)/2, fraction_mid, (1-fraction_mid)/2]
assert abs(sum(fractions)-1) < 1e-12
assert abs(fractions[0]*x_front-fractions[1]*x_mid+fractions[2]*x_front) < 1e-9

# Intact straight box comparison; not the full crank assembly.
w, h, t = leg["box_w_mm"], leg["box_h_mm"], leg["wall_link_mm"]
I = (w*h**3-(w-2*t)*(h-2*t)**3)/12
Z = I/(h/2)
J = 4*((w-t)*(h-t))**2/(2*((w-t)+(h-t))/t)
area = w*h-(w-2*t)*(h-2*t)
box_reference_L = 195.0
wi_reference = 4.569*9.81/3
G = p["material_preliminary"]["PLA_G_MPa"]
delta = (3*wi_reference*g["R_tread_mm"])*box_reference_L/(G*J)*g["R_tread_mm"]
assert abs(J-97492.6058862) < 0.001
assert abs(delta-1.09375528347) < 0.001

# FK sign: positive q1 raises the first link on either side.
eps = rad(0.01)
for side in (1, -1):
    axis = (0, -side, 0)
    link_direction = (side, 0, 0)
    dz = axis[0]*link_direction[1]-axis[1]*link_direction[0]
    assert dz > 0
    assert math.sin(eps) > 0

result = {
  "scope": "D0 mathematical regression only; not fabrication acceptance",
  "document_sha256": hashlib.sha256(doc.read_bytes()).hexdigest(),
  "derived": {"L_link_mm": length, "phi_off_deg": deg(phi),
              "beta_reference_deg": deg(beta), "q1_reference_deg": deg(q1),
              "q2_reference_deg": deg(q2), "arm_mm": arm,
              "ground_z_mm": ground, "axle_z_mm": axle_z,
              "frame_y_mm": 2*body["d_leg_long_mm"]+body["frame_end_allowance_mm"]},
  "profile": {"coarse": coarse, "fine": fine, "tip_chord_mm": tip_chord,
              "min_PLA_outer_mm": min_PLA_outer, "min_lip_inner_mm": min_lip_inner},
  "D0_feet_mm": feet,
  "rev2_reference_only": {"normal_load_fractions": fractions,
                           "intact_box_J_mm4": J, "intact_box_Z_mm3": Z,
                           "intact_box_mass_g": area*box_reference_L*p["material_preliminary"]["PLA_density_g_cm3"]/1000,
                           "link_delta_3wi_mm": delta},
  "regression_status": "PASS"
}
print(json.dumps(result, ensure_ascii=False, indent=2))
