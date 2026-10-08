#!/usr/bin/env python3
"""SCONEv3 링크 길이 / 자세 / 토크 / 보폭 / 간섭 / 안정마진.  docs/30 §6,§7 재생성."""
import math
import numpy as np
g = 9.81
Ro = 122.5
B_SECTOR = 20.0
CLEAR = 10.0
S_TAU, ETA = 1.5, 0.75
TAU1_STALL, TAU2_STALL, TAU_STEER_STALL = 4.1, 3.0, 2.5     # W350 / W210 / MX-28AT @12V
LINK_A, GAMMA, LINK_B = 90.0, 25.0, 110.0
X_P, X_S, STAGGER = 108.0, 50.0, 25.0
MASS = 4.569
H_STAND = 255.0
L_LINK = math.sqrt(LINK_A**2 + LINK_B**2 + 2*LINK_A*LINK_B*math.cos(math.radians(GAMMA)))
PHI_OFF = math.degrees(math.atan2(LINK_B*math.sin(math.radians(GAMMA)),
                                  LINK_A + LINK_B*math.cos(math.radians(GAMMA))))
Wi = MASS*g/3.0

def theta_for(H, L=L_LINK): return math.degrees(math.asin((105.0-H)/L))
def arm_for(H, L=L_LINK):   return L*math.cos(math.radians(theta_for(H, L)))
def tau1(H, alpha=0.0, L=L_LINK):
    return Wi*(arm_for(H, L) + Ro*math.sin(math.radians(alpha)))/1000.0*S_TAU/ETA
def tau2(H, alpha):
    return abs(Wi*Ro*math.sin(math.radians(alpha)))/1000.0*S_TAU/ETA
def alpha_star(H, L=L_LINK):
    return math.degrees(math.asin(-3*arm_for(H, L)/(7.1*Ro)))
def L_swing(L=L_LINK, jmin=-110.0, jmax=115.0):
    return L*(math.sin(math.radians(jmax+PHI_OFF)) - math.sin(math.radians(jmin+PHI_OFF)))
def stride(H, psi, L=L_LINK):   return 2*(X_S+8.0+arm_for(H, L))*math.sin(math.radians(psi))
def e_env(psi, b=B_SECTOR):
    return Ro*abs(math.sin(math.radians(psi))) + 0.5*b*math.cos(math.radians(psi))
def psi_max(d_leg, b=B_SECTOR, c=CLEAR):
    ps = [p/10 for p in range(0, 901) if e_env(p/10, b) <= (d_leg-c)/2.0]
    return max(ps) if ps else 0.0
def margin(half_track, d_leg, stagger):
    P = np.array([[half_track, d_leg], [-(half_track+stagger), 0.0], [half_track, -d_leg]])
    c = P.mean(0); mm = 1e9
    for i in range(3):
        p, q = P[i], P[(i+1) % 3]
        d = q-p; n = np.array([-d[1], d[0]]); n /= np.linalg.norm(n)
        if np.dot(c-p, n) < 0: n = -n
        mm = min(mm, np.dot(-p, n))
    return mm

if __name__ == "__main__":
    print(f"L_link={L_LINK:.2f}  phi_off={PHI_OFF:.2f} deg  m={MASS} kg  W_i={Wi:.2f} N")
    print(f"L_swing={L_swing():.1f} mm (요구 230)   arm_max={TAU1_STALL*ETA/S_TAU/Wi*1000:.1f} mm")
    print(f"\n{'L':>7}{'theta':>8}{'arm':>7}{'t1_raw':>8}{'t1_dsg':>8}{'%':>5}"
          f"{'alpha*':>8}{'t1*':>7}{'%':>5}{'보폭@30':>9}{'v@1Hz':>8}{'L_swing':>9}")
    for L in [180, 195.31, 210, 225]:
        th = theta_for(H_STAND, L); a = arm_for(H_STAND, L)
        al = alpha_star(H_STAND, L); st = stride(H_STAND, 30, L)
        print(f"{L:7.2f}{th:8.2f}{a:7.1f}{Wi*a/1000:8.2f}{tau1(H_STAND,0,L):8.2f}"
              f"{100*tau1(H_STAND,0,L)/TAU1_STALL:5.0f}{al:8.1f}{tau1(H_STAND,al,L):7.2f}"
              f"{100*tau1(H_STAND,al,L)/TAU1_STALL:5.0f}{st:9.1f}{st/1000:8.3f}{L_swing(L):9.0f}")
    a = arm_for(H_STAND); al = alpha_star(H_STAND); sh = Ro*math.sin(math.radians(al))
    ht = X_P + a
    print(f"\n표준자세 H={H_STAND:.0f}: theta={theta_for(H_STAND):.2f} arm={a:.1f}")
    print(f"  half-track FR={ht:.1f} MID={ht+STAGGER:.1f} | 전폭 {2*(ht+STAGGER+Ro):.0f} mm")
    print(f"  alpha*={al:.1f} shift={sh:.1f} -> half-track {ht+sh:.1f}")
    print(f"\n{'d_leg':>7}{'psi_max':>9}{'margin st=0':>13}{'margin st=25':>14}")
    for d in [130, 150, 175, 200, 260]:
        print(f"{d:7.0f}{psi_max(d):9.1f}{margin(ht,d,0):13.1f}{margin(ht,d,STAGGER):14.1f}")
    print(f"  @alpha* (175/25): {margin(ht+sh,175,STAGGER):.1f} mm")
    trail = X_S + 8.0 + a
    print(f"\n트레일 {trail:.1f} mm | 지지중 조향 {Wi*trail/1000:.2f} | 스윙중(0.15W) "
          f"{0.15*Wi*trail/1000:.2f} | stall {TAU_STEER_STALL}")
