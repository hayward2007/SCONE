#!/usr/bin/env python3
"""SCONEv3 부채꼴 기하 — heave / throat / dx_rev.  docs/30 §2 재생성."""
import math, numpy as np
R = math.radians
Ro = 122.5

def heave(rho_fn, n=2880, nq=1441):
    psi = np.linspace(-180, 180, n, endpoint=False)
    rho = rho_fn(psi); ok = rho > 0
    ps = np.radians(psi[ok]); rr = rho[ok]
    q = np.radians(np.linspace(0, 360, nq))
    Z = np.max(rr[None, :] * (-np.sin(ps[None, :] + q[:, None])), axis=1)
    return Z.max(), Z.min(), Z.max() - Z.min()

def profile(occ, delta=0.0, r_tip=None, spiral=True):
    a = occ / 2.0
    def f(psi):
        r = np.zeros_like(psi); ab = np.abs(psi)
        r[ab <= a] = Ro
        if delta > 0:
            m = (ab > a) & (ab <= a + delta)
            r[m] = Ro + (r_tip - Ro) * ((ab[m] - a) / delta) if spiral else r_tip
        return r
    return f

def closed_form(occ, delta, r_tip):
    th_open = 360 - occ - 2 * delta
    return (Ro - r_tip * math.cos(R(th_open / 2)),
            2 * r_tip * math.sin(R(th_open / 2)),
            2 * math.pi * Ro * occ / 360.0)

if __name__ == "__main__":
    print("A. 단일 반경 부채꼴")
    diag = math.hypot(200, 240)
    print(f"{'occ':>5}{'open':>6}{'heave':>8}{'L_open':>9}{'q_open':>8}")
    for occ in [225, 250, 270, 290, 300]:
        h = heave(profile(occ))[2]
        Lo = 2 * Ro * math.sin(R((360 - occ) / 2))
        print(f"{occ:5.0f}{360-occ:6.0f}{h:8.1f}{Lo:9.1f}{Lo/diag:8.3f}")
    print("\nB. 나선 로브")
    print(f"{'occ':>5}{'delta':>7}{'r_tip':>7}{'heave':>8}{'throat':>8}{'dx_rev':>8}")
    for occ, d, rt in [(225,40,112.5),(225,45,112.5),(225,50,112.5),(225,45,107.5),(215,50,112.5)]:
        hv, th, dx = closed_form(occ, d, rt)
        hv_num = heave(profile(occ, d, rt))[2]
        assert abs(hv - hv_num) < 0.3, (hv, hv_num)
        print(f"{occ:5.0f}{d:7.1f}{rt:7.1f}{hv:8.2f}{th:8.2f}{dx:8.0f}")
    print("\n채택: occ=225 delta=45 r_tip=112.5 -> heave 18.56 / throat 86.10 / dx_rev 481")
