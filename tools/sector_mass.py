#!/usr/bin/env python3
"""SCONEv3 부채꼴 질량·관성·면외 강성.  docs/30 §3 재생성."""
import math
Ro, Rt = 122.5, 112.5
OCC, DELTA = 225.0, 45.0
ARC = OCC + 2 * DELTA
RHO = {'AL6061':2.70,'AL7075':2.81,'PLA_infill':0.848,'TPU_infill':0.494,
       'CFRP':1.55,'PA6_CF':1.20}
E   = {'AL6061':69e3,'AL7075':71.7e3,'PLA_infill':2.6e3,'CFRP':70e3,'PA6_CF':6.5e3}
WEB_SOLID = 0.25          # v2 STL 캘리브레이션 0.312 대비 알루미늄 심화 포켓

def ann(ro, ri, ang, t):
    return math.pi * (ro*ro - ri*ri) * (ang/360.0) * t

def design(b, t_rim, t_web, mat, r_hub=22.0, web_solid=WEB_SOLID, tire_rho='TPU_infill'):
    r_wo = Rt - t_rim
    V_rim = ann(Rt, r_wo, ARC, b)
    V_web = ann(r_wo, r_hub, ARC, t_web) * web_solid
    V_hub = ann(r_hub, 9.75, 360, t_web + 6.0)
    m_sec = (V_rim + V_web + V_hub) * RHO[mat] / 1000.0
    V_tire = ann(Ro, Rt, ARC, b)
    m_tire = V_tire * RHO[tire_rho] / 1000.0
    I = ((V_rim*RHO[mat]/1e6)*0.5*(Rt**2+r_wo**2)
         + (V_web*RHO[mat]/1e6)*0.5*(r_wo**2+r_hub**2)
         + (V_hub*RHO[mat]/1e6)*0.5*(r_hub**2+95.06)
         + (V_tire*RHO[tire_rho]/1e6)*0.5*(Ro**2+Rt**2)) * 1e-6
    EI = E[mat] * (t_rim * b**3 / 12.0)
    return m_sec, m_tire, I, EI

if __name__ == "__main__":
    print("v2 실측: 부채꼴 117.4 g / 타이어 99.8 g / 다리당 217.2 g @225deg, Izz 2.02e-3")
    hdr = f"{'b':>4}{'t_rim':>6}{'t_web':>6}{'mat':>12}{'sec':>8}{'tire':>7}{'다리당':>8}{'Izz':>10}{'I/v2':>7}{'EI':>10}"
    print(hdr); print('-'*len(hdr))
    for b,tr,tw,m in [(44,3.9,8.0,'PLA_infill'),(20,3.0,3.0,'AL6061'),(15,3.0,2.5,'AL6061'),
                      (12,2.5,2.5,'AL6061'),(15,2.5,2.0,'AL7075'),(16,2.5,2.5,'CFRP'),
                      (16,3.0,3.0,'PA6_CF')]:
        ms,mt,I,EI = design(b,tr,tw,m)
        print(f"{b:4.0f}{tr:6.1f}{tw:6.1f}{m:>12}{ms:8.1f}{mt:7.1f}{ms+mt:8.1f}{I:10.2e}{I/2.02e-3:7.2f}{EI:10.2e}")
    print("\n채택: b=15 t_rim=3.0 t_web=2.5 AL6061 -> 155.6 + 47.9 = 203.4 g/다리, Izz 1.92e-3")
    print("면외 E*I 기준(v2 rim b44 t3.9 PLA, E 1.8~2.6 GPa): 4.98e7 ~ 7.20e7")
