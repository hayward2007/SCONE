#!/usr/bin/env python3
"""SCONEv3 부채꼴 립·리브 단면 구조 최적화.  docs/30 §3 재생성.
제약: 횡변형 <= 0.53 mm @3xW_i (서보 백래시 등가),  SF >= 4.0 @5xW_i,  b in [16,22]."""
import math
Ro, Rt = 122.5, 112.5
OCC, DELTA = 225.0, 45.0
ARC = OCC + 2 * DELTA                      # 315 deg
PLA = {'rho': 1.18, 'E': 3200.0, 'sy': 48.0, 'G': 1230.0}   # [미측정] 시편시험으로 갱신할 것
TPU_RHO = 0.494
Wi = 14.94                                  # m=4.569 kg, 3다리 지지
L_CANT = Rt - 22.0                          # 허브~접지 캔틸레버 90.5 mm

def ann(ro, ri, ang, t):
    return math.pi * (ro * ro - ri * ri) * (ang / 360.0) * t

def section(b, t_rim, t_lip, h_lip):
    """면외(축방향) 굽힘 단면 특성. 재료를 축방향 극단에 두는 립이 핵심."""
    y = b / 2.0 - t_lip / 2.0
    I = t_rim * b ** 3 / 12.0 + 2 * (t_lip * h_lip) * y * y
    return I, I / (b / 2.0)

def section_radial(b, t_rim, t_lip, h_lip):
    """반경 방향(수직 하중) 국부 단면계수."""
    I = (b * t_rim) * (h_lip / 2 + t_rim / 2) ** 2 + 2 * t_lip * h_lip ** 3 / 12
    return I, I / ((h_lip + t_rim) / 2)

def sector(b, t_rim, t_lip, h_lip, t_web, n_rib, t_rib, pocket, r_hub=22.0):
    r_li = Rt - t_rim - h_lip
    V = dict(tread=ann(Rt, Rt - t_rim, ARC, b),
             lip=2 * ann(Rt - t_rim, r_li, ARC, t_lip),
             web=ann(r_li, r_hub, ARC, t_web) * (1 - pocket),
             rib=n_rib * (r_li - r_hub) * t_rib * (b - 2 * t_lip),
             hub=ann(r_hub, 9.75, 360, t_web + 8.0))
    m = sum(V.values()) * PLA['rho'] / 1000.0
    mt = ann(Ro, Rt, ARC, b) * TPU_RHO / 1000.0
    I = ((V['tread'] * PLA['rho'] / 1e6) * 0.5 * (Rt ** 2 + (Rt - t_rim) ** 2)
         + (V['lip'] * PLA['rho'] / 1e6) * 0.5 * ((Rt - t_rim) ** 2 + r_li ** 2)
         + ((V['web'] + V['rib']) * PLA['rho'] / 1e6) * 0.5 * (r_li ** 2 + r_hub ** 2)
         + (ann(Ro, Rt, ARC, b) * TPU_RHO / 1e6) * 0.5 * (Ro ** 2 + Rt ** 2)) * 1e-6
    return m, mt, I, V

def deflect(I, n=3):        # 횡변형 mm @ n x W_i
    return n * Wi * L_CANT ** 3 / (3 * PLA['E'] * I)

def sf(Z, n=5):             # 굽힘 안전율 @ n x W_i 횡력
    return PLA['sy'] / (n * Wi * 100.0 / Z)

if __name__ == "__main__":
    print("v2 실측: 판 117.4 + 타이어 99.8 = 217.2 g/다리 @225deg, Izz 2.02e-3")
    print(f"{'b':>4}{'h_lip':>6}{'I mm4':>8}{'Z':>7}{'d@3x':>8}{'SF@5x':>7}{'판g':>7}{'타이어g':>8}{'다리당':>8}")
    for b, h_lip in [(18, 12), (18, 14), (20, 10), (20, 12), (20, 14), (22, 12)]:
        I, Z = section(b, 3.0, 3.0, h_lip)
        m, mt, Iz, V = sector(b, 3.0, 3.0, h_lip, 2.0, 7, 2.0, 0.72)
        ok = " <=" if (deflect(I) <= 0.53 and sf(Z) >= 4.0) else ""
        print(f"{b:4.0f}{h_lip:6.0f}{I:8.0f}{Z:7.0f}{deflect(I):8.3f}{sf(Z):7.2f}"
              f"{m:7.1f}{mt:8.1f}{m+mt:8.1f}{ok}")
    B, TR, TL, HL, TW, NR, TRB, PK = 20.0, 3.0, 3.0, 12.0, 2.0, 7, 2.0, 0.72
    m, mt, Iz, V = sector(B, TR, TL, HL, TW, NR, TRB, PK)
    I, Z = section(B, TR, TL, HL)
    Ir, Zr = section_radial(B, TR, TL, HL)
    span = 2 * math.pi * Rt * (ARC / NR) / 360.0
    print(f"\n채택 b={B:.0f} t_rim={TR} t_lip={TL} h_lip={HL} t_web={TW} n_rib={NR} t_rib={TRB} pocket={PK}")
    for k, v in V.items():
        print(f"   V_{k:6s}={v:8.0f} mm3 -> {v*PLA['rho']/1000:6.1f} g")
    print(f"   판 {m:.1f} + 타이어 {mt:.1f} = {m+mt:.1f} g/다리 | 단위각도 {(m+mt)/ARC:.3f} g/deg (v2 0.965)")
    print(f"   Izz {Iz:.3e} | Z_면외 {Z:.0f} | 횡변형 {deflect(I):.3f} mm @3xW_i | SF {sf(Z):.2f} @5xW_i")
    print(f"   반경방향 국부: 스팬 {span:.1f} mm, M={5*Wi*span/8:.0f} N mm, Z_rad={Zr:.0f}"
          f" -> sigma={5*Wi*span/8/Zr:.2f} MPa (SF {PLA['sy']/(5*Wi*span/8/Zr):.0f})")
    print(f"   6다리 {6*(m+mt)/1000:.3f} kg (v2 1.303)")
