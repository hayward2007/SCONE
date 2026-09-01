# SCONE 부채꼴 후킹 조건과 `scone-stair` 계단 알고리즘

이 문서는 SCONEv2가 C자형/부채꼴 말단 프레임으로 계단 모서리를 걸고
올라가기 위한 조건을 현재 CAD 도면, MuJoCo 충돌 메시, 보관 영상, 코드,
1차 연구 문헌에서 다시 계산한 기록이다. 또한 단순 연속 회전, 기존
`Climb`, 대각 삼각보 후킹, 주행-후킹 hybrid를 같은 시뮬레이션 조건에서
비교하고 최종 `scone-stair`를 선택한 이유를 남긴다.

가장 중요한 결론은 다음과 같다.

- SCONE의 강점은 말단을 한 발끝으로 고정하는 것이 아니라, 반경이 거의
  일정한 225° 접촉 호를 계단 모서리와 윗면에 연속 접촉시키는 데 있다.
- 현재 외반경은 `122.5 mm`, 내반경은 `112.5 mm`, 폭은 `44 mm`다.
- 쉬운/중간 계단에서는 여섯 말단을 계속 회전하는 방식이 가장 빠르고
  적은 기계일을 사용했다. 이 경우 굳이 보행 phase를 넣지 않는다.
- 현재 `stairs-3`의 최대 `120 mm` 단에서는 순수 회전이 마지막 단에서
  정체됐다. 대각 삼각보 후킹을 필요할 때만 켠 adaptive 방식은 상단 조건을
  통과했다.
- 구현된 `scone-stair`는 **MuJoCo 전용**이다. 실물 Dynamixel 제어 경로는
  바꾸지 않았다. 실제 TPU 마찰, 모터 전류, 백래시, 계단 nosing을 측정하기
  전에는 실물 안전 동작으로 간주하면 안 된다.

---

## 1. 조사 근거와 증거 등급

### 1.1 프로젝트 내부 1차 자료

| 자료 | 확인한 내용 |
|---|---|
| [`SCONEv2 Arc-Shaped Wheel.pdf`](../archive/assets/SCONEv2%20Arc-Shaped%20Wheel.pdf) | `R112.5`, `R122.5`, 폭 `44`, 전체 높이 `244.94 mm` 도면 표기 |
| [`TIRE.stl`](../src/assets/meshes/TIRE.stl) | MuJoCo가 실제 접촉에 사용하는 TPU 메시 |
| [`model.xml`](../src/assets/model.xml) | 질량, 축, 하단 모터 `3.0 N·m` stall cap, 마찰 `1.0`, 접촉 compliance |
| [`SCONEv2_stairs.mp4`](../archive/videos/SCONEv2_stairs.mp4) | 실제 SCONEv2가 계단을 옆으로 두고 여섯 C자 프레임을 회전해 올라간 한 사례 |
| [`presets.py`](../src/simulation/terrain/presets.py) | `stairs-1/2/3`의 단별 rise, tread, width |
| [`climb.py`](../src/locomotion/climb.py) | 기존 한쪽 지지 자세 + 전체 하단 회전 등반 순서 |

도면의 `148.27°` 표기는 C자 타이어의 빈 개구각 자체로 사용하지 않았다.
도면상 내부 spoke/기준선 사이의 각도이기 때문이다. 현재 `TIRE.stl`의 모든
vertex를 하단 관절축 기준 극좌표로 변환한 결과는 다음과 같다.

| 메시 계산값 | 여섯 다리 공통 결과 |
|---|---:|
| 관절축에서 최소 반경 | `0.112500 m` |
| 중앙 반경 | 약 `0.121406 m` |
| 최대 반경 | `0.122500 m` |
| 축 방향 폭 | `0.044000 m` |
| 충돌 호가 차지하는 각도 | 약 `225.001°` |
| 가장 큰 빈 각 구간 | 약 `134.999°` |

즉 수식과 코드에서는 도면 radii/width와 메시의 `225°/135°`를 사용한다.

### 1.2 관련 1차 연구

- Moore et al.의 RHex 계단 연구는 반원형 다리가 접촉점을 굴리면서
  hip-to-contact 수평 거리를 줄이고, 계단 경사와 가까운 hip 궤적을 만드는
  것이 에너지·pitch 측면에서 유리하다고 보고했다. 일반 tripod gait보다
  계단 geometry에 맞춘 wave/phase가 필요했고, 계단과 phase가 어긋나는
  문제는 다음 riser에 일부러 밀어 self-alignment하는 단계로 줄였다.
  [McGill 연구실 출판 목록](https://library.cim.mcgill.ca/data/websites/mll/publications.html),
  [ICRA 2002 논문](https://www.rhex.web.tr/moore_campbell_grimminger_buehler.icra2002.pdf)
- Seo와 Kang은 curved spoke가 높은 tread에 닿아 굴러가며 wheel/body를
  끌어올리는 조건을 단계별 기구학 부등식으로 만들고, 접촉 가능 반경,
  tread 잔여 길이, slip margin, 필요한 호 길이를 함께 확인해야 한다고
  제시했다. 또한 spoke 속도를 phase 경계에서 선형 변화시켜 chattering과
  impact를 줄이는 프로필을 사용했다.
  [Biomimetics 2024 논문](https://doi.org/10.3390/biomimetics9100633)
- Ordoñez-Avila et al.은 legged-wheel opening chord와 한 계단의 rise/tread
  대각선 비를 비교했다. 이 지표는 SCONE과 정확히 같은 구조는 아니므로
  아래에서는 보조 비교값으로만 사용한다.
  [Symmetry 2023 논문](https://doi.org/10.3390/sym15112071)

이 문헌의 성능 수치를 SCONE 성능으로 옮겨 쓰지 않는다. 형태와 제어
아이디어만 비교하며 SCONE 결과는 현재 모델에서 다시 측정했다.

---

## 2. 좌표와 용어

계단 한 단을 2차원 단면에서 다음과 같이 둔다.

| 기호 | 의미 | 단위 |
|---|---|---:|
| `h` | 한 단의 수직 rise | m |
| `d` | 한 단의 tread depth | m |
| `w` | 계단 폭 | m |
| `r_n` | 둥근 nosing 반경 | m |
| `R_i` | C자 타이어 내반경, 현재 `0.1125` | m |
| `R_o` | C자 타이어 외반경, 현재 `0.1225` | m |
| `b` | 타이어 축 방향 폭, 현재 `0.044` | m |
| `C=(x_c,z_c)` | 하단 회전축 중심 | m |
| `E=(x_e,z_e)` | 목표 계단 모서리 | m |
| `ρ=||E-C||` | 축 중심에서 계단 모서리까지 거리 | m |
| `φ_E` | 하단 프레임 기준 모서리 polar angle | rad |
| `μ` | 접촉면 Coulomb 마찰계수 | - |
| `W_i` | 한 hooked sector가 부담하는 중량 | N |
| `τ_i` | 해당 하단 모터가 내야 하는 torque | N·m |

현재 procedural course는 world `+Y`로 올라간다. SCONE은 계단에 옆면을
향하도록 네 번 좌회전한 뒤, legacy Drive의 `left` 방향과 같은 하단 회전으로
`+Y`를 진행한다. `scone-stair`의 A/D 전용 입력은 이 배치를 전제로 한다.

---

## 3. 후킹 가능 조건

아래 조건은 각각 **필요 조건 또는 보수적 검사**다. 하나만 만족한다고 전체
로봇이 반드시 올라가는 충분조건이 되지는 않는다.

### 3.1 계단 모서리가 TPU annulus에 들어와야 한다

sharp edge를 점으로 근사하면 기본 radial 조건은 다음과 같다.

```text
ρ = sqrt((x_e - x_c)^2 + (z_e - z_c)^2)

R_i - ε_c <= ρ <= R_o + ε_c
```

`ε_c`는 TPU 변형과 MuJoCo 접촉 허용 오차다. 현재 코드는 이 검사를
`ArcWheelGeometry.edge_in_radial_band()`로 제공한다.

반경만 맞아도 개구부가 모서리를 향하면 접촉하지 않는다. 하단 관절각을
`q_l`, 메시의 접촉 호 각 구간을 `A_arc`라고 하면 다음 각도 조건도 필요하다.

```text
wrap(φ_E - q_l) ∈ A_arc
```

현재 접촉 호는 약 `225°`, 빈 구간은 약 `135°`다. 따라서 회전 중 모서리가
빈 구간에 들어가는 짧은 phase가 있으며, 다른 다리 또는 이미 윗면에 걸린
호가 그 구간 동안 차체를 지지해야 한다.

### 3.2 평지 접근에서의 보수적 단높이 조건

하단 축이 lower tread 위에서 외반경 높이에 있다고 단순화하면 다음을 먼저
검사할 수 있다.

```text
h_eff = h + r_n + c_z
h_eff <= R_o
```

여기서 `c_z`는 원하는 수직 여유다. 현재 `R_o=122.5 mm`이므로:

- 여유와 nosing을 0으로 두면 이론상 `h <= 122.5 mm`
- `3 mm` clearance를 두면 `h <= 119.5 mm`
- 둥근 nosing이 크면 유효 한계는 더 낮아진다.

이 식은 SCONE 전체 leg articulation을 무시한 보수적 flat-approach 검사다.
중단 관절이 하단 축을 들어 올리고 앞으로 보낼 수 있으므로 `h > R_o`가
무조건 불가능하다는 뜻은 아니다. 반대로 `h < R_o`만으로 성공이 보장되지도
않는다.

### 3.3 sharp edge를 중심으로 도는 순간의 기하

반경 `R`인 접촉 호가 높이 `h`의 sharp edge를 중심으로 pivot한다고 하자.
초기 축 높이가 lower tread에서 `R`이면 edge-to-center의 수직 성분은
`R-h`이고 수평 성분은 다음과 같다.

```text
x_Q = sqrt(R^2 - (R-h)^2)
    = sqrt(2Rh - h^2)
```

`0 <= h <= 2R`에서만 실수다. flat approach의 보수 조건은 앞 절처럼 보통
`h <= R`을 사용한다.

이 값은 [`wheel_edge_offset()`](../src/locomotion/stair_geometry.py)에 구현돼
있다.

### 3.4 중력 모멘트를 이기는 하단 torque

한 sector가 부담하는 중량을 `W_i`라 하면 edge 주위 중력 모멘트의 이상적
하한은 다음과 같다.

```text
τ_ideal = W_i x_Q

τ_required = S_τ W_i x_Q / η
```

`S_τ >= 1`은 충격·하중 불균형 safety factor, `0 < η <= 1`은 전달 효율이다.
현재 MuJoCo 모델 총질량 `4.160952 kg`, 세 hooked sector가 중량을 정확히
나눠 가진다고 가정해 `W_i=mg/3`, `R=0.1225 m`를 넣으면:

| rise | 이상적 sector당 torque | 비고 |
|---:|---:|---|
| `35 mm` | `1.166 N·m` | 쉬운 첫 단 |
| `85 mm` | `1.587 N·m` | 중간 preset 최대 단 |
| `120 mm` | `1.666 N·m` | 외반경에 거의 같은 높은 단 |

하단 XM430-W210의 모델 stall cap은 `3.0 N·m`다. 그러나 이 표는 정적·균등
분담·sharp edge·효율 100% 가정이다. 예를 들어 `S_τ=1.5`, `η=0.7`을
적용하면 `120 mm`에 약 `3.57 N·m`가 되어 모델 cap을 넘는다. 따라서
`1.666 < 3.0`만 보고 실물 여유가 충분하다고 결론 내리면 안 된다.

수평으로 축을 밀어 edge 모멘트를 만든다고 보면 필요한 force는:

```text
F_x = W_i x_Q / (R-h)
```

`h -> R`이면 분모가 0에 가까워진다. 즉 `120/122.5 mm`처럼 반경에 거의
같은 단은 기하상 닿더라도 순수 수평 push에는 매우 불리하다. 여기서
중단 관절 lift와 다른 tripod의 pull이 중요해진다.

### 3.5 마찰 조건과 형상 구속

Coulomb 접촉의 기본 조건은 다음과 같다.

```text
|F_t| <= μ F_n
μ_required = |F_t| / F_n
```

edge가 C자 내부로 들어오면 단순 마찰뿐 아니라 형상 구속이 생겨 미끄럼
의존성이 줄 수 있다. 그래도 다음 경우에는 후킹이 쉽게 풀린다.

- nosing이 매우 둥글어 edge가 점/선이 아니라 넓은 곡면인 경우
- TPU가 젖거나 먼지가 있어 `μ`가 낮아진 경우
- 하단 호가 개구 방향으로 회전해 edge가 빠지는 경우
- 다음 tripod가 닿기 전에 현재 contact normal이 작아지는 경우
- 계단 overhang이 타이어 또는 링크를 밀어내는 경우

현재 terrain과 tire의 sliding friction은 모두 `1.0`이다. 실제 계단 재질별
측정값은 아직 없다.

### 3.6 tread에 남아 있을 길이

모서리에 걸린 뒤 굴러 올라가도 다음 riser에 너무 일찍 막히거나 뒤로
미끄러지면 실패한다. 최소 조건을 다음처럼 둔다.

```text
d_contact + d_slip + c_front <= d
```

`d_contact`는 호가 윗면에 올라간 뒤 필요한 진행 길이, `d_slip`은 허용할
후방 slip, `c_front`는 다음 riser까지 남길 margin이다. 정확한 값은 하단
프레임 phase와 중단/상단 관절 자세로 계산하거나 접촉 로그로 측정해야 한다.

보조 비교로 legged-wheel opening chord를 계산하면:

```text
L_open = 2 R_o sin(θ_open / 2)
q_open = L_open / sqrt(h^2 + d^2)
```

현재 `θ_open=135°`, `L_open=0.226350 m`다. 이 비는 N-spoke wheel 연구의
지표와 구조가 달라 SCONE 성공 판정에는 쓰지 않고 상대 비교만 한다.

### 3.7 세 다리 지지의 정적 안정성

한 대각 tripod의 접촉점들을 `p_1,p_2,p_3`, CoM의 수평 투영을 `p_G`라고
하면 기본 정적 안정 조건은:

```text
p_G ∈ conv{p_1,p_2,p_3}
```

코드의 `support_polygon_margin()`은 각 support edge까지 signed distance의
최솟값을 반환한다.

```text
m_support > 0 : 내부
m_support = 0 : 경계
m_support < 0 : 외부
```

계단에서는 세 contact의 높이가 다르고 동적 관성도 있으므로 이 2D margin은
정적 1차 검사다. 그래도 한 tripod를 lift할 시점과 body pitch/yaw가 잘못돼
CoM이 support triangle 밖으로 나가는 오류를 찾는 데 유용하다.

### 3.8 추가로 동시에 만족해야 할 조건

후킹 성공에는 아래 항목도 필요하다.

- 중단/상단 관절 workspace 안에서 swing tripod가 다음 tread를 넘을 것
- swing 최저 높이가 `h + clearance`보다 높을 것
- body plate와 링크가 riser/nosing에 충돌하지 않을 것
- 세 hooked sector의 load가 한 모터에 몰리지 않을 것
- 하단 관절 속도가 접촉 전에 감속돼 impact가 제한될 것
- 다음 contact가 생길 때까지 반대 tripod support polygon이 유지될 것
- 배선과 TPU 변형이 개구부 후킹을 방해하지 않을 것

현재 MJCF hinge에는 실제 mechanical range가 설정돼 있지 않다. 따라서
simulation IK/position이 된다는 사실만으로 실물 가동범위를 증명할 수 없다.

---

## 4. 현재 계단 preset의 기하 평가

각 행은 한 물리 단이다. `h/R_o`가 1에 가까울수록 pure rolling의 수평
force 조건이 불리해진다. `τ_ideal`은 앞 절의 `mg/3`, 효율 100% 가정이다.

| preset/단 | rise `h` | tread `d` | `h/R_o` | 등가 경사 | `q_open` | `τ_ideal` |
|---|---:|---:|---:|---:|---:|---:|
| stairs-1/1 | 35 mm | 300 mm | 0.286 | 6.65° | 0.749 | 1.166 N·m |
| stairs-1/2 | 45 mm | 270 mm | 0.367 | 9.46° | 0.827 | 1.291 N·m |
| stairs-1/3 | 55 mm | 240 mm | 0.449 | 12.91° | 0.919 | 1.391 N·m |
| stairs-2/1 | 55 mm | 270 mm | 0.449 | 11.51° | 0.821 | 1.391 N·m |
| stairs-2/2 | 70 mm | 230 mm | 0.571 | 16.93° | 0.941 | 1.506 N·m |
| stairs-2/3 | 85 mm | 200 mm | 0.694 | 23.03° | 1.042 | 1.587 N·m |
| stairs-3/1 | 80 mm | 240 mm | 0.653 | 18.43° | 0.895 | 1.563 N·m |
| stairs-3/2 | 100 mm | 200 mm | 0.816 | 26.57° | 1.012 | 1.638 N·m |
| stairs-3/3 | 120 mm | 170 mm | 0.980 | 35.22° | 1.088 | 1.666 N·m |

`stairs-3/3`은 외반경보다 겨우 `2.5 mm` 낮다. `3 mm` clearance만 적용해도
보수적 flat-approach 조건을 벗어난다. 순수 rolling이 이 preset의 마지막
단에서 정체된 결과와 방향이 일치한다.

### “일반적인 계단은 모두 회전만으로 가능” 주장에 대한 범위

보관 영상은 SCONEv2가 한 실제 계단을 연속 회전으로 오른다는 강한 설계
증거다. 그러나 다음 이유로 “모든 일반 계단”을 검증했다고 쓰지는 않는다.

- 영상 계단의 rise, tread, nosing radius, 마찰을 실측한 기록이 없다.
- 현재 preset의 최대 rise는 `120 mm`이고, RHex 논문의 실제 계단 표에는
  `130–200 mm` rise가 포함된다. 로봇 형상은 다르지만 검증 범위 차이는
  분명하다.
- 둥근 모서리, overhang, 젖은 표면, 나선 계단, 폭 변화는 현재 반복 실험에
  포함되지 않았다.
- 현재 결과는 deterministic MuJoCo 한 모델/한 friction 설정이다.

따라서 문서상 현재 주장은 “보관 영상의 한 실제 계단과 MuJoCo `stairs-1/2`는
연속 회전으로 통과했고, `stairs-3`는 assist가 필요했다”까지다.

---

## 5. 비교한 제어 가설

### H0: 순수 연속 rolling

여섯 하단 sector를 같은 지면 이동 방향으로 `velocity=150`에 해당하는
속도로 계속 회전한다. 짝수/홀수 관절축은 MuJoCo adapter가 반대 raw sign으로
보정한다.

예상 장점:

- SCONE의 C자형 말단 자체가 하는 passive length change를 그대로 사용
- phase planner가 없어 빠르고 단순함
- 쉬운 계단과 평지에서 불필요한 중단 관절 motion이 없음

예상 실패:

- `h/R_o`가 1에 가까우면 모서리에 닿아도 수평 push leverage가 나빠짐
- 모든 sector가 같은 속도로 돌면 지지와 회수 역할을 구분할 수 없음
- 마지막 높은 단에서 계속 헛돌거나 edge phase를 놓칠 수 있음

### H1: 현재 legacy `Climb`

한쪽 middle group을 load-bearing 자세로 만든 뒤 모든 하단 sector를 일정
시간 회전하고, 매 command 뒤 기본 stance로 복구한다.

예상 장점:

- 기존 하드웨어 동작 순서를 보존
- 동작 사이에 확실한 자세 복구가 있음

실제 문제:

- simulation target settle을 여러 번 기다려 한 실험이 약 `46 s` 걸림
- 한쪽 physical side를 지지하는 개념이지 대각 tripod A/B를 다음 단으로
  교대시키는 구조가 아님
- 현재 세 preset의 상단 판정에 도달하지 못함

### H2: 대각 삼각보 후킹

tripod A `(1,4,5)`와 tripod B `(2,3,6)`를 교대한다.

- support tripod middle: `250°`
- swing tripod middle: `165°`
- support lower speed magnitude: `45`
- swing lower speed magnitude: `210`
- phase: `0.75 s`

support는 edge를 천천히 유지하고 swing은 다음 단을 빨리 찾도록 했다.

장점은 높은 단을 안정적으로 넘는 것이고, 단점은 쉬운 계단에서도 회전을
과도하게 분리해 pure rolling보다 느리고 기계일이 커진다는 것이다.

### H3: 주행 + 후킹 hybrid

모든 sector를 계속 전진시키되 두 tripod 속도를 다르게 둔다.

- support lower speed magnitude: `105`
- swing lower speed magnitude: `185`
- middle target과 phase는 H2와 같음

H2처럼 support를 거의 멈추지 않으므로 C자 호의 rolling 장점을 유지하면서
swing tripod가 다음 edge를 먼저 잡도록 했다.

### H3 파라미터 변형

`stairs-3`에서 첫 접근을 끝낸 같은 상태로 아래 변형도 실행했다. 이 표는
최종 동등조건 표와 달리 **접근 이후 tuning sweep**이므로 서로 간 비교에만
사용한다.

| 변형 | middle support/swing | lower support/swing | phase | 상단 시간 | 상단까지 일 | 관찰 |
|---|---:|---:|---:|---:|---:|---|
| hybrid 기본 | 250°/165° | 105/185 | 0.75 s | 4.600 s | 54.713 J | 빠르지만 phase 전환 충격 존재 |
| soft | 240°/170° | 80/170 | 0.75 s | 8.142 s | 94.618 J | 접촉은 부드러우나 너무 느림 |
| balanced | 240°/170° | 100/170 | 0.65 s | 6.166 s | 74.219 J | 예상과 달리 peak impact가 증가 |
| fast | 245°/170° | 120/210 | 0.60 s | 3.536 s | 47.630 J | 가장 빠르지만 전체 run peak force `281 N`, 제외 |
| ramped | 250°/165° | 105/185 | 0.75 s | 4.172 s | 52.267 J | phase 시작 `0.18 s` smoothstep 적용 |

`fast`는 시간/일만 보면 좋지만 contact peak가 너무 커 채택하지 않았다.
`ramped`는 기본보다 빠르고 일이 작으며 upright도 나았지만, 전체 wheel이
최대 속도로 도는 도중 바로 A/B 속도를 나누면 차체가 옆으로 전복됐다.
따라서 최종 알고리즘은 assist 진입 직전에 여섯 하단 속도를 한 번 0으로
동기화한 뒤 ramp를 시작한다.

### H4: 최종 adaptive roll-first

최종 선택은 H0과 ramped H3의 조건부 결합이다.

```text
IDLE
  └─ A/D 입력 ─> ROLLING
                   ├─ 쉬운/중간 계단, 정상 진행 ─> 계속 ROLLING
                   ├─ 알려진 높은 단 근접 ─┐
                   └─ 0.8 s 동안 25 mm 미만 진행 ─┤
                                                   v
                                      lower 6개 속도 0 동기화
                                                   v
                                      TRIPOD_ASSIST 6 phase
                                                   v
                                               ROLLING
```

알려진 stair preset에서 `max(h)/R_o >= 0.75`면 첫 riser `0.27 m` 전부터
pre-hook 전환을 준비한다. 그렇지 않거나 `mixed`처럼 위치별 profile이 다른
경우에는 진행 정체 검출을 fallback으로 사용한다.

이 설계가 SCONE의 장점을 살리는 이유는 다음과 같다.

- 항상 보행시키지 않고 sector rolling이 충분한 구간은 그대로 굴린다.
- 높은 단에서만 대각 tripod를 “현재 단 지지 / 다음 단 탐색”으로 분리한다.
- support sector도 완전히 고정하지 않고 낮은 속도로 굴려 곡면 contact의
  passive length change를 유지한다.
- phase 경계의 속도/자세를 `0.18 s` smoothstep으로 바꾼다.
- assist 6 phase 뒤 rolling으로 돌아가 긴 landing과 평지에서 불필요한
  leg motion을 줄인다.

---

## 6. 동등조건 시뮬레이션 방법

### 6.1 공통 조건

| 항목 | 값 |
|---|---|
| 모델 | 현재 [`model.xml`](../src/assets/model.xml) |
| total model mass | `4.16095234368 kg` |
| profile | `standard` |
| base | floating |
| physics timestep | `0.002 s` |
| controller update | 매 physics step |
| 계단 | `stairs-1`, `stairs-2`, `stairs-3` |
| terrain/tire sliding friction | `1.0` |
| 초기 배치 | Walk 초기화 → 좌회전 4회 → Drive 자세 → 0.5 s settle |
| 진행 방향 | world `+Y`, side-on |
| 최대 관찰 시간 | rolling/adaptive `16 s`; H2/H3 `11 s`; legacy 약 `46 s` |

상단 통과 판정은 순간적으로 앞 sector 하나가 걸린 것을 성공으로 세지 않기
위해 다음 두 조건을 동시에 사용했다.

```text
y_root >= 0.35 + sum(마지막 전 tread) + 0.4 * 마지막 tread
z_root >= z_start + 0.70 * total_stair_height
```

### 6.2 지표

상단까지 절대 기계일은 각 actuator force와 관절 속도의 절댓값을 적분했다.

```text
E_abs = integral sum_i |τ_i qdot_i| dt
```

회생 에너지를 빼는 전기 소비량이 아니라 actuator motion의 비교 지표다.

upright는 body rotation matrix의 `R_zz`를 사용했다.

```text
upright = e_z_world · e_z_body = R_zz
```

`1`은 직립, `0`은 90° 기울어짐, 음수는 뒤집힘이다. contact force는
`mj_contactForce()`의 3축 force norm 최대값이다.

### 6.3 한계

- 각 조건은 deterministic model의 한 번 실행이다. 통계적 신뢰구간이 없다.
- 에너지는 배터리 전력이나 Dynamixel 전류 적분이 아니다.
- 모델 friction, compliance, motor torque-speed가 실물과 완전히 교정되지 않았다.
- 상단 통과 뒤 계속 command를 주면 landing을 벗어나므로 결과는 최초 상단
  통과 시점까지만 비교한다.

### 6.4 재현 명령

[`src/simulation/stair_benchmark.py`](../src/simulation/stair_benchmark.py)는
H0–H4의 세 preset 비교와 H3 파라미터 변형을 같은 초기화·상단 판정·지표로
다시 실행한다.

```bash
python -m src.simulation.stair_benchmark --all --tuning
```

출력은 실험별 JSON Lines다. `time_to_top_s`, `work_to_top_j`,
`minimum_upright_to_top`, `peak_contact_force_to_top_n`은 최초 상단 판정까지만
집계한다. `minimum_upright`, `peak_contact_force_n`은 설정된 전체 관찰 구간을
집계하므로 tuning 중 늦게 생긴 충격을 따로 발견할 수 있다.

---

## 7. 모든 가설의 동등조건 결과

`실패`는 해당 최대 관찰 시간 안에 상단 조건을 동시에 만족하지 못했다는
뜻이다.

### 7.1 상단까지 시간

| 전략 | stairs-1 | stairs-2 | stairs-3 |
|---|---:|---:|---:|
| H0 pure rolling | 3.278 s | 3.408 s | 실패 |
| H1 legacy Climb | 실패 | 실패 | 실패 |
| H2 tripod hook | 5.968 s | 7.422 s | 7.764 s |
| H3 hybrid | 3.728 s | 5.946 s | 6.112 s |
| H4 adaptive `scone-stair` | **3.278 s** | **3.408 s** | **4.718 s** |

### 7.2 상단까지 절대 기계일

| 전략 | stairs-1 | stairs-2 | stairs-3 |
|---|---:|---:|---:|
| H0 pure rolling | 18.779 J | 23.225 J | 실패 전 16 s 총 97.729 J |
| H1 legacy Climb | 실패 전 총 275.027 J | 실패 전 총 273.733 J | 실패 전 총 277.790 J |
| H2 tripod hook | 51.686 J | 68.350 J | 73.815 J |
| H3 hybrid | 32.726 J | 66.692 J | 68.676 J |
| H4 adaptive `scone-stair` | **18.779 J** | **23.225 J** | **50.861 J** |

### 7.3 최종 알고리즘의 자세·접촉

| terrain | assist 진입 | 상단 시간 | 상단까지 일 | 상단까지 최소 upright | 상단까지 peak contact |
|---|---:|---:|---:|---:|---:|
| stairs-1 | 0 | 3.278 s | 18.779 J | 0.987 | 37.495 N |
| stairs-2 | 0 | 3.408 s | 23.225 J | 0.965 | 50.396 N |
| stairs-3 | 1 | 4.718 s | 50.861 J | 0.870 | 86.372 N |

### 7.4 해석

- 쉬운 두 계단에서는 adaptive가 assist를 한 번도 켜지 않아 pure rolling과
  정확히 같은 상단 시간/일을 냈다.
- 높은 계단에서 pure rolling은 16초 뒤 `y=0.810 m`, `z=0.216 m` 부근에서
  마지막 상단 판정을 얻지 못했다.
- H2는 세 preset을 모두 통과하지만 쉬운 계단에서도 불필요한 큰 middle
  motion과 속도 차이를 사용해 느리고 일이 크다.
- H3는 H2보다 빠르지만 모든 계단에서 assist를 사용한다.
- adaptive는 높은 계단에서만 한 번 assist를 사용했고 H2보다 약 39%, H3보다
  약 23% 빠르게 상단 조건에 도달했다. 이 백분율은 현재 단일 deterministic
  run의 계산값이며 일반 성능 보장은 아니다.
- legacy Climb은 자세 settle과 복구 대기가 누적되고 현재 계단 진행 phase와
  맞지 않아 상단 조건을 통과하지 못했다. 이 결과는 실물 legacy 동작이
  불가능하다는 뜻이 아니라 현재 MuJoCo/계단 배치와의 비교다.

---

## 8. 구현 구조

### 8.1 기하 계산

[`src/locomotion/stair_geometry.py`](../src/locomotion/stair_geometry.py)는
backend와 무관한 식을 제공한다.

| API | 역할 |
|---|---|
| `ArcWheelGeometry` | 내/외반경, 폭, 접촉 호, opening chord |
| `SCONE_V2_ARC_WHEEL` | 현재 `112.5/122.5/44 mm`, `225°` 상수 |
| `wheel_edge_offset()` | `sqrt(2Rh-h²)` |
| `quasi_static_pivot_torque()` | safety/efficiency 포함 edge pivot torque |
| `quasi_static_horizontal_push()` | 수평 axle push 하한 |
| `required_friction_coefficient()` | `|F_t|/F_n` |
| `stair_slope()` | `atan2(h,d)` |
| `legged_wheel_opening_ratio()` | opening chord/step diagonal 보조 지표 |
| `support_polygon_margin()` | CoM projection의 convex support signed margin |

### 8.2 adaptive controller

[`src/simulation/core/stair_climber.py`](../src/simulation/core/stair_climber.py)는
MuJoCo 전용 state machine을 구현한다.

| 설정 | 현재값 | 의미 |
|---|---:|---|
| `rolling_velocity` | 150 | assist가 필요 없을 때 여섯 sector 속도 |
| `assist_support_velocity` | 105 | 현재 단을 지지하는 tripod 속도 magnitude |
| `assist_swing_velocity` | 185 | 다음 단을 찾는 tripod 속도 magnitude |
| `support_middle_degrees` | 250° | support tripod middle target |
| `swing_middle_degrees` | 165° | swing tripod middle target |
| `neutral_middle_degrees` | 180° | Drive 기준 자세 |
| `assist_phase_seconds` | 0.75 s | 한 A/B phase 시간 |
| `transition_seconds` | 0.18 s | phase 시작 smoothstep 시간 |
| `assist_phase_count` | 6 | assist 뒤 rolling 복귀 |
| `stall_window_seconds` | 0.80 s | 정체 측정 구간 |
| `minimum_progress_metres` | 0.025 m | 구간 내 이 값 미만이면 assist |
| `tall_rise_ratio` | 0.75 | `max rise / R_o` pre-hook 기준 |
| `first_riser_y` | 0.35 m | procedural stair 시작 위치 |
| `prehook_distance` | 0.27 m | 알려진 높은 단의 assist 준비 거리 |

대각 tripod mapping은 프로젝트 기존 값과 같다.

```text
tripod A = legs (1, 4, 5)
tripod B = legs (2, 3, 6)
```

assist 진입 시 하단 여섯 속도를 먼저 0으로 보낸다. 이 단계가 없던 prototype은
`stairs-3` 첫 support swap에서 차체가 옆으로 기울고 뒤집혔다. 그 뒤 각
phase의 이전 command에서 새 command까지 cubic smoothstep을 적용한다.

### 8.3 CLI route

키 입력 없이 두 방식을 보려면 루트 메뉴의 `시뮬레이션 (자동 데모)`를
선택하거나 다음 명령을 사용한다.

```bash
mjpython -m src.simulation --demo compare --terrain stairs-2
```

`compare`는 feedback 없는 H0 hardcoded viewer 뒤에 H4 improved viewer를 같은
terrain으로 연다. 기본 stairs-2는 둘 다 통과하며, stairs-3는 H0 정체와 H4
assist 통과를 보여 준다. 구현·검증 과정은 12번 재설계 기록에 있다.

직접 실행:

```bash
mjpython -m src.simulation \
  --control scone-stair \
  --profile standard \
  --terrain stairs-3
```

통합 메뉴에서는 다음 순서로 선택한다.

```text
시뮬레이션 조종
  -> scone-stair
  -> Standard
  -> stairs-1 / stairs-2 / stairs-3
```

controller가 자동으로 네 번 좌회전해 procedural course에 side-on 정렬하고
Drive 자세에 들어간다. 그 뒤:

- `A`: world `+Y` 계단 상승 방향
- `D`: 반대 방향
- `SPACE`: 하단 속도 0
- `Q`: 종료

W/S와 yaw는 `scone-stair`에서 0으로 제한한다. 이 controller는 regular
side-on stair 전용이며 평면 omnidirectional gait가 아니다.

### 8.4 hardware 경로를 바꾸지 않은 이유

`SconeStairClimber`는 생성 시 `MuJoCoController`만 허용한다. 실물 CLI의
`Walk/Drive/Climb`이나 `DynamixelController`에는 새 mode를 추가하지 않았다.

실물 전환 전에 필요한 최소 측정은 다음과 같다.

1. 타이어 재질별 static/dynamic friction
2. 실제 계단의 rise, tread, width, nosing radius, overhang
3. ID 13–18의 연속/순간 전류, 전압 sag, 온도
4. 각 joint의 실제 zero, 방향, mechanical range, backlash
5. 세 sector 하중 분담과 support polygon
6. 하단 velocity 105/150/185가 실제 rpm에서 만드는 contact speed
7. 비상정지 시 계단에서 미끄러지지 않는 hold 자세

---

## 9. 테스트와 재검증

### 9.1 수식 회귀

[`tests/test_stair_geometry.py`](../tests/test_stair_geometry.py)는 다음을 고정한다.

- 현재 radius/width/arc opening과 opening chord
- `120 mm`가 zero-clearance에서는 reach 가능하지만 `3 mm` clearance에서는
  보수 조건을 넘는다는 경계
- pivot offset, torque, horizontal push 식
- friction coefficient와 stair slope
- support polygon 내부/외부 signed margin

### 9.2 동역학 회귀

[`tests/test_stair_climber.py`](../tests/test_stair_climber.py)는 실제 MuJoCo
floating model에서:

- `stairs-1`은 assist 0회로 상단 조건 통과
- `stairs-3`은 assist 1회 이상 사용
- 두 경우 모두 제한 시간 안에 상단 위치/높이 동시 통과
- 높은 계단 상단까지 최소 upright `>0.84`

를 검사한다.

실행:

```bash
python -m unittest \
  tests.test_stair_geometry \
  tests.test_stair_climber \
  tests.test_simulation -v
```

전체 회귀:

```bash
python -m unittest discover -s tests -v
```

### 9.3 변경 시 다시 돌릴 범위

| 변경 | 최소 재검증 |
|---|---|
| `TIRE.stl`, wheel radius/opening | geometry test + 세 stairs 동역학 + 문서 표 재계산 |
| motor torque/speed/PID | stair time/work/contact + flat Drive |
| terrain rise/tread/friction | 모든 가설 또는 최소 H0/H4 재비교 |
| tripod mapping | gait tests + stair assist + legacy Walk |
| middle target/phase | stairs-3 upright/contact + stairs-1 불필요 assist 확인 |
| stall threshold | mixed terrain false-positive와 tall stair false-negative |

---

## 10. 실패, 미해결 문제, 다음 실험

### 10.1 이번에 실제로 발생한 실패

| 시도 | 문제 | 처리 |
|---|---|---|
| pure rolling / stairs-3 | 16 s 내 마지막 상단 판정 실패 | tall rise에서만 assist 추가 |
| legacy Climb | 약 46 s settle/복구 후에도 세 preset 상단 실패 | 최종 route로 선택하지 않음; 하드웨어 경로는 보존 |
| 항상 tripod hook | 모든 계단 통과하지만 쉬운 계단 시간·일 증가 | assist를 조건부로 변경 |
| 항상 hybrid | rolling보다 쉬운 계단 비효율 | roll-first로 변경 |
| hybrid-fast | 가장 빠르지만 전체 run peak contact 약 281 N | 제외 |
| hybrid-soft | contact를 줄이려 했으나 8.142 s, 94.618 J | 제외 |
| full rolling에서 바로 A/B 분리 | `stairs-3`에서 lateral pitch 후 전복 | assist 직전 lower 0 동기화 추가 |
| assist가 끝난 뒤 tall 조건 재평가 | 같은 계단에서 assist가 반복 진입 | known pre-hook을 1회로 제한, 이후 stall fallback만 사용 |

### 10.2 아직 검증하지 않은 것

- 계단 하강
- 첫 단에 비스듬히 접근하거나 yaw가 틀어진 경우
- 좌우 rise가 다른 계단과 나선 계단
- 둥근/고무/금속 nosing, overhang
- 마찰계수 sweep과 젖은 표면
- 계단 폭이 44 mm sector 세 개의 lateral support에 부족한 경우
- payload 변화와 배터리 위치 변화
- 실물 전류/온도/TPU 변형
- 센서 기반 실제 stair geometry 추정
- 여러 random perturbation과 반복 run의 성공률/신뢰구간
- GUI에서 장시간 사람이 관찰한 contact sequence

### 10.3 다음 권장 실험 순서

1. `stairs-3`에서 friction `1.0 → 0.8 → 0.6 → 0.4` sweep
2. rise `80–130 mm`, tread `160–300 mm`, nosing `0–20 mm` grid
3. payload `0/0.5/1.0 kg`와 CoM 위치 sweep
4. `stall_window`와 `minimum_progress` false-positive/negative map
5. contact force peak뿐 아니라 impulse와 95 percentile 기록
6. GUI slow-motion으로 edge가 개구부에 들어오고 빠지는 phase 확인
7. 실물은 tether, current limit, spotter, emergency stop을 갖춘 단일 낮은 단부터 시작
8. 실제 센서가 준비되면 root world-Y 대신 odometry/IMU/contact 추정으로 stall
   detector를 교체

---

## 11. 최종 판단

SCONE의 부채꼴 말단은 “보행 발”과 “바퀴”를 별개로 바꾸는 구조가 아니라,
같은 회전에서 아래 tread 접촉, riser edge 후킹, 위 tread rolling을 연속적으로
만드는 것이 핵심이다. 따라서 계단 제어의 기본값은 복잡한 발끝 planning이
아니라 연속 rolling이어야 한다.

다만 단높이가 외반경에 가까워지면 수평 leverage가 급격히 나빠지고 pure
rolling이 정체될 수 있다. 이때 SCONE의 18-DoF 장점을 사용해 대각 세 다리는
현재 단을 지지하고, 반대 세 다리는 더 빠르게 다음 단을 찾게 하면 된다.
`scone-stair`는 이 assist를 상시 사용하지 않고 geometry/stall 조건에서만
켜므로, 쉬운 계단의 속도·효율과 높은 계단의 통과성을 동시에 보존하는 현재
시뮬레이션 기준 최선의 가설이다.

이 결론은 현재 MuJoCo 모델과 세 preset에 대한 구현 결론이다. 실물에서
“일반 계단 모두 가능”을 주장하려면 위 미해결 실험과 반복 성공률 측정이
추가로 필요하다.
