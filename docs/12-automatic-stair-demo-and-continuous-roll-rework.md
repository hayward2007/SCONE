# 자동 계단 데모와 연속 회전형 `scone-gait` 재설계 기록

이 문서는 2026-09-01에 수행한 시뮬레이션 보행 재설계의 요구사항, 재현
조건, 실패한 가설, 채택 수치, 구현 경계, 테스트 결과를 시간 순서대로 남긴다.
결론만 적는 문서가 아니라 왜 기존 동작이 답답하고 `tripod-gait`와 비슷해
보였는지, 어떤 수정이 효과가 없었는지까지 재현할 수 있게 기록한다.

기존 계단 기하·후킹 공식과 H0–H4 비교는
[`11-scone-stair-climbing.md`](11-scone-stair-climbing.md)가 기준이다. 이 문서는
그 결과를 사람이 키를 누르지 않아도 볼 수 있는 자동 데모로 연결하고,
평지 `scone-gait`를 **말단 프레임 연속 회전** 방식으로 다시 만든 과정에
집중한다.

## 1. 이번 작업에서 해결할 문제

요청을 코드 기준으로 다음 네 문제로 분리했다.

1. 루트 메뉴의 `시뮬레이션 조종`과 별개로 입력이 전혀 필요 없는
   `시뮬레이션 (자동 데모)`를 만들고, 계단에서 하드코딩 고정 회전과 개선형
   adaptive 알고리즘을 보여준다.
2. 교대 삼각보에서 세 다리만 지지할 때 MuJoCo 차체가 아래로 처지거나 크게
   출렁이는 현상을 줄인다.
3. 기존 model-based gait의 모터 profile 제한과 60 mm 보폭 때문에 PPO보다
   답답해 보이는 문제를 측정하고, 안정성을 잃지 않는 범위에서 속도·보폭을
   높인다.
4. 위치 목표를 ±30° 왕복하던 기존 `scone-gait` 대신 ID 13–18의 C자형
   말단을 실제 바퀴처럼 여러 바퀴 연속 회전시키고, 빈 개구부가 동시에
   지면을 향해 차체가 빠지는 문제를 phase로 줄인다.

## 2. 수정 경계

이번 수정은 의도적으로 다음 경계를 지킨다.

- 실물 Dynamixel controller와 기존 `Walk/Drive/Climb` 동작은 바꾸지 않는다.
- PPO replay와 RL 환경의 기본 PID·profile·기준 모션 의미를 바꾸지 않는다.
- 기존 PPO checkpoint는 학습 당시 `hardcoded` reference로 재생한다.
- 연속 회전은 position reference 18개로 표현할 수 없으므로 현재
  **비-RL MuJoCo 조종 전용**으로 둔다.
- RL의 `scone-gait` 선택에는 checkpoint 호환성을 위해 bounded-position
  `SconeGait` reference를 남긴다.
- 계단 자동 데모도 MuJoCo 전용이다. 실물 안전 동작이라고 주장하지 않는다.

이 분리는 중요하다. 같은 이름의 알고리즘을 개선했다는 이유로 과거 정책에
새 기준 모션을 얹으면 policy residual과 reference가 서로 상쇄돼 이전에
관찰된 꼬인 움직임이 재발할 수 있다.

## 3. 재현 방법

### 3.1 평지 보행 공통 조건

| 항목 | 값 |
|---|---|
| model | 현재 `src/assets/model.xml` |
| terrain | `flat` |
| base | floating |
| profile | `standard` |
| physics timestep | 0.002 s |
| gait command/update | `vx=0.18 m/s`, 0.02 s |
| 측정 구간 | 6.0 s |
| 방향 측정 | 측정 시작 시 body frame으로 world displacement 회전 |
| 안정 지표 | 시작 대비 min/max root Z, 최소 `R_zz` upright |
| IK 지표 | 실패 frame 수, 최소 backoff scale |
| 추종 지표 | position target과 실제 관절각의 최대 오차 |

각 후보는 새 model/data/controller에서 `SCONE.initialize()`를 다시 실행해
이전 후보의 contact phase나 속도가 다음 후보로 섞이지 않게 했다. 표의 값은
현재 deterministic model의 1회 실행이며 평균·분산이나 실물 성능이 아니다.

### 3.2 계단 공통 조건

계단은 `stair_benchmark.py`의 기존 동등조건을 그대로 사용했다.

- Walk 초기화 → 네 번 좌회전 → Drive 자세 → 0.5초 settle
- world `+Y` 방향 side-on 상승
- hardcoded는 여섯 lower를 fixed velocity 150으로 연속 회전
- improved는 `SconeStairClimber`의 roll-first + stall/pre-hook + tripod assist
- 최대 관찰 16초
- root Y와 root Z 상단 조건을 동시에 만족해야 성공

## 4. 왜 기존 `tripod-gait`가 느렸는가

기존 시뮬레이션 조종 경로는 초기화 후 다음 제한을 그대로 사용했다.

| 항목 | 기존 값 |
|---|---:|
| all-motor profile velocity | `walking_speed=100` |
| XM profile acceleration | `20` |
| gait cadence | `0.7 Hz` |
| 전후 stride | `60 mm` |
| 측면 stride | `50 mm` |

PPO 환경의 simulation controller profile은 학습 동역학을 보존하기 위해
기본적으로 무제한이다. 반면 non-RL 경로는 실물용 profile 값을 그대로
유한 제한으로 사용했다. 즉 gait planner가 50 Hz로 다음 자세를 만들어도
position setpoint가 그 속도를 따라가지 못했다.

또한 최대 전진 명령의 이론 stroke는 다음과 같다.

```text
stroke_requested = vx * duty_factor / cycle_frequency
                 = 0.18 * 0.5 / 0.7
                 = 0.1286 m
```

기존 상한은 0.060 m이므로 최대 명령에서 거의 모든 frame이 같은 포화
stroke를 냈다. 실험의 평균 `stride_clip_fraction=0.9867`은 이 현상을
확인한다. 명령을 더 세게 해도 보폭이 거의 커지지 않아 사용자에게 모터
이동 제약이 심한 것처럼 보였다.

## 5. 세 다리 지지 시 높이 출렁임 진단

### 5.1 관찰을 두 종류로 분리

처짐으로 보이는 현상에는 두 가지가 섞여 있었다.

1. 초기화 중 mode/torque/profile 전환에서 생기는 시작 transient
2. alternating tripod support가 바뀔 때 middle 관절 position tracking이
   실제 servo보다 부드러워 생기는 주기적 차체 높이 변화

초기 standing seed의 root Z는 약 0.1616 m였다. blocking 초기화 과정에서
순간적으로 약 0.0257 m까지 내려간 뒤, position loop가 회복하고 settle하면
약 0.1518 m가 됐다. 따라서 seed만 고치거나 gait foot target에 임의 상향
offset을 넣는 것은 원인을 해결하지 못한다.

### 5.2 실패: `qfrc_bias` 중력 feed-forward

첫 가설은 MuJoCo의 `qfrc_bias`를 actuator torque로 나눠 position PID 출력에
더하는 중력 보상이었다. 정지 middle 관절 오차를 줄일 것으로 예상했다.

실제 결과는 반대였다.

- settle root Z가 약 0.1518 m에서 약 0.1454 m로 더 낮아졌다.
- middle 평균 tracking error가 약 3.6°에서 약 6.8°로 커졌다.
- contact constraint와 이미 들어 있는 DC motor/PID 동역학에 generalized
  bias를 단순 중복 보상해 자세를 더 밀었다.

이 코드는 즉시 제거했다. 최종 controller에는 `qfrc_bias` feed-forward가 없다.

### 5.3 채택: model gait만 middle position stiffness 2배

두 번째 가설은 torque cap을 올리지 않고 ID 7–12의 position loop만 실제
servo hold에 가깝게 만드는 것이었다.

```text
kp_gait = 2.0 * kp_default
kd_gait = sqrt(2.0) * kd_default
```

`kd`를 같은 2배가 아니라 제곱근으로 올린 이유는 stiffness 증가를 따라
damping을 늘리면서 새 진동을 만들지 않기 위해서다. 이 adapter는
`MuJoCoController.set_gait_position_stiffness()`이며 다음 조건을 지킨다.

- 허용 배수 0.5–4.0
- 기본 대상 ID 7–12
- motor torque cap 불변
- `tripod-gait`와 continuous-roll simulation route만 opt-in
- PPO replay에는 자동 적용하지 않음

기존 100/20/0.7 Hz/60 mm 조건에서 6초 root Z 범위는 최대 +19.91 mm,
최소 0 mm였다. stiffness 2배 후 최대 +9.05 mm, 최소 −0.92 mm로 주기적
변화 폭이 약 절반이 됐다. 최종 160/50/0.8 Hz/80 mm 조건에서는 최소
−0.10 mm, 최대 +10.75 mm였다.

이 결과는 “차체 높이를 강제로 고정”한 것이 아니다. 세 접촉점과 C자 메시가
움직이는 이상 위쪽 variation은 남는다. 대신 support 전환 때 실제보다 과하게
무너지는 하방 tracking lag를 줄였다.

## 6. `tripod-gait` 속도·보폭 sweep

다음 후보를 모두 6초씩 실행했다.

| 후보 | speed/accel | cadence/stride | stiffness | 전진 | 평균 속도 | 측면 | min/max ΔZ | 최소 upright | 판정 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---|
| 기존 | 100/20 | 0.70 Hz/60 mm | 1.0 | 0.3833 m | 0.0639 m/s | +40.0 mm | 0/+19.91 mm | 0.99937 | 기준 |
| 기존 + hold | 100/20 | 0.70/60 | 2.0 | 0.4055 m | 0.0676 m/s | +37.9 mm | −0.92/+9.05 mm | 0.99961 | 높이 개선 |
| speed150 | 150/40 | 0.80/75 | 2.0 | 0.6075 m | 0.1013 m/s | −29.6 mm | −1.48/+9.38 mm | 0.99859 | 양호 |
| **speed160-stride80** | **160/50** | **0.80/80** | **2.0** | **0.6348 m** | **0.1058 m/s** | **+25.5 mm** | **−0.10/+10.75 mm** | **0.99866** | **채택** |
| speed175-cadence85 | 175/50 | 0.85/80 | 2.0 | 0.5382 m | 0.0897 m/s | +34.4 mm | −27.52/+12.96 mm | 0.99512 | 하방 붕괴, 기각 |
| stiffness2.5 | 150/40 | 0.80/75 | 2.5 | 0.6133 m | 0.1022 m/s | −33.6 mm | −2.73/+8.08 mm | 0.99846 | 추가 이득 작음 |
| speed200 | 200/60 | 0.90/80 | 2.0 | 0.5864 m | 0.0977 m/s | −50.2 mm | −19.96/+12.12 mm | 0.99259 | slip/붕괴, 기각 |
| unlimited | 0/0 | 1.00/90 | 2.0 | 0.7457 m | 0.1243 m/s | −7.2 mm | −0.12/+5.49 mm | 0.99990 | 빠르나 기존 PPO/실물 의미 혼동, 기각 |

모든 후보의 IK 실패 frame은 0이었다. `unlimited`는 수치가 가장 좋지만
profile 0이 DYNAMIXEL 의미상 무제한이고 IK backoff가 한 번 0.8까지 내려갔다.
이번 목표는 PPO 환경을 non-RL에 복사하는 것이 아니라 보수적인 유한 profile
안에서 답답함을 줄이는 것이므로 채택하지 않았다.

최종 `tripod-gait`는 기존 평균 0.0639 m/s에서 0.1058 m/s로 약 65.6% 빨라졌다.
80 mm 보폭은 기존보다 33.3% 크다. 측면/yaw는 별도 복합 명령 장기 sweep이
필요하므로 60 mm 측면 상한을 유지했다.

## 7. 기존 `scone-gait`가 tripod처럼 보인 이유

기존 `src/locomotion/SconeGait`는 `TripodGait.step()`의 18개 position target을
먼저 만든 뒤 lower target에 최대 30° sector sweep을 섞었다.

```text
lower_target = nominal_lower
             - polarity * roll_coordinate * 30 deg
```

한 cycle 뒤에는 같은 lower 각도로 되돌아오기 때문에 실제 C자 프레임은
계속 굴러가지 않고 앞뒤로 왕복했다. 상단/중단 발 궤적과 support phase는
그대로 tripod였으므로 외형과 속도가 일반 `tripod-gait`와 비슷한 것이
정상적인 결과였다.

이 bounded 방식은 RL 기준 모션에는 장점이 있다. 매 frame 0–360°의 18개
position target을 반환해 residual을 더할 수 있기 때문이다. 그러나 SCONE
말단을 바퀴처럼 사용하는 실제 non-RL 제어에는 맞지 않는다.

따라서 클래스 하나를 억지로 두 용도에 쓰지 않고 다음처럼 분리했다.

| 용도 | 구현 | lower 의미 |
|---|---|---|
| RL reference/호환 | `locomotion.SconeGait` | bounded position sweep |
| 비-RL MuJoCo `scone-gait` 조종 | `simulation.core.SconeRollingGait` | continuous velocity |

## 8. 연속 회전형 `scone-gait` 설계

### 8.1 상단/중단 stabilizer

`SconeRollingGait`는 작은 `SconeGait` planner를 내부에 둔다. 이 planner의
lower position 출력은 보내지 않고 ID 1–12만 사용한다.

| 필드 | 최종 값 |
|---|---:|
| cadence | 0.8 Hz |
| duty factor | 0.64 |
| step height | 4 mm |
| 전후/측면 stabilizer stride | 25/25 mm |
| upper steering blend | 0.20 |
| steering limit | 45° |
| command/velocity filter τ | 0.10 s |
| IK tolerance/backoff | 1 mm / 4회 |

큰 35 mm lift와 60–80 mm Cartesian stroke를 lower 연속 회전과 동시에 쓰면
같은 추진을 두 번 요구하고 차체 흔들림이 커졌다. 따라서 upper/middle은
몸체 자세와 조향만 보조하는 작은 creep trajectory로 줄였다.

### 8.2 lower 연속 회전 식

각 다리의 접촉 tangent는 기존 mesh 기반 `steering_solution()`으로 계산한다.
다리 `i`의 목표 raw velocity는 다음과 같다.

```text
activity = max(|vx|/vx_max, |vy|/vy_max, |yaw|/yaw_max)

phase_ratio_i = 0.80  if i is stance
                1.00  if i is swing

v_lower,i* = -polarity_i
             * 175
             * activity
             * alignment_i
             * phase_ratio_i
```

`-polarity`는 mesh active contact가 움직이는 방향과 ground reaction으로
차체가 추진되는 방향이 반대이기 때문이다. 출력은 다음 1차 filter로
부드럽게 만든다.

```text
alpha = 1 - exp(-dt / 0.10)
v_i <- v_i + alpha * (v_lower,i* - v_i)
```

ID 13–18은 `VELOCITY` mode라 6초 측정에서 평균 3.05회전했다. 기존 ±30°
왕복과 달리 한 cycle에서 되돌아오지 않는다.

### 8.3 C자 개구부 동시 접지 문제

C자 메시의 약 135° 빈 개구부가 여섯 다리에서 같은 phase면 여섯 support
반경이 동시에 작아질 수 있다. no-stagger 실험은 빠르게 전진했지만 root Z가
최대 63.5 mm 내려가 사용자가 지적한 “세 다리 지지 때 몸이 빠지는” 문제를
오히려 키웠다.

최종 해결은 tripod A `(1,4,5)`를 nominal lower angle로 두고 tripod B
`(2,3,6)`를 시작 전에 +72° 회전시키는 것이다.

```text
q_lower,start,i = 255 deg                  i in tripod A
                  255 deg + 72 deg = 327   i in tripod B
```

두 support group의 개구 phase를 벌려 한 tripod가 빈 구간을 지날 때 반대
tripod의 호가 지면을 받도록 한다. 시작 phase position에 도달한 뒤에만 여섯
lower를 velocity mode로 전환한다.

## 9. 연속 회전 phase/속도 가설 전부

모든 행은 6초, `vx=0.18 m/s`, lower 평균 약 3.05–3.14회전이다.

| 후보 | phase | 전진/속도 | 측면 | min/max ΔZ | 최소 upright | 판정 |
|---|---|---:|---:|---:|---:|---|
| no stagger | 모두 0° | 0.9642 m / 0.1607 m/s | −24.2 mm | **−63.52/+21.11 mm** | 0.9821 | 동시 개구, 기각 |
| arbitrary six-way | 0/60/120/30/90/150° | 1.1011 / 0.1835 | **−44.4 mm** | −49.11/+25.65 mm | **0.9642** | 빠르나 drift/tilt, 기각 |
| tripod 45° | B +45° | 0.9795 / 0.1632 | +33.7 mm | −28.75/+21.11 mm | 0.9796 | 지지 부족 |
| tripod 60° | B +60° | 1.0437 / 0.1739 | +19.9 mm | −19.47/+19.53 mm | 0.9851 | 개선 |
| tripod 67.5° | B +67.5° | 1.0999 / 0.1833 | +15.6 mm | −15.54/+19.26 mm | **0.9898** | 빠르나 drift 남음 |
| **175/B72/steer20** | **B +72°** | **0.9788 / 0.1631** | **−10.6 mm** | **−12.97/+20.22 mm** | **0.9891** | **채택** |
| 180/B72/steer20 | B +72° | 1.0178 / 0.1696 | +7.4 mm | −13.25/+20.42 mm | 0.9857 | 속도 이득 대비 upright 감소 |
| 180/B75/steer15 | B +75° | 0.9195 / 0.1532 | +5.7 mm | −11.50/+21.14 mm | 0.9793 | drift 좋으나 tilt/속도 기각 |
| 180/B67.5/low | B +67.5°, stride20/lift3 | 0.9859 / 0.1643 | +5.3 mm | −15.46/+21.06 mm | 0.9878 | 높이 이득 없음 |
| 180/B90/steer20 | B +90° | 1.1153 / 0.1859 | +27.7 mm | −23.11/+19.96 mm | 0.9781 | 빠르나 drift/drop, 기각 |

72° 후보는 no-stagger의 하방 변화 63.52 mm를 12.97 mm로 약 79.6% 줄였다.
180보다 175를 택한 이유는 최고 직진 수치보다 upright, lateral drift, phase
민감도를 함께 보수적으로 선택했기 때문이다.

최종 continuous-roll 속도 0.1631 m/s는 기존 `tripod-gait` 0.0639 m/s의
약 2.55배다. 이 값은 현 model/contact의 단일 run이며 실물 속도 약속이 아니다.

## 10. 입력 없는 자동 계단 데모

### 10.1 루트 메뉴

루트 launcher를 다음 두 메뉴로 분리했다.

```text
시뮬레이션 (자동 데모)
시뮬레이션 조종
```

자동 데모는 keyboard joystick을 만들지 않는다. 사용자는 전략과 계단만
고르고 MuJoCo viewer에서 자동 동작을 본다.

### 10.2 전략

| 표시 | 코드 | 동작 |
|---|---|---|
| 하드코딩 | `hardcoded` | 상태·진행량 feedback 없이 lower 6개를 velocity 150으로 고정 회전 |
| 개선형 | `improved` | 정상 진행은 같은 rolling, tall/pre-hook 또는 0.8초에 25 mm 미만이면 tripod assist |
| 비교 | `compare` | hardcoded viewer 종료 후 improved viewer를 같은 terrain에서 순서대로 실행 |

기본 terrain은 `stairs-2`다. 이 preset에서는 두 방식 모두 상단에 올라가므로
사용자가 “하드코딩과 개선형이 계단을 오르는 모습”을 바로 비교할 수 있다.
차이를 분명히 보려면 `stairs-3`를 고른다. 이때 hardcoded는 제한 시간 안에
상단 판정을 얻지 못하고 improved는 한 번 assist해 통과한다.

### 10.3 자동 실행 순서

```text
viewer main thread
  controller.update(0.002)
  -> mj_step
  -> viewer.sync

automatic worker
  SCONE.initialize
  -> 좌회전 4회
  -> Drive posture
  -> hardcoded.update 또는 SconeStairClimber.update(0.02)
  -> top Y/Z 판정
  -> lower velocity 0
  -> 최종 자세 1.5초 표시
```

상단 판정은 benchmark와 같은 식을 공유한다.

```text
y_root >= 0.35 + sum(last 이전 tread) + 0.4 * last tread
z_root >= z_start + 0.70 * total height
```

### 10.4 실행법

macOS passive viewer는 main thread 제약 때문에 `mjpython`을 사용한다.

```bash
mjpython SCONE.py
# 시뮬레이션 (자동 데모) -> 비교 -> stairs-2
```

직접 실행:

```bash
mjpython -m src.simulation --demo compare --terrain stairs-2
mjpython -m src.simulation --demo hardcoded --terrain stairs-1
mjpython -m src.simulation --demo improved --terrain stairs-3
```

`--demo`에서 terrain을 생략하면 control CLI의 평지 기본값을 자동으로
`stairs-2`로 바꾼다. `flat`, slope, uneven, mixed를 명시하면 stair 전용
경계 검사에서 거부한다.

## 11. 계단 재검증 결과

2026-09-01에 현재 코드로 H0 hardcoded와 H4 improved를 세 preset에서 다시
실행했다.

| terrain | hardcoded | improved | improved assist | 해석 |
|---|---:|---:|---:|---|
| stairs-1 | 3.278 s, 18.779 J | 3.278 s, 18.779 J | 0 | rolling만 사용 |
| stairs-2 | 3.408 s, 23.225 J | 3.408 s, 23.225 J | 0 | rolling만 사용 |
| stairs-3 | 16초 내 실패, y=0.810/z=0.216 m | 4.718 s, 50.861 J | 1 | 높은 단에서만 tripod assist |

improved의 상단까지 자세/contact 지표:

| terrain | 최소 upright | peak contact force |
|---|---:|---:|
| stairs-1 | 0.987 | 37.495 N |
| stairs-2 | 0.965 | 50.396 N |
| stairs-3 | 0.870 | 86.372 N |

쉬운 계단에서 개선형 수치가 hardcoded와 완전히 같은 것은 버그가 아니다.
정체가 없으면 assist를 켜지 않아 SCONE의 단순 연속 회전 장점을 그대로
사용하도록 설계했기 때문이다.

## 12. 구현 파일과 역할

| 파일 | 이번 역할 |
|---|---|
| `src/simulation/core/controller.py` | model gait opt-in middle stiffness API |
| `src/simulation/core/cli_bridge.py` | 160/50/2x tripod tuning, interactive `scone-gait`를 continuous roll로 route |
| `src/simulation/core/scone_rolling_gait.py` | phase stagger, upper/middle stabilizer, lower velocity controller |
| `src/simulation/core/stair_demo.py` | no-feedback baseline, adaptive automatic worker, sequential compare viewer |
| `src/simulation/core/simulator_cli.py` | `--demo`, strategy/terrain selector |
| `src/cli.py` | 자동 데모와 조종 메뉴 분리 |
| `tests/test_scone_rolling_gait.py` | 6초 연속 회전, 3회전 이상, 전진/높이/upright/IK 회귀 |
| `tests/test_stair_demo.py` | 전략 경계, no-feedback lower mode, CLI route |
| `tests/test_simulation.py` | 80 mm gait, opt-in stiffness, 높이/속도 회귀 |

## 13. 테스트와 판정 기준

새 회귀 테스트의 핵심 기준은 다음과 같다.

### `tripod-gait`

- 3초 전진 > 0.25 m
- 모든 frame IK 수렴
- 측정 시작 대비 root Z 하방 변화 > −5 mm
- 최종 upright > 0.98

### continuous-roll `scone-gait`

- 6초 전진 > 0.80 m
- 측면 drift < 30 mm
- root Z 하방 변화 > −20 mm
- 최소 upright > 0.98
- lower 평균 회전 > 2.5회
- stabilizer IK 실패 0

### 자동 데모

- `hardcoded/improved/compare` 외 전략 거부
- stair preset 외 terrain 거부
- hardcoded는 lower 6개만 velocity mode로 전환하고 feedback state를 갖지 않음
- direct `--demo compare`는 생략 terrain을 `stairs-2`로 선택
- 루트 launcher 자동 데모 route와 기존 RL 조종 route를 각각 검증

### 실제 viewer smoke

다음 명령을 `mjpython`으로 실제 실행했다.

```bash
mjpython -m src.simulation --demo compare --terrain stairs-2
```

hardcoded viewer가 먼저 자동 실행돼 상단을 3.68초에 판정하고 종료한 뒤,
improved viewer가 열려 같은 3.68초에 상단을 판정하고 종료했다. 두 실행 모두
최종 lower 속도를 0으로 보냈고 `assist=0`이었다. 이 값은 wall-clock worker와
실시간 viewer pacing이 들어간 GUI smoke 결과라, physics-only benchmark의
3.408초를 대체하는 성능 수치로 사용하지 않는다. 화면 픽셀을 자동 분석한 것은
아니며 process route·자동 동작·상단 state 판정·순차 종료를 확인한 결과다.

### 전체 회귀

```bash
python -m compileall -q SCONE.py src tests
python -m unittest discover -s tests -v
```

전체 122개 테스트가 통과했다. 그 뒤 config validation과 phase 준비 순서를
정리한 최종 소규모 변경은 관련 simulation/rolling/demo 17개 테스트를 다시
통과했다. 시스템 Python 환경에는 별도 `pytest` package가 없어 이 저장소가
기준으로 사용하는 `unittest` discovery로 실행했다.

## 14. 시간 순 시행 기록

1. 기존 route와 config를 읽어 PPO replay와 non-RL profile 차이를 확인했다.
2. 기존 최대 전진 6초를 재현해 0.3833 m, 98.67% stride clipping을 측정했다.
3. 세 다리 support 처짐을 startup transient와 gait tracking으로 분리했다.
4. `qfrc_bias` gravity feed-forward를 넣어 봤으나 height/error가 악화돼 제거했다.
5. middle stiffness 2배를 적용해 Z variation을 약 절반으로 줄였다.
6. speed/cadence/stride/stiffness 후보 8개를 각각 새 simulation에서 실행했다.
7. unlimited와 speed200은 빠르거나 공격적이지만 호환성·drop 때문에 기각했다.
8. 유한 160/50, 0.8 Hz, 80 mm, stiffness 2배를 tripod 최종값으로 선택했다.
9. 기존 `SconeGait`가 lower를 왕복 position으로만 움직여 tripod처럼 보이는
   구조적 원인을 확인했다.
10. lower velocity mode prototype을 만들어 6초 3회전 이상을 확인했다.
11. 여섯 sector를 같은 phase로 돌렸더니 root Z가 63.5 mm 빠지는 실패를
    재현했다.
12. arbitrary six-way와 tripod B 45/60/67.5/72/75/90° phase를 모두 실행했다.
13. 최고 속도만 택하지 않고 height/upright/lateral을 함께 보고 72°/175를
    선택했다.
14. RL bounded reference와 simulation continuous controller를 별도 클래스로
    분리했다.
15. 자동 hardcoded/improved/compare viewer route를 구현했다.
16. stairs-1/2/3에서 hardcoded와 adaptive를 다시 benchmark했다.
17. 단위·동역학·launcher 회귀 테스트를 추가하고 전체 suite로 검증했다.

## 15. 남은 한계와 다음 실험

- 72°는 현재 mesh/friction/Standard 자세의 단일 deterministic 최적점이다.
  표면·질량·TPU compliance가 바뀌면 다시 sweep해야 한다.
- continuous-roll은 평지 최대 전진을 중심으로 골랐다. `vx+vy+yaw` 복합 명령,
  uneven, slope, 긴 run은 추가 GUI/접촉 로그가 필요하다.
- root Z 하방 변화는 13 mm까지 줄었지만 0은 아니다. C자 open sector 구조에서
  완전한 constant-radius wheel과 같은 높이를 기대할 수 없다.
- 자동 viewer는 사람이 직접 보게 하는 기능이며 영상 녹화·화면 기반 자동
  판정은 구현하지 않았다. 성공 판정은 물리 state의 root Y/Z를 사용한다.
- simulation `kp` 보정은 실제 servo hold를 정확히 식별한 system ID가 아니다.
  실물 step response를 계측한 뒤 PID/모터 모델을 다시 맞춰야 한다.
- continuous lower velocity를 RL에 쓰려면 action/reference 표현부터 position
  residual이 아닌 hybrid position+velocity로 새로 설계하고 0 step부터
  학습해야 한다. 기존 PPO checkpoint에 연결하면 안 된다.
- stairs-3 upright 0.870과 86.4 N peak는 통과만 보여 주는 값이다. 더 빠른
  `fast` hybrid는 281 N peak가 나와 이미 기각했다. 실물 적용 전 current,
  nosing, overhang, 마찰, 반복 성공률을 반드시 측정해야 한다.

## 16. 최종 결론

이번 문제는 단순히 stride 한 값을 올리는 것으로 해결되지 않았다.

- `tripod-gait`는 유한 motor profile 안에서 80 mm/0.8 Hz와 middle hold를
  함께 조정해 65.6% 빨라지고 하방 처짐을 억제했다.
- `scone-gait`는 bounded lower position 왕복을 유지하면 구조적으로 tripod와
  비슷할 수밖에 없었다. simulation 조종 경로를 continuous velocity로 분리해
  SCONE의 부채꼴 말단을 실제로 여러 바퀴 굴리도록 바꿨다.
- C자 개구부 때문에 단순 동기 회전은 빠르지만 차체가 63.5 mm 빠졌다.
  대각 tripod 72° phase stagger로 이를 13.0 mm까지 줄였다.
- 계단은 복잡한 보행을 항상 켜지 않고, 쉬운 구간은 고정 rolling과 같은
  속도로 지나가며 높은 단에서만 tripod hook을 켜는 adaptive 방식이 SCONE의
  장점을 가장 잘 보존했다.
- 자동 데모를 별도 메뉴로 만들어 이 차이를 키보드 조종 없이 직접 볼 수
  있게 했다.
