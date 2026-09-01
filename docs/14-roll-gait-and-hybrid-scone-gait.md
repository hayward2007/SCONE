# `roll-gait` 분리와 PPO/점접지 하이브리드 `scone-gait`

이 문서는 2026-09-01에 적용한 이름 교정, 새 평지 보행 제어, 계단 준비
상태기 복구, 실물 1단 모터 설정 검증을 현재 코드 기준으로 설명한다. 이전
연속 회전 실험의 시간 순 기록은
[`12-automatic-stair-demo-and-continuous-roll-rework.md`](12-automatic-stair-demo-and-continuous-roll-rework.md)에
남아 있지만, 실행 이름과 현재 route는 이 문서가 우선한다.

## 1. 최종 이름과 역할

| CLI 이름 | 핵심 제어 | checkpoint | 말단 모드 | 현재 용도 |
|---|---|---:|---|---|
| `tripod-gait` | 고전 교대 삼각보 + IK | 불필요 | position | 비-RL 기준 보행 |
| `roll-gait` | 기본 보행 + 여섯 C-frame 연속 회전 | 불필요 | velocity | 기존 `scone-gait`의 정확한 새 이름 |
| `scone-gait` | 저속/제자리 yaw PPO + 고속 점접지/누적 말단회전 | 필수 | multi-turn position | 새 하이브리드 보행 |
| `rl` | 선택한 residual reference + PPO | 필수 | position | 순수 PPO 재생 |
| `scone-stair` | 여섯 말단 공통 위상 계단 모션 | 불필요 | extended position | MuJoCo 계단 전용 |

기존 `SconeRollingGait`, `SconeRollingGaitConfig`,
`SconeRollingSample` Python 이름은 외부 import가 즉시 깨지지 않도록 alias로만
남겼다. 새 코드의 정식 이름은 `RollGait`, `RollGaitConfig`,
`RollGaitSample`이다. `SimulationControl.SCONE_GAIT`와
`SimulationControl.ROLL_GAIT`는 서로 다른 enum이다.

## 2. 왜 기존 동작을 `roll-gait`로 바꿨는가

기존 시뮬레이션 `scone-gait`는 ID 13–18을 velocity mode로 전환하고 명령이
있는 동안 계속 회전했다. 상단과 1단의 기본 보행 및 하단 기본 보행 속도
성분도 들어 있었지만, 화면에서 가장 큰 동작은 여러 바퀴 도는 C-frame이었다.
따라서 “한 점을 지지하는 보행과 필요한 구간의 말단 회전을 섞는다”는 뜻보다
주행형 rolling에 가까웠다.

현재 `roll-gait`는 이 동작을 그대로 보존한다.

```text
upper/stage-1 target = bounded SconeGait IK position
lower velocity       = continuous roll velocity
                     + d(bounded lower offset)/dt × blend
```

`roll-gait`는 연속 회전 성능 비교와 C-frame 주행 실험에 쓴다. 새
`scone-gait`와 섞어 해석하지 않는다.

## 3. 새 `scone-gait`의 전체 데이터 흐름

새 route는 PPO와 model-based gait가 서로 다른 MuJoCo 인스턴스를 열지 않는다.
하나의 `SconeWalkEnv`, 하나의 `MuJoCoController`, 하나의 50 Hz loop를 공유한다.

```text
[vx, vy, yaw_rate]
  ├─ PPO observation → checkpoint → residual action
  └─ SconeGait → tripod IK + phase-gated multi-turn sector reference
             ↓ speed-dependent smooth blend
  SconeWalkEnv reference + attenuated residual
             ↓
  18 position targets → profile/PID/dcmotor → MuJoCo
```

구현 위치는 다음과 같다.

- route와 checkpoint 요구: [`src/simulation/core/cli_bridge.py`](../src/simulation/core/cli_bridge.py)
- PPO/하이브리드 전환: [`src/rl/joystick_control.py`](../src/rl/joystick_control.py)
- replay-only reference 합성: [`src/rl/walk_learn.py`](../src/rl/walk_learn.py)
- 점접지/말단 회전 궤적: [`src/locomotion/scone_gait.py`](../src/locomotion/scone_gait.py)
- 기존 연속 회전: [`src/simulation/core/scone_rolling_gait.py`](../src/simulation/core/scone_rolling_gait.py)

## 4. 저속 PPO와 고속 hybrid 전환식

평면 명령 속도를 다음처럼 정의한다.

```text
v = sqrt(vx² + vy²)
v0 = 0.10 m/s
v1 = 0.18 m/s
u = clamp((v - v0) / (v1 - v0), 0, 1)
b = 3u² - 2u³
```

`b`는 hybrid 비율이다.

- `v <= 0.10`: `b=0`, checkpoint의 원래 reference와 PPO residual만 사용
- `0.10 < v < 0.18`: PPO와 새 reference를 smoothstep으로 연속 합성
- `v >= 0.18`: `b=1`, 고속 model reference가 동작하고 PPO residual은 0
- 제자리 yaw: `vx=vy=0`이므로 yaw 크기와 무관하게 항상 `b=0`, PPO가 담당

reference와 residual 합성은 다음과 같다.

```text
q_ref = (1-b) q_checkpoint_reference + b q_scone_hybrid
a_used = (1-b) a_ppo
q_target = q_ref + residual_scale ⊙ a_used
```

전환 중 lower operating mode를 바꾸지 않는다. 모든 관절은 RL 환경의 position
경로를 유지하므로 velocity/position mode 전환 충격이 없다. hybrid가 처음
켜지는 frame에는 `SconeGait` phase를 현재 RL phase에 맞춘다.

## 5. 한 점 지지 + 실제 누적 말단 회전

### 5.1 이전 bounded 식이 회전처럼 보이지 않은 원인

최초 구현은 stance 후반에 약 `20~42°` 움직인 뒤 swing에서 같은 각도를
반대로 되돌렸다. 한 cycle의 순 회전량이 0이어서 코드상 lower 각도 변화는
있어도 화면에서는 일반적인 2단 보행 진동처럼 보였다.

현재 고속 `scone-gait`는 기본 몸통, 상단, 1단, 2단 IK 보행을 그대로 수행하고
말단 회전각만 **한 방향으로 누적**한다. bounded `roll_coordinate()`는 학습용
`scone-gait` reference 호환을 위해 남아 있지만, interactive high-speed route는
`continuous_rotation=True`와 아래 식을 쓴다.

### 5.2 다리별 필요한 말단 회전속도

몸체 중심 기준 다리 `i`의 접점 위치를 `(x_i,y_i)`, 조종 명령을
`(v_x,v_y,omega_z)`라 하면 접점에서 필요한 평면 속도는 다음과 같다.

```text
u_i = [v_x - omega_z y_i, v_y + omega_z x_i]
omega_roll,i* = clamp((180/pi) ||u_i|| / R_eff, 0, omega_max)
R_eff = 0.1225 m
omega_max = 360 deg/s
```

`R_eff`는 현재 C-frame 외반지름이다. 실제 TPU 변형과 미끄럼이 있으므로 이는
실물 no-slip 보장이 아니라 MuJoCo 시작값이다. 각 다리의 CAD 축 방향이
다르므로 `steering_solution()`이 접선 방향을 측정하고
`sigma_i in {-1,+1}` 극성을 정한다.

### 5.3 점접지와 회전을 나누는 위상 gate

stance 진행률을 `p`, swing 진행률을 `s`, 점접지 비율을 `rho=0.55`, swing
회전 유지 비율을 `eta=0.70`, quintic smoothstep을
`S(x)=10x³-15x⁴+6x⁵`라 한다.

```text
g_stance(p) = 0                                      (p <= rho)
g_stance(p) = S((p-rho)/(1-rho))                    (p > rho)

g_swing(s)  = 1                                      (s <= eta)
g_swing(s)  = 1 - S((s-eta)/(1-eta))                (s > eta)
```

- stance 앞 55%: `g=0`, 말단 추가 회전이 없어 한 점 지지
- stance 뒤 45%: 접지 상태에서 회전을 가속해 추진력 추가
- swing 앞 70%: 하중이 빠진 동안 회전을 계속 진행
- swing 뒤 30%: 다음 착지 전에 회전속도를 0으로 감속

다리별 누적 회전과 최종 lower 목표는 다음과 같다.

```text
Theta_i[k+1] = Theta_i[k]
             - sigma_i A_i omega_roll,i* g_i(phi_i) Delta_t

q_lower,i = q_IK,lower,i + Theta_i
```

`A_i`는 현재 접선과 명령 방향의 alignment다. `Theta_i`는 매 cycle 0으로
되돌리지 않으므로 실제 MuJoCo 관절도 360°를 넘어 계속 회전한다. 반면
`q_IK,lower,i`가 남아 있으므로 2단 기본 보행과 회전이 동시에 합성된다.

### 5.4 PPO reference와 다회전 branch 합성

checkpoint 기준각이 `255°`, hybrid 목표가 `615°`이면 물리적으로 같은 말단
방향인데 숫자만 직접 보간할 경우 불필요한 역회전이 생긴다. 따라서 먼저
checkpoint lower를 hybrid 목표에 가장 가까운 360° branch로 옮긴다.

```text
q_base,eq = q_base + 360 round((q_hybrid - q_base) / 360)
q_ref     = (1-b) q_base,eq + b q_hybrid
```

실제 MuJoCo qpos와 목표는 multi-turn 상태를 유지한다. checkpoint 관측과
joint-limit 계산에서만 lower 각도를 동등한 `[-180°,180°)` 위상으로 접는다.
이 예외는 interactive simulation hybrid에만 적용하며 upper/1단 보호 범위는
그대로 유지한다.

고속 route의 현재 설정은 다음과 같다.

| 변수 | 값 | 역할 |
|---|---:|---|
| `cycle_frequency` | `1.2 Hz` | 최대 명령에서 채택한 cadence |
| `duty_factor` | `0.60` | 한 tripod의 stance 비율 |
| `point_support_ratio` | `0.55` | stance 중 추가 sector phase 고정 비율 |
| `swing_roll_hold_ratio` | `0.70` | 무부하 swing에서 회전을 유지하는 비율 |
| `step_height` | `25 mm` | swing clearance |
| `max_stride` | `90 mm` | 전후 최대 Cartesian stroke |
| `max_lateral_stride` | `70 mm` | 측면 최대 stroke |
| `effective_roll_radius` | `122.5 mm` | 속도→말단 각속도 변환 반지름 |
| `max_roll_rate_degrees` | `360°/s` | multi-turn 목표 각속도 상한 |

이 수치는 MuJoCo 검증값이지 실물 안전 한계가 아니다.

## 6. 계단 준비 상태기 복구

기존 사용자 정의 계단 준비는 네 번 좌회전한 뒤 `Walk → Drive`까지만 실행하고
곧바로 `SconeStairClimber`가 lower 제어를 덮었다. 실제 Legacy 상태기의
`Drive → Climb` 준비가 빠져 있었다.

현재 `prepare_scone_stair_pose()`는 다음 순서를 강제한다.

```text
Walk
  → 네 번 left로 계단 +Y 방향에 side-on 정렬
  → change_mode(): Drive 준비 완료 및 centre 확인
  → change_mode(): Climb tripod 준비 완료 및 centre 확인
  → 앞 stage-1 brace
  → 여섯 C-frame 공통 위상 획득
  → synchronized stair motion 활성화
```

두 전환 중 하나라도 예상 mode 이름을 반환하지 않으면 계단 제어를 시작하지
않는다. 자동 데모, joystick 계단 제어, benchmark가 같은 준비 함수를 쓴다.

## 7. Drive 1단 모터 설정 재검증

### 7.1 코드가 보장하는 설정

ID 7–12는 `XM430-W350-T` stage-1이다. 초기화와 Walk→Drive 전환 뒤의 live
기대값은 다음과 같다.

| register/상태 | 기대값 |
|---|---:|
| operating mode | `POSITION (3)` |
| torque enable | `ON (1)` |
| profile acceleration | `20` |
| profile velocity | `safety_speed=50` |
| goal position | `2048 raw = 180°` |
| present position | `2048 ± 64 raw` |

`SCONE.initialize()`는 이제 lower뿐 아니라 모든 XM, 즉 ID 7–18의 position
mode를 명시한다. 외부 도구나 이전 세션이 stage-1 mode를 바꿔 두었더라도
초기화가 position mode로 복구한다.

물리 `Controller.wait_until_raw_positions()`도 추가했다. Legacy 전환이 이미
사용하던 settle hook이 이제 실물에서도 present-position을 읽어 목표 도달을
확인한다. Drive 객체가 만들어지면 `verify_drive_stage1_settings()`가 위 여섯
register를 다시 read-back하고 하나라도 다르면 motion을 계속하지 않고
`ControllerError`를 낸다. 이 검사는 쓰기 동작이 아니다.

### 7.2 흔들림이 남아도 자동으로 “문제 없음”은 아니다

코드의 read-back이 통과하면 적어도 mode, torque, profile, 목표/현재 위치
설정 오류는 아니다. 그러나 현재 물리 adapter는 XM의 position P/I/D gain,
current, hardware error status를 읽거나 설정하지 않는다. 따라서 read-back
통과 후 흔들림은 다음을 따로 확인해야 한다.

- horn, 볼트, 프레임 유격과 케이블 장력
- 하중 상태에서 goal/present position의 진폭과 주파수
- servo current, temperature, input voltage, hardware error status
- 실제 XM position gain과 return delay/bus update 주기

MuJoCo의 Drive stage-1 `kd` 2배는 시뮬레이션 전용 adapter다. 실물 gain이
2배로 바뀌었다는 뜻이 아니다. read-back 통과와 기계 점검 뒤 작고 감쇠되는
움직임이면 정상 compliance일 수 있지만, 진폭이 커지거나 계속 증폭되면
정상으로 간주하지 않는다.

## 8. checkpoint 호환성

새 `scone-gait`는 checkpoint가 필수다. 저속/yaw 구간에서 policy가 학습 때의
reference를 그대로 전제로 하기 때문이다. `scone_walk_15410928_steps.zip`은
이번 검증에서 `hardcoded` reference와 Standard stance로 재생했다.

고속 구간은 학습 reference를 몰래 바꾸고 PPO residual을 그대로 더하지 않는다.
`b`가 커지는 만큼 PPO residual을 줄이고 full hybrid에서는 0으로 만든 이유가
바로 reference/action 상쇄를 막기 위해서다. 다만 이는 replay-time
supervisor이며, hybrid reference로 학습된 PPO라고 주장할 수 없다. 향후 PPO가
고속 hybrid 오차까지 보정하게 하려면 이 환경 버전으로 0 step부터 새로
학습해야 한다. 기존 checkpoint resume는 금지한다.

## 9. 실행 방법

통합 launcher가 가장 안전하다.

```bash
mjpython SCONE.py
```

`시뮬레이션 조종 → scone-gait`를 선택한 뒤 checkpoint, 원래 reference,
standing pose, terrain을 선택한다. 연속 회전만 비교하려면 `roll-gait`를
선택하며 checkpoint는 묻지 않는다.

직접 실행 예시는 다음과 같다.

```bash
# 기존 연속 회전
mjpython -m src.simulation --control roll-gait --profile standard --terrain flat

# 15.4M Standard/hardcoded checkpoint의 새 hybrid route
mjpython -m src.simulation \
  --control scone-gait \
  --checkpoint runs/walk_full_standard/checkpoints/scone_walk_15410928_steps.zip \
  --rl-reference-motion hardcoded \
  --rl-standing-pose-degrees \
    135 135 180 180 225 225 \
    240 240 240 240 240 240 \
    255 255 255 255 255 255
```

HUD에는 `scone-gait/ppo`, `scone-gait/mix-X/roll-Yturn`,
`scone-gait/hybrid/roll-Yturn`이 표시돼 현재 supervisor 상태와 누적 회전수를
확인할 수 있다.

## 10. 실행한 검증과 결과

### 10.1 자동 테스트

이전 bounded 구현은 전체 134개 테스트를 통과했다. 누적 회전 수정에서는
다음 항목을 새 회귀 테스트로 추가했고 최종 전체 137개 테스트가 통과했다.

- stance 앞 55%에서 누적 회전 gate가 정확히 `0`임
- 최대 전진 4초 동안 누적 각도가 `250°`를 넘고 부호가 역전되지 않음
- multi-turn lower target과 PPO 기준각을 같은 360° branch에서 합성함
- 제자리 yaw와 `0.08 m/s` 저속 명령에서 PPO action이 그대로 통과함
- `0.5 m/s` 명령에서 hybrid reference가 100% 적용되고 PPO residual이 0임
- 전환 band가 smoothstep의 `0 → 0.5 → 1`을 만족함
- `scone-gait`와 `roll-gait` route가 분리됨
- 계단 준비가 정확히 Drive, Climb 두 전환을 순서대로 호출함
- 초기화가 ID 7–18을 position mode로 명시함
- physical stage-1 live register 정상/오류 read-back을 구분함

### 10.2 `scone_walk_15410928_steps.zip` headless 4초 검증

공통 조건은 flat, Standard stance, hardcoded checkpoint reference, seed 7,
50 Hz policy/500 Hz physics다. 아래 값은 deterministic 1회이며 실물 성능이나
통계적 평균이 아니다.

| 명령/route | body X | body Y | yaw | 종료 |
|---|---:|---:|---:|---|
| PPO 저속 `vx=0.06` | `+0.0704 m` | `+0.1154 m` | `-11.25°` | 없음 |
| PPO 제자리 `yaw=0.6` | `+0.1290 m` | `-0.1977 m` | `+119.96°` | 없음 |
| 이전 bounded hybrid 최대 `vx=0.5` | `+0.456 m` | `-0.013 m` | `-2.2°` | 없음 |
| 현재 누적회전 hybrid 최대 `vx=0.5` | `+0.298 m` | `+0.0119 m` | `-4.11°` | 없음 |
| 기존 PPO 최대 `vx=0.5` 비교 | `+0.918 m` | `+0.103 m` | `-11.7°` | 없음 |

현재 누적회전 hybrid에서 실제 ID 13–18 회전량은 4초 동안
`+436.8/+434.1/+342.4/+344.8/-432.7/-460.3°`였다. 동시에 ID 1–12의
관절별 peak-to-peak도 `14.0~24.1°`였으므로 “회전만” 또는 “보행만”이 아니라
두 동작이 실제로 함께 발생했다. `fallen`, forbidden collision, hard joint
limit는 모두 false였다.

역방향 `vx=-0.5`도 별도 4초 실행에서 종료 없이 body-X `-0.407 m`였고 말단
회전 부호가 전진과 반대로 바뀌었다. 주어진 PPO보다 절대 전진 속도는 여전히
느리지만 lateral/yaw drift는 작다. 따라서 현재 결과는 “더 빠른 PPO”가 아니라
“빠른 입력에서 점접지 보행과 실제 multi-turn 말단 회전을 동시에 수행하는
supervisor”로 해석해야 한다. 저속 PPO와 제자리 yaw의 translation drift는
해당 checkpoint 자체의 후속 재학습 과제다.

같은 checkpoint와 `vx=0.5` 명령을 macOS `mjpython` human viewer에서도
실시간 pace로 200 frame(4초) 재생했다. 비정상 종료 없이 끝났고 마지막 HUD는
`scone-gait/hybrid/roll-1.3turn`이었다. 즉 headless 관절 계측뿐 아니라 GUI
재생에서도 말단이 한 바퀴 이상 누적 회전하는 route가 실행됐다.

누적회전 전의 bounded 후보 선정에서는 cadence
`0.8/1.0/1.2/1.3/1.4/1.5 Hz`도 실행했다. 최대 `vx=0.5`,
`point_support_ratio=0.55`에서 4초 body-X는 각각 약 `0.160/0.418/0.456/
0.365/0.246/0.389 m`였다. `1.2 Hz`가 속도, backtracking, yaw의 균형이 가장
좋아 채택됐다. `1.3 Hz` 이상은 phase/contact 혼합으로 역방향 누적이 다시
커졌다.

## 11. 수정 방법과 안전 경계

### PPO/hybrid 전환 속도

[`SconeHybridControlConfig`](../src/rl/joystick_control.py)의
`hybrid_start_speed`, `hybrid_full_speed`를 수정한다. 항상 두 값 사이에서
smoothstep이 단조 증가하는지, 제자리 yaw가 `b=0`인지 테스트한다.

### 점접지/회전 비율

`point_support_ratio`를 키우면 한 점 지지 시간이 길어지고 접지 회전 추진
시간이 짧아진다. `swing_roll_hold_ratio`를 키우면 공중에서 더 오래 회전하지만
착지 전 감속 시간이 줄어든다. `effective_roll_radius`,
`max_roll_rate_degrees`, `cycle_frequency`, `duty_factor`를 한 번에 바꾸지 말고
하나씩 sweep한다.

### checkpoint reference

`--rl-reference-motion`은 저속 PPO가 학습할 때 쓴 값과 같아야 한다. 이 값은
`scone-gait` supervisor 이름과 별개다. checkpoint reference가
`hardcoded`인데 `tripod-gait`나 bounded `scone-gait`로 바꾸면 residual 의미가
달라져 모션이 다시 상쇄될 수 있다.

### 실물 적용

새 `scone-gait`, `roll-gait`, `scone-stair`는 현재 MuJoCo 검증 경로다. 특히
multi-turn lower 예외는 MuJoCo의 무제한 hinge에서만 허용한다. 실물
기본 `Walk/Drive/Climb`을 자동으로 교체하지 않는다. 실물 승격 전에는 current,
temperature, hard stop, TPU 마찰/변형, e-stop, tether, 한 다리 무부하 시험을
별도로 통과해야 한다.
