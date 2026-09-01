# `tripod-gait`와 `scone-gait` 상세 설계·사용·검증 가이드

이 문서는 2026-09-01 기준 현재 코드에 구현된 두 model-based 보행
제어기를 한곳에서 설명한다.

- `tripod-gait`: 고전 교대 삼각보(alternating tripod)와 3차원 발 위치
  inverse kinematics를 결합한 기준 보행
- 비-RL MuJoCo `scone-gait`: 몸통·1단·2단 기본 보행과 SCONE의 부채꼴 TPU
  말단 **연속 velocity 회전**을 결합한 실험 보행
- RL `scone-gait` reference: 기존 checkpoint/action 형식과 호환되는 bounded
  18-position rolling/creep sweep

구현의 최종 기준은 다음 소스다.

- [`src/locomotion/tripod_gait.py`](../src/locomotion/tripod_gait.py)
- [`src/locomotion/scone_gait.py`](../src/locomotion/scone_gait.py)
- [`src/simulation/core/scone_rolling_gait.py`](../src/simulation/core/scone_rolling_gait.py)
- [`src/cli.py`](../src/cli.py)
- [`src/simulation/core/cli_bridge.py`](../src/simulation/core/cli_bridge.py)
- [`src/simulation/core/simulator_cli.py`](../src/simulation/core/simulator_cli.py)
- [`src/rl/walk_learn.py`](../src/rl/walk_learn.py)
- [`src/rl/inquiry.py`](../src/rl/inquiry.py)

개발 기록, 저장된 원격 job 설정 또는 과거 실행 명령에 `Non-RL`,
`NonRLWalk`, `non_rl`이 등장하면 현재의 `tripod-gait` 계열을 뜻한다.
새 문서와 UI에서는
`tripod-gait`, `scone-gait`를 정식 이름으로 사용한다.

## 1. 이름과 호환성 규칙

### 1.1 정식 이름

| 용도 | 정식 이름 | Python 클래스 | 구현 파일 |
|---|---|---|---|
| 고전 교대 삼각보 + IK | `tripod-gait` | `TripodGait` | `tripod_gait.py` |
| SCONE 부채꼴 연속 회전 조종 | `scone-gait` | `SconeRollingGait` | `simulation/core/scone_rolling_gait.py` |
| RL bounded 부채꼴 기준 모션 | `scone-gait` reference | `SconeGait` | `locomotion/scone_gait.py` |

CLI 선택값은 하이픈을 사용하고 Python 클래스는 CamelCase를 사용한다.
따라서 `tripod_gait`, `scone_gait`는 파일 이름에는 쓰이지만 CLI의 정식
선택값은 아니다.

### 1.2 남겨 둔 호환 별칭

기존 실행 스크립트, 저장된 원격 작업 기록, checkpoint 메타데이터가 즉시
깨지지 않도록 다음 별칭을 남겼다.

| 과거 이름 | 현재 해석 |
|---|---|
| CLI/reference 문자열 `non_rl` | `tripod-gait`로 정규화 |
| Python `NonRLWalk` | `TripodGait` 클래스 별칭 |
| Python `PhoenixTripodGait` | `TripodGait` 클래스 별칭 |
| `src/locomotion/non_rl_walk.py` | `tripod_gait.py`를 다시 export하는 shim |
| `SimulationControl.NON_RL` | `SimulationControl.TRIPOD_GAIT` enum 별칭 |
| `NON_RL_SIMULATION_GAIT_CONFIG` | `TRIPOD_GAIT_SIMULATION_CONFIG` 별칭 |

새 코드에서는 호환 이름을 사용하지 않는다. 별칭은 입력과 import 호환을
위한 것이며 UI에 별도 알고리즘으로 표시되지 않는다.

## 2. 왜 두 알고리즘으로 분리했는가

기존 model-based 보행은 각 발의 지면 접촉을 하나의 고정 support point로
가정한다. 이 가정은 일반적인 점발 또는 작은 구형 발에는 이해하기 쉽고
IK 기반 삼각보를 안정적으로 구성하기 좋다.

SCONE의 말단은 점발이 아니라 회전 가능한 부채꼴 프레임이다. 하단 관절이
회전하면 실제 최저 접촉점이 프레임을 따라 이동한다. 이 기구적 특징을 모두
고정점 IK 안에 숨기면 다음 기회를 사용하지 못한다.

- stance 동안 접촉점을 프레임 곡면을 따라 이동시켜 추진력을 만들 수 있다.
- 한 꼭짓점만 밟는 대신 접촉 패치가 이동하면서 하중을 분산할 수 있다.
- 완전한 순수 구름이 불가능한 방향에서는 제한된 횡방향 creepage를 허용할
  수 있다.

반대로 rolling만 사용해 기존 삼각보를 완전히 제거하면 아직 측정되지 않은
TPU 마찰, 변형, 실물 관절 지연에 강하게 의존한다. 따라서 현재 구조는
다음과 같이 분리했다.

```text
tripod-gait
  = 명령 필터 + 교대 삼각보 + 발 위치 IK + IK 안전장치

scone-gait (MuJoCo 조종)
  = upper/middle tripod IK 기본 보행
  + lower 기본 보행 offset의 시간 미분
  + 말단 mesh 기반 rolling 방향/조향 보정
  + lower 6개 continuous velocity
  + tripod-B 60° opening phase offset

scone-gait (RL reference)
  = tripod-gait position 결과
  + bounded stance/swing sector sweep
```

`SconeGait` reference는 `TripodGait`를 상속한다. `SconeRollingGait`는 이를
기본 보행/steering planner로 사용한다. lower는 velocity mode라 position
target을 직접 보낼 수 없으므로 nominal 대비 offset의 미분값을 연속 회전에
합성한다. 두 용도를 분리한 진단과 모든
phase 후보는 [`12-automatic-stair-demo-and-continuous-roll-rework.md`](12-automatic-stair-demo-and-continuous-roll-rework.md)를 따른다.

## 3. 공통 입력, 좌표계, 관절 순서

### 3.1 속도 명령

두 gait는 같은 `VelocityCommand`를 받는다.

```python
VelocityCommand(vx, vy, yaw_rate)
```

| 필드 | 단위 | 의미 |
|---|---:|---|
| `vx` | m/s | body frame 전진/후진 속도 |
| `vy` | m/s | body frame 좌우 속도 |
| `yaw_rate` | rad/s | body Z축 회전 속도, 양수는 왼쪽 회전 |

배열 입력은 shape `(3,)`이어야 한다. NaN/무한대 입력은 상위 RL 명령
경계에서 거부하며 gait 내부에서는 설정된 최대 속도로 clamp한다.

### 3.2 터미널 키 매핑

| 키 | joystick 축 | body 명령 |
|---|---|---|
| `W` / `S` | `+y` / `-y` | `+vx` / `-vx` |
| `A` / `D` | `-x` / `+x` | `+vy` / `-vy` |
| 왼쪽 / 오른쪽 화살표 | `+yaw` / `-yaw` | `+yaw_rate` / `-yaw_rate` |
| `Space` | 모든 축 0 | 즉시 neutral |
| `Q` | 종료 | 종료 전에 neutral frame 전송 |

UI의 오른쪽 `+x`와 model body frame의 좌우 부호가 반대이므로
`vy = -joystick_x * max_vy`로 변환한다.

### 3.3 18개 actuator 순서

모든 배열은 actuator ID 순서를 따른다.

```text
index 0..5    = motor ID 1..6   = upper/body joint
index 6..11   = motor ID 7..12  = middle/stage1 joint
index 12..17  = motor ID 13..18 = lower/stage2 sector joint
```

한 다리 `leg`의 배열 index는 다음과 같다.

```text
upper = leg - 1
middle = leg + 5
lower = leg + 11
```

motor degree와 MuJoCo joint radian의 공통 변환은 다음 의미를 갖는다.

```text
joint_radian = radians(motor_degree - 180)
motor_degree = degrees(joint_radian) + 180
```

## 4. 공통 support point 계산

`TIRE_n` body origin은 CAD transform의 원점이지 실제 지면 접촉점이 아니다.
따라서 `TripodGait` 초기화 시 각 `TIRE_n_geom` mesh에서 nominal 자세의
support point를 추론한다.

1. 선택한 Standard/Sport 자세를 18개 joint에 적용한다.
2. 각 `TIRE_n_geom`의 mesh vertex를 geom local에서 world frame으로 옮긴다.
3. world vertex를 body frame으로 변환한다.
4. 가장 낮은 Z값을 찾는다.
5. 최저점부터 `SUPPORT_PATCH_DEPTH = 1e-4 m`, 즉 0.1 mm 안의 vertex를
   support patch로 선택한다.
6. patch 평균을 계산한다.
7. 평균점을 해당 tire body의 local 좌표로 저장해 IK end effector로 쓴다.

한 개의 절대 최저 vertex만 선택하지 않는 이유는 TPU 프레임 폭의 한쪽
모서리로 IK가 치우치는 것을 피하기 위해서다. 필요하면 실측한 tire-local
좌표를 `end_effector_points`로 주입해 자동 추론을 대체할 수 있다.

## 5. `tripod-gait` 알고리즘

### 5.1 처리 순서

한 control frame은 다음 순서로 생성된다.

```text
VelocityCommand
  → 축별 clamp
  → 1차 지수 low-pass filter
  → command activity 계산
  → gait phase 갱신
  → 다리별 body twist 속도 계산
  → stride ellipse 제한
  → stance/swing 발 목표 생성
  → 6개 다리 DLS IK
  → 실패 시 stride backoff
  → 18개 motor degree 생성
  → 유한값/0..360°/IK 수렴 검사
  → controller.set_positions() 한 번으로 batch 전송
```

### 5.2 명령 clamp와 필터

각 명령 성분은 `GaitConfig`의 한계로 자른다.

```text
vx ∈ [-max_vx, +max_vx]
vy ∈ [-max_vy, +max_vy]
yaw_rate ∈ [-max_yaw_rate, +max_yaw_rate]
```

필터 time constant를 `tau`, frame 간격을 `dt`라고 하면 다음 계수를 쓴다.

```text
alpha = 1                                  (tau = 0)
alpha = 1 - exp(-dt / tau)                 (tau > 0)
filtered = filtered + alpha(target-filtered)
```

RL reference에서는 이미 환경의 명령 갱신 흐름이 있으므로
`command_time_constant=0.0`을 사용한다. 일반 gait 기본값은 `0.15 s`다.

### 5.3 activity

정규화된 세 축 중 가장 큰 값을 gait activity로 사용한다.

```text
activity = clip(max(
    |vx| / max_vx,
    |vy| / max_vy,
    |yaw_rate| / max_yaw_rate
), 0, 1)
```

activity가 `idle_epsilon` 이하면 phase를 진행하지 않고 모든 발을 nominal
support point에 둔다. 이때 stance 다리는 `(1,2,3,4,5,6)`으로 보고한다.

### 5.4 교대 tripod

지원 그룹은 기존 SCONE wiring/legacy gait와 동일하다.

```text
TRIPOD_A = (1, 4, 5), phase offset 0.0
TRIPOD_B = (2, 3, 6), phase offset 0.5
```

global phase `p`와 다리 offset `o_i`에서 다리 phase는 다음과 같다.

```text
p_i = (p + o_i) mod 1
```

기본 `duty_factor=0.5`이므로 `p_i < 0.5`는 stance, 나머지는 swing이다.
활성 명령이 있을 때 phase는 다음처럼 진행한다.

```text
p = (p + dt * cycle_frequency) mod 1
```

### 5.5 body twist에서 다리별 속도로 변환

다리 `i`의 nominal support 위치를 `(x_i, y_i)`라고 하면 body yaw가 만드는
접선 속도를 병진 속도에 합친다.

```text
v_i,x = vx - yaw_rate * y_i
v_i,y = vy + yaw_rate * x_i
```

이는 2차원 `omega × r`이다. 모든 다리에 같은 sideways offset을 주는 대신
각 발의 몸체 중심으로부터의 위치에 따라 서로 다른 접선 방향을 만든다.
따라서 제자리 회전과 병진+회전 명령을 같은 식으로 처리할 수 있다.

stance 시간은 다음과 같다.

```text
stance_time = duty_factor / cycle_frequency
stroke_i = v_i * stance_time
```

### 5.6 타원형 보폭 제한

전후 한계를 `Sx=max_stride`, 측면 한계를 `Sy=max_lateral_stride`라고 하면
요청 stroke가 다음 타원 안에 있는지 검사한다.

```text
rho = sqrt((stroke_x/Sx)^2 + (stroke_y/Sy)^2)
```

`rho > 1`이면 `stroke /= rho`로 방향은 유지하면서 타원 경계까지 줄인다.
제한된 다리 수를 6으로 나눈 값이 `stride_clip_fraction`이다.

이 값이 계속 1에 가까우면 사용자가 요청한 속도와 실제 reference 보폭이
일치하지 않는다는 의미다. 이런 상태에서 PPO가 모든 속도 오차를 residual로
해결하도록 맡기면 action saturation이 발생하기 쉽다.

### 5.7 stance 궤적

stance 내부 진행률을 `u ∈ [0,1)`라고 하면 nominal 발 위치에서 다음 offset을
사용한다.

```text
offset_stance = (0.5 - u) * stroke
```

stance 시작에서는 stroke 앞쪽 `+0.5`, 끝에서는 뒤쪽 `-0.5`로 이동한다.
지면에 고정된 발이 body 움직임 반대 방향으로 이동하도록 관절을 생성해
차체를 전진시킨다.

### 5.8 swing 궤적

swing 진행률을 `u ∈ [0,1)`라고 한다. 수평 복귀에는 minimum-jerk quintic을
사용한다.

```text
H(u) = u^3 * (10 - 15u + 6u^2)
offset_xy = (H(u) - 0.5) * stroke
```

수직 lift는 다음 함수다.

```text
L(u) = 16u^2(1-u)^2
offset_z = step_height * activity * L(u)
```

두 함수는 touchdown 부근에서 속도가 급변하지 않도록 endpoint 기울기를
줄인다. lift는 중앙에서 1이 되어 설정된 `step_height`까지 올라간다.

### 5.9 IK와 backoff

여섯 다리의 목표 위치는 `RobotKinematics.inverse()`로 전달된다.

- frame: body
- initial angle: 직전 유효 frame의 18개 각도
- solver: damped least squares
- 한 iteration 최대 step: `ik_max_step`
- 성공 기준: `ik_tolerance`
- 최대 반복: `ik_max_iterations`

일부 다리가 수렴하지 않으면 simulation/RL 설정에서는 목표를 nominal 발
위치 쪽으로 축소한다.

```text
scale_0 = 1
scale_(k+1) = scale_k * ik_stride_backoff_factor
target_backoff = nominal + (requested-nominal) * scale
```

현재 simulation/RL은 최대 4회, 회당 `0.8`배로 재시도한다. 최종 배율은
`ik_backoff_scale`에 기록된다. 다리별 IK가 실패하면 해당 다리는 직전 유효
각도를 유지하지만, 기본 `send(require_converged=True)`는 한 다리라도 실패한
frame 전체를 전송하지 않는다.

### 5.10 출력 안전 검사

전송 전에 다음 조건을 검사한다.

- controller가 존재하는가
- 모든 다리 IK가 수렴했는가
- 18개 motor target이 모두 finite인가
- 모든 target이 `0..360°` 안에 있는가

통과하면 `ControllerProtocol.set_positions()`에 ID 1..18 전체를 한 batch로
전달한다. gait 객체는 torque를 켜거나 serial port를 열지 않는다.

## 6. RL bounded-position `SconeGait` 알고리즘

### 6.1 기본 구조

`SconeGait.step()`은 먼저 `TripodGait.step()`을 호출한다. 따라서 다음 정보는
그대로 유지된다.

- filtered command
- gait phase와 stance/swing 그룹
- 발 위치 목표
- IK 수렴 결과
- stride clipping과 backoff 진단값
- 기본 18개 motor target

그다음 각 다리에 대해 부채꼴 rolling 방향을 계산하고 upper/lower 목표를
혼합한다. idle이면 base 결과를 수정하지 않고 즉시 반환한다.

### 6.2 active support point

`SconeGait`는 하단 관절 회전에 따라 현재 최저 접촉 패치가 어디로 이동하는지
계산한다.

1. 시험할 18개 motor degree를 FK에 적용한다.
2. `TIRE_n_geom` mesh vertex를 body frame으로 변환한다.
3. 최저 Z부터 0.1 mm 안의 patch를 선택한다.
4. patch 중심의 body-frame 3차원 위치를 반환한다.

`TripodGait`의 support point는 nominal 자세에서 고정된 IK end effector이고,
여기서 계산하는 active support point는 sector 회전에 따른 rolling 방향을
측정하기 위한 동적 기하량이라는 차이가 있다.

### 6.3 sector tangent 수치 미분

다리 `i`의 lower joint를 nominal에서 `-1°`, `+1°` 움직인 두 자세를 만든다.
두 자세의 active support point 차이에서 XY 성분을 꺼내 정규화한다.

```text
t_i = normalize(
    support(lower+1°) - support(lower-1°)
)[:2]
```

이 벡터는 lower motor degree가 증가할 때 접촉 패치가 body frame에서
움직이는 방향이다. 벡터 norm이 거의 0이면 rolling tangent를 정의할 수
없으므로 초기화를 실패시킨다.

### 6.4 upper 조향 gain 보정

nominal upper 자세와 `steering_probe_degrees=5°` offset 자세에서 각각 sector
tangent 각도를 측정한다.

```text
theta_0 = atan2(t_nominal_y, t_nominal_x)
theta_probe = atan2(t_probe_y, t_probe_x)
gain = wrap(theta_probe - theta_0) / radians(probe)
```

gain 절댓값이 `0.25`보다 작으면 upper joint가 rolling tangent를 충분히
바꾸지 못한다고 보고 초기화를 실패시킨다. 측정된 값은 다리별
`_nominal_roll_angles`, `_steering_gains`에 저장한다.

자세를 `reset()` 또는 `reset_from_controller()`로 바꾸면 nominal 기하가
달라지므로 tangent와 gain을 다시 보정한다.

### 6.5 명령 방향과 rolling 극성 선택

다리 위치 `(x_i,y_i)`에서 원하는 body point velocity는 `tripod-gait`와 같은
식을 사용한다.

```text
body_velocity_i = [
    vx - yaw_rate*y_i,
    vy + yaw_rate*x_i
]
desired_contact_i = -body_velocity_i
```

부채꼴은 같은 tangent를 정방향 또는 역방향으로 사용할 수 있으므로
`polarity=+1`, `polarity=-1` 두 후보를 비교한다. 역방향 후보는 nominal
tangent angle에 π를 더한다.

각 후보에서 필요한 upper steering offset을 계산하고
`±max_steering_degrees`로 제한한다. 제한 후 실제 가능한 방향과 원하는 접촉
방향의 cosine을 alignment로 사용한다.

```text
alignment = max(0, cos(desired_angle - actual_angle))
```

alignment가 큰 후보를 선택하며 동률이면 작은 steering offset을 선호한다.
반환값은 다음 세 값이다.

```text
(upper steering degree, lower polarity, alignment 0..1)
```

### 6.6 stance/swing sector sweep

각 다리 phase에서 rolling coordinate `r_i`를 만든다.

stance 진행률 `u`:

```text
r_i = u - 0.5
```

swing 진행률 `u`:

```text
r_i = 0.5 - H(u)
```

stance에서는 `-0.5 → +0.5`로 선형 sweep하고, swing에서는 quintic으로
`+0.5 → -0.5` 복귀한다. 이 때문에 touchdown/다음 stance 시작에서 lower
목표가 불연속으로 뛰지 않는다.

### 6.7 lower sector 목표

alignment가 지나치게 낮아도 완전히 멈추지 않도록 다음 값을 쓴다.

```text
usable_alignment = max(alignment, minimum_roll_alignment)
```

최종 rolling 목표는 다음 구조다.

```text
roll_target = nominal_lower
    - polarity
    * roll_coordinate
    * sector_sweep_degrees
    * activity
    * usable_alignment
```

여기서 `-polarity`가 중요하다. mesh에서 측정한 것은 body frame에서 active
patch 위치가 이동하는 방향이고, 차체를 미는 ground-reaction 진행 방향은
반대 부호로 나타났다. 초기 구현은 이 부호를 그대로 사용해 전진 명령에서
뒤로 움직였다. 자유 몸체 MuJoCo 실험 후 부호를 반전했다.

lower joint는 base IK 결과와 rolling 목표를 혼합한다.

```text
lower_output =
    (1-rolling_blend) * base_ik_lower
    + rolling_blend * roll_target
```

현재 기본 `rolling_blend=0.75`다. 즉 rolling을 주로 반영하면서 base IK가
만든 자세 안정 성분을 25% 남긴다.

### 6.8 선택적 upper steering

upper steering 목표는 다음과 같다.

```text
steering_weight = steering_blend * activity
steering_target = nominal_upper + steering_offset
upper_output =
    (1-steering_weight) * base_ik_upper
    + steering_weight * steering_target
```

현재 `steering_blend=0.0`이므로 계산과 보정 코드는 존재하지만 실제 출력에는
섞이지 않는다. MuJoCo sweep에서 큰 steering blend가 좌우 yaw 비대칭과 병진
drift를 키웠기 때문이다. 물리 TPU 마찰과 접촉 패치 데이터를 얻기 전까지
기본값을 0으로 유지한다.

### 6.9 creepage의 의미

모든 다리 rolling tangent를 원하는 body twist 방향과 완전히 일치시키려면
기구적으로 매우 큰 upper 조향이 필요할 수 있다. 현재 알고리즘은 다음처럼
타협한다.

- 가능한 polarity 중 alignment가 높은 방향을 선택한다.
- upper steering은 제한 범위로 계산하지만 기본 출력 혼합은 끈다.
- alignment로 lower sweep 크기를 줄인다.
- `minimum_roll_alignment`만큼은 sweep을 유지한다.
- 남는 횡방향 성분은 접촉 모델의 제한된 미끄러짐, 즉 creepage로 처리한다.

따라서 `scone-gait`의 rolling/creep은 무미끄럼 순수 구름을 보장한다는 뜻이
아니다. SCONE 기구가 허용하는 방향으로 접촉점을 이동시키면서, 불가능한
성분은 작게 미끄러지는 hybrid 전략이다.

### 6.10 비-RL MuJoCo의 full-body + continuous lower 회전

위 6.1–6.9는 RL residual에 더할 수 있는 18-position reference 설명이다.
실제 `--control scone-gait`는 2026-09-01부터
`SconeRollingGait`를 사용한다.

- ID 1–6: body/upper 기본 보행과 rolling 조향 position
- ID 7–12: stage-1 기본 보행 position
- ID 13–18: `VELOCITY` mode, 기본 보행 속도 성분 + mesh tangent 연속 회전
- tripod A `(1,4,5)`: lower 시작 255°
- tripod B `(2,3,6)`: lower 시작 315°, 즉 +60° phase
- lower 속도: 175, stance는 0.8배, swing은 1.0배
- roll velocity filter: 0.10초
- 기본 lower 속도 미분 filter: 0.04초, 최대 80 unit, 최종 blend 0.35

하단은 position과 velocity mode를 동시에 사용할 수 없다. 따라서 bounded
기본 보행의 하단 목표를 nominal 기준 offset `Δq_basic(t)`로 분리하고 다음과
같이 속도 형태로 합성한다.

```text
q_lower(t) = q_continuous_roll(t) + 0.35 Δq_basic(t)

qdot_lower(t)
  = qdot_roll(t)
  + 0.35 lowpass(clip(d(Δq_basic)/dt, -80, +80))
```

XM430 속도 1 unit은 `0.229 rpm = 1.374 deg/s`로 변환한다. `Δq_basic`에는
IK가 만든 stage-2 굽힘과 bounded sector sweep가 모두 들어간다. 결과적으로
말단은 여러 바퀴 계속 회전하지만 gait 주기에 맞춰 속도가 가감되어 2단의
기본 보행 성분도 사라지지 않는다.

최초 회전 전용 구현의 동기 회전은 root Z가 −63.5 mm까지 내려갔다. 이후
full-body 합성 기준으로 phase를 다시 sweep해 72°에서 60°로 바꿨고, 8초
최저 Z는 −51.2 mm에서 −20.7 mm, 최대 yaw는 19.7°에서 8.0°로 줄었다.
전체 식과 후보는 12번 문서에 있다.

## 7. 설정값

### 7.1 `GaitConfig` 기본값

| 필드 | 기본값 | 역할 |
|---|---:|---|
| `control_frequency` | 50 Hz | gait frame 생성 빈도 |
| `cycle_frequency` | 0.8 Hz | 한 gait cycle 빈도 |
| `duty_factor` | 0.5 | stance cycle 비율 |
| `step_height` | 0.035 m | swing 최대 높이 |
| `max_stride` | 0.070 m | 전후 stroke 한계 |
| `max_lateral_stride` | `None` | `None`이면 전후 한계와 동일 |
| `max_vx` | 0.18 m/s | 전후 명령 clamp |
| `max_vy` | 0.12 m/s | 측면 명령 clamp |
| `max_yaw_rate` | 0.9 rad/s | yaw 명령 clamp |
| `command_time_constant` | 0.15 s | 명령 low-pass 시간 |
| `idle_epsilon` | 0.001 | idle 판정 경계 |
| `ik_tolerance` | 0.0001 m | IK 위치 residual 한계 |
| `ik_max_iterations` | 80 | 다리별 최대 반복 |
| `ik_damping` | 0.002 | DLS damping |
| `ik_max_step` | 0.15 rad | iteration당 최대 각도 변화 |
| `ik_stride_backoff_attempts` | 0 | 기본 IK 재시도 횟수 |
| `ik_stride_backoff_factor` | 0.8 | 재시도별 목표 축소 배율 |

### 7.2 simulation용 `tripod-gait`

`TRIPOD_GAIT_SIMULATION_CONFIG`는 다음 항목을 override한다.

| 필드 | 값 |
|---|---:|
| `cycle_frequency` | 1.0 Hz |
| `step_height` | 0.025 m |
| `max_stride` | 0.090 m |
| `max_lateral_stride` | 0.070 m |
| `ik_tolerance` | 0.001 m |
| `ik_stride_backoff_attempts` | 4 |

이 route에는 초기화 뒤 profile velocity/acceleration 0(무제한)과 middle
position stiffness 2배를 적용한다. 무제한은 토크를 없애는 뜻이 아니라 profile
ramp만 제거하며 MuJoCo dcmotor PID·전압·토크 제한은 그대로다. 8초 측정에서
0.9469 m(0.1184 m/s), 역방향 누적 3.7 mm, 측면 0.7 mm, 최대 yaw 1.17°였고,
20초에는 2.4263 m/측면 5.0 mm/IK 실패 0이었다. 이 값은 비-RL 조종 전용이며
아래 9장의 RL reference 0.7 Hz/60 mm를 바꾸지 않는다.

### 7.3 `SconeGaitConfig` 기본값

상속된 값 중 다음을 바꾸거나 추가한다.

| 필드 | 값 | 역할 |
|---|---:|---|
| `cycle_frequency` | 0.65 Hz | sector sweep 포함 cycle |
| `step_height` | 0.025 m | 작은 Cartesian 안정화 lift |
| `max_stride` | 0.035 m | base IK 전후 stroke |
| `max_lateral_stride` | 0.025 m | base IK 측면 stroke |
| `sector_sweep_degrees` | 30° | lower rolling 전체 sweep scale |
| `max_steering_degrees` | 55° | 계산상 upper 조향 한계 |
| `rolling_blend` | 0.75 | base IK와 rolling lower 혼합 |
| `steering_blend` | 0.0 | upper 조향 출력 혼합, 현재 비활성 |
| `steering_probe_degrees` | 5° | tangent gain 보정 probe |
| `minimum_roll_alignment` | 0.15 | 최소 sector sweep 비율 |

검증 범위는 다음과 같다.

- `sector_sweep_degrees`: `(0, 60]`
- `max_steering_degrees`: `(0, 90]`
- `steering_probe_degrees`: `(0, 15]`
- `rolling_blend`, `steering_blend`, `minimum_roll_alignment`: `[0,1]`

simulation용 `SCONE_GAIT_SIMULATION_CONFIG`는 IK tolerance를 1 mm로 하고
backoff 4회를 켠다.

### 7.4 `SconeRollingGaitConfig` 기본값

| 필드 | 값 |
|---|---:|
| `roll_velocity` | 175 |
| `support_velocity_ratio` | 0.80 |
| `tripod_b_phase_offset_degrees` | 60° |
| `velocity_time_constant` | 0.10 s |
| `basic_velocity_time_constant` | 0.04 s |
| `basic_lower_motion_blend` | 0.35 |
| `max_basic_lower_velocity` | 80 |
| `profile_velocity/profile_acceleration` | 160/50 |
| `middle_stiffness_multiplier` | 2.0 |
| basic gait cadence/duty | 0.8 Hz/0.58 |
| basic gait stride/lift | 55 mm/20 mm |
| steering blend/limit | 0.20/45° |

## 8. CLI와 시뮬레이션 연결

### 8.1 직접 실행

macOS에서는 passive viewer가 main thread를 사용하도록 `mjpython`으로 실행한다.

```bash
mjpython -m src.simulation \
  --control tripod-gait \
  --profile standard \
  --terrain flat
```

```bash
mjpython -m src.simulation \
  --control scone-gait \
  --profile standard \
  --terrain flat
```

`scone-gait`는 현재 Standard 자세를 권장한다. Sport는 차체가 낮아 swing
접지 여유와 sector rolling 패치가 부족해질 수 있다.

### 8.2 대화형 선택

루트 launcher에서 시뮬레이션 조종을 선택하면 다음 다섯 제어 경로가 나타난다.

```text
Legacy mode control
tripod-gait
scone-gait
scone-stair
RL control
```

`tripod-gait`와 `scone-gait`는 같은 terminal x/y/yaw joystick을 사용한다.
MuJoCo viewer는 SCONE용 키 callback을 가지지 않으므로 robot 명령과 viewer
camera key가 섞이지 않는다.

### 8.3 simulation thread 경계

```text
main thread
  MuJoCo controller.update(dt)
  → mujoco.mj_step()
  → viewer.sync()

worker thread
  terminal key
  → KeyboardJoystick
  → VelocityCommand
  → TripodGait/SconeRollingGait.update()
```

controller의 공유 상태는 lock으로 보호한다. physics loop는 MuJoCo timestep에
맞춰 pace하며 lock 안에서 긴 terminal I/O를 수행하지 않는다.

### 8.4 simulation에서 controller 자세를 재보정하지 않는 이유

일반 `_run_gait_joystick_cli()`는 실제 controller의 현재 raw position을 읽어
stroke 중심으로 사용할 수 있다. 그러나 simulation은 초기 Standard 자세가
중력으로 잠시 처지는 시점의 값을 nominal로 다시 잡으면 workspace 경계에
있는 다리 2/5의 IK가 바로 실패할 수 있다.

따라서 simulation bridge의 `tripod-gait`는
`calibrate_from_controller=False`로 실행하고, 선택한 검증 완료 profile 자세를
nominal로 유지한다. `SconeRollingGait`도 내부 planner의 Standard nominal을
유지하되 lower phase start pose에 도달한 뒤 velocity mode로 전환한다. 실물에서
bounded gait를 직접 시험할 때는 현재 측정 자세를 사용하는
`reset_from_controller()`가 필요하다.

## 9. Residual RL 기준 모션 연결

### 9.1 선택지

RL reference motion은 다음 세 정식 값을 제공한다.

| 값 | 기준 모션 | 용도 |
|---|---|---|
| `tripod-gait` | 연속 교대 삼각보 + IK | 새 check/train 기본값 |
| `scone-gait` | 삼각보 + sector rolling/creep | 새 실험 학습 |
| `hardcoded` | 기존 분석적 sine tripod | 과거 PPO replay 기본값 |

`non_rl` 입력은 로딩 시 `tripod-gait`로 정규화한다. `TrainingConfig`와
`RemoteJob`도 저장 전에 정식 이름으로 바꾼다.

### 9.2 environment 안의 reference 생성

`SconeWalkEnv`는 reference에 따라 `_reference_gait`를 만든다.

`tripod-gait` RL 설정:

- control frequency: `1/control_dt`, 현재 50 Hz
- cycle: 0.7 Hz
- 전후/측면 stroke: 0.060/0.050 m
- command 한계: `[0.50, 0.25, 0.80]`
- command filter: 0
- IK tolerance: 1 mm
- backoff: 최대 4회

`scone-gait` RL 설정:

- `SconeGaitConfig`의 0.65 Hz, 35/25 mm base stroke, 30° sector sweep
- command 한계: `[0.50, 0.25, 0.80]`
- command filter: 0
- IK tolerance: 1 mm
- backoff: 최대 4회

선택한 standing pose를 `_reference_gait.reset(motor_degrees=...)`에 전달해
reference의 nominal 자세와 RL 환경의 기본 자세를 맞춘다.

### 9.3 reference와 residual 합성

model-based reference의 `step()` 결과가 18개 기준 motor degree가 된다.
정책 action은 `[-1,1]` 범위이며 관절 단계별 scale을 곱한다.

```text
upper ID 1..6:    residual scale 10°
middle ID 7..12:  residual scale 12°
lower ID 13..18:  residual scale 15°

target = reference + residual_scale * clipped_action
```

reference gait가 자체 phase를 소유하므로 환경은 sample phase를 다시 복사해
관측의 `sin(phase)`, `cos(phase)`와 기준 모션을 맞춘다. 또한 다음 진단값을
TensorBoard state로 전달한다.

- `reference_cycle_frequency`
- `reference_stride_clip_fraction`
- `reference_ik_backoff_scale`

### 9.4 checkpoint 호환성

reference가 바뀌면 같은 policy action의 물리적 의미가 달라진다.

```text
hardcoded reference + action
!= tripod-gait reference + 같은 action
!= scone-gait reference + 같은 action
```

따라서 다음 규칙을 지킨다.

- 과거 hardcoded checkpoint는 기본적으로 `hardcoded`로 재생한다.
- `non_rl`로 기록된 checkpoint는 `tripod-gait`로 읽되, 실제 학습 당시 설정이
  동일한지 확인한다.
- `tripod-gait` checkpoint를 `scone-gait`로 바꿔 resume하지 않는다.
- `scone-gait`는 기존 checkpoint를 재사용하지 말고 0 step부터 새 run으로
  학습한다.
- standing pose, actuator profile, support geometry, reward 또는 observation이
  바뀐 경우에도 기존 run과 분리한다.

현재 Stable-Baselines3 PPO ZIP 파일 이름만으로는 학습 당시 reference를
신뢰성 있게 판별할 수 없다. `scone_walk_700000_steps.zip` 같은 prefix의
`scone_walk`는 프로젝트/학습 작업 이름이지 `scone-gait`로 학습했다는 뜻이
아니다. 다음 근거를 함께 확인해야 한다.

- `runs/.remote_jobs.json`의 `reference_motion`
- 학습 시작 명령의 `--reference-motion`
- `train.log` 또는 당시 실행 기록
- 별도로 기록한 standing pose와 환경 버전

근거가 없는 구형 checkpoint는 자동으로 `scone-gait` 또는 `tripod-gait`라고
추정하지 않는다. 현재 replay CLI가 명시 입력이 없을 때 `hardcoded`를
선택하는 이유도 과거 PPO의 action 의미를 보수적으로 보존하기 위해서다.

### 9.5 직접 명령

환경 smoke test:

```bash
PYTHONPATH=. python -m src.rl.walk_learn \
  --reference-motion tripod-gait \
  check --steps 500 --curriculum easy
```

```bash
PYTHONPATH=. python -m src.rl.walk_learn \
  --reference-motion scone-gait \
  check --steps 500 --curriculum easy
```

새 PPO 학습:

```bash
PYTHONPATH=. python -m src.rl.walk_learn \
  --reference-motion scone-gait \
  train \
  --curriculum easy \
  --timesteps 1000000 \
  --num-envs 4 \
  --output runs/scone_gait_easy_v1
```

## 10. Python API

### 10.1 offline frame 생성

controller 없이 frame을 생성해 궤적과 IK를 검사할 수 있다.

```python
from src.locomotion import TripodGait, VelocityCommand

gait = TripodGait(profile="standard")
sample = gait.step(
    VelocityCommand(vx=0.05, vy=0.0, yaw_rate=0.2),
    dt=0.02,
)

print(sample.converged)
print(sample.stance_legs)
print(sample.motor_degrees)
print(sample.stride_clip_fraction)
print(sample.ik_backoff_scale)
```

### 10.2 `scone-gait` frame 생성

```python
from src.locomotion import SconeGait, VelocityCommand

gait = SconeGait(profile="standard")
sample = gait.step(
    VelocityCommand(vx=0.05, vy=0.02, yaw_rate=0.15),
    dt=0.02,
)
```

다리별 rolling 후보를 따로 진단할 수도 있다.

```python
for leg in range(1, 7):
    steering, polarity, alignment = gait.steering_solution(
        leg,
        VelocityCommand(vx=0.05),
    )
    print(leg, steering, polarity, alignment)
```

### 10.3 controller 전송

```python
gait = TripodGait(controller, profile="standard")
gait.reset_from_controller()

sample = gait.update(
    VelocityCommand(vx=0.04),
    dt=0.02,
    send=True,
)
```

이 호출은 18개 position target을 보내지만 controller 초기화, torque enable,
비상정지, 충돌 감시는 별도 상위 계층의 책임이다. 현재 하드웨어 launcher는
검증된 legacy discrete gait를 기본으로 유지하며 두 연속 gait를 자동으로
활성화하지 않는다.

## 11. 검증 결과

### 11.1 자동 테스트

2026-09-01 전체 회귀 결과:

```text
python -m unittest discover -s tests -v
Ran 107 tests
OK
```

관련 테스트의 역할은 다음과 같다.

| 테스트 | 검증 내용 |
|---|---|
| `test_tripod_gait.py` | idle, tripod phase, support patch, yaw 접선, IK, backoff, 18개 batch |
| `test_scone_gait.py` | tangent 보정, polarity/alignment 범위, lower sweep, idle, config 검증 |
| `test_simulation.py` | 정식 CLI 이름, `non_rl` alias, 자유 몸체 전진, upright 유지 |
| `test_rl_reference_motion.py` | RL에서 `TripodGait`/`SconeGait` 실제 생성, lower reference 차이 |
| `test_rl_inquiry.py` | UI 기본값과 과거 `non_rl` 설정 정규화 |
| `test_api.py` | 공통 terminal joystick, neutral 종료, gait CLI dispatch |

### 11.2 bounded position reference의 과거 3초 자유 몸체 비교

동일한 검사 구조에서 다음 조건을 사용했다.

- terrain: flat
- floating base: enabled
- profile: Standard
- controller: `MuJoCoController`의 실제 position profile/PID 경로
- duration: 150 frame × 0.02 s = 3.0 s
- command: 각 gait의 `max_vx`, 현재 둘 다 0.18 m/s
- 시작 자세의 body frame으로 최종 위치 차이를 회전해 비교

측정 결과:

| gait | body X | body Y | 최종 upright Z축 cosine |
|---|---:|---:|---:|
| `tripod-gait` | +0.1357 m | -0.0006 m | 0.999984 |
| `scone-gait` | +0.1248 m | -0.0186 m | 0.999954 |

이 수치는 현재 model/contact/PID에서 방향과 자세 안정성을 확인한 회귀값이지
실물 성능 보증이나 최고 속도 측정이 아니다. `scone-gait`는 3초 동안 약
18.6 mm 측면 drift가 남으므로 계속 실험 모드로 분류한다.

이 표의 `scone-gait`는 RL 호환용 bounded `SconeGait`다. 현재 비-RL 조종
route의 6초 측정은 다음과 같다.

| interactive route | body X | 평균 속도 | body Y | 최소 ΔZ | 최소 upright |
|---|---:|---:|---:|---:|---:|
| SCONE-tuned `tripod-gait` (8 s) | +0.9469 m | 0.1184 m/s | −0.0007 m | −0.00002 m | 0.99993 |
| full-body continuous-roll `scone-gait` (6 s) | +1.2556 m | 0.2093 m/s | −0.0525 m | −0.0207 m | 0.9811 |

`tripod-gait`는 같은 8초 동안 역방향 누적 3.7 mm, 최대 yaw 1.17°였고
20초에서도 측면 편향 5.0 mm였다. continuous route는 lower 평균 3.09회전을
수행했고 기본 보행 명령 진폭은 upper 17.3°, middle 9.6°, planned lower
14.7°, lower basic 속도 성분 24.3 unit이었다. 두 route 모두 IK 실패가 없었다.

### 11.3 구현 중 발견한 반대 방향 문제

초기 sector sweep은 mesh active point의 이동 방향을 actuator 추진 방향으로
그대로 사용했다. 그 결과 전진 명령에서 약 `-0.025 m`로 뒤로 이동하는
실험이 나왔다. 다음 두 수정을 함께 적용했다.

- ground-reaction travel에 맞춰 lower sweep polarity를 반전
- 큰 upper steering blend가 만든 yaw 비대칭 때문에 기본
  `steering_blend`를 0으로 변경

수정 후 동일 계열의 자유 몸체 검사에서 양의 X 전진으로 돌아왔고 현재
회귀값은 위 표처럼 +0.1248 m다.

### 11.4 GUI 검증 상태

bounded reference 시기에는 `mjpython -m src.simulation --control scone-gait`의
GUI 기동과 terminal 입력까지만 확인했다. 2026-09-01에는 자동 계단 route를
`mjpython -m src.simulation --demo compare --terrain stairs-2`로 직접 실행해
hardcoded viewer 3.68초 상단 판정/종료 뒤 improved viewer 3.68초 상단
판정/종료까지 확인했다. 이는 continuous 평지 joystick의 장시간 화면 분석이
아니며 자동 viewer/physics/state route의 smoke 검증이다.

따라서 다음 항목은 아직 완료됐다고 주장하지 않는다.

- continuous-roll 평지 joystick의 사람이 본 장시간 직진 궤적
- 복합 `vx+vy+yaw`에서 말단 접촉 패치의 시각적 연속성
- uneven/stairs/slope/mixed 전 구간 GUI 완주
- 실물 TPU 변형과 소음, 발열, 전류, 마모

## 12. 현재 한계와 위험

### 12.1 `scone-gait`는 simulation-first다

실물에서 다음 값이 아직 계측되지 않았다.

- TPU-바닥 마찰계수와 속도 의존성
- 부채꼴 접촉 패치의 실제 폭과 변형
- lower joint backlash와 하중별 tracking lag
- rolling/creep 중 motor current와 온도
- 표면별 slip 방향과 마모

따라서 현재 기본 하드웨어 보행을 `scone-gait`로 교체하지 않았다.

### 12.2 modified target의 최종 Cartesian 재검증

`scone-gait`의 `GaitSample.ik_results`는 base `tripod-gait` IK 결과다. 그 뒤
upper/lower target을 rolling용으로 혼합하므로 modified 18개 target 전체를
다시 Cartesian IK residual로 평가하는 구조는 아직 없다. 출력은 0..360°로
clip되지만, 이는 실물 하중 안전 영역을 증명하지 않는다.

향후에는 modified target에 대해 다음 검사를 추가하는 것이 좋다.

- FK로 최종 접촉 패치 높이 재계산
- stance 다리의 지면 침투/이탈 범위 검사
- per-joint velocity/acceleration 제한
- 예상 motor current와 torque margin 검사

### 12.3 upper steering은 계산만 하고 기본 비활성이다

`steering_solution()`과 gain calibration은 향후 실험을 위해 남아 있지만
`steering_blend=0`이다. 이 값을 올리면 전진 alignment가 좋아질 수 있으나
현재 MuJoCo에서는 좌우 yaw 비대칭과 병진 drift가 커졌다.

### 12.4 contact 모델 의존성

mesh 최저점과 MuJoCo contact가 실물 TPU 접촉을 완전히 재현하지는 않는다.
mesh 해상도, geom friction, solver parameter가 바뀌면 tangent, slip, 추진
결과도 달라질 수 있다. model/contact 변경 후에는 기존 benchmark를 다시
수행해야 한다.

### 12.5 기준 모션 변경은 checkpoint 의미 변경이다

`tripod-gait`와 `scone-gait`는 lower target의 phase와 크기가 다르다. 정책이
학습한 residual action을 다른 reference에 얹으면 보정이 아니라 상쇄가 될 수
있다. 사용자에게 처음 발생했던 “기준 모션과 정책 방향이 반대라 움직임이
꼬이는” 증상이 다시 생길 수 있으므로 reference를 자동 추정하지 않는다.

## 13. 권장 튜닝 절차

한 번에 여러 값을 바꾸면 원인을 분리할 수 없으므로 다음 순서를 권장한다.

### 13.1 1단계: zero-residual 기준 모션

PPO 없이 각 gait만 실행한다.

- 정지 자세 drift
- 전진/후진 부호
- 좌/우 `vy` 부호
- 좌/우 yaw 부호
- 3초/6초 body-frame displacement
- upright cosine
- stride clipping
- IK backoff scale
- joint target 속도
- 접촉 slip

### 13.2 2단계: sector sweep

다음 순서로 하나씩 바꾼다.

1. `sector_sweep_degrees`
2. `rolling_blend`
3. `minimum_roll_alignment`
4. `cycle_frequency`
5. base `max_stride`, `max_lateral_stride`

처음에는 `steering_blend=0`을 유지한다.

### 13.3 3단계: upper steering 실험

zero-residual에서 양/음 yaw가 대칭인지 확인하면서 `steering_blend`를 매우
작게 올린다. 한 방향 yaw만 좋아지고 반대 방향이 나빠지면 gain 부호,
좌우 mesh axis, contact friction을 먼저 검사한다.

### 13.4 4단계: RL smoke test

새 run 이름으로 다음을 확인한다.

- reset 성공
- 500 step finite observation/reward
- random residual에서도 termination 원인이 합리적인가
- `reference_stride_clip_fraction`이 지속적으로 포화되지 않는가
- `reference_ik_backoff_scale`이 항상 1보다 작은 것은 아닌가
- neutral command에서 residual gate가 action bias를 0으로 감쇠하는가

### 13.5 5단계: 새 PPO 학습

기존 checkpoint를 resume하지 않고 `scone-gait` 전용 run을 만든다. 처음에는
easy 전진 범위와 달성 가능한 `0..0.06 m/s` 부근에서 reference와 reward를
검증한 후 curriculum을 넓힌다. mean reward 하나만 보지 말고 속도 오차,
action saturation, slip, current, fall, clipping을 함께 기록한다.

### 13.6 6단계: 실물 진입 조건

다음 조건을 충족한 뒤에만 낮은 속도로 실물 시험을 고려한다.

- joint별 안전 각도/속도/가속도 확정
- 비상 torque-off 경로 별도 검증
- 로봇을 들어 올린 무부하 방향 시험
- 지면 접촉 전류 제한
- TPU 접촉 패치와 slip 고속 촬영
- 짧은 stance sweep부터 단계적으로 증가
- Standard measured pose에서 `reset_from_controller()` 보정

## 14. 문제 진단표

| 증상 | 우선 확인할 값 | 가능한 원인 |
|---|---|---|
| 전진 명령인데 후진 | reference 이름, lower polarity | 잘못된 checkpoint reference, sector sweep 부호 |
| gait가 거의 안 움직임 | `stride_clip_fraction`, `activity` | 명령 clamp, reference 보폭 포화, neutral 판정 |
| 특정 다리 IK 실패 | `failed_legs`, `ik_backoff_scale` | nominal 자세, 과도한 stride, gravity-sagged 재보정 |
| 좌우 yaw 비대칭 | `steering_blend`, 다리별 polarity | upper steering gain/축/접촉 비대칭 |
| 측면 drift 증가 | body Y, alignment, slip | sector tangent 불일치, creepage, friction |
| 정지 중 떨림 | filtered command, phase, residual action | key timeout, policy bias, neutral gate 미적용 |
| PPO가 기준 모션을 상쇄 | checkpoint의 원래 reference | 다른 reference로 replay/resume |
| lower 목표가 급격함 | sweep, frequency, profile velocity | 과도한 `sector_sweep_degrees`, actuator 추종 한계 |
| GUI는 켜지지만 키가 안 먹음 | terminal focus | viewer가 아니라 실행 terminal이 키 입력 소유 |

## 15. 변경 시 같이 확인할 파일

| 변경 대상 | 함께 확인할 파일/테스트 |
|---|---|
| gait 이름/alias | `simulator_cli.py`, `cli_bridge.py`, `inquiry.py`, `test_simulation.py`, `test_rl_inquiry.py` |
| `GaitConfig` | `tripod_gait.py`, simulation config, RL env config, `test_tripod_gait.py` |
| `SconeGaitConfig` | `scone_gait.py`, `test_scone_gait.py`, 자유 몸체 simulation test |
| tripod/phase | `actuator_index.py`, legacy `walk.py`, RL phase observation |
| support point | `model.xml`, `TIRE.stl`, kinematics tests, slip reward |
| RL reference | `runs/.remote_jobs.json`, 학습 명령/log, `walk_learn.py`, inquiry serialization |
| command 부호 | `KeyboardJoystick`, `VelocityCommand`, yaw/lateral sign tests |
| contact/friction | model/terrain XML, slip reward, 모든 gait benchmark |

## 16. 설계 참고자료

현재 구현은 참고자료를 SCONE 기구에 맞게 재구성한 것이며 동일 로봇이나 동일
제어기를 그대로 이식한 것이 아니다.

- [CMU RHex: A Simple and Highly Mobile Hexapod Robot](https://publications.ri.cmu.edu/rhex-a-simple-and-highly-mobile-hexapod-robot)
  — clock-driven alternating tripod과 회전형 다리의 설계 배경
- [Keep Rollin' – Whole-Body Motion Control and Planning for Wheeled Quadrupedal Robots](https://arxiv.org/abs/1809.03557)
  — 이동하는 접촉점과 rolling constraint를 다루는 wheel-legged 제어 배경
- [Lynxmotion Phoenix implementation](https://github.com/KurtE/Phantom_Phoenix)
  — 다리별 phase, stance/swing, body translation, IK 구조 참고

SCONE의 `scone-gait`는 위 시스템의 성능을 재현한다고 주장하지 않는다.
현재 코드의 실제 동작과 한계는 이 문서의 source/test/benchmark 항목을 기준으로
판단한다.

계단을 side-on으로 오르는 별도 `scone-stair` state machine은 일반 평지 gait나
RL reference가 아니다. 후킹 조건, pure rolling/tripod/hybrid/adaptive 비교와
실패 기록은 [`11-scone-stair-climbing.md`](11-scone-stair-climbing.md)를 따른다.
