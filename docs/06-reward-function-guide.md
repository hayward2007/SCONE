# 보상함수 수정 가이드

> 범위: 이 문서의 본문은 기존 70차원 `walk_learn` 정책을 설명한다. 새 82차원
> `walk_v2`의 현재 보상식, 실제 35.4M-step 항별 기여와 문제점은
> [`21-walk-v2-ppo-training-analysis.md`](21-walk-v2-ppo-training-analysis.md)를
> 기준으로 한다. 두 환경의 숫자와 checkpoint를 섞어 쓰면 안 된다.

## 1. 기준 파일과 변경 지점

기존 정책 보상함수의 최종 기준은 [`src/rl/walk_learn.py`](../src/rl/walk_learn.py)의 다음 세 부분이다.

1. `RewardConfig`: sigma, weight, soft/hard limit
2. `SconeWalkEnv._reward()`: raw 측정값, 정규화, aggregate, 종료 조건
3. `RewardTermsCallback`: TensorBoard에 기록하는 항목

[`08-rl-development-log.md`](08-rl-development-log.md)의 숫자는 개발 기록이며 현재 기본값과 일부 다르다. 실험 설정을 기록할 때는 `RewardConfig` 전체와 Git commit/working-tree diff를 함께 저장한다.

## 2. 현재 보상 수식

기호:

- `Δt = control_dt = physics_timestep × frame_skip`, 기본 `0.02 s`
- `v = (vx, vy)`, `c = (command_vx, command_vy)`
- `ω = (roll_rate, pitch_rate, yaw_rate)`, `cω = command_yaw_rate`
- `g`는 body frame projected gravity
- `a_t`는 현재 18차원 residual action
- 모든 `w_*`와 `σ_*`는 `RewardConfig`의 대응 필드

### 명령 추종과 방향

```text
linear_tracking = exp(-||v - c||² / σ_linear²)
yaw_tracking    = exp(-(ωz - cω)² / σ_yaw²)
heading_tracking = exp(-heading_error² / σ_heading²)
upright          = exp(-||gxy||² / σ_gravity²)
```

각 tracking 값은 최적일 때 1, 오차가 커질수록 0에 가까워진다.

### 자세와 진동

```text
height_drop = max(0, reference_height - root_height)
height_penalty = (height_drop / σ_height)²

oscillation_penalty =
    (vz / σ_vertical)²
  + (ωroll / σ_roll_pitch)²
  + (ωpitch / σ_roll_pitch)²
```

높이는 one-sided다. 기준보다 높아지는 것은 벌하지 않고, 낮아지는 collapse만 벌한다.

### action과 idle

```text
action_rate_penalty      = mean((a_t - a_(t-1))²)
action_magnitude_penalty = mean(a_t²)

activity = clip(max(abs(command / command_scale)), 0, 1)
idle_fraction = clip(1 - activity / idle_activity_threshold, 0, 1)

idle_velocity_tracking = exp(
    -||v||² / σ_idle_linear²
    -ωz² / σ_idle_yaw²
)
idle_action_penalty = idle_fraction * action_magnitude_penalty
```

`idle_fraction`은 완전 neutral에서 1이고 threshold에 도달하면 0이다. 일반 tracking sigma보다 훨씬 작은 idle sigma가 미세한 정지 진동을 구분한다.

### 미끄러짐, 전류, 관절, 충돌

접촉점 미끄러짐:

```text
point_velocity = J_contact * qvel
tangential = point_velocity - normal * dot(point_velocity, normal)
excess = max(0, ||tangential|| - slip_deadzone)
slip_penalty = mean((excess / σ_slip)²) over valid tire contacts
```

normal force가 `contact_force_threshold` 미만인 접촉과 swing leg는 계산하지 않는다.

전류:

```text
current = (voltage - K * joint_velocity) / R
stall_current = stall_torque / K
current_penalty = mean((current / stall_current)²)
```

관절:

```text
joint_offset = abs(joint_position - default_radians)
excess = max(0, joint_offset - soft_joint_offset)
joint_limit_penalty = mean((excess / radians(15))²)
```

`forbidden_collision`은 1 N 이상인 ground 접촉 중 tire가 아닌 robot body가 닿으면 참이다. `collision_penalty`는 그 boolean의 0/1 값이다.

## 3. aggregate와 최종 합계

```text
velocity_term = Δt * (
    w_velocity * linear_tracking
  + w_idle_velocity * idle_fraction * idle_velocity_tracking
)

direction_term = Δt * (
    w_yaw * yaw_tracking
  + w_heading * heading_tracking
)

stability_term = Δt * (
    w_upright * upright
  - w_height * height_penalty
  - w_oscillation * oscillation_penalty
  - w_slip * slip_penalty
  - w_joint_limit * joint_limit_penalty
  - w_collision * collision_penalty
)

damping_term = -Δt * (
    w_action_rate * action_rate_penalty
  + w_action_magnitude * action_magnitude_penalty
  + w_idle_action * idle_action_penalty
  + w_current * current_penalty
)

total = velocity_term + direction_term + stability_term + damping_term
```

종료 조건이면 마지막에 `termination_penalty=5.0`을 한 번 뺀다. termination penalty만 `Δt`를 곱하지 않는다.

중요: `info["reward_terms"]`에는 세부 항과 aggregate가 함께 있다. 다음처럼 전부 합하면 중복 계산된다.

```python
# 잘못된 검증: velocity/yaw/upright 등의 세부 항과 aggregate가 겹친다.
sum(info["reward_terms"].values())
```

정확한 비종료 합계 검증은 네 aggregate만 사용한다.

```python
terms = info["reward_terms"]
expected = terms["velocity"] + terms["direction"] + terms["stability"] + terms["damping"]
```

종료 시에는 `terms["termination"]`도 한 번 더한다.

## 4. 현재 기본값

### sigma와 한계

| 필드 | 기본값 | 작게 만들면 | 크게 만들면 |
|---|---:|---|---|
| `linear_velocity_sigma` | 0.25 m/s | 정확한 xy 추종만 높은 점수 | 큰 속도 오차도 비슷한 점수 |
| `yaw_velocity_sigma` | 0.15 rad/s | yaw-rate 정밀 추종 | yaw 오차에 관대 |
| `heading_error_sigma` | 0.60 rad | heading drift에 민감 | heading drift에 관대 |
| `projected_gravity_sigma` | 0.25 | 작은 기울기도 구분 | 기울어져도 upright 보존 |
| `height_sigma` | 0.05 m | 작은 하강도 큰 penalty | collapse guard가 느슨 |
| `vertical_velocity_sigma` | 0.30 m/s | bounce에 민감 | 수직 진동에 관대 |
| `roll_pitch_rate_sigma` | 0.80 rad/s | 흔들림에 민감 | 회전 진동에 관대 |
| `slip_deadzone` | 0.02 m/s | 접촉 노이즈까지 penalty | 작은 실제 slip도 무시 |
| `slip_sigma` | 0.20 m/s | deadzone 이후 penalty 급증 | slip에 관대 |
| `soft_joint_offset` | 60° | 더 일찍 관절 penalty | 큰 자세 변화 허용 |
| `hard_joint_offset` | 90° | 더 일찍 종료 | 위험 각도까지 episode 유지 |

### weight

| 보상 | weight | penalty | weight |
|---|---:|---|---:|
| linear velocity | 2.0 | height | 0.05 |
| yaw rate | 1.0 | oscillation | 0.1 |
| heading | 0.75 | action rate | 0.02 |
| upright | 0.5 | action magnitude | 0.25 |
| idle velocity | 1.0 | idle action | 0.5 |
|  |  | current | 0.02 |
|  |  | slip | 0.1 |
|  |  | joint limit | 0.2 |
|  |  | collision | 1.0 |
|  |  | termination | 5.0 |

## 5. 가장 안전한 수정 절차

### 1단계: 증상을 측정한다

먼저 “잘 못 걷는다”를 구체적인 관측값으로 바꾼다.

| 증상 | 먼저 볼 로그 |
|---|---|
| 명령보다 느림 | `state/vx`, `reward/velocity`, action magnitude, current |
| 회전 명령 부정확 | `state/yaw_rate`, heading error, `reward/yaw`, `reward/heading` |
| 정지 시 떨림 | idle velocity/action, action rate, z/roll/pitch 진동 |
| 자세가 낮아짐 | height, height_drop, upright, current |
| 발이 끌림 | stance contact 수, slip, reference phase |
| 관절이 끝으로 감 | joint limit, residual distribution, termination |
| 자주 넘어짐 | fallen/collision/hard limit 원인별 빈도 |

### 2단계: sigma와 weight 중 하나만 바꾼다

- sigma는 “오차의 의미 있는 크기”를 정한다.
- weight는 정규화된 목표끼리의 우선순위를 정한다.
- tracking이 너무 넓어 좋고 나쁜 상태를 구분하지 못하면 sigma를 먼저 조정한다.
- scale은 적절하지만 다른 목표에 비해 중요도가 낮으면 weight를 조정한다.
- 한 실험에서 한 항 또는 한 가족만 바꾼다. 보통 25–50% 범위 변화부터 시작한다.

### 3단계: 코드 기본값 대신 설정 객체로 먼저 검증한다

환경을 직접 만들 때는 기본값을 영구 수정하지 않고 실험 설정을 주입할 수 있다.

```python
from src.rl.walk_learn import RewardConfig, SconeWalkEnv

reward_config = RewardConfig(
    velocity_weight=2.5,
    action_magnitude_weight=0.30,
    slip_weight=0.15,
)
env = SconeWalkEnv(reward_config=reward_config, curriculum="full")
```

현재 CLI의 `_build_env()`는 `reward_config` argument를 노출하지 않는다. 정식 실험 방법은 다음 중 하나다.

- 실험 branch에서 `RewardConfig` 기본값을 바꾸고 diff를 보존한다.
- CLI에 reward preset/설정 파일 argument를 추가하고 `_build_env()`까지 전달한다.

재현성이 필요한 여러 실험은 두 번째 방식이 낫다. 문자열로 임의 Python 표현을 실행하는 방식은 사용하지 않는다.

### 4단계: 고정 명령과 동일 seed로 비교한다

- zero action baseline으로 reference gait 자체를 확인한다.
- forward `(0.25,0,0)`, reverse, yaw, idle을 각각 분리한다.
- 같은 terrain/seed/standing pose로 before/after를 비교한다.
- 최소 3개 이상의 학습 seed와 여러 episode 평균을 사용한다.
- 평균 reward만 보지 말고 성공률, 속도 오차, fall 원인, current, slip을 같이 본다.

### 5단계: 짧은 검증 후 새 run을 시작한다

```bash
python -m src.rl.walk_learn check --steps 500 --curriculum easy
python -m src.rl.walk_learn check --steps 500 --curriculum full --random-actions
python -m src.rl.walk_learn train \
  --timesteps 100000 \
  --curriculum easy \
  --num-envs 4 \
  --output runs/reward_trial_name \
  --tensorboard-log runs/tensorboard
```

terrain 같은 global option은 subcommand 앞에 둔다.

```bash
python -m src.rl.walk_learn --terrain uneven --terrain-seed 7 check --steps 500
```

## 6. 항목별 조정 예시

### 속도를 더 잘 따라가게 하고 싶다

1. `linear_velocity_sigma`가 실제 달성 가능한 오차보다 너무 작은지 확인한다.
2. 너무 작으면 tracking이 거의 항상 0이 되어 gradient 신호가 약하므로 먼저 약간 늘린다.
3. tracking 값이 상태를 잘 구분하는데 우선순위만 낮으면 `velocity_weight`를 올린다.
4. policy가 큰 residual로 속도만 얻는다면 action/current/slip도 함께 확인하되 한 번에 모두 바꾸지 않는다.

### 정지 시 떨림을 줄이고 싶다

1. `idle_fraction`이 실제 neutral command에서 1인지 확인한다.
2. drift가 남으면 `idle_linear_velocity_sigma`/`idle_yaw_velocity_sigma`가 구분 가능한 범위인지 확인한다.
3. residual bias가 크면 `idle_action_weight`를 올린다.
4. active gait까지 둔해지면 일반 `action_magnitude_weight` 대신 idle 항만 조정한다.
5. 재생 시에는 reward와 별도로 `NeutralResidualGate`가 활성인지 확인한다.

### 넘어짐을 줄이고 싶다

- 자세가 계속 기울면 `upright_weight` 또는 gravity sigma를 검토한다.
- 높이만 낮아지면 one-sided `height_weight`를 검토한다.
- 매우 빠른 흔들림이면 `oscillation_weight`를 검토한다.
- 너무 빠른 termination은 policy가 회복을 배우지 못하게 할 수 있으므로 fall 경계와 penalty를 동시에 과도하게 강화하지 않는다.
- actuator가 포화되어 자세를 못 버티는 문제는 reward만으로 고치지 말고 motor spec, PID, 질량, 초기 자세를 먼저 확인한다.

### 미끄러짐을 줄이고 싶다

1. contact normal/point velocity가 맞는지 시각·수치로 확인한다.
2. `slip_deadzone`은 접촉 solver 노이즈보다 약간 크게 둔다.
3. `slip_sigma`로 “나쁜 slip 속도”의 규모를 정한다.
4. 마지막에 `slip_weight`를 올린다.
5. rolling drive를 학습할 계획이라면 현재 접선속도 penalty가 의도한 wheel rolling까지 벌할 수 있으므로 contact model을 분리한다.

### 에너지 사용을 줄이고 싶다

`current_weight`를 올리기 전에 motor `K/R/stall_torque`, XML dcmotor nominal 값과 Python `DCMotorSpec`이 일치하는지 확인한다. 값이 틀리면 reward가 물리 비용이 아니라 모델링 오차를 최적화한다.

## 7. 새 보상 항을 추가하는 방법

예를 들어 foot clearance 항을 추가한다면 다음 순서를 따른다.

1. `RewardConfig`에 `clearance_sigma`, `clearance_weight`를 추가한다.
2. `__post_init__()`에서 sigma가 양수이고 weight가 finite/non-negative인지 검증한다.
3. `_reward()`에서 물리량을 측정하고 먼저 무차원 raw penalty로 정규화한다.
4. `stability_term` 또는 `damping_term` 중 한 aggregate에 정확히 한 번 포함한다.
5. `raw_terms["clearance"]`에 weighted 값을 넣어 TensorBoard에 기록한다.
6. `total`에는 aggregate만 사용하며 세부 clearance를 또 더하지 않는다.
7. 단위 테스트를 만든다: 최적/나쁜 상태의 부호, weight=0, aggregate 합계, termination과 독립성.
8. zero action baseline과 random action smoke test를 실행한다.

권장 형태:

```python
clearance_penalty = normalized_clearance_error
stability_term -= reward.clearance_weight * clearance_penalty * self.control_dt
raw_terms["clearance"] = (
    -reward.clearance_weight * clearance_penalty * self.control_dt
)
```

## 8. 반드시 유지할 불변 조건

- 모든 sigma는 `> 0`이어야 한다.
- reward와 observation은 finite여야 한다.
- 보상/penalty weight는 의미상 음수가 되지 않게 한다.
- penalty term은 총합에 음수, tracking term은 양수로 들어간다.
- step reward는 termination을 제외하고 `control_dt`로 시간 정규화한다.
- action과 관측 shape를 바꾸면 기존 PPO checkpoint와 학습 resume가 호환되지 않는다.
- 관절 제한은 기준 자세 offset가 아니라 최종적으로 실제 기계 범위를 사용해야 한다.
- collision/slip 항은 정확한 geom 이름과 contact force threshold에 의존한다.
- reward 계산에서 physical controller를 열거나 명령하지 않는다.

현재 `RewardConfig.__post_init__()`는 idle threshold와 idle sigma만 검증한다. 다른 sigma를 0으로 바꾸면 division-by-zero가 발생할 수 있으므로, 보상 수정 작업에서는 모든 sigma/weight에 대한 validation을 함께 추가하는 것이 안전하다.

## 9. checkpoint 재사용 판단

| 변경 | 기존 checkpoint 재생 | 기존 checkpoint에서 학습 재개 | 권장 |
|---|---|---|---|
| weight/sigma만 변경 | 가능 | 기술적으로 가능 | 비교 실험은 새 run 권장 |
| termination 기준 변경 | 가능 | 가능하지만 데이터 분포 급변 | 새 run 또는 명시적 fine-tune |
| observation 값/순서/차원 변경 | shape가 같아도 의미 불일치 위험 | 권장하지 않음 | 새 run |
| 68→70차원 변경 | legacy adapter로 재생만 | 불가 | 새 run |
| action 차원/scale/관절 순서 변경 | 위험/불가 | 불가 | 새 run |
| standing pose/reference gait 변경 | 로드는 가능 | objective/동작 의미 변경 | 새 run 또는 별도 fine-tune 이름 |

on-policy PPO라도 과거 objective로 만들어진 policy/value state를 새 reward로 이어갈 수는 있다. 하지만 새 reward의 순수한 효과를 비교하는 실험이라면 동일 seed의 새 학습을 사용한다.

## 10. heading 적분 회귀 계약

`SconeWalkEnv.step()`은 아래 heading 적분을 policy step마다 정확히 한 번 수행한다.

```python
self._target_heading += self._command[2] * self.control_dt
```

heading/yaw weight나 step 순서를 바꿀 때는 이 적분이 중복되거나 빠지지 않도록 다음 계약을 유지한다.

- 일정 yaw command에서 N step 후 target heading이 `yaw_rate×control_dt×N`인지
- zero yaw에서 target heading이 변하지 않는지
- `±π` 경계를 지날 때 `_heading_error()`가 wrap되는지

## 11. 최소 회귀 테스트 목록

- 완벽한 속도/자세 상태에서 tracking 항이 최대인지
- linear/yaw error를 늘리면 각 tracking이 단조 감소하는지
- reference보다 높은 body는 height penalty 0인지
- reference 아래 body만 height penalty가 생기는지
- neutral에서 idle action penalty가 활성인지
- active command에서 idle fraction이 0인지
- action이 고정이면 action-rate penalty 0인지
- tire contact가 없으면 slip penalty 0/contact count 0인지
- forbidden body contact가 collision/termination을 만드는지
- soft joint limit 전에는 0, 이후 증가, hard limit 이후 종료인지
- `total = velocity+direction+stability+damping(+termination)`인지
- 모든 reward/diagnostic이 finite인지
- 70차원 관측과 18차원 action 계약이 유지되는지
