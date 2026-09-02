# `walk_v2` PPO 학습 분석 (2026-09-02)

이 문서는 현재 실행 중인 `walk-v2_full_20260902_103338` 학습을 코드, TensorBoard,
`monitor.csv`, 체크포인트 내부값, 고정 명령 재생으로 분석한 기록이다. 분석 시점의
체크포인트는 `scone_walk_v2_35399646_steps.zip`이고, 원격 학습은 중단하거나 설정을
바꾸지 않았다. 따라서 이후 총 step은 계속 증가할 수 있다.

분석 기준 Git commit은 `4e449e2`다. 이 문서는 원인과 재학습 설계를 제시하지만 PPO
환경이나 실행 중인 학습을 수정하지는 않는다.

---

## 1. 결론

현재 체크포인트는 "안 넘어지는 정책"은 만들었지만, 명령을 잘 따라 움직이는 정책은
아니다.

- 20초 episode는 대부분 1,000 policy step을 끝까지 버틴다.
- 하지만 고정 명령 평가에서 최신 PPO는 시험한 모든 명령에서 **zero residual보다
  return이 낮았다.**
- `vx=0.25 m/s` 명령에서 최신 정책의 평균 전진 속도는 `0.012 m/s`였고, zero
  residual 기준 모션도 `0.010 m/s`였다. PPO가 실질적인 전진 성능을 만들지 못했다.
- 환경에 들어가는 행동 성분의 약 **94.7%가 -1 또는 +1에 붙었다.** 정책의 raw 평균은
  행동 범위보다 최대 약 19배 컸다.
- 약 5M step에서 최고였던 episode return이 이후 지속적으로 낮아졌다. 35M 부근의
  KL divergence와 clipping fraction은 정상적인 PPO 업데이트라고 보기 어려울 만큼
  커졌다.
- 매 reset마다 `actuator_gainprm[:, 0]`을 이전 값에 다시 곱하는 코드가 있어 모터
  동역학이 episode마다 누적 변한다. 이름은 strength randomization이지만 실제로는
  DC motor 저항 `R`을 누적 변경한다.

따라서 현재 run을 더 오래 돌리는 것보다 **행동 분포와 domain randomization을 먼저
고치고 새 run으로 학습하는 것**이 우선이다. 이 두 항목을 바꾸면 환경의 의미가
달라지므로 현재 체크포인트를 resume해서는 안 된다.

---

## 2. 현재 PPO가 실제로 계산하는 흐름

```text
[vx, vy, yaw 명령]
        │
        ▼
82차원 관측 ──► MLP actor/critic [512, 256, 128]
        │
        ▼
18차원 대각 Gaussian raw action
        │
        ├─ SB3가 환경 호출 직전에 [-1, 1]로 clip
        ▼
hardcoded 기준 모션 + residual [20°, 22°, 26°]
        │
        ├─ 최종 관절 목표는 기본 자세 ±65°로 clip
        ▼
MuJoCoController PID + MuJoCo DC motor/접촉 모델
        │
        ▼
보상 16개 항 + 다음 관측
```

학습은 `50 Hz`다. MuJoCo physics timestep은 `0.002 s`, frame skip은 10이므로 PPO
한 step은 `0.02 s`다. 한 episode는 20초, 즉 최대 1,000 policy step이다.

### 2.1 관측 82차원

| 인덱스 | 크기 | 내용 | 정규화 |
| --- | ---: | --- | --- |
| `0:3` | 3 | 몸체 선속도 | `/ 2.0` |
| `3:6` | 3 | 몸체 각속도 | `/ 5.0` |
| `6:9` | 3 | 몸체 좌표 투영 중력 | 그대로 |
| `9:27` | 18 | 기본 자세 기준 관절 위치 | `/ π` |
| `27:45` | 18 | 관절 속도 | `/ 10.0` |
| `45:63` | 18 | 직전 적용 residual action | 그대로 |
| `63:66` | 3 | `[vx, vy, yaw]` 명령 | `[0.70, 0.25, 0.90]`으로 나눔 |
| `66:68` | 2 | gait phase sin/cos | 그대로 |
| `68:70` | 2 | heading error sin/cos | 그대로 |
| `70:76` | 6 | 각 발 접촉 여부 | 0 또는 1 |
| `76:82` | 6 | 각 발 수직력 | 체중/3으로 나누고 0~2 clip |

episode 단위 좌우 mirror와 관측 노이즈 `σ=0.01`이 적용된다. 82차원 shape와
체크포인트 shape는 일치했다.

### 2.2 행동과 기준 모션

행동 공간은 18차원 `[-1, 1]`이다. 각 관절군 residual 범위는 다음과 같다.

| 모터 | residual scale |
| --- | ---: |
| ID 1~6 | ±20° |
| ID 7~12 | ±22° |
| ID 13~18 | ±26° |

현재 run의 reference는 `hardcoded`, stance는 `standard`, curriculum은 `full`,
terrain은 `flat`, seed는 7, 병렬 환경 수는 9다.

`hardcoded` 기준 모션은 명령 활성도에 따라 `0.6~1.5 Hz`로 움직이고, upper motor와
ID 7~12 lift를 22°로 생성한다. 전진과 yaw는 기준 모션에 들어가지만 **lateral
명령은 보폭 방향을 만들지 않고 activity/lift만 키운다.** 따라서 lateral 학습은
처음부터 18개 residual이 전부 만들어야 한다.

### 2.3 명령과 curriculum

| 단계 | `|vx|` | `|vy|` | `|yaw|` |
| --- | ---: | ---: | ---: |
| easy | 0.20 | 0.00 | 0.00 |
| medium | 0.45 | 0.15 | 0.60 |
| full | 0.70 | 0.25 | 0.90 |

코드에는 easy → medium → full 자동 승급기가 없다. 이번 run은 첫 step부터 `full`을
사용했다. 명령은 2~5초마다 바뀌고 15% 확률로 idle이며, command filter의 time
constant는 0.30초다.

---

## 3. 보상함수

제어 주기 `dt=0.02`가 대부분 항에 곱해진다. 핵심 tracking은 다음과 같다.

```text
shortfall = max(0, |command_xy| - command 방향 실제 속도)
velocity_tracking = exp(-(shortfall² + lateral²) / 0.30²)
yaw_tracking      = exp(-(actual_yaw - command_yaw)² / 0.25²)
heading_tracking  = exp(-heading_error² / 0.20²)
upright            = exp(-||gravity_xy||² / 0.25²)

stability = (upright * exp(-oscillation))²
speed     = max(0, command 방향 실제 속도) * stability
```

전진 tracking은 one-sided이므로 명령보다 빠른 것은 벌하지 않는다. 각 항의 현재
가중치는 다음과 같다.

| 항 | 가중치 | 부호/역할 |
| --- | ---: | --- |
| velocity | 2.0 | 명령 방향 속도 shortfall와 lateral 오차 |
| speed | 4.0 | 안정 자세에서 실제 전진 속도 보상 |
| yaw | 0.8 | yaw rate 추종 |
| heading | 1.0 | 누적 heading 유지 |
| upright | 0.6 | 수평 자세 |
| air time | 0.25 | 명령 중 적절한 swing 후 touchdown |
| height | 0.4 | reset 후 기준 높이 유지 벌점 |
| oscillation | 0.10 | 상하 속도와 roll/pitch rate 벌점 |
| inactivity | 1.0 | 명령 중 1초 이상 접촉하지 않은 다리 벌점 |
| load share | 0.3 | 현재 접지 다리 사이의 수직력 불균형 벌점 |
| impact | 0.5 | 수직력 증가율 제곱 벌점 |
| slip | 0.15 | 접지점 tangential slip 벌점 |
| joint limit | 0.3 | 기본 자세에서 55°를 넘는 offset 벌점 |
| torque | 0.02 | actuator force 제곱 벌점 |
| action rate | 0.05 | 직전 행동과 차이 제곱 벌점 |
| action magnitude | 0.05 | 행동 크기 제곱 벌점 |
| collision | 1.0 | 발 이외 geom의 지면 충돌 |

넘어짐, 금지 충돌, hard joint limit 또는 비유한 상태에는 추가로 `-5`가 들어간다.

### 3.1 최근 1M step의 보상 기여

TensorBoard의 policy step당 평균은 다음과 같다.

| 항 | 평균 기여 |
| --- | ---: |
| velocity | `+0.019952` |
| impact | `-0.016011` |
| upright | `+0.009365` |
| yaw | `+0.006690` |
| air time | `+0.005843` |
| heading | `+0.002917` |
| slip | `-0.002481` |
| speed | `+0.001943` |
| torque | `-0.001068` |
| load share | `-0.000954` |
| action magnitude | `-0.000942` |
| oscillation | `-0.000902` |
| height | `-0.000569` |
| action rate | `-0.000502` |
| inactivity | `-0.000096` |

impact 벌점은 실제 speed 보상의 약 8.2배이고 velocity 보상의 대부분을 상쇄한다.
반대로 action이 거의 포화됐는데도 action magnitude 벌점은 `-0.000942`에 불과하다.

`vx=0.25`에서 정지한 로봇도 velocity 항만으로 대략
`2 × exp(-(0.25/0.30)²) × 0.02 ≈ 0.01997`을 받는다. 즉 움직이지 않아도
velocity, yaw, heading, upright의 양의 기본 보상이 크다. 총 return만 보면 명령
추종이 개선됐는지 분리하기 어렵다.

---

## 4. PPO 설정

| 항목 | 현재 값 |
| --- | ---: |
| algorithm | Stable-Baselines3 PPO |
| policy | MLP, actor/critic 각각 `[512, 256, 128]` |
| learning rate | `3e-4` 고정 |
| rollout | `512 step × 9 env = 4,608 sample` |
| batch size | `1,024` |
| epoch | 5 |
| gamma | 0.995 |
| GAE lambda | 0.95 |
| clip range | 0.2 |
| entropy coefficient | 0.003 |
| max gradient norm | 1.0 |
| target KL | 없음 |

`4,608`은 `1,024`로 나누어떨어지지 않는다. epoch마다 1,024 sample batch 4개와
512 sample batch 1개가 생기며 SB3도 truncated mini-batch 경고를 낸다. 병렬 환경
수가 호스트에 따라 달라질 수 있으므로 batch size는 실제 rollout buffer의 약수로
선택해야 한다.

---

## 5. 학습 로그 분석

### 5.1 1M~35M 변화

| step | episode return | 길이 | approx KL | clip fraction | action std | explained variance | velocity | speed |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 1M | 31.5 | 1000 | 0.024 | 0.213 | 0.95 | 0.91 | 0.0190 | 0.0042 |
| 5M | **40.8** | 1000 | 0.038 | 0.302 | 0.82 | 0.89 | 0.0286 | 0.0034 |
| 10M | 34.8 | 1000 | 0.047 | 0.321 | 0.89 | 0.93 | 0.0214 | 0.0019 |
| 15M | 29.4 | 1000 | 0.064 | 0.352 | 0.95 | 0.92 | 0.0141 | 0.0022 |
| 20M | 27.7 | 1000 | 0.086 | 0.409 | 1.10 | 0.88 | 0.0182 | 0.0035 |
| 25M | 29.4 | 1000 | 0.153 | 0.471 | 1.43 | 0.91 | 0.0192 | 0.0022 |
| 30M | 26.2 | 1000 | 0.365 | 0.562 | 2.03 | 0.87 | 0.0235 | 0.0012 |
| 35M | 24.0 | 1000 | **0.640** | **0.648** | **3.32** | 0.91 | 0.0234 | 0.0043 |

최근 1M에서 approx KL 평균은 `0.6875`, clip fraction 평균은 `0.6638`이었다.
정책 업데이트의 약 2/3가 PPO clip에 걸리고 있는데도 `target_kl`이 없어 epoch를
계속 수행한다. critic의 explained variance가 0.9 전후인 것은 actor가 명령을 잘
수행한다는 뜻이 아니다. critic이 현재의 낮은-quality return을 예측할 수 있다는
뜻일 뿐이다.

`monitor.csv` 37,143 episode를 시간순 10등분하면 return은 초기에 38.10까지
올랐다가 마지막 구간 24.55로 내려갔다. 초반 10%를 제외하면 episode는 거의 전부
1,000 step을 버텼다. 즉 실패는 주로 넘어짐이 아니라 **생존하면서 보상을 악화시키는
정책 업데이트**다.

### 5.2 고정 조건 체크포인트 비교

환경 randomization, 관측 노이즈, push, mirror, action delay를 끄고 `standard`,
`hardcoded`, `flat` 조건에서 phase seed 3개를 10초씩 평가했다.

| checkpoint | return | 평균 vx | 행동 포화율 |
| --- | ---: | ---: | ---: |
| zero residual | **32.68** | 0.010 m/s | 0.0% |
| 16.199M | 28.19 | 0.015 m/s | 93.3% |
| 16.299M | 27.48 | 0.010 m/s | 92.3% |
| 32.999M | 27.44 | 0.020 m/s | 95.1% |
| 33.099M | 25.63 | 0.022 m/s | 94.9% |
| 35.399M | 27.37 | 0.012 m/s | 94.7% |

현재 보존된 checkpoint 중 `vx=0.25` 고정 평가에서 zero residual을 이긴 모델은
없었다. 약 5M에서 학습 return이 가장 높았지만 rolling checkpoint가 최신 10개만
남기므로 당시 모델은 이미 삭제됐다.

### 5.3 최신 모델 명령별 평가

| 명령 | 정책 | 생존 | vx | vy | yaw rate | 평균 heading error 절댓값 | 포화율 | return |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| idle | zero | 3/3 | -0.000 | -0.000 | -0.000 | 0.000 | 0.0% | **43.95** |
| idle | PPO | 3/3 | -0.003 | -0.005 | -0.023 | 0.107 | 88.5% | 36.43 |
| vx=0.10 | zero | 3/3 | 0.001 | 0.000 | -0.002 | 0.007 | 0.0% | **41.61** |
| vx=0.10 | PPO | 3/3 | 0.002 | -0.002 | 0.003 | 0.067 | 94.3% | 35.84 |
| vx=0.25 | zero | 3/3 | 0.010 | 0.000 | -0.004 | 0.021 | 0.0% | **32.68** |
| vx=0.25 | PPO | 3/3 | 0.012 | -0.004 | -0.014 | 0.031 | 94.7% | 27.37 |
| vx=0.50 | zero | 3/3 | 0.074 | 0.001 | 0.017 | 0.066 | 0.0% | **26.48** |
| vx=0.50 | PPO | 3/3 | 0.078 | 0.008 | 0.032 | 0.182 | 95.7% | 15.19 |
| vy=0.10 | zero | 3/3 | 0.000 | 0.000 | -0.000 | 0.000 | 0.0% | **41.49** |
| vy=0.10 | PPO | 3/3 | 0.004 | -0.007 | -0.051 | 0.354 | 87.9% | 30.96 |
| yaw=0.40 | zero | 3/3 | 0.002 | 0.000 | 0.142 | 1.914 | 0.0% | **29.86** |
| yaw=0.40 | PPO | 3/3 | 0.010 | -0.004 | 0.084 | 2.346 | 90.1% | 25.41 |

lateral 명령은 부호도 맞지 않고, yaw 명령은 zero residual보다 느리다. 이 수치는
GUI로 보기 전에 정책 품질 자체가 낮음을 보여 준다. 단, nominal 3-seed 10초
평가이므로 sim-to-real 성능이나 모든 terrain의 성능을 의미하지는 않는다.

---

## 6. 원인 분석

### P0-1. Gaussian raw action과 환경 clipping이 분리돼 있다

현재 policy의 기본 분포는 squash하지 않은 대각 Gaussian이다. SB3 on-policy
rollout은 다음처럼 처리한다.

1. Gaussian에서 범위 제한 없는 raw action을 sample한다.
2. raw action의 log probability를 계산한다.
3. 환경에 보낼 때만 `[-1, 1]`로 clip한다.
4. rollout buffer에는 raw action과 raw log probability를 저장한다.

최신 policy의 `vx=0.25` deterministic raw mean을 측정하면 다음과 같다.

| 지표 | 값 |
| --- | ---: |
| raw action 평균 절댓값의 평균 | 8.169 |
| raw action 절댓값 p95 | 16.244 |
| raw action 절댓값 최대 | 18.752 |
| 환경 입력 행동 포화율 | 94.58% |
| mean log standard deviation | 0.532 |
| mean standard deviation | 3.469 |
| standard deviation 범위 | 0.098 ~ 7.776 |

예를 들어 raw action 2와 18은 모두 환경에서 +1이 된다. 환경 reward는 둘을 구분할 수
없는데 PPO 확률비와 entropy는 서로 다른 값으로 취급한다. 이 flat 영역에서 평균과
표준편차가 계속 커질 수 있고, 실제로 16.2M의 평균 std 약 0.99가 35.4M에서 3.47로
증가했다.

필요한 변경은 단순 env clip 유지가 아니라 **log probability까지 일관된 bounded
action distribution**이다. Stable-Baselines3에서 가능한 후보는 gSDE와
`squash_output=True` 조합 또는 correct log-prob correction을 포함한 tanh 분포다.
어느 쪽이 이 프로젝트에 맞는지는 짧은 학습으로 검증해야 한다. entropy coefficient와
초기 log std를 낮추는 것은 보조책이며 bounded mapping을 대신하지 못한다.

### P0-2. actuator randomization이 reset마다 누적된다

현재 reset은 body mass와 geom friction은 nominal에서 다시 계산하지만 actuator는
다음처럼 현재 값에 곱한다.

```python
self.model.actuator_gainprm[actuator, 0] *= self._strength_scale
```

`actuator_gainprm[:, 0]`의 nominal 사본을 저장하고 복원하는 코드가 없다. 따라서
episode `n`의 값은 `R0 × S1 × S2 × ... × Sn`이다.

더 중요한 점은 MuJoCo `mjGAIN_DCMOTOR`에서 이 항은 strength가 아니라 **저항 R**이다.
예를 들어 model의 첫 값은 다음과 같다.

| 모터군 | `gainprm[0]` R | `gainprm[1]` K |
| --- | ---: | ---: |
| ID 1~6 | 10.0007 | 2.08348 |
| ID 7~12 | 7.2911 | 2.49112 |
| ID 13~18 | 5.9528 | 1.4882 |

`S ~ Uniform(0.85, 1.15)`일 때 `E[log S] = -0.0037756`이다. median 누적 비율은
100 reset에서 0.686, 500 reset에서 0.151, 1,000 reset에서 0.0229다. 35.4M step,
9 env, 1,000-step episode를 단순 환산한 약 3,933 reset에서는 median이
`3.56e-7`까지 내려간다. 실제 각 worker의 값은 seed에 따라 다르지만 **동역학이
비정상적으로 누적 변한다는 사실은 확정적**이다.

R이 작아지면 `K/R`가 커지고 DC motor가 forcerange에 빨리 붙는 bang-bang 성향이
강해진다. 이는 학습 중 action 포화와 충격 벌점 증가를 악화시킬 수 있다.

수정할 때는 nominal `actuator_gainprm`을 저장해 reset마다 먼저 복원해야 한다.
그리고 의도가 토크, 전류, 전압, 기어 효율 중 무엇인지 정한 뒤 물리적으로 맞는
파라미터를 randomize해야 한다. R 변경을 strength라고 부르면 안 된다. body mass만
바꾸고 inertia를 그대로 두는 현재 방식도 물리적으로 일관되지 않으므로 함께 검토한다.

### P0-3. best-model 평가와 보존이 없다

현재 callback은 일정 주기의 checkpoint를 저장하고 최신 10개만 남긴다. 고정 seed
평가, `best_model.zip`, 성능 악화 시 rollback이 없다. 그래서 5M 부근의 상대적으로
좋은 정책이 삭제됐고 35M의 나쁜 정책만 남았다.

training return은 명령 분포와 randomization에 따라 흔들리므로 best 기준으로 바로
쓰면 안 된다. 별도 nominal evaluation env에서 idle/전진/lateral/yaw 고정 suite를
돌리고, command tracking과 안정성을 통과한 모델을 따로 보존해야 한다.

### P1-1. PPO update가 지나치게 크다

- 최근 approx KL 평균 `0.6875`
- 최근 clip fraction 평균 `0.6638`
- `target_kl` 없음
- learning rate `3e-4` 고정, epoch 5

행동 분포 문제를 먼저 고친 뒤 `target_kl` 약 0.01~0.03, learning rate 약 `1e-4`
또는 decay, epoch 3, max gradient norm 0.5를 시작 후보로 비교한다. 이는 확정값이
아니며 짧은 ablation에서 KL, clip fraction, 고정 평가를 함께 보고 선택한다.

### P1-2. 첫 step부터 full curriculum이다

hardcoded reference가 lateral scaffold를 제공하지 않는데 첫 step부터 `vy=±0.25`,
`yaw=±0.90`, `vx=±0.70`까지 동시에 요구한다. 먼저 easy의 고정 전진 명령에서
bounded action과 nominal dynamics가 실제 개선을 만드는지 확인한 뒤, 기준을 통과할
때 medium과 full로 승급하는 편이 원인 분리가 쉽다.

### P1-3. 보상 총합이 명령 추종 품질을 가린다

정지해도 velocity/yaw/heading/upright의 양의 reward가 들어가며 impact는 speed보다
훨씬 크다. reward를 즉시 크게 바꾸기 전에 먼저 action mapping과 dynamics를
정상화해야 한다. 이후 다음 순서로 조정한다.

1. zero residual과 PPO를 같은 fixed command로 평가한다.
2. tracking error, 실제 속도, idle drift, slip/impact를 return과 별도로 기록한다.
3. contact force difference의 단위와 impact scale을 확인한다.
4. action magnitude가 saturation을 실제로 억제하는지 확인한다.
5. 한 번에 하나의 reward family만 바꾸고 ablation한다.

---

## 7. 권장 수정·재학습 순서

1. **현재 run 보존**
   - 진단 자료로 checkpoint와 TensorBoard를 보존한다.
   - 아래 환경 변경 뒤에는 이 checkpoint를 resume하지 않는다.
2. **domain randomization 복구**
   - actuator nominal을 저장/복원하는 회귀 검사를 추가한다.
   - 여러 reset 후 같은 seed의 파라미터가 누적되지 않는지 검사한다.
   - mass와 inertia의 일관성을 정한다.
3. **bounded action distribution 적용**
   - raw action, env action, saturation, log std를 TensorBoard에 추가한다.
   - 수천~수만 step 스모크 학습에서 action mean/std가 발산하지 않는지 본다.
4. **PPO update 안정화**
   - `target_kl`, 낮은 learning rate/entropy, rollout buffer의 약수 batch를 비교한다.
5. **고정 평가와 best-model 보존**
   - idle, `vx=0.10/0.25/0.50`, `vy=0.10`, `yaw=0.40`을 같은 seed로 평가한다.
   - zero residual보다 나은 모델만 best로 저장한다.
6. **단계식 curriculum**
   - easy 전진 → medium lateral/yaw → full 순서로 승급한다.
7. **보상 ablation**
   - mechanics와 PPO가 정상화된 뒤 impact, constant tracking baseline, action cost를
     한 계열씩 조정한다.

### 7.1 최소 통과 기준

다음은 새 run을 수천만 step까지 늘리기 전의 권장 gate다.

- 여러 reset 후 actuator/mass/friction 값이 nominal 기준 지정 범위 안에 있고 누적되지
  않는다.
- nominal fixed-command 평가의 action saturation이 10% 미만이며, 가능하면 5% 미만이다.
- approx KL이 대체로 0.03 이내, clip fraction이 대체로 0.25 이내다.
- idle에서 PPO가 zero residual보다 drift와 return을 악화시키지 않는다.
- `vx=0.10`, `vx=0.25`에서 PPO가 zero residual보다 속도 추종과 return을 모두
  개선한다.
- lateral과 yaw는 적어도 명령과 같은 부호이며, heading drift가 zero보다 나쁘지 않다.
- 3개 이상의 평가 seed에서 같은 결론이 나오고 best checkpoint가 별도 보존된다.

이 수치는 장기 목표 성능을 보장하는 합격선이 아니라, 명백히 망가진 run을 조기에
중단하기 위한 최소 기준이다.

---

## 8. 재현과 시각 확인

최신 로컬 checkpoint를 GUI로 재생하려면 macOS에서는 시스템 Python 대신
`mjpython`을 사용한다.

```bash
mjpython -m src.rl.walk_v2 \
  --reference-motion hardcoded \
  --terrain flat \
  enjoy runs/<run>/checkpoints/scone_walk_v2_<steps>_steps.zip \
  --command 0.25 0 0 --episodes 3
```

원격 watcher와 실시간 중계도 이제 checkpoint의 관측 shape를 보고 V2 환경을
선택한다. 이 viewer 복구는 commit `4e449e2`에 포함돼 있다. 수정 전에는 V2의
82차원 checkpoint를 구형 70차원 환경으로 열거나, V2 `step()`이 viewer를 sync하지
않아 MuJoCo 창이 나타나지 않았다.

TensorBoard는 run의 event 디렉터리를 지정해 다음 항목을 함께 본다.

```bash
tensorboard --logdir runs/<run>/tensorboard
```

필수 plot은 `rollout/ep_rew_mean`, `train/approx_kl`, `train/clip_fraction`,
`train/std`, `train/explained_variance`, `reward/*`다. 다음 학습부터는 raw action
mean/std와 env action saturation도 반드시 기록해야 한다.

---

## 9. 분석 범위와 남은 한계

- 이 결과는 MuJoCo nominal flat 환경과 현재 모델/컨트롤러에 대한 것이다. 실물
  성능을 증명하지 않는다.
- latest checkpoint를 3개 phase seed로 10초씩 평가했다. 더 확실한 비교에는 여러
  terrain과 10개 이상 seed가 필요하다.
- 실행 중인 원격 학습의 프로세스와 설정은 건드리지 않았다. 수집 이후 로그는 이
  문서의 snapshot에 포함되지 않는다.
- 현재 정책이 넘어지지 않는 것은 확인했지만, 포화된 actuator 명령이 실물에 안전하다는
  뜻은 아니다. 이 checkpoint를 실물에 배포해서는 안 된다.

---

## 10. 후속 수정 완료 (commit `ec5b7d7`)

§6~§7의 P0 항목과 PPO update guard, 고정 평가를 실제 코드에 적용했다. 기존
`walk-v2_full_20260902_103338` 프로세스는 당시 Python 코드를 이미 메모리에 올려
실행 중이므로 이 수정의 영향을 받지 않으며, 중단하거나 덮어쓰지 않았다.

### 10.1 누적 motor randomization 수정

환경을 만들 때 다음 nominal 배열을 별도로 저장한다.

```text
nominal body_mass
nominal body_inertia
nominal geom_friction
nominal actuator_gainprm
nominal actuator_forcerange
```

매 reset은 반드시 nominal에서 다시 시작한다. mass scale `M`, friction scale `F`,
motor strength scale `S`에 대해 현재 적용식은 다음과 같다.

```text
body_mass    = nominal_body_mass × M
body_inertia = nominal_body_inertia × M
sliding_friction = nominal_sliding_friction × F

R = nominal_R / S
K = nominal_K
torque_limit = nominal_torque_limit × S
```

DC motor의 토크는 `τ = K/R × (V - Kω)`이므로 `R/S`는 같은 속도와 전압에서
토크-속도 직선 전체를 `S`배 한다. no-load speed `V/K`는 변하지 않는다. torque
limit도 `S`배 해야 stall 부근에서 같은 scale을 유지한다. 이전처럼 `R *= S`를
누적하지 않는다.

mass만 바꾸고 inertia를 그대로 두던 비일관성도 함께 수정했다. 변경 뒤
`mj_setConst()`로 model 파생 상수를 다시 계산한다. 고정 scale로 두 번 reset해도
gain, torque limit, mass, inertia, friction이 첫 reset과 완전히 같은 회귀 검사를
추가했다.

### 10.2 bounded PPO action

새 학습 기본 분포는 Stable-Baselines3가 log-probability correction을 지원하는
gSDE + tanh squash다.

```text
use_sde          = true
squash_output    = true
use_expln        = true
sde_sample_freq  = 4 policy steps
log_std_init     = -1.5  (초기 std 약 0.223)
```

이제 policy가 학습하는 행동과 환경이 받는 행동이 같은 `[-1, 1]` 공간에 있다. 이전
unbounded Gaussian checkpoint는 `use_sde=False`, `squash_output=False`이므로
`--resume` 시 명시적으로 거부한다. 재생은 분석·비교를 위해 계속 허용하지만 새
환경 학습에 섞지 않는다.

### 10.3 PPO update 기본값

| 항목 | 실패 run | 새 기본값 |
| --- | ---: | ---: |
| learning rate | `3e-4` | `1e-4` |
| epoch | 5 | 3 |
| entropy coefficient | 0.003 | 0.0 |
| max gradient norm | 1.0 | 0.5 |
| target KL | 없음 | 0.02 |
| batch size | 1,024 | 512 |
| rollout | `512 × num_envs` | 동일 |

batch size는 `n_steps × num_envs`를 나누어떨어져야 한다. 기본 512는 병렬 환경 수가
몇 개이든 `512 × num_envs`의 약수다. 직접 잘못된 값을 지정하면 SB3 warning으로
넘기지 않고 학습 전에 유효한 인접 약수와 함께 `ValueError`를 낸다.

새 튜닝 인자는 `--learning-rate`, `--n-epochs`, `--target-kl`,
`--entropy-coefficient`, `--max-grad-norm`, `--sde-sample-freq`,
`--log-std-init`이다. 기본값부터 시작하고 한 번에 하나만 ablation한다.

### 10.4 올바른 TensorBoard 평균

기존 `RewardTermsCallback`은 vector env 사이만 평균하고 매 policy step마다 같은
logger key를 덮어썼다. SB3가 rollout 끝에 dump할 때는 사실상 마지막 step 값만
남았다. §3.1의 과거 값은 TensorBoard point 사이의 추세 비교에는 쓸 수 있지만
rollout 전체 시간평균으로 해석하면 안 된다.

수정 후 callback은 `n_steps × num_envs` 전체의 reward term을 누적한 뒤 rollout
끝에 한 번 기록한다. 다음 행동 진단도 함께 추가했다.

```text
action/abs_mean
action/abs_max
action/saturation_fraction  # |action| >= 0.98
action/latent_abs_mean
action/latent_abs_max
```

### 10.5 fixed-command 평가와 best model 승격

checkpoint interval마다 randomization이 없는 flat nominal 환경에서 각 3 episode,
10초씩 다음 여섯 명령을 평가한다.

```text
idle
vx = 0.10 / 0.25 / 0.50 m/s
vy = 0.10 m/s
yaw = 0.40 rad/s
```

평가 score는 dense reward 합이 아니다.

```text
tracking_error = mean(|achieved - command| / [0.70, 0.25, 0.90])

score = survival_rate
      - tracking_error
      - 0.10 × mean_abs_heading_error / π
      - 0.05 × action_saturation_fraction
```

생성 파일:

| 파일 | 의미 |
| --- | --- |
| `evaluation_baseline.json` | 같은 reference의 zero residual 기준 |
| `evaluation_history.jsonl` | 매 평가의 명령별 실제 속도·return·포화·생존 |
| `best_candidate_model.zip` | 지금까지 policy 중 score가 가장 높은 후보 |
| `best_candidate_metrics.json` | 후보 선택 근거 |
| `best_model.zip` | zero residual보다 score가 높고 모든 이동축 부호가 맞는 모델만 승격 |
| `best_model_metrics.json` | 승격 모델 근거 |

따라서 학습 초기의 덜 나쁜 정책은 candidate로 보존되지만, 기준 모션보다 못하면
실사용 best로 표시되지 않는다. `--eval-every`, `--eval-episodes`, `--eval-seconds`로
비용을 조절할 수 있다.

### 10.6 실제 64-step 스모크 결과

`hardcoded`, `standard`, easy, 1 env, 32-step rollout을 두 번 실행했다.

| 검증 항목 | 결과 |
| --- | ---: |
| 저장 policy `use_sde` | `True` |
| 저장 policy `squash_output` | `True` |
| target KL | 0.02 |
| 초기 std | 0.223 |
| 2번째 update approx KL | 0.000106 |
| rollout 행동 포화율 | 2.26% → 1.74% |
| deterministic fixed-eval 포화율 | 0% |
| zero score | 0.8816 |
| 초기 policy score | 0.8810 |
| `best_candidate_model.zip` | 생성됨 |
| `best_model.zip` | 생성 안 됨 — zero를 못 이겨 정상적으로 승격 거부 |

이것은 성능 학습 결과가 아니라 plumbing 검증이다. 64 step policy가 zero residual을
못 이기는 것이 정상이며, 여기서는 분포·학습·평가·저장 계약이 함께 작동하는지만
확인했다.

전체 테스트는 다음 명령으로 167개가 통과했다.

```bash
python -m unittest discover -s tests -v
```

### 10.7 새 학습 시작 방법과 남은 순서

새 환경에서는 기존 checkpoint를 resume하지 말고 easy에서 별도 run을 만든다.

```bash
python -m src.rl.walk_v2 \
  --reference-motion tripod-gait \
  --stance standard \
  --terrain flat \
  --terrain-seed 7 \
  train \
  --curriculum easy \
  --timesteps 50000000 \
  --num-envs 4 \
  --checkpoint-every 100000 \
  --keep-checkpoints 20 \
  --output runs/walk-v2-bounded-easy \
  --tensorboard-log runs/walk-v2-bounded-easy/tensorboard
```

현재 남은 단계는 easy 결과가 §7.1 gate를 통과하는지 확인한 뒤 medium/full로
진행하는 것과, 그 후에만 impact·constant tracking reward를 ablation하는 것이다.

### 10.8 기존 run 후속 상태

수정 직전 원격 run을 다시 읽었을 때 40,877,568 step이었다.

```text
episode return 22.2
approx KL      0.8497
clip fraction  0.698
action std     6.37
```

35M snapshot보다 더 악화됐고 목표가 1,000,000,000 step이라 자동 종료를 기다리기에는
매우 길다. 다만 이번 수정·문서화 작업에서는 기존 프로세스를 멈추지 않았다. 새 run을
같은 호스트에서 정상 env 수로 시작하려면 기존 run을 graceful pause해 CPU를 확보하는
운영 결정이 별도로 필요하다.
