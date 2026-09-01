# Non-RL·Residual RL 보행 성능 분석과 개선 로드맵

> 이름 변경 안내: 이 문서의 과거 `Non-RL` 수치는 현재
> `tripod-gait`에 해당한다. `scone-gait`는 2026-09-01 추가된 SCONE
> 부채꼴 rolling/creep 실험 모드이므로 아래 기존 비교 수치에 포함되지 않는다.

## 1. 결론

Non-RL이 하드코드보다 느려야 하는 구조적 이유는 없다. IK를 쓰는 비용은 학습
FPS에는 영향을 주지만, 생성된 관절 목표를 같은 50 Hz로 전송한 뒤의 로봇 속도를
직접 제한하지는 않는다. 현재 느림은 다음 세 문제가 겹친 결과다.

1. **명령보다 작은 보폭 상한**: Non-RL 전후 stroke는 `60 mm`에서 포화된다.
2. **관절 속도 한계를 넘는 발 궤적**: 특히 ID 13–18 목표가 프로파일 속도보다
   빠르게 바뀌어 큰 추종 지연이 생긴다.
3. **기준 보행을 덮어쓰는 policy**: 보유 checkpoint의 deterministic action은
   평균 절댓값 약 `0.95`로 거의 전 관절에서 포화되어, 더 빠른 zero-residual
   기준 모션을 오히려 느리게 만든다.

따라서 단순히 cadence나 모터 speed 숫자만 올리는 방식은 맞지 않는다. 명령 범위,
접지 궤적, actuator-aware 시간 배분, residual 학습을 순서대로 다시 맞춰야 한다.

### 1.1 2026-09-01 비-RL 조종 경로 적용 결과

이 문서 아래의 60 mm/0.7 Hz 분석은 RL reference와 2026-08-31 기준을 설명하는
역사적 측정으로 그대로 보존한다. 최초 비-RL MuJoCo 조종 sweep의
80 mm/0.8 Hz, speed 160, acceleration 50은 평균 속도만 보면 나아졌지만 후속
방향 진단에서 다시 교체됐다. 현재는 90/70 mm, 1.0 Hz, 25 mm lift, profile
무제한, middle stiffness 2배다.

| 경로 | 평균 속도 | root Z 하방 변화 | 비고 |
|---|---:|---:|---|
| 이전 interactive tripod | 0.0639 m/s | 0 mm, 위로 +19.91 mm 변동 | speed100/acc20, 60 mm |
| 1차 tuned tripod | 0.1058 m/s | −0.10 mm | 유한 profile, 80 mm; 방향 흔들림으로 교체 |
| 현재 SCONE-tuned tripod | 0.1184 m/s | −0.02 mm | 8초, 역방향 3.7 mm, 측면 0.7 mm, yaw 1.17° |
| 현재 full-body continuous-roll scone | 0.2093 m/s | −20.68 mm | 6초, lower 3.09회전, B +60° |

PPO/RL reference 값은 이 표로 자동 변경하지 않았다. 모든 후보·실패·phase
가설은 [`12-automatic-stair-demo-and-continuous-roll-rework.md`](12-automatic-stair-demo-and-continuous-roll-rework.md)에 있다.

## 2. 비교 대상부터 분리해야 한다

프로젝트에서 “하드코드”는 세 가지를 가리킬 수 있다.

| 이름 | 코드 경로 | 동작 |
|---|---|---|
| Legacy `Walk` | `src/locomotion/walk.py` | `time.sleep()`과 20° 위치 명령으로 한 stride씩 실행 |
| RL `hardcoded` reference | `SconeWalkEnv._reference_motion_degrees()` | 기존 tripod 동작을 연속 사인파로 만든 residual 기준 |
| 학습된 hardcoded policy | 위 기준 + PPO residual | checkpoint가 18개 관절 보정을 추가 |

Legacy, zero-residual reference, 학습된 policy는 같은 알고리즘이 아니다. 특히
학습된 결과가 느리다고 해서 Non-RL 기준 생성기 자체가 느리다는 뜻은 아니다.
아래 측정은 Standard 자세, 평지, 50 Hz policy/500 Hz physics를 공통으로
사용했다. 물리 profile benchmark는 `walking_speed=100`, XM acceleration 20을
별도로 적용했고, 기존 PPO checkpoint 비교는 실제 학습과 같은 무제한 simulation
profile을 사용했다. 두 결과를 섞어 checkpoint 성능으로 해석하면 안 된다.

## 3. 2026-08-31 재측정 결과

측정 기준은 commit `9fd774a` 위 working tree다. 현재 변경에는 말단 최저 패치
중심 지지점, 0.7 Hz cadence, 전후 60 mm/측면 50 mm 작업공간이 포함된다. 기존
PPO는 무제한 simulation profile에서 학습됐으므로 현재 RL reset도 그 동역학을
보존한다. 물리 profile 수치는 회귀 원인을 분리하기 위한 별도 진단이다. GUI와
headless 결과를 함께 사용했지만 실물 장시간 측정은 아니므로 절대 안전 성능으로
해석하지 않는다.

### 3.1 기준 모션만 비교하면 Non-RL이 더 빠르다

residual action을 전부 0으로 두고 seed 3/7/11의 10초 결과를 평균했다.

| 전진 명령 | RL hardcoded reference | Non-RL reference | Non-RL / hardcoded |
|---:|---:|---:|---:|
| `0.18 m/s` | `0.00721 m/s` | `0.04554 m/s` | `6.32×` |
| `0.30 m/s` | `0.01400 m/s` | `0.05521 m/s` | `3.94×` |

즉 현재 체감 속도 차이의 원인을 “IK라서 원래 느리다”로 결론 내릴 수 없다.
zero-residual Non-RL은 같은 물리 제한에서 오히려 더 빠르다.

### 3.2 현재 저장된 policy는 기준 모션을 느리게 만든다

hardcoded 기준 checkpoint를 seed 7, deterministic으로 10초 재생했다. 기준
선택 기능 도입 전 9.8M이 후속 15.4M·22.1M보다 빨랐고, action 포화는 모든 큰
checkpoint에 남아 있었다.

| checkpoint/동역학 | 명령 | 전진 속도 | 평균 `|action|` | 결과 |
|---|---:|---:|---:|---|
| hardcoded 9.8M / 학습 당시 | `0.18` | `0.0727 m/s` | `0.973` | 비교군 중 가장 빠름 |
| hardcoded 15.4M / 학습 당시 | `0.18` | `0.0509 m/s` | `0.975` | 전진 복구, yaw drift 잔존 |
| hardcoded 22.1M / 학습 당시 | `0.18` | `0.0175 m/s` | `0.975` | 후속 학습에서 성능 퇴행 |
| hardcoded 15.4M / 뒤늦게 물리 profile 강제 | `0.18` | `-0.0225 m/s` | `0.972` | 전진 명령에서 후진; 해당 변경은 되돌림 |

action 범위는 `[-1, 1]`이고 residual scale은 upper/middle/lower 각각
`±10°/±12°/±15°`다. 평균 절댓값이 `0.95`라는 것은 policy가 작은 보정을 하는
residual controller가 아니라 거의 항상 최대 관절 오프셋을 쓰는 별도 gait가
되었다는 뜻이다. 여기에 학습하지 않은 actuator profile까지 덧붙이면 위상 관계가
깨져 후진까지 발생한다. profile 회귀는 복구했지만 policy 포화와 yaw drift는
checkpoint 자체의 남은 문제다.

### 3.3 사용자의 “움직임 반경이 작다”는 판단은 일부 맞다

목표 관절각을 동일한 부채꼴 말단 support point의 body-frame 궤적으로 다시
변환했다.

| 기준 모션 | 명령 | 평균 전후 범위 | 다리별 최대 전후 범위 | 수직 범위 |
|---|---:|---:|---:|---:|
| Non-RL | CLI 최대 `0.18` | `60.0 mm` | `60.0 mm` | `35.0 mm` |
| hardcoded | `0.18` | `36.3 mm` | `54.4 mm` | `13.9 mm` |
| hardcoded | full 최대 `0.50` | `108.0 mm` | `156.1 mm` | `48.8 mm` |

일상적인 `0.18` 명령에서는 Non-RL 범위가 더 크지만, full curriculum의 큰
명령에서는 hardcoded upper 관절 `±20°`가 Non-RL의 고정 60 mm 상한보다 훨씬
큰 궤적을 만든다. 다만 hardcoded 궤적은 전후 동작과 비슷한 크기의 불필요한
측면 궤적도 만들고, 실제 actuator가 그 큰 사인파를 모두 추종하지 못한다.
“범위를 키우면 무조건 빨라진다”가 아니라 유효 전후 stroke와 추종 가능한 관절
속도를 함께 최적화해야 한다.

### 3.4 Non-RL 말단 관절 목표가 물리 프로파일보다 빠르다

CLI 최대 전진, 0.7 Hz, 60 mm/35 mm 궤적을 50 Hz로 생성한 target 분석이다.

| 관절 그룹 | 목표각 peak-to-peak 평균 | target 속도 p95 | target 속도 최대 | speed=100 한계 |
|---|---:|---:|---:|---:|
| upper, MX ID 1–6 | `18.4°` | `48.5°/s` | `60.0°/s` | `68.4°/s` |
| middle, XM ID 7–12 | `16.2°` | `73.3°/s` | `116.5°/s` | `137.4°/s` |
| lower, XM ID 13–18 | `43.8°` | `181.2°/s` | `279.1°/s` | `137.4°/s` |

upper/middle은 대부분 한계 안이지만 lower는 p95부터 한계의 `1.32×`, 순간
최대는 `2.03×`다. 실제 동역학에서 `0.30 m/s` 명령을 주면 lower의
target-to-setpoint p95 지연이 다리별 최대 약 `33.8°`까지 커졌다. 현재 알고리즘은
발끝의 매끄러운 Cartesian 곡선만 보장하고, 그 곡선을 만드는 각 관절의 속도와
가속도는 제한하지 않는다. 이것이 가장 직접적인 actuator 병목이다.

### 3.5 학습 명령 대부분이 같은 60 mm stroke로 뭉개진다

stance 시간은 `duty_factor / cycle_frequency = 0.5 / 0.7 = 0.714 s`다.
따라서 60 mm stroke가 포화되기 시작하는 순수 전진 명령은 이론상 약
`0.060 / 0.714 = 0.084 m/s`다. 그러나 curriculum은 최대 `0.30`, `0.40`,
`0.50 m/s`를 요구한다.

현재 분포를 10,000개 표본으로 계산하면 다음과 같다.

- `easy` 전진 표본의 `56.79%`가 이미 stride clipping에 걸린다.
- `full` 3축 균등 표본의 `99.71%`에서 한 다리 이상 clipping된다.
- `full` 표본의 `85.11%`는 여섯 다리가 모두 clipping된다.
- 평균 leg clip fraction은 `96.68%`다.

결과적으로 policy 관측에는 서로 다른 `0.1~0.5 m/s` 명령이 들어오지만 기준
보행은 거의 같은 60 mm 궤적을 낸다. 달성 불가능한 속도 오차를 residual만으로
메우라고 요구하는 셈이며, command-conditioned policy가 명령 크기를 학습하기
어렵다.

### 3.6 같은 `0.18` 명령인데 CLI와 RL의 발 들기 높이가 다르다

`step_height`는 현재 `activity = |command| / max_command`에 곱해진다.

- CLI Non-RL: `max_vx=0.18`, 명령 `0.18` → activity `1.0` → `35 mm`
- RL Non-RL: `max_vx=0.50`, 명령 `0.18` → activity `0.36` → `12.6 mm`

전후 stroke는 60 mm로 이미 포화되는데 발은 12.6 mm만 든다. 즉 큰 수평 이동에
비해 clearance가 부족해 발 끌림과 접촉 충격이 늘 수 있다. `activity`를 전체
curriculum 최대값이 아니라 실제 stride utilization 또는 별도 lift schedule로
계산해야 한다.

## 4. 알고리즘 구조의 핵심 한계

### 4.1 stance 발이 월드에 고정되지 않는다

현재 stance 목표는 매 frame마다 명목 body-frame 발 위치에 명령 속도로 계산한
offset을 더한다. touchdown 순간의 실제 world contact를 저장하지 않고, 측정된
body 속도나 slip을 다음 목표에 피드백하지도 않는다. 몸체가 예상보다 느리면 발이
지면에 고정되는 대신 계속 미끄러진다.

개선 방향은 touchdown에서 support point의 world 좌표를 저장하고, stance 동안
현재 body pose로 다시 변환한 target을 IK에 넣는 것이다. 여기에 측정 body 속도
오차를 이용한 Raibert형 foot-placement 보정을 더하면 open-loop command와 실제
이동량의 차이를 줄일 수 있다.

### 4.2 부채꼴 접촉을 고정된 점 하나로 근사한다

현재 개선된 support point는 한쪽 mesh vertex가 아니라 최저 0.1 mm 패치 중심이다.
이것은 lateral bias를 줄였지만, 관절이 움직여 부채꼴 자세가 바뀐 뒤에도 같은
local point를 발끝으로 사용한다. 실제 최저점과 접촉점은 부채꼴을 따라 이동한다.

3관절로 한 점의 xyz를 정확히 맞추면 말단 자세를 독립적으로 지정할 자유도는
남지 않는다. 따라서 다음 중 하나가 필요하다.

- 현재 관절 자세에서 실제 최저 contact 후보를 다시 선택하는 active support point
- 접촉 위치·법선·관절속도를 함께 최소화하는 constrained optimization
- 미리 계산한 contact-aware joint-space 궤적을 사용하고 Cartesian IK는 작은
  보정에만 사용

### 4.3 Cartesian minimum-jerk가 joint-space 제한을 보장하지 않는다

발끝 경로는 부드러워도 Jacobian이 나쁜 구간에서는 작은 발 움직임이 lower 관절의
큰 회전으로 바뀐다. 현재 damped least-squares IK는 위치 오차와 수렴만 보며
관절별 속도, 가속도, torque margin을 비용에 넣지 않는다.

필요한 것은 actuator-aware time scaling과 weighted/constrained IK다. 각 frame의
`dq = J⁺ dx`를 계산해 ID별 profile velocity/acceleration 한계를 넘으면 phase
증분이나 해당 축의 stroke를 줄여야 한다. 반대로 여유가 있는 구간에서만 cadence와
stride를 늘려야 한다.

### 4.4 RL 목표가 현재 기준 보행의 도달 범위와 맞지 않는다

Non-RL zero-residual의 실측 최대가 현재 약 `0.055~0.062 m/s`인데 full command는
`0.50 m/s`다. 목표가 너무 멀면 velocity reward가 넓은 오차 영역에서 작은 차이만
보이고, policy는 residual 포화로 다른 보상 항을 공략할 수 있다. 기존 policy의
평균 `|action|≈0.95`는 이 실패를 직접 보여준다.

## 5. 우선순위별 개선 계획

### P0 — 측정 기준과 checkpoint를 먼저 고정

1. reference, support point, actuator profile, reward를 하나의 환경 버전으로
   고정한다. 물리 profile 정책은 기존 checkpoint를 resume하지 않고 0 step부터
   별도 학습한다.
2. 학습 전·중·후에 같은 command grid와 seed로 다음 세 대상을 자동 비교한다.
   - hardcoded zero residual
   - Non-RL zero residual
   - PPO policy
3. checkpoint 승격 조건에 전진/측면/yaw 오차, neutral drift, action saturation,
   stride clip, joint tracking lag, contact slip을 모두 넣는다.
4. policy가 zero-residual baseline보다 나쁘면 “학습 step이 많다”는 이유만으로
   최신 checkpoint를 채택하지 않는다.

### P1 — Non-RL 궤적을 actuator/contact-aware하게 변경

1. lift activity를 stride utilization에서 계산하고, 보행이 시작되면 최소 clearance를
   보장한다.
2. IK 결과의 관절 속도·가속도로 phase를 time-scale한다. 먼저 lower p95 target
   속도를 `137.4°/s` 아래로 만드는 것이 목표다.
3. touchdown world anchor와 측정 body-velocity feedback을 추가해 stance slip을
   닫힌고리로 줄인다.
4. 고정 sector-tip point를 실제 active contact point 또는 contact-aware 사전
   궤적으로 교체한다.
5. 그 뒤에만 stride와 cadence를 joint limit sweep으로 확장한다. 이전처럼
   cadence만 1.4 Hz로 올리지 않는다.

### P2 — residual RL이 기준 모션을 보존하도록 변경

1. 첫 curriculum은 현재 달성 가능한 전진 범위, 예를 들어 `0~0.06 m/s`에서
   시작하고 baseline 개선이 확인될 때만 범위를 넓힌다.
2. residual scale을 현재 `10°/12°/15°`보다 작게 시작하고 학습 진행에 따라
   늘리는 curriculum을 사용한다.
3. `mean(|action|)`, `action saturation fraction`, baseline 대비 속도 개선량을
   TensorBoard와 checkpoint 평가에 기록한다.
4. reward는 절대 속도 추종뿐 아니라 `policy 결과 - zero-residual 결과`의 개선을
   평가하도록 별도 benchmark를 둔다. 학습 reward 자체를 바꿀 때는
   [`06-reward-function-guide.md`](06-reward-function-guide.md)의 한-family-at-a-time
   절차를 따른다.

### P3 — 실물 데이터로 시뮬레이터를 닫는다

1. ID별 step response에서 목표/현재 위치, 속도, 전류 또는 load, supply voltage를
   기록한다.
2. 실제 기계 hard stop과 안전 관절범위를 측정해 임시 `±60°/±90°`를 교체한다.
3. 말단 접촉점 영상 또는 접촉 표식으로 실제 sector support 위치와 slip을 측정한다.
4. 이 데이터로 profile speed, acceleration, PD gain, friction을 맞춘 뒤 동일
   command grid를 실물과 MuJoCo에서 비교한다.

## 6. 다음 변경의 합격 기준

다음 튜닝은 “보기에 빨라짐” 대신 아래 기준으로 판정한다.

| 지표 | 첫 목표 |
|---|---|
| Non-RL zero-residual 전진 | 현재 `0.055 m/s` 이상 유지하며 증가 |
| PPO policy | 모든 핵심 명령에서 자신의 zero-residual baseline 이상 |
| easy stride clipping | 현재 `56.79%`에서 `20%` 미만 |
| lower target 속도 | p95가 profile limit `137.4°/s` 이하 |
| lower profile lag | p95 최대 다리 `5°` 이하 |
| neutral drift | runtime neutral gate 적용 후 별도 허용치 이내 |
| slip·자세 | 속도 증가 때문에 기존 slip/upright 기준이 악화되지 않음 |

이 기준을 통과하기 전에는 full curriculum 장기 학습보다 P1의 궤적과 P2의 작은
명령 curriculum을 짧게 반복하는 편이 효율적이다.

## 7. 이번 분석에서 제외한 것

- 실제 SCONE 하드웨어의 장시간 보행 속도와 발열은 측정하지 않았다.
- MuJoCo GUI에서 사람이 키를 누르는 시간, terminal key-repeat, 카메라 체감은
  headless 수치에 포함되지 않는다.
- 현재 checkpoint는 최신 dynamics와 호환된 신규 학습 결과가 아니다.
- 마찰계수와 TPU 변형은 실제 측정으로 보정되지 않았다.

따라서 이 문서의 핵심 결론은 “현재 알고리즘 병목이 수치로 확인됐다”까지다.
실물 안전 속도를 바로 올려도 된다는 근거는 아니다.
