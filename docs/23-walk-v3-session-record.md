# 2026-09-02/03 작업 기록: 원격 재생 오류 수정과 `walk_v3` 신설

이 문서는 요청부터 검증까지의 **과정**을 기록한다. `walk_v3`의 설계 근거와 수식은
[`22-walk-v3-residual-design.md`](22-walk-v3-residual-design.md)에, v2의 실패 진단은
[`21-walk-v2-ppo-training-analysis.md`](21-walk-v2-ppo-training-analysis.md)에 있다.
여기서는 무엇을 요청받고, 무엇을 측정했고, 무엇을 왜 채택/기각했는지를 남긴다.

작업 시점 commit은 `9d44054`(작업 시작)이며, 이 문서의 변경은 그 위에 쌓였다.

---

## 1. 요청 1 — 원격 학습 실시간 중계가 즉시 종료된다

### 1.1 증상

```text
? Select a remote run to watch walk-v2_full_20260902_173610 · ssh.hayward.kim
remote_watch.py: error: argument --reference-motion: invalid choice: 'none'
  (choose from tripod-gait, scone-gait, hardcoded, non_rl)
```

viewer가 열리지도 않고 argparse 단계에서 종료됐다.

### 1.2 원인

| 지점 | 상태 |
| --- | --- |
| `runs/.remote_jobs.json`의 해당 run | `task=walk-v2`, `reference_motion='none'` |
| `walk_v2.REFERENCE_CHOICES` | `tripod-gait, scone-gait, hardcoded, none` |
| `walk_learn.REFERENCE_MOTION_CHOICES` | `tripod-gait, scone-gait, hardcoded, non_rl` |
| `remote_watch.build_parser()` | **walk_learn의 목록만** 사용 |
| `inquiry.watch_remote_job()` | job의 `reference_motion`을 그대로 전달 |

즉 v2가 추가한 end-to-end 모드 `none`을 런처는 저장하고 전달하는데, viewer의
인자 정의는 구형 학습기의 목록에 고정돼 있었다. 런처가 자기 자신이 만든 명령을
거부하는 구조였다.

### 1.3 수정

1. `remote_watch`의 `--reference-motion`을 **모든 학습기 목록의 합집합**으로 확장
   (`REFERENCE_MOTION_CHOICES_ALL`).
2. `reference_motion_for_environment(value, task=...)` 추가. 열릴 환경이 지원하지
   않는 이름은 예외 대신 `hardcoded`로 강등하고 `[RL]` 알림을 출력한다.
3. 같은 결함을 가진 형제 경로도 같은 함수로 통일했다.
   - `inquiry.view_local_model()`: 구형 checkpoint + `none` 조합이 walk_learn의
     argparse에서 같은 방식으로 죽었다.
   - `joystick_control.run_rl_joystick()`: `unknown reference motion 'none'`이라는
     오해를 부르는 ValueError를 냈다(이름은 존재하지만 그 학습기에 없을 뿐이다).

### 1.4 회귀 방지

`tests/test_rl_inquiry.py`에 **런처가 생성한 argv를 그대로 `remote_watch`의 파서에
넣는** 테스트를 추가했다. 두 쪽이 다시 어긋나면 이 테스트가 먼저 깨진다.
`tests/test_remote_watch.py`에는 강등/정규화/거부 규칙 5개를 추가했다.

---

## 2. 요청 2 — v2가 residual RL을 못 이기니 v1 기반 v3를 만들 것

요구사항은 세 가지였다: **속도 상한 없음**, **몸체 높이는 움직여도 됨**,
**자세(orientation)는 움직이면 안 됨**.

### 2.1 먼저 확인한 두 가지 해석

구현이 크게 갈리는 지점이라 진행 전에 확인했다.

| 질문 | 선택지 | 사용자 결정 |
| --- | --- | --- |
| 자세 고정 방법 | ① 모델의 freejoint를 병진 3축으로 바꿔 물리적으로 회전을 막는다(harness) ② 보상·종료로만 강제 ③ 플래그로 둘 다 | **② 보상·종료** |
| 속도 상한 없음의 의미 | ① 명령 유지 + 단측 보상 ② 명령 제거, 순수 최대속도 ③ 대칭 추종 유지 + 상한만 상향 | **① 명령 유지 + 단측 보상** |

①의 결과로 v3는 실물 전개 가능한 free-floating base를 유지하고, joystick 명령
조건부 정책이라는 프로젝트 목표([`01`](01-project-overview.md))를 깨지 않는다.

높이 처리는 별도로 묻지 않고 결정했다: 높이 항과 높이 종료를 모두 제거하고,
붕괴 방지는 기존 `forbidden_collision`(발 이외 접촉) 종료에 맡긴다.

### 2.2 측정부터 시작했다

v1의 하드코딩 기준 모션을 Standard stance, flat, residual 0으로 직접 측정했다.
문서 [`17`](17-ppo-diagnosis-and-fix-plan.md) §2는 이 기준이 0.084 m/s에서 포화한다고
기록했지만, 실제로 재현한 값은 달랐다.

| 조건 | 실제 전진 속도 | 최대 tilt | 평균 접촉 다리 |
| --- | ---: | ---: | ---: |
| stride 20°, lift 20°, 1.4 Hz, `vx=0.50` 명령 | **0.101 m/s** | 1.6° | 3.47 |

즉 v1의 기준 모션 자체는 제대로 된 교대 삼각보였고, 문제는 "명령이 능력의 5배"
라는 불일치였다. 이어서 stride×lift×cadence 격자를 sweep했다(각 조건 6초, 앞
1.5초 제외).

- **stride가 유일한 속도 레버다.** 20°→50°에서 0.10→0.21 m/s.
- **cadence는 레버가 아니다.** 1.4 Hz 이상에서 오히려 느려진다.
- **stride가 길면 clearance도 커져야 한다.** stride 20°는 lift 20°, stride 85°는
  lift 40°가 최적이었다.
- zero residual 최고 속도는 stride 85°, lift 40°, 1.4 Hz에서 **0.324 m/s**,
  이때 최대 tilt는 4.8°였다.

마지막 값이 자세 제약의 근거가 됐다. 전 속도 구간에서 tilt가 1.6~4.8°이므로
종료 임계값 15°는 낙상 감지가 아니라 **여유 3배의 실제 제약**이다.

### 2.3 단일 gain을 기각하고 측정 역산표를 채택

stride→속도 관계를 고정 stride로 다시 측정하니 선형이 아니었다.

| stride | 10° | 20° | 30° | 40° | 50° | 70° | 80° | 90° |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| m/s | 0.020 | 0.101 | 0.154 | 0.192 | 0.203 | 0.242 | 0.301 | 0.336 |

저속에서 급하고 중간에서 평평하다. 단일 gain(`stride = 300·v`)으로는 0.10 m/s에서
**152% 초과**, 0.30 m/s에서 112%가 됐다. 그래서 이 측정을 그대로
`WalkConfig.stride_calibration`에 넣고 역보간하는 방식으로 바꿨다. 회전축도 같은
방법으로 `yaw_stride_calibration`을 만들었다(초기 계수 55는 0.30 rad/s에서 133%를
줬다).

보정 후 zero residual 추종률:

| 명령 | 0.05 | 0.10 | 0.20 | 0.30 | 0.40 | 0.60 | yaw 0.30 | yaw 0.60 | yaw 0.90 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 추종률 | 100% | 101% | 103% | 100% | 84% | 56% | 104% | 100% | 105% |

0.336 m/s 위에서는 관절 여유 한계로 포화하지만 **명령을 거부하지 않는다.** 남은
요구량은 residual이 만들고, 단측 보상이므로 못 만들어도 벌점은 없다.

### 2.4 측정 중 드러난 두 결함

| 결함 | 증상 | 수정 |
| --- | --- | --- |
| heading 비용이 무한 적분 | `yaw=0.9` 명령의 6초 return이 **-31.7**. 목표 heading이 yaw 명령을 적분하는데 실제 rate가 조금 부족하면 오차가 계속 쌓였다 | heading을 **직진 유지 전용** 목표로 바꿨다. yaw 명령이 `0.05 rad/s`를 넘으면 목표를 현재 heading으로 재고정하고 비용도 끈다 |
| `reset()`의 첫 관측이 미정의 | `settle_seconds=0`이면 `mj_resetData` 직후 상태를 그대로 관측해 접촉 flag와 속도가 전부 0 | settle 루프 뒤 `mj_forward` 1회 추가 |

두 번째는 테스트가 먼저 잡았다. 같은 테스트가 "무접촉"으로 실패했을 때 확인해
보니 로봇은 정착 자세보다 **4.5 mm 위에서 생성**되므로 settle 0에서 접촉이 없는
것이 물리적으로 옳았다. 코드는 고치고 테스트의 기대값을 정정했다.

### 2.5 v2에서 가져온 것과 버린 것

| 항목 | 판단 |
| --- | --- |
| bounded action (gSDE + squashed policy) | **채택.** v2가 측정한 84~95% action 포화는 목적함수 문제가 아니라 분포 문제였다 |
| 학습/재생 randomization 분리 | **채택.** 재생·중계·조이스틱은 항상 명목 로봇 |
| zero-residual 기준 승격(`best_model`) | **채택.** "residual RL을 이기는가"를 학습 중에 자동 측정 |
| 정규 좌표계 회전(canonical frame) | 기각. v1 규약 유지 |
| 좌우 mirror 증강 | 기각. v2에 있었지만 실패를 되돌리지 못했고 관측 레이아웃별 sign 위험만 늘린다 |
| 접촉력·air-time·load-share 보상군 | 기각. 접촉 **flag 6개**만 관측에 넣고 보상은 유휴 다리 벌점 하나로 축소 |
| backlash, action delay | 기각. v1과 같은 명목 구동 경로 유지 |

### 2.6 승격 규칙을 한 번 고쳤다

첫 설계는 "기준선보다 점수가 높고 **방향 실패 0**"이었다. 그런데 하드코딩 기준
모션은 lateral scaffold가 없어 `vy` 명령에서 스스로 방향 실패 1개를 낸다. 즉
구조적으로 승격 불가였다. 실제 smoke 학습에서 이 조건이 드러나 규칙을
"**기준선보다 점수가 높고, 기준선이 성공한 축에서 방향을 틀리지 않을 때**"로
바꿨다.

### 2.7 배선

`walk-v3`는 런처의 1급 작업이다. 관측 폭이 학습기를 식별한다.

```text
68/70 -> walk (walk_learn)     76 -> walk-v3     82 -> walk-v2
```

판정 지점은 `policy_compat.task_for_observation_shape()` 하나뿐이고, 로컬 재생,
원격 실시간 중계(`--task auto`), 조이스틱 조종이 모두 이 함수를 거친다.
환경 테스트 flow도 작업 선택형으로 바꿨다(세 학습기의 `check` 인자가 동일하다).

### 2.8 검증

| 검증 | 결과 |
| --- | --- |
| 전체 회귀 테스트 | **206/206 통과** (약 20초). v3 테스트 23개, remote_watch/inquiry 신규 8개 포함 |
| 2,048 step 실제 학습 | 통과. checkpoint 명명, resume pointer, zero-residual 기준선, 승격 판정까지 동작 |
| 승격 동작 | 1,024 step에서 승격, 2,048 step에서 미승격(점수 하락) — 판정이 실제로 작동 |
| headless 재생 | `best_model.zip`이 `0.30` 명령에서 평균 progress `+0.285 m/s` |
| 라우팅 | 실제 76차원 checkpoint가 로컬 재생·원격 auto·조이스틱에서 모두 `walk-v3`로 라우팅 |
| 미사용 import 검사 | 변경 모듈 5개 모두 clean |

---

## 3. 재현 명령

```bash
# 기준 모션과 보상을 한 rollout으로 확인
PYTHONPATH=. python -m src.rl.walk_v3 --stance standard check --command 0.30 0 0 --steps 300

# 짧은 학습(파이프라인 점검용)
PYTHONPATH=. python -m src.rl.walk_v3 --stance standard train \
    --curriculum easy --timesteps 2048 --num-envs 2 --n-steps 256 --batch-size 512 \
    --checkpoint-every 1024 --eval-every 1024 --eval-episodes 1 --eval-seconds 2.0 \
    --device cpu --output runs/walk_v3_smoke

# 회귀 테스트
python -m unittest discover -s tests
python -m unittest tests.test_walk_v3
```

## 4. 이번 작업에서 하지 않은 것

1. **장기 학습을 돌리지 않았다.** v3가 zero residual을 실제로 이기는지는 아직
   미검증이다. 승격 로직이 그것을 자동 판정하도록 만들어 둔 상태다.
2. **lateral scaffold를 만들지 않았다.** 하드코딩 기준 모션은 `vy` 명령에
   0.000 m/s로 답한다. 좌우 이동은 전부 residual의 몫이다.
3. **v1/v2를 수정하지 않았다.** 기존 checkpoint 재생 경로는 그대로다.
4. **관절 하드스톱을 측정하지 않았다.** ±70° 목표 제한은 보수적 가정이며 v3의
   실질 최고 속도를 정하는 값이다.
5. **좌우 대칭 강제 장치가 없다.** 필요하면 mirror 증강을 v3 관측 레이아웃에
   맞춰 새로 유도해야 한다.

## 5. 다음 단계

1. `easy` curriculum으로 원격 장기 학습을 시작하고 `eval/beats_zero_residual`을
   본다. 이 값이 계속 0이면 오래 돌리는 것은 의미가 없다.
2. 승격된 `best_model.zip`이 나오면 `--command`를 0.05~0.60까지 훑어 추종 곡선을
   기준선과 겹쳐 본다.
3. 관절 하드스톱 실측 후 `joint_target_bound_degrees`와
   `stride_degrees_max`를 갱신하고 보정표를 다시 측정한다.
4. lateral scaffold가 필요하면 발 arc의 접선 기하를 측정한 뒤 기준 모션에 lateral
   항을 추가한다([`26`](26-stair-and-hybrid-locomotion-theory.md) §4.6 참고).
