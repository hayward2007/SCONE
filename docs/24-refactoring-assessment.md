# 전체 프로젝트 리팩토링 진단과 방향 (2026-09-03)

이 문서는 `src/`, `benchmark/`, `tests/` 전체를 기계적으로 계측한 뒤 리팩토링이
필요한지, 무엇을 어떤 순서로 해야 하는지, 그리고 **무엇을 건드리면 안 되는지**를
정리한다. 계측은 AST 분석과 import 그래프로 수행했고 수치는 모두 재현 가능하다.

---

## 1. 결론

**필요하다. 단, 전면 재구조화가 아니라 네 곳의 국소 수술이고, ICRA 데이터 동결
전에 손대도 되는 것과 동결 후에만 손대야 하는 것이 명확히 갈린다.**

| # | 항목 | 근거 | 규모 | ICRA 경로 | 우선도 |
| --- | --- | --- | --- | --- | --- |
| R1 | RL 학습기 3종의 **학습 인프라 중복 추출** | 같은 이름의 함수·콜백 17개가 2~3개 모듈에 복제, 이미 버그 1건 전파 | 약 700행 → 1개 모듈 | 무관 | **높음** |
| R2 | `MuJoCoController`/환경의 **사유 속성 공개 API화** | 모듈 경계를 넘는 `_` 접근 25건 | 작음 | 접촉 | 중간 |
| R3 | `inquiry.py` **분할** | 2,399행에 등록부·SSH·프롬프트·flow 혼재 | 4개 모듈로 분리 | 무관 | 중간 |
| R4 | `src/cli` ↔ `src/simulation` ↔ `src/rl` **순환 의존 제거** | 패키지 간 순환 3개, 함수 내부 지연 import 18곳 | 중간 | 접촉 | 낮음(동결 후) |
| R5 | **죽은 호환 shim 제거**와 내부 import 정리 | importer 0인 shim 4개, core가 shim을 거쳐 자기 구현을 import | 매우 작음 | 접촉 | 낮음 |
| R6 | `tmp/` 실험 스크립트 정리 | 일회성 4개가 저장소 루트에 잔존 | 매우 작음 | 무관 | 낮음 |

지금 당장 착수할 것은 **R1**이다. 이미 실제 버그를 만든 유일한 항목이고, ICRA
실험 경로(`benchmark/`, `src/simulation`, `src/locomotion`)를 전혀 건드리지 않는다.

---

## 2. 계측 결과

### 2.1 규모

| 영역 | 행 수 |
| --- | ---: |
| `src/` + `benchmark/` | 22,223 |
| `tests/` | 4,512 |
| 테스트 개수 | 206 (약 20초) |

상위 파일: `rl/inquiry.py` 2,399, `rl/walk_v3.py` 2,079, `rl/walk_v2.py` 1,957,
`rl/walk_learn.py` 1,501, `cli.py` 990, `benchmark/common.py` 700,
`simulation/stair_benchmark.py` 619, `rl/remote_watch.py` 608,
`simulation/core/controller.py` 595.

### 2.2 중복 정의 (모듈 2개 이상에 같은 이름)

학습기 3종 사이의 복제가 압도적이다.

| 이름 | 위치 |
| --- | --- |
| `PruningCheckpointCallback` | walk_learn, walk_v2(중첩), walk_v3(중첩) |
| `RewardTermsCallback` | walk_learn, walk_v2(중첩), walk_v3(중첩) |
| `GracefulStopCallback` | walk_learn, walk_v2(중첩), walk_v3(중첩) |
| `FixedCommandEvalCallback` | walk_v2(중첩), walk_v3(중첩) |
| `_write_resume_pointer` | walk_learn, walk_v2, walk_v3 |
| `_ppo_training_kwargs` | walk_v2, walk_v3 |
| `_validate_training_batch_size` | walk_v2, walk_v3 |
| `_require_bounded_resume_policy` | walk_v2, walk_v3 |
| `_validate_evaluation_arguments` | walk_v2, walk_v3 |
| `_fixed_evaluation_score` | walk_v2, walk_v3 |
| `_write_json_atomic` | walk_v2, walk_v3 |
| `_env_factory`, `_make_callbacks`, `_bar`, `training_walk_config` | walk_v2, walk_v3 |
| `run_check` / `run_train` / `run_enjoy` / `build_parser` | 3종 전부 |

`_make_callbacks`만 walk_v2에서 282행, walk_v3에서 269행이다. 학습기 하나를 더
만들면 또 700행이 복제된다.

### 2.3 패키지 간 순환 의존

```text
src/simulation -> src/(cli, main, cli_i18n, cli_ui)   24회
src/rl         -> src/simulation                      16회
src/rl         -> src/(cli, main)                      9회
src/simulation -> src/rl                               4회      <-- 순환
src/(root)     -> src/simulation                       4회      <-- 순환
src/(root)     -> src/rl                               2회      <-- 순환
```

즉 **시뮬레이션 런타임과 학습기가 CLI/UI 계층을 import한다.** 그 결과 함수 본문
안의 지연 import가 18곳 생겼다(`cli_bridge`가 `rl.joystick_control`을,
`simulator_cli`가 `rl.inquiry`를, `scone_rolling_gait`가 `cli`를 함수 안에서
import). 순환을 끊지 않으면 이 지연 import를 없앨 수 없다.

### 2.4 모듈 경계를 넘는 사유 속성 접근

| 횟수 | 접근 | 대표 위치 |
| ---: | --- | --- |
| 10 | `env._viewer` | `rl/joystick_control.py`, `rl/remote_watch.py` |
| 5 | `controller._joint_velocity` | `rl/walk_learn.py`, `rl/walk_v3.py` |
| 3 | `controller._joint_position` | 학습기 3종 |
| 3 | `controller._actuator_ids` | 학습기 3종 |
| 2 | `env._phase` | `rl/joystick_control.py` |
| 1 | `planner._activity` | `simulation/core/scone_rolling_gait.py` |
| 1 | `stair_benchmark._Trial` | `benchmark/capture.py` (monkeypatch 대상) |

`controller._joint_position()`은 학습기 3종이 매 step 18번씩 호출하는 **사실상의
공개 API**다. 이름만 사유다.

### 2.5 죽은 호환 shim

| 파일 | 내부 importer |
| --- | --- |
| `simulation/controller.py` | 없음 |
| `simulation/cli_bridge.py` | 없음 |
| `simulation/simulator_cli.py` | 없음 |
| `locomotion/non_rl_walk.py` | 없음 |
| `simulation/model.py` | `core/cli_bridge`, `core/stair_climber`, `core/stair_demo` |
| `simulation/pid.py` | `core/controller` |

뒤의 두 개는 더 나쁘다. **`core/` 안의 구현이 자기 자신의 shim을 거쳐 import한다.**

---

## 3. 중복은 이미 버그를 만들었다 (측정 증거)

리팩토링을 "정리 취향" 문제로 볼 수 없는 근거가 두 건 있다.

### 3.1 heading 비용의 무한 적분이 v2에 그대로 남아 있다

`walk_learn`은 heading을 **유계 양의 보상** `exp(-e²/σ²)`(σ=0.60)으로 썼다.
`walk_v2`는 이를 **비용** `-0.25·min(25,(e/0.35)²)`으로 바꾸면서, 목표 heading을
yaw 명령으로 적분하는 v1의 코드는 그대로 복사했다. 실제 rate가 명령보다 조금이라도
부족하면 오차가 계속 쌓인다. `walk_v2` 환경에서 residual 0으로 직접 측정했다.

| 명령 | 달성 yaw rate | 2초 후 heading 오차 / 누적 heading 항 | 10초 후 |
| --- | ---: | ---: | ---: |
| `yaw=0.65` | 0.391 rad/s | -0.685 rad / **-0.83** | -2.877 rad / **-39.82** |
| `yaw=0.30` | 0.039 rad/s | -0.493 rad / -0.33 | -2.444 rad / **-32.71** |
| `vx=0.25` | - | -0.017 rad / -0.00 | -0.002 rad / **-0.01** |

`walk_v2`의 6.6M-step episode return이 약 42.6이었다([`21`](21-walk-v2-ppo-training-analysis.md) §11.0).
즉 **yaw 명령이 걸린 구간에서는 정책이 제거할 수 없는 -30~-40 규모의 항이
episode return 전체와 맞먹는다.** medium/full curriculum은 yaw를 샘플링하므로
학습 신호의 상당 부분이 이 항에 지배됐다. v2가 회전을 배우지 못한 데에는 이
구조적 원인이 있을 가능성이 크다.

`walk_v3`에서는 heading을 **직진 유지 전용**으로 바꾸고 yaw 명령 중에는 목표를
재고정하도록 고쳤다([`22`](22-walk-v3-residual-design.md) §3). v1은 유계 보상이라
문제가 없다. 결과적으로 **같은 코드의 세 사본이 서로 다른 정합성 상태에 있다.**

> 조치 권고: v2를 다시 살릴 경우에만 같은 gate를 적용한다. 지금 v2를 수정하면
> 환경 의미가 바뀌어 기존 run과 비교 불가가 되므로, 활성 라인이 v3인 동안에는
> 문서화만 한다.

### 3.2 `reset()`의 미정의 첫 관측

`mj_resetData` 직후에는 접촉과 파생 상태가 없다. settle 길이가 0이면 첫 관측의
접촉 flag와 속도가 전부 0이 된다. v3에서 테스트가 이를 잡아 `mj_forward` 한 줄을
추가했다. **같은 구조가 v1·v2에도 있고**, 기본 settle이 0보다 크다는 이유로만
드러나지 않는다.

---

## 4. 권고안

### R1. 학습 인프라 추출 (우선 착수)

**문제.** 학습기 3종이 checkpoint 저장·정리, resume pointer, graceful stop, 보상
로깅, 고정 명령 평가, PPO 인자 검증, JSON 원자적 쓰기, CLI 레이아웃을 각자
복제한다. 환경/보상은 **의도적으로** 분리돼야 하지만(§5 참고) 이 인프라에는
checkpoint 의미가 없다.

**방향.**

```text
src/rl/training/
  checkpoints.py   PruningCheckpointCallback, _write_resume_pointer
  logging.py       RewardTermsCallback (기록 키 목록만 주입)
  control.py       GracefulStopCallback, 시그널 핸들러 설치
  ppo.py           _validate_training_batch_size, _require_bounded_resume_policy,
                   _ppo_training_kwargs(기본값은 호출자가 주입)
  evaluation.py    FixedCommandEvalCallback (환경 생성자와 점수 함수를 주입)
  cli.py           공통 인자군(--terrain/--terrain-seed/--reference-motion/
                   --standing-pose-degrees/train 하위 인자)
```

각 학습기는 자기 `RewardConfig`/`WalkConfig`/`SconeWalkEnv*`와 자기 점수 함수만
남긴다. 평가 콜백은 "환경을 만드는 함수"와 "레코드→점수 함수"를 인자로 받는다.
`_fixed_evaluation_score`는 v2(추종 오차 중심)와 v3(전달률 중심)가 **의미가 다르므로
합치지 않는다.**

**위험.** 낮다. 세 학습기의 CLI 인자 레이아웃은 이미 동일해야 하고
(`build_training_arguments` 하나가 세 모듈을 구동한다) 테스트가 그것을 검사한다.

**검증.** ① 전체 206개 테스트 통과 ② 세 학습기 각각 `check` 1회, 2,048-step
`train` 1회 ③ 기존 checkpoint 3종(70/76/82차원)의 `enjoy` 재생 ④ 원격 런처의
`build_remote_launch_command` 문자열 회귀.

**비용.** 약 700행 삭제, 약 350행 신설. 반나절.

### R2. 컨트롤러/환경의 공개 API 확정

**방향.** `MuJoCoController`에 `joint_position(motor_id)`,
`joint_velocity(motor_id)`, `actuator_id(motor_id)`를 공개 메서드로 추가하고
사유 이름은 얇은 위임으로 남긴다. 환경에는 `viewer_is_open()`,
`gait_phase` property를 추가해 `env._viewer`/`env._phase` 접근을 없앤다.

**위험.** 낮지만 `benchmark/common.py`와 `capture.py`가 같은 경로를 쓰므로 ICRA
데이터 동결 전이라면 같은 revision에서 재실행이 필요하다. → **동결 후 또는
동결 전 일괄 실행 직전**에 수행.

### R3. `inquiry.py` 분할

2,399행에 ① 작업 등록부와 dataclass ② SSH 셸 명령 빌더 ③ 용량 조회 ④ InquirerPy
프롬프트 ⑤ 대화형 flow ⑥ 메뉴가 섞여 있다. 제안:

```text
src/rl/launcher/
  tasks.py     TrainingTask, TRAINING_TASKS, 참조 모션 목록
  config.py    TrainingConfig, RemoteJob, RemoteSettings, 검증
  remote.py    SSH 명령 빌더와 실행(_run_ssh, build_remote_*)
  prompts.py   prompt_* 함수
  flows.py     _*_flow, main 메뉴
```

**위험.** 중간. `tests/test_rl_inquiry.py`가 함수 단위로 import하므로 재export
shim을 두면 테스트 변경 없이 옮길 수 있다. 단 **새 shim을 만드는 리팩토링은
R5의 교훈(죽은 shim이 남는다)을 반복하므로, import를 함께 갱신하고 shim을 두지
않는다.**

### R4. 순환 의존 제거 (동결 후)

**방향.** 조종 입력을 계층이 아니라 **프로토콜**로 만든다.

```text
현재: simulation/core/stair_climber.py -> src/cli.run_velocity_joystick_cli
목표: stair_climber는 "명령 공급자(Callable[[], VelocityCommand])"만 받는다.
      CLI/조이스틱은 상위(엔트리포인트)에서 주입한다.
```

`src/main.py`의 `SCONE`과 `src/cli_i18n`은 최하층으로 내려도 되지만(순환에
기여하지 않음), `src/cli.py`의 조이스틱 루프는 반드시 상위로 올려야 한다. 이
변경 후 함수 내부 지연 import 18곳 중 12곳 이상이 모듈 상단으로 올라간다.

**위험.** 높음. `cli_bridge`, `stair_climber`, `scone_rolling_gait`,
`stair_demo`가 모두 대상이고 이들이 ICRA 계단/전환 실험의 실행 경로다.
**논문 데이터 동결 이후에만 수행한다.**

### R5. 죽은 shim 제거

`simulation/controller.py`, `simulation/cli_bridge.py`,
`simulation/simulator_cli.py`, `locomotion/non_rl_walk.py`는 importer가 없다.
삭제하고, `core/*`가 `..model`/`..pid` shim을 거쳐 import하는 3+1곳을
`.model`/`.pid` 직접 import로 바꾼다. 외부 사용자가 있을 수 있으므로 삭제 대신
`README`에 이동 안내를 남기는 것도 가능하지만, 이 저장소는 애플리케이션이므로
삭제를 권한다.

### R6. `tmp/` 정리

`tmp/{stair,gait_direction,scone_combined,gait_rework}_experiment.py`는 일회성
실험이다. 결과가 문서에 반영됐으므로 `archive/experiments/`로 옮기거나 삭제한다.
`packages/dynamixel-mujoco`는 독립 패키지 구조가 의도된 것이므로 그대로 둔다.

---

## 5. 하지 말아야 할 리팩토링

| 대상 | 이유 |
| --- | --- |
| 학습기 3종의 **환경을 하나로 통합** | 관측 폭·보상 의미·명령 규약이 다르고, 저장된 checkpoint가 그 의미에 묶여 있다. "새 설계는 새 모듈"은 이 저장소의 **의도된 정책**이다([[project-scone-rl-trainer-lineage]] 및 각 모듈 docstring). 통합하면 과거 checkpoint 재생이 깨진다 |
| `RewardConfig` 필드 통일/개명 | 세 환경의 항 의미가 다르다. 이름을 맞추면 서로 다른 것이 같아 보인다 |
| `locomotion/climb.py`(실물)와 `simulation/core/stair_climber.py`(시뮬) 병합 | 전자는 blocking 하드웨어 상태기, 후자는 50 Hz 공통 위상 제어기다. 병합하면 실물 안전 경로에 시뮬 전용 로직이 들어간다 |
| model.xml의 좌표계·L/R 매핑 정리 | 이미 검증·문서화된 규약이고 미해결 항목(전방 축)은 측정으로만 닫힌다([`18`](18-actuator-model-and-frame-convention.md)) |
| `benchmark/`를 `src/`로 이동 | 실험 코드와 제품 코드의 분리는 논문 재현성에 유리하다 |

---

## 6. 순서와 게이트

```text
지금 ──> R1 (학습 인프라 추출)            ICRA 경로 무관, 버그 전파 차단
     └─> R3 (inquiry 분할)               ICRA 경로 무관
     └─> R6 (tmp 정리)                   무해

ICRA 평가 실행 및 결과 아카이브 (동결)
     │
     └─> R2 (컨트롤러 공개 API)
     └─> R5 (죽은 shim 제거)
     └─> R4 (순환 의존 제거)              가장 큰 변경, 마지막
```

각 단계의 완료 판정은 동일하다.

1. `python -m unittest discover -s tests` 206개 유지(감소 금지).
2. 세 학습기 `check` 실행 결과의 보상 항 합이 리팩토링 전과 동일(결정론적 seed).
3. 70/76/82차원 checkpoint 각각 `enjoy` 1 episode 재생 성공.
4. `benchmark flat --controller all`의 nominal 수치가 동일(R2/R4 전후 필수).
5. 공개 동작 변경 없음. 동작을 바꾸려면 별도 commit으로 분리한다.

## 7. 하지 않은 계측

- 런타임 프로파일링(어느 함수가 학습 FPS를 지배하는지)은 측정하지 않았다.
  `num_envs=2`에서 489 FPS라는 값만 있다.
- 순환 복잡도, 타입 커버리지(`mypy`), 정적 검사(`ruff`)는 환경에 도구가 없어
  실행하지 못했다. 도입한다면 R1과 함께 CI에 넣는 것이 가장 싸다.
- `tests/`의 실제 라인 커버리지는 측정하지 않았다. 위 §2.5의 "테스트 없음"
  목록은 import 기반 추정이며, 패키지 재export를 통한 간접 테스트를 놓친다.
