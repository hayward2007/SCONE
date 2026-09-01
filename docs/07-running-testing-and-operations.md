# 실행·학습·운영·검증

## 1. 환경 설치

권장 Python 가상환경에서 실행한다.

```bash
python -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
```

CLI, Dynamixel SDK, MuJoCo, RL 의존성을 하나의 파일에서 설치한다.

```bash
python -m pip install -r requirements.txt
```

아카이브 SCONEv1의 OpenCV/imutils vision 코드는 현재 runtime 의존성이 아니다.

## 2. 통합 launcher

macOS에서 MuJoCo viewer를 열 때는 viewer가 main thread를 사용할 수 있도록 `mjpython`을 권장한다.

```bash
mjpython SCONE.py
```

메뉴에서 다음을 선택할 수 있다.

- 시뮬레이션 (자동 데모): hardcoded/improved/compare와 stairs-1..3 선택, 조종 입력 없음
- 시뮬레이션 조종: profile, control 방식, terrain, 필요 시 RL checkpoint
- 하드웨어 조종: 탐색된 serial port와 legacy discrete 제어
- 하드웨어 다시 탐색: torque/position을 바꾸지 않는 ping
- 강화학습 관리: 환경 검사, 새 학습, 원격 상태/pause/resume/download/watch/reset

velocity joystick:

| 키 | 동작 |
|---|---|
| `W/S` | 전진/후진 `vx` |
| `A/D` | 좌/우 `vy` |
| 좌/우 화살표 | yaw |
| `Space` | 즉시 neutral |
| `R` | Legacy/RL에서 Walk→Drive→Climb→Walk 전환 |
| `H` | Legacy에서 home 자세 복귀 |
| `Q` | 현재 조종 종료 |

키를 놓은 뒤 terminal key-repeat가 끊기면 약 0.35초 후 해당 축이 자동으로 0이 된다. 종료 시에도 explicit neutral을 전송한다.

## 3. Python API와 실제 하드웨어

최소 예제:

```bash
python example.py
```

코드에서 사용할 때:

```python
import SCONE
from src.hardware import Controller, discover_hardware

probe = discover_hardware()
if probe.available:
    with SCONE.SCONE(Controller(probe.device_name), profile="standard") as robot:
        robot.forward()
        robot.left()
```

안전 확인:

1. 로봇을 지지대에 올리고 비상 전원 차단 수단을 준비한다.
2. ID 1, 7, 13의 모델 응답과 전체 bus 전원을 확인한다.
3. `SCONE_DEVICE` 또는 탐색 결과가 실제 포트인지 확인한다.
4. 처음에는 낮은 speed/제한된 motion으로 각 ID 방향과 zero를 확인한다.
5. `with` 또는 `try/finally`를 사용해 `close()`와 torque-off를 보장한다.
6. RL checkpoint를 실제 controller에 직접 연결하지 않는다. 현재 RL 환경은 MuJoCo body state에 의존한다.

## 4. 시뮬레이션 직접 실행

```bash
mjpython -m src.simulation
```

조종 없이 hardcoded와 improved 계단 동작을 순서대로 보려면:

```bash
mjpython -m src.simulation --demo compare --terrain stairs-2
```

기본 `stairs-2`에서는 두 방식 모두 통과한다. fixed rolling이 실패하고 adaptive
assist가 필요한 차이를 보려면 `--terrain stairs-3`를 사용한다. 자동 데모는
terminal joystick을 열지 않는다.

argparse option은 다음 명령으로 확인한다.

```bash
python -m src.simulation --help
```

계단 전용 adaptive controller는 SCONE을 자동으로 side-on Drive 자세로 만든다.
`A`는 preset 계단의 `+Y` 상승, `D`는 반대 방향이며 W/S와 yaw는 이 경로에서
비활성화된다.

```bash
mjpython -m src.simulation \
  --control scone-stair \
  --profile standard \
  --terrain stairs-3
```

`scone-stair`는 MuJoCo 전용이다. 실물 적용 전에 후킹 기하, 모터 여유,
마찰과 지지 조건을 [계단 알고리즘 문서](11-scone-stair-climbing.md)대로
별도 계측한다.

문서의 H0–H4 전체 비교와 H3 튜닝 sweep은 headless로 다시 실행할 수 있다.
출력은 한 실험당 JSON 한 줄이며 상단 성공, 시간, 절대 기계일, upright,
contact force와 adaptive assist 횟수를 포함한다.

```bash
python -m src.simulation.stair_benchmark --all --tuning
```

코드에서 모델만 만들 수 있다.

```python
from src.simulation import load_model

model = load_model(
    floating_base=True,
    terrain="mixed",
    terrain_seed=42,
)
```

`flat`, `uneven`, `stairs-1..3`, `slope-1..3`, `mixed`가 지원된다. random rough patch를 비교할 때는 seed를 고정한다.

지형 geom은 viewer 기본 표시 그룹 0에 있고 카메라는 몸체를 추적한다. 긴 `mixed` 코스 때문에 초기 카메라가 과도하게 멀어지지 않도록 거리를 2.2–3.0 범위로 제한한다.

## 5. RL 환경 검사

기본 reference와 보상 finite 여부:

```bash
python -m src.rl.walk_learn --reference-motion tripod-gait \
  check --steps 500 --curriculum easy
```

무작위 residual과 full 명령 범위:

```bash
python -m src.rl.walk_learn check \
  --steps 500 \
  --curriculum full \
  --random-actions
```

지형 option은 subcommand 앞에 둔다.

```bash
python -m src.rl.walk_learn \
  --terrain uneven \
  --terrain-seed 7 \
  --reference-motion scone-gait \
  check \
  --steps 500
```

출력되는 mean weighted reward term은 항의 부호와 scale을 보는 smoke 지표다. 학습 성능 결론으로 사용하지 않는다.

모델 기반 gait 기준을 선택하면 TensorBoard `state/` 아래에 다음 튜닝 지표가 추가된다.

- `reference_cycle_frequency`: 현재 기준 모션 cadence. `tripod-gait`는 `0.7 Hz`, `scone-gait`는 `0.65 Hz`
- `reference_stride_clip_fraction`: 작업공간 한계에 걸린 다리 비율
- `reference_ik_backoff_scale`: IK 재시도로 실제 적용된 발 오프셋 배율. 항상 1보다 작다면 stance·보폭을 다시 조정한다.

## 6. PPO 학습

새 학습:

```bash
python -m src.rl.walk_learn --reference-motion tripod-gait train \
  --timesteps 1000000 \
  --curriculum easy \
  --num-envs 4 \
  --checkpoint-every 100000 \
  --keep-checkpoints 10 \
  --output runs/walk_easy_experiment \
  --tensorboard-log runs/tensorboard
```

학습 재개:

```bash
python -m src.rl.walk_learn --reference-motion tripod-gait train \
  --timesteps 1000000 \
  --curriculum easy \
  --num-envs 4 \
  --resume runs/walk_easy_experiment/checkpoints/scone_walk_100000_steps.zip \
  --output runs/walk_easy_experiment
```

재개 전 확인:

- 관측 70차원, action 18차원, 순서가 동일한가
- standing pose와 reference gait 의미가 동일한가
- reward 변경 후 fine-tune인지 완전한 비교 실험인지 구분했는가
- 68차원 legacy policy가 아닌가
- checkpoint ZIP이 완전한가

SIGINT/SIGTERM을 보내면 현재 step을 마치고 final model/resume pointer를 남기는 경로를 사용한다.

`--reference-motion tripod-gait`는 고전 교대 삼각보+IK를 사용한다. `scone-gait`는 여기에 부채꼴 rolling/creep sweep을 더한 실험 기준이며, `hardcoded`는 기존 사인파 tripod를 보존한다. `non_rl`은 `tripod-gait` 호환 별칭이다. 기준 모션이 다른 checkpoint를 재개하면 action 의미가 달라지므로, 원래 설정과 반드시 일치시킨다.

두 gait의 정확한 기본값, reference+residual 합성, checkpoint 호환 표와
튜닝 순서는 [`10-tripod-gait-and-scone-gait.md`](10-tripod-gait-and-scone-gait.md)를
참고한다.

`tripod-gait` 기준 모션은 2026-08-31에 stride 작업공간과 IK backoff가 추가되었고, support point는 부채꼴 말단의 최저 0.1 mm 패치 중심으로 교정됐다. RL reference는 checkpoint 의미를 보존해 0.7 Hz, 60/50 mm를 유지한다. 2026-09-01부터 비-RL MuJoCo 조종만 0.8 Hz, 80/60 mm, speed 160, XM acceleration 50, middle hold 2배를 opt-in한다. 기존 PPO는 무제한 simulation profile에서 학습됐으므로 RL reset도 그 동역학을 보존한다. 새 profile/dynamics 학습은 기존 checkpoint를 resume하지 말고 환경 버전을 기록해 0 step부터 시작한다.

`--num-envs 1`은 단일 프로세스이고 2 이상은 `SubprocVecEnv`로 각 MuJoCo 환경을 별도 프로세스에서 실행한다. 환경 수를 늘리면 PPO rollout 크기도 `n_steps × num_envs`로 커지므로 CPU 사용률뿐 아니라 업데이트 지연과 메모리를 함께 확인한다.

## 7. 정책 재생과 조이스틱

고정 명령 재생:

```bash
mjpython -m src.rl.walk_learn --reference-motion tripod-gait enjoy \
  runs/walk_full_standard/checkpoints/scone_walk_6100000_steps.zip \
  --command 0.25 0.0 0.0 \
  --curriculum full \
  --episodes 3
```

실시간 조이스틱은 통합 launcher의 RL control 또는 `src.rl.joystick_control` API를 사용한다. 기본으로 neutral residual gate가 켜진다. `R`을 누르면 policy target을 일시 중단하고 같은 controller에서 Drive/Climb으로 전환하며, Walk 복귀 시 heading·기준 높이·residual 상태를 재정렬한다. `--raw-policy` option은 neutral gate 없이 policy 자체 bias를 진단할 때만 사용한다.

68차원 checkpoint는 마지막 heading 두 관측이 없으므로 현재 70차원 관측의 앞 68개를 전달해 재생한다. 이 adapter를 학습 재개에 사용하지 않는다.

## 8. 원격 학습 운영

대화형 진입:

```bash
python -m src.rl
```

일반 흐름:

1. `학습 환경/보상 스모크 테스트`
2. `새 학습 시작`에서 SSH/로컬 위치를 먼저 선택
3. SSH이면 머신의 코어·가용 메모리·load를 조회해 병렬 환경 수 추천 확인
4. reference, curriculum, terrain, stance, timestep, env 수 선택
5. 필요 시 현재 코드를 SSH host로 동기화
6. Python 3.12 `.venv`와 RL 의존성 확인
7. `nohup` background trainer의 PID/로그/checkpoint 확인
8. pause 시 SIGTERM과 resume checkpoint 사용
9. resume 시 같은 run 설정과 호환성 확인
10. watcher/download는 `.part`를 받은 뒤 ZIP 검증과 atomic publish

추천 환경 수는 다음 보수식의 최솟값이다.

```text
CPU 한도 = max(1, 물리 코어 - 1)
메모리 한도 = max(1, floor((가용 메모리 - 2 GiB) / 768 MiB))
추천 = min(CPU 한도, 메모리 한도)
```

자동 조회가 실패하면 4개를 편집 가능한 기본값으로 제안한다. 추천은 “최대 안전 성능”의 보장이 아니라 시작점이며, 원격 머신에 다른 작업이 있으면 실제 FPS와 load를 보고 낮춘다.

`runs/.remote_jobs.json`은 접속/실험 metadata이므로 공개 저장소에 올리기 전에 host/path 정보가 민감하지 않은지 확인한다.

reset은 삭제가 아니라 원격 `runs/.reset_backup/`으로 이동한다. 복구 가능성을 확인한 뒤 오래된 백업을 별도로 정리한다.

## 9. 테스트

전체 테스트:

```bash
python -m unittest discover -s tests -v
```

영역별 빠른 실행:

```bash
python -m unittest tests.test_actuators tests.test_api
python -m unittest tests.test_kinematics tests.test_tripod_gait tests.test_scone_gait
python -m unittest tests.test_stair_geometry tests.test_stair_climber
python -m unittest tests.test_simulation tests.test_terrain
python -m unittest tests.test_remote_watch tests.test_rl_inquiry
python -m unittest tests.test_rl_joystick tests.test_rl_reference_motion
```

변경별 최소 검증:

| 변경 영역 | 최소 테스트 |
|---|---|
| actuator ID/register | actuator + API fake controller |
| 초기화/모드 | API + simulation |
| model joint/axis | kinematics + tripod-gait/scone-gait + reference motion |
| terrain | terrain + simulation |
| arc-wheel/계단 controller | stair geometry + stair climber + simulation |
| reward/observation | remote-watch reward tests + environment check |
| checkpoint/remote command | remote-watch + RL inquiry |
| joystick | API joystick + RL joystick |

실제 serial이나 SSH를 호출하는 검증은 unit test와 분리하고, 명시적으로 선택한 장치/host에서만 한다.

## 10. 문서와 코드 변경 체크리스트

- 새로운 파일을 추가하면 [파일 지도](05-file-and-folder-map.md)를 갱신한다.
- 상수·데이터 클래스·상태 필드를 추가하면 [변수 사전](variables/README.md)을 갱신한다.
- 관측 차원을 바꾸면 shape, 순서, compatibility policy, 테스트, checkpoint 안내를 모두 갱신한다.
- reward를 바꾸면 [보상 가이드](06-reward-function-guide.md)의 수식·기본값·회귀 테스트를 갱신한다.
- model motor 사양을 바꾸면 XML과 `src/simulation/core/pid.py`를 함께 바꾼다.
- profile/stance를 바꾸면 hardware profile, simulation seed, RL stance의 의도적 차이를 기록한다.
- 논문 숫자는 원 로그/영상/설정과 연결된 evidence가 있을 때만 확정한다.

## 11. 현재 운영상 주의사항

- `requirements.txt`는 CLI·Dynamixel·MuJoCo·RL을 통합한다. 새 장치에서는 설치 뒤 대표 ID 1/7/13 ping과 MuJoCo import를 각각 확인한다.
- reward/heading 비교 전에 `SconeWalkEnv.step()`의 heading target 이중 적분을 확인한다.
- `runs/remote_watch/scone_walk_260000_steps.zip.part`는 미완성 파일이므로 policy로 열지 않는다.
- `archive/ICRA`의 결과 표는 아직 실험 증거로 채워지지 않았다.
