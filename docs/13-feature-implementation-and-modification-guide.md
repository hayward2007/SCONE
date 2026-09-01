# SCONE 기능 구현 및 코드 수정 가이드

이 문서는 2026-09-01 현재 SCONE의 주요 기능이 **어떤 경로로 실행되고,
어느 코드에서 구현되며, 기능을 바꾸려면 무엇을 함께 수정하고 검증해야
하는지**를 한곳에 정리한 개발자용 안내서다. 세부 수식과 시행착오는 기존
주제별 문서에 남기고, 여기서는 실제 코드 수정의 출발점과 영향 범위를
중심으로 설명한다.

코드와 문서가 다를 때의 최종 기준은 다음 순서다.

1. `src/`와 루트 진입점의 현재 코드
2. `tests/`가 고정하는 동작
3. 이 문서와 주제별 문서
4. `archive/`의 구형 코드·논문·영상

---

## 1. 전체 구조와 수정 경계

### 1.1 공통 제어 흐름

실물과 시뮬레이션은 상위 API를 공유하지만 controller 구현은 분리돼 있다.

```text
SCONE.py / python -m src.simulation / python -m src.rl
                         │
                    src/cli.py
                         │
       ┌─────────────────┼──────────────────┐
       │                 │                  │
 Legacy Walk/Drive  model-based gait    PPO joystick
       │                 │                  │
       └───────────── SCONE facade ─────────┘
                         │
                ControllerProtocol
                  ┌──────┴──────┐
                  │             │
       hardware.Controller  MuJoCoController
                  │             │
             DYNAMIXEL       model.xml + terrain
```

핵심은 [`ControllerProtocol`](../src/hardware/interface.py)이다. 실제
[`Controller`](../src/hardware/controller.py)와 시뮬레이션
[`MuJoCoController`](../src/simulation/core/controller.py)가 같은 위치·속도·
torque·mode API를 구현하므로 [`SCONE`](../src/main.py)과 legacy locomotion은
backend를 몰라도 동작한다.

단, 다음 기능은 의도적으로 MuJoCo 전용이다.

- 연속 lower velocity 회전을 사용하는 비-RL `scone-gait`
- 정체 감지와 대각 삼각 후킹을 사용하는 `scone-stair`
- 자동 계단 `hardcoded/improved/compare` 데모
- MuJoCo body state와 contact를 관측하는 PPO 환경

이 기능을 실물로 옮기는 것은 controller 이름만 바꾸는 작업이 아니다. 실제
joint hard stop, TPU 마찰·변형, 전류·온도, 낙상 정지, 통신 지연을 측정한
안전 계층이 먼저 필요하다.

### 1.2 기능별 기준 파일

| 기능 | 실행 진입점 | 핵심 구현 | 주 회귀 테스트 |
|---|---|---|---|
| 통합 launcher와 조이스틱 | [`SCONE.py`](../SCONE.py), [`src/cli.py`](../src/cli.py) | `KeyboardJoystick`, `run_velocity_joystick_cli()`, `main()` | [`test_api.py`](../tests/test_api.py) |
| 공용 robot API | [`src/main.py`](../src/main.py) | `SCONE`, `RobotCommand`, `RobotStatus` | [`test_api.py`](../tests/test_api.py) |
| 하드웨어 탐색·통신 | [`src/hardware/`](../src/hardware) | `discover_hardware()`, `Controller`, actuator table | [`test_actuators.py`](../tests/test_actuators.py), [`test_api.py`](../tests/test_api.py) |
| Legacy Walk/Drive/Climb | [`src/locomotion/`](../src/locomotion) | `Walk`, `Drive`, `Climb`, `LegacyVelocityAdapter` | [`test_api.py`](../tests/test_api.py), [`test_simulation.py`](../tests/test_simulation.py) |
| FK/IK | [`src/kinematics/`](../src/kinematics) | `LegKinematics`, `RobotKinematics` | [`test_kinematics.py`](../tests/test_kinematics.py) |
| MuJoCo 모델·motor loop | [`src/simulation/core/`](../src/simulation/core) | `load_model()`, `MuJoCoController`, `DCMotorPID` | [`test_simulation.py`](../tests/test_simulation.py) |
| 절차형 지형 | [`src/simulation/terrain/`](../src/simulation/terrain) | `TerrainGenerator`, `STAIR_PRESETS`, `SLOPE_PRESETS` | [`test_terrain.py`](../tests/test_terrain.py) |
| `tripod-gait` | [`tripod_gait.py`](../src/locomotion/tripod_gait.py) | `TripodGait`, `GaitConfig` | [`test_tripod_gait.py`](../tests/test_tripod_gait.py), [`test_simulation.py`](../tests/test_simulation.py) |
| RL bounded `scone-gait` | [`scone_gait.py`](../src/locomotion/scone_gait.py) | `SconeGait`, `SconeGaitConfig` | [`test_scone_gait.py`](../tests/test_scone_gait.py), [`test_rl_reference_motion.py`](../tests/test_rl_reference_motion.py) |
| 연속 회전 `scone-gait` | [`scone_rolling_gait.py`](../src/simulation/core/scone_rolling_gait.py) | `SconeRollingGait` | [`test_scone_rolling_gait.py`](../tests/test_scone_rolling_gait.py) |
| 계단 후킹·자동 데모 | [`stair_climber.py`](../src/simulation/core/stair_climber.py), [`stair_demo.py`](../src/simulation/core/stair_demo.py) | `SconeStairClimber`, `HardcodedStairRoller` | [`test_stair_geometry.py`](../tests/test_stair_geometry.py), [`test_stair_climber.py`](../tests/test_stair_climber.py), [`test_stair_demo.py`](../tests/test_stair_demo.py) |
| RL 환경·reward·PPO | [`walk_learn.py`](../src/rl/walk_learn.py) | `SconeWalkEnv`, `RewardConfig`, `WalkConfig` | [`test_remote_watch.py`](../tests/test_remote_watch.py), [`test_rl_reference_motion.py`](../tests/test_rl_reference_motion.py) |
| RL 조종·호환성 | [`joystick_control.py`](../src/rl/joystick_control.py), [`policy_compat.py`](../src/rl/policy_compat.py) | `NeutralResidualGate`, `_RLModeRouter`, 68→70 replay adapter | [`test_rl_joystick.py`](../tests/test_rl_joystick.py), [`test_remote_watch.py`](../tests/test_remote_watch.py) |
| 원격 학습·checkpoint | [`inquiry.py`](../src/rl/inquiry.py), [`remote_watch.py`](../src/rl/remote_watch.py) | SSH job lifecycle, atomic mirror, ZIP 검사 | [`test_rl_inquiry.py`](../tests/test_rl_inquiry.py), [`test_remote_watch.py`](../tests/test_remote_watch.py) |

### 1.3 수정 전에 지켜야 할 세 원칙

첫째, `src/simulation/cli_bridge.py`, `controller.py`, `model.py`, `pid.py`,
`simulator_cli.py`는 과거 import를 살리는 호환 shim이다. 실제 구현은 대부분
`src/simulation/core/`에 있으므로 shim에 새 로직을 중복해서 넣지 않는다.

둘째, `non_rl`과 `NonRLWalk`은 이전 이름의 호환 경로다. 현재 정식 명칭은
`tripod-gait`이다. 새 UI·설정·문서에는 정식 이름을 쓰고, 저장된 job과 외부
코드 때문에 alias만 유지한다.

셋째, 학습 당시의 reference motion, standing pose, 관측 순서, action scale,
motor dynamics는 checkpoint 의미의 일부다. 이 중 하나를 바꾼 뒤 기존 PPO를
그대로 재개하면 같은 action 값이 다른 관절 목표를 뜻하게 된다.

---

## 2. 모터 ID, 단위와 방향

### 2.1 ID 배열

[`ActuatorIndex`](../src/hardware/actuator_index.py)가 모든 그룹의 기준이다.

| 관절 단계 | ID | 다리 `n`의 ID |
|---|---|---|
| 몸체/upper | 1–6 | `n` |
| 다리 1단/middle | 7–12 | `n + 6` |
| 말단 sector/lower | 13–18 | `n + 12` |

대각 tripod는 다음 두 그룹이다.

- `TRIPOD_A` / `*_DIAGONAL_RIGHT`: 다리 `(1, 4, 5)`
- `TRIPOD_B` / `*_DIAGONAL_LEFT`: 다리 `(2, 3, 6)`

여기서 LEFT/RIGHT는 물리적인 한쪽 면이 아니라 기존 코드의 tripod phase
이름이다. 그룹 이름만 보고 좌우 세 다리로 해석하면 gait가 꼬인다.

### 2.2 단위

| 값 | 단위/규약 |
|---|---|
| 조이스틱 `vx, vy` | m/s, body frame |
| 조이스틱 `yaw_rate` | rad/s, 양수는 좌회전 |
| locomotion motor target | degree |
| 실제 DYNAMIXEL position | `0..4095` raw/회전 |
| MuJoCo `qpos/qvel` | rad, rad/s |
| lower goal velocity | DYNAMIXEL velocity unit |
| 지형 치수 | metre |
| physics timestep | second |

조이스틱의 화면 `+x`는 오른쪽이지만 model body `+y`는 왼쪽이다. 변환은
[`JoystickState.to_velocity_command()`](../src/cli.py)에서
`vy = -x * max_vy`로 한 번만 수행한다. 방향을 바꿀 때 gait 내부와
controller의 부호를 동시에 뒤집지 말고, 다음 순서로 원인을 나눈다.

1. UI axis → body command
2. body command → foot/sector 목표
3. mirrored lower axis → raw velocity
4. contact reaction → 실제 body displacement

---

## 3. 통합 launcher와 키보드 조이스틱

### 3.1 구현 방식

[`src/cli.py`](../src/cli.py)의 `main()`은 장치를 비파괴 탐색한 뒤 다음 메뉴를
연다.

- 시뮬레이션 자동 데모
- 시뮬레이션 조종
- 실제 하드웨어 조종
- 하드웨어 재탐색
- 강화학습 관리

시뮬레이션 세부 선택은
[`src/simulation/core/simulator_cli.py`](../src/simulation/core/simulator_cli.py),
실행 route는
[`src/simulation/core/cli_bridge.py`](../src/simulation/core/cli_bridge.py)가
담당한다. `SimulationControl`은 현재 `old`, `tripod-gait`, `scone-gait`,
`scone-stair`, `rl`을 제공한다.

`KeyboardJoystick`은 terminal에 key-up event가 없다는 점을 timeout으로
처리한다. 키 입력은 0.35초 동안 axis를 유지하고 OS key repeat가 deadline을
갱신한다. `run_velocity_joystick_cli()`는 50 Hz로 다음 순서를 반복한다.

```python
state = joystick.state()
command = state.to_velocity_command(limits)
apply_command(command, dt)
```

`Space`와 종료 시에는 명시적으로 zero command를 보내므로 마지막 속도
명령이 남지 않는다.

### 3.2 조이스틱 기능 수정 방법

키를 추가하거나 바꾸려면:

1. `_JOYSTICK_KEY_BINDINGS` 또는 `_TERMINAL_KEY_SEQUENCES`를 수정한다.
2. `_normalize_key()`에 platform alias가 필요한지 확인한다.
3. `render_joystick_ui()`의 도움말과 표시를 같이 수정한다.
4. [`test_api.py`](../tests/test_api.py)의 axis, timeout, neutral 테스트를
   추가한다.

속도 한계를 바꾸려면 UI 배율을 하드코딩하지 말고 해당 controller의
`GaitConfig` 또는 `JoystickLimits`를 바꾼다. PPO 조종 한계는 학습 관측의
`OBSERVATION_COMMAND_SCALE`에서 오므로 일반 gait 한계와 따로 관리한다.

새 simulation control을 추가하려면:

1. `SimulationControl` enum에 정식 문자열을 추가한다.
2. `select_simulation_control()`에 사용자 label을 추가한다.
3. `cli_bridge.run()`의 worker 분기에 adapter를 연결한다.
4. 직접 CLI가 필요하면 `build_parser()` 선택지와 필수 argument 검사를
   추가한다.
5. 메뉴 dispatch는 `test_api.py`, simulation route는 `test_simulation.py`에
   테스트한다.

---

## 4. 공용 `SCONE` API와 수명주기

### 4.1 구현 방식

[`src/main.py`](../src/main.py)의 `SCONE`은 construction 시 port를 열거나
움직이지 않는다. controller를 주입한 뒤 `initialize()`해야 명령을 받는다.

초기화 순서는 다음과 같다.

1. 전체 torque off
2. lower 13–18을 position mode로 복귀
3. XM acceleration 20 설정
4. torque on
5. safety speed에서 middle 시작 자세
6. upper profile 자세
7. lower profile 자세
8. middle profile 자세
9. walking speed로 복귀하고 `Walk` mode 생성

`forward/backward/left/right`는 현재 `Mode` 객체로 위임된다. `change_mode()`는
`Walk → Drive → Climb → Walk`의 객체 교체를 수행한다. `shutdown()`은 middle을
종료 자세로 옮긴 뒤 torque를 끄고, `close()`는 controller까지 닫는다.

### 4.2 API 수정 방법

새 자세 profile을 추가하려면
[`src/locomotion/profile.py`](../src/locomotion/profile.py)에 immutable
`MotionProfile`을 만들고 `PROFILES`에 등록한다. 다음을 함께 확인한다.

- upper tuple이 정확히 6개인가
- middle/lower 초기 자세가 실제 기구 한계 안인가
- walking/driving/climbing speed 의미가 기존 model과 같은가
- `initialize()`의 이동 순서에서 충돌하지 않는가
- MuJoCo standing seed 또는 RL standing pose와 혼동되지 않는가

새 public command를 추가하려면 `RobotCommand`, `SCONE.execute()`의 dispatch,
현재 mode의 구현과 `SCONE.py` export를 함께 수정한다. mode에 없는 동작은
`UnsupportedCommandError`로 실패해야 하며 조용히 무시하지 않는다.

실제 로봇 수명주기를 바꿀 때는 fake controller 테스트만으로 충분하지 않다.
지지대 위에서 torque-off, mode switch, 목표 위치 순서와 비상 정지를 별도로
확인한다.

---

## 5. 실제 DYNAMIXEL 하드웨어

### 5.1 구현 방식

하드웨어 계층은 네 부분으로 나뉜다.

- [`actuator_index.py`](../src/hardware/actuator_index.py): ID와 관절 그룹
- [`actuator_control_table.py`](../src/hardware/actuator_control_table.py):
  model별 protocol, register address와 byte 크기
- [`discovery.py`](../src/hardware/discovery.py): 후보 port를 열고 ID 1/7/13을
  ping하는 비파괴 탐색
- [`controller.py`](../src/hardware/controller.py): 실제 read/write, model별
  `GroupSyncWrite`, torque/mode/position/velocity API

upper 1–6은 MX-28AT Protocol 1.0, middle/lower 7–18은 XM430 Protocol 2.0이다.
`Controller._sync_write()`는 요청을 `(protocol, register)`로 묶으므로 서로
다른 register width를 한 packet으로 섞지 않는다.

`set_mode()`는 지원 model에서 torque를 끄고 operating mode를 쓴 뒤 다시
켠다. upper MX에는 operating-mode register가 없으므로 무시한다. lower만
position/velocity mode를 전환하는 것이 현재 locomotion의 전제다.

### 5.2 하드웨어 수정 방법

motor model 또는 register를 바꾸려면:

1. datasheet에서 model number, protocol, address, byte size를 확인한다.
2. `ActuatorModel`과 `ControlTable`을 추가/수정한다.
3. [`actuator.py`](../src/hardware/actuator.py)의 `model_for_id()` 매핑을
   갱신한다.
4. read/write가 signed 값인지 확인한다. 특히 goal velocity의 음수 encoding을
   sync write의 bit mask와 함께 검증한다.
5. `test_actuators.py`에서 ID별 model/register 선택을 고정한다.
6. 실제 bus에서는 한 ID씩 torque off 상태로 검증한 뒤 group write로 넓힌다.

serial port 기본값을 바꾸는 것보다 `SCONE_DEVICE` 환경 변수를 우선 사용한다.
탐색을 수정할 때도 torque, mode, position write를 넣지 않는다.

---

## 6. Legacy `Walk / Drive / Climb`

### 6.1 구현 방식

Legacy 계열은 검증된 blocking motor sequence를 보존한다.

- [`Walk`](../src/locomotion/walk.py): 대각 tripod를 hold/release하며 전후진,
  upper/middle 목표로 좌우 회전
- [`Drive`](../src/locomotion/drive.py): lower를 velocity mode로 바꿔 C자
  말단을 주행 바퀴처럼 회전
- [`Climb`](../src/locomotion/climb.py): 한 tripod를 지지하고 반대 tripod의
  middle/lower를 교대하는 기존 등반 sequence
- [`LegacyVelocityAdapter`](../src/locomotion/legacy_velocity.py): 연속
  `VelocityCommand`를 가장 가까운 discrete 명령으로 바꾸고 background
  thread에서 최신 명령만 실행

이 경로는 model-based `tripod-gait`와 다르다. `Walk`는 정해진 position과
sleep 순서를 실행하고, `TripodGait`는 50 Hz 발 궤적과 IK를 생성한다.

### 6.2 Legacy 동작 수정 방법

자세나 순서를 바꾸려면:

1. `walk.py`, `drive.py`, `climb.py` 중 실제 mode를 먼저 특정한다.
2. 반복되는 ID는 숫자 tuple을 새로 만들지 말고 `Actuator.Index` 그룹을
   사용한다.
3. profile 속도는 `MotionProfile`에서, 한 동작의 임시 speed만 mode 내부에서
   바꾼다.
4. mode 종료 시 lower operating mode와 zero velocity가 다음 mode의 전제와
   맞는지 확인한다.
5. blocking sleep을 바꾸면 실물뿐 아니라 MuJoCo worker와 자동 benchmark의
   준비 시간도 달라짐을 기록한다.

`LegacyVelocityAdapter`의 방향 임계값을 바꿀 때는 translation과 yaw가 동시에
들어온 경우 어떤 discrete 동작을 우선할지 테스트해야 한다.

---

## 7. FK/IK와 말단 접촉점

### 7.1 구현 방식

[`LegKinematics`](../src/kinematics/leg.py)는 MJCF 이름으로 한 다리의 세
joint와 `TIRE_n` body를 찾는다. 별도 링크 길이 상수를 복제하지 않고 MuJoCo
forward kinematics와 Jacobian을 사용한다.

inverse kinematics는 position-only damped least squares다.

```text
Δq = Jᵀ (J Jᵀ + λ²I)⁻¹ e
```

큰 step은 `max_step`으로 제한하고 backtracking으로 residual이 줄어드는
candidate만 받는다. 세 관절로 말단 위치 3개 축은 풀지만 tire orientation을
독립적으로 지정하지는 않는다.

[`RobotKinematics`](../src/kinematics/robot.py)는 여섯 `LegKinematics`가 하나의
MuJoCo model/data를 공유하게 하고, `(6, 3)` 또는 actuator 순서 `(18,)`를
상호 변환한다.

`TripodGait`는 `TIRE_n` body origin 대신 nominal pose에서 tire mesh의 가장
낮은 0.1 mm patch 중심을 support point로 추론한다. 한쪽 mesh vertex만
선택해서 생기는 불필요한 lateral moment를 피하기 위해서다.

### 7.2 FK/IK 수정 방법

MJCF joint/body 이름을 바꾸면 `LegKinematics.__init__()`의 이름 규칙과
`MuJoCoController._find_actuator()`의 `A01_..A18_` 규칙을 함께 수정한다.

IK tolerance/damping을 조정할 때는:

- `test_kinematics.py`의 FK→IK round trip
- `test_tripod_gait.py`의 여섯 다리 convergence
- gait의 `ik_backoff_scale`과 failed leg
- 실제 frame의 연속성

을 같이 본다. convergence 비율만 높이기 위해 tolerance를 느슨하게 하면
발 접촉점 오차와 slip을 숨길 수 있다. MJCF에 실제 mechanical joint range가
아직 없으므로 simulation IK 성공을 실물 안전 범위로 해석하지 않는다.

---

## 8. MuJoCo 모델, controller와 physics loop

### 8.1 모델 load

[`load_model()`](../src/simulation/core/model.py)은 원본
[`model.xml`](../src/assets/model.xml)을 runtime에 XML로 수정한다.

1. fixed/floating base 선택에 따라 root freejoint를 삽입하거나 제거
2. contact mesh의 최저점으로 simulation floor 높이 계산
3. 선택 terrain primitive geom 삽입
4. STL asset bytes와 함께 MuJoCo model compile

원본 MJCF에 매 terrain을 직접 복사하지 않으므로 robot asset과 실험 course를
분리할 수 있다.

### 8.2 simulated motor

[`MuJoCoController`](../src/simulation/core/controller.py)는 actuator 이름
`A01_..A18_`로 joint, qpos, dof address를 찾고 18개의
[`DCMotorPID`](../src/simulation/core/pid.py)를 만든다. position mode에서는
profile velocity/acceleration으로 setpoint를 만든 뒤 PID가 voltage를
출력하고, velocity mode에서는 goal velocity를 추종한다. 최종 torque는
motor spec, voltage, back-EMF와 saturation의 영향을 받는다.

초기 qpos는 raw CAD rest가 아니라 standing pose로 seed하고, floating root를
들어 올려 최저 contact mesh가 floor 위 2 mm에 놓이게 한다. 이 과정은
초기 `initialize()` torque-off에서 차체가 접힌 자세로 무너지는 transient를
줄인다.

비-RL MuJoCo `tripod-gait`는 `configure_model_gait_controller()`가 profile
limit를 `0`으로 풀고 middle position stiffness를 2배로 올린다. 연속 회전형
`scone-gait`는 별도의 `SconeRollingGaitConfig`에서 160/50 profile과 stiffness
2배를 적용한다. 두 opt-in 튜닝 모두 PPO replay에는 적용하지 않는다.

### 8.3 simulation 수정 방법

motor 성능을 바꾸려면 `pid.py`의 model spec과 `controller.py`의 outer-loop
gain을 구분한다.

- torque/speed/전기 사양 변경: `DCMotorSpec`
- position 응답 변경: `default_gains_for_motor_id()` 또는 opt-in stiffness
- DYNAMIXEL profile 의미 변경: `set_speed()`, `set_acceleration()`
- mirrored lower 방향 변경: `arc_wheel_velocities()`

이 값은 학습 동역학이다. 바꾼 뒤 기존 checkpoint 성능이 달라지는 것은
호환성 오류가 아니라 환경 자체가 달라진 결과일 수 있다. 새 정책을 학습할
경우 run 이름과 설정에 변경점을 남긴다.

viewer loop는 controller update 후 `mujoco.mj_step()`을 실행한다. 자동 계단
데모는 60 Hz render와 2 ms physics를 분리하고 timeout도 simulation time을
사용한다. GUI timeout을 wall clock으로 되돌리면 느린 rendering 환경에서
실험이 조기 종료된다.

---

## 9. 절차형 terrain

### 9.1 구현 방식

[`TerrainType`](../src/simulation/terrain/types.py)은 `flat`, `uneven`,
`stairs-1..3`, `slope-1..3`, `mixed`를 정의한다.
[`TerrainGenerator`](../src/simulation/terrain/generator.py)는 box/ramp primitive를
현재 cursor 뒤에 이어 붙이고 geom 이름, 시작/끝 위치, 최대 높이를 반환한다.
`uneven`과 `mixed`의 random 요소는 `terrain_seed`로 재현된다.

현재 계단 preset은 다음과 같다.

| preset | 각 riser | tread |
|---|---:|---|
| `stairs-1` | 100 mm × 3 | 300/270/240 mm |
| `stairs-2` | 150 mm × 3 | 270/230/200 mm |
| `stairs-3` | 200 mm × 3 | 350/350/350 mm |

200 mm preset의 tread 350 mm는 한 tripod가 tread를 지지하는 동안 다른
tripod가 simulated hook assist를 수행할 공간을 확보하기 위한 현재 검증값이다.

### 9.2 terrain 수정 방법

단높이·폭·tread만 바꿀 때는
[`presets.py`](../src/simulation/terrain/presets.py)의 `StairProfile`을 수정한다.
새 지형 종류를 추가할 때는:

1. `TerrainType`과 label 추가
2. 필요한 profile dataclass/preset 추가
3. `TerrainGenerator.build()` route 추가
4. mixed course 포함 여부 결정
5. simulator argparse와 interactive picker 노출 확인
6. `test_terrain.py`에서 XML compile, 치수, seed 재현성 검증

계단 치수를 바꾸면 `SconeStairClimber.maximum_rise`의 direct-roll 판정,
자동 데모의 `_top_thresholds()`, benchmark 성공 threshold까지 다시
검증한다. 단순히 terrain이 compile되는 것만으로 climbing 성공을 보장하지
않는다.

---

## 10. `tripod-gait`

### 10.1 구현 방식

[`TripodGait`](../src/locomotion/tripod_gait.py)은 Phoenix 계열의 고전 교대
삼각보를 SCONE의 3D IK와 결합한다.

각 다리의 nominal contact 위치 `r_i = (x_i, y_i)`에서 body twist가 요구하는
발 속도는 다음과 같다.

```text
v_foot,i = [vx - yaw_rate*y_i,
            vy + yaw_rate*x_i]

stroke_i = v_foot,i * duty_factor / cycle_frequency
```

요청 stroke가 전후/측면 타원 workspace를 넘으면 방향은 유지한 채 경계까지
축소한다. stance에서는 발을 body 명령 반대 방향으로 이동하고, swing에서는
minimum-jerk 보간과 `16s²(1-s)²` lift로 다음 위치에 되돌린다. 여섯 foot
target을 IK로 풀어 18개 motor degree를 만든다.

현재 비-RL MuJoCo 조종은
[`TRIPOD_GAIT_SIMULATION_CONFIG`](../src/simulation/core/cli_bridge.py)를 사용한다.

- 50 Hz control
- 1.0 Hz cycle
- duty 0.5
- 25 mm lift
- 90 mm 전후, 70 mm 측면 workspace
- 최대 4회 0.8배 IK backoff
- simulation profile velocity/acceleration 제한 해제

이 값은 simulation-only measured tuning이다. `GaitConfig`의 library default와
RL reference 설정은 의도적으로 다르다.

### 10.2 `tripod-gait` 수정 방법

튜닝은 다음 순서로 한 변수군씩 진행한다.

1. 좌표/부호: 작은 `vx`에서 여섯 foot target과 root displacement 확인
2. nominal stance/support point: idle IK가 현재 자세를 유지하는지 확인
3. cadence와 duty
4. `max_stride`/`max_lateral_stride`와 clipping 비율
5. step height
6. IK tolerance/backoff
7. simulated motor profile/stiffness

`max_vx`만 올려도 stroke workspace가 이미 포화라면 실제 보폭은 늘지 않는다.
다음 식으로 먼저 clipping을 예측한다.

```text
requested_stride = |v| * duty_factor / cycle_frequency
```

수정 후에는 최소한 `test_tripod_gait.py`와 `test_simulation.py`를 실행하고,
8초 flat run에서 전진·측면 drift·yaw·역방향 누적·root Z·IK failure를 함께
측정한다. 평균 전진만 보면 앞뒤 움직임이 상쇄되는 문제를 놓칠 수 있다.

실물 적용 시 `reset_from_controller()`로 현재 joint 위치를 nominal stance로
잡는 것이 권장된다. 현재 simulation route는 초기 중력 sag에서 불안정한 IK
branch를 잡지 않도록 검증된 profile pose를 유지한다.

---

## 11. 두 종류의 `scone-gait`

같은 이름이지만 구현 목적이 다르므로 수정 전에 반드시 구분한다.

| 경로 | 클래스 | lower 제어 | 용도 |
|---|---|---|---|
| RL reference/position controller | [`SconeGait`](../src/locomotion/scone_gait.py) | bounded position sweep | residual 기준 모션, checkpoint 호환 |
| 비-RL MuJoCo 조종 | [`SconeRollingGait`](../src/simulation/core/scone_rolling_gait.py) | continuous velocity + basic motion rate | 실제로 여러 바퀴 회전하는 실험 gait |

### 11.1 RL bounded `SconeGait`

`SconeGait`은 `TripodGait`의 upper/middle/lower position frame에 sector
tangent와 steering solution을 더한다. 기본값은 0.65 Hz, 35/25 mm stride,
25 mm lift, ±30° sector sweep이다. output이 항상 18개 bounded position이므로
다음 residual 합성이 가능하다.

```text
target = reference_motor_degrees
       + residual_scale_degrees * policy_action
```

이 클래스를 연속 회전으로 바꾸면 기존 position residual의 의미가 사라진다.
RL에서 unbounded roll을 학습하려면 action/observation/controller 설계를
별도 버전으로 만들고 새 checkpoint를 학습해야 한다.

### 11.2 비-RL 연속 회전 `SconeRollingGait`

`SconeRollingGait`은 내부 `SconeGait` planner로 몸통·middle 기본 보행을
계산하면서 lower를 velocity mode로 운용한다.

1. ID 1–12에는 planner position 전송
2. lower rolling target을 contact tangent와 command로 계산
3. planner lower의 nominal 대비 offset을 미분
4. 미분값을 velocity unit으로 바꾸고 limit/low-pass
5. continuous roll과 0.35배 basic lower motion을 합성

```text
v_lower = lowpass(v_roll, 0.10 s)
        + 0.35 * lowpass(d(Δq_basic)/dt, 0.04 s)
```

현재 주요 값은 roll velocity 175, support ratio 0.80, tripod B phase 60°,
planner 0.8 Hz/duty 0.58/55 mm stride/20 mm lift다. 시작할 때 `prepare()`가
C자 개구부 phase를 나눠 position으로 정렬하고, `activate()`가 lower를
velocity mode로 바꾼다.

### 11.3 `scone-gait` 수정 방법

화면에서 “회전만 한다”면 먼저 다음 telemetry를 확인한다.

- upper nominal 대비 최대 명령각
- middle nominal 대비 최대 명령각
- planned lower offset과 basic velocity
- lower 실제 회전수

full-body 보행 크기를 바꾸려면 `SconeRollingGaitConfig`의
`cycle_frequency`, `duty_factor`, `step_height`, `max_stride`를 조정한다.
말단 추진을 바꾸려면 `roll_velocity`, `support_velocity_ratio`,
`basic_lower_motion_blend`를 조정한다. 두 군을 동시에 크게 올리면 같은 추진을
중복 요구해 yaw와 height drop이 커질 수 있다.

phase를 바꿀 때는 한 번의 전진 거리뿐 아니라:

- 여섯 opening이 동시에 바닥을 향하는지
- lateral excursion과 최종 drift
- 최대 yaw
- root Z 최저값과 upright
- lower 회전수
- IK failure와 합성식 오차

를 6–20초 동안 본다. `test_scone_rolling_gait.py`는 full-body movement,
연속 회전, phase, drift/height 회귀를 고정한다.

---

## 12. 부채꼴 후킹과 계단 이동

### 12.1 기하 계산

[`stair_geometry.py`](../src/locomotion/stair_geometry.py)는 현재 SCONEv2
arc-wheel 외반경, 내반경, opening angle을 묶고 다음 계산을 제공한다.

- riser가 radial band에 들어오는지
- clearance를 포함한 보수적 direct-roll 한계
- sharp edge offset `sqrt(2Rh-h²)`
- quasi-static pivot torque와 horizontal push
- 필요한 마찰계수
- support polygon margin

현재 외반경 `R = 122.5 mm`이고
`SconeStairConfig.direct_roll_clearance = 3 mm`이므로 단순 접근의
direct-roll 분류는 대략 `rise + 3 mm <= 122.5 mm`다. 이는 기하 1차
판정이며 실제 성공 보장이 아니다.

전체 공식과 가정은
[`11-scone-stair-climbing.md`](11-scone-stair-climbing.md)를 기준으로 한다.

### 12.2 adaptive `scone-stair`

[`SconeStairClimber`](../src/simulation/core/stair_climber.py)은 정지
`IDLE`에서 시작하고, 명령 중에는 두 active 상태를 오간다.

```text
IDLE
  └─ command ─> ROLLING
                   ├─ progress sufficient ─> ROLLING
                   ├─ known tall riser ─────> TRIPOD_ASSIST
                   └─ stall detected ───────> TRIPOD_ASSIST
                                                └─ phases complete ─> ROLLING
```

`ROLLING`은 여섯 lower를 함께 회전한다. `TRIPOD_ASSIST`는
`(1,4,5)`와 `(2,3,6)` bank를 교대하며 support middle 250°, swing middle
165°, support/swing lower velocity 105/185를 smooth transition으로 적용한다.
0.80초 동안 진행이 25 mm 미만이면 stall로 판단한다. 150/200 mm처럼
direct-roll 범위를 넘는 알려진 preset은 첫 riser 전에 pre-hook을 시작한다.

`prepare_scone_stair_pose()`는 Legacy Walk에서 네 번 회전해 course `+Y`에
side-on으로 놓고 Drive 자세로 전환한다. 이 준비 sequence는 terrain 방향과
결합돼 있으므로 course axis를 바꾸면 함께 수정해야 한다.

### 12.3 자동 데모와 benchmark

[`stair_demo.py`](../src/simulation/core/stair_demo.py)는 조종 입력 없이 다음을
실행한다.

- `hardcoded`: 상태 feedback 없이 lower velocity 150 고정
- `improved`: `SconeStairClimber`
- `compare`: 두 viewer를 순서대로 실행

[`stair_benchmark.py`](../src/simulation/stair_benchmark.py)는 GUI 없이 H0–H4와
튜닝 variant를 같은 준비 조건에서 JSONL로 출력한다.

계단 controller를 수정할 때는:

1. geometry predicate와 config validation 수정
2. state transition 단위 테스트
3. headless 동일 조건 benchmark
4. 자동 데모 result/timeout 테스트
5. macOS `mjpython` viewer에서 실제 route 확인
6. 시도·실패·채택 이유를 계단 문서에 기록

현재 결정론적 결과는 100 mm에서 두 방식이 통과하고, 150/200 mm에서는 fixed
rolling이 제한 시간 안에 실패하며 adaptive가 통과한다. 이는 현재 model과
마찰 설정의 simulation 결과이지 실물 성능 보증이 아니다.

---

## 13. Residual RL 환경

### 13.1 environment step

[`SconeWalkEnv`](../src/rl/walk_learn.py)는 command-conditioned residual PPO
환경이다.

```text
command update
  → target heading/phase update
  → reference motion 생성
  → reference + 18D residual
  → position target 전송
  → 10 × (controller update + 2 ms physics)
  → observation/reward/termination
```

`WalkConfig.physics_timestep = 0.002`, `frame_skip = 10`이므로 physics는
500 Hz, policy는 50 Hz다.

### 13.2 action

action은 raw current가 아니라 `[-1, 1]` 범위의 18개 position residual이다.
관절군별 scale은 upper 10°, middle 12°, lower 15°이며 현재 target은:

```text
target = reference + residual_scale * clipped_action
target = clip(target, standing_pose - 60°, standing_pose + 60°)
```

MX upper가 current control을 지원하지 않으므로 18개 joint에 공통으로 적용할
수 있는 position residual을 선택했다. ±60° clip은 실제 hard stop 측정값이
아닌 임시 보수 제한이다.

### 13.3 observation

현재 관측은 70차원이다.

| 항목 | 차원 |
|---|---:|
| body linear velocity | 3 |
| body angular velocity | 3 |
| projected gravity | 3 |
| joint position offset | 18 |
| joint velocity | 18 |
| previous action | 18 |
| normalized command | 3 |
| gait phase sin/cos | 2 |
| heading error sin/cos | 2 |

`policy_compat.py`는 과거 68차원 checkpoint를 재생할 때만 마지막 heading
2개를 잘라 준다. 학습 재개에는 이 adapter를 사용하지 않는다.

### 13.4 reference motion

- `hardcoded`: 기존 사인파 upper stride + middle lift
- `tripod-gait`: model-based alternating tripod + IK
- `scone-gait`: bounded position rolling/creep reference
- `non_rl`: `tripod-gait`의 저장 호환 alias

새 check/train의 기본은 `tripod-gait`이고, `enjoy`와 interactive replay의
기본은 기존 checkpoint를 위해 `hardcoded`다. checkpoint가 학습된 reference를
모르면 저장된 run 설정과 log를 먼저 확인한다. 임의 선택으로 움직임을
“고치려” 하면 policy residual과 reference가 반대 방향으로 상쇄될 수 있다.

### 13.5 reward와 종료

`RewardConfig`는 다음 네 합성군을 만든다.

- `velocity`: command linear tracking과 idle velocity
- `direction`: yaw rate와 누적 target heading
- `stability`: upright, one-sided height drop, oscillation, slip, joint limit,
  forbidden collision
- `damping`: action rate/magnitude, idle residual, estimated current

각 항은 `control_dt`를 곱해 policy frequency 변화에 덜 민감하게 합산한다.
NaN/Inf, 과도한 tilt/height drop, forbidden floor collision, hard joint offset은
episode를 종료하고 termination penalty를 더한다.

reward의 전체 수식과 double-count 방지 규칙은
[`06-reward-function-guide.md`](06-reward-function-guide.md)를 따른다.

### 13.6 RL 기능 수정 방법

reward weight만 바꿀 때:

1. `RewardConfig`의 한 family만 수정
2. `_reward()`의 raw term과 합성군에서 중복 합산되지 않는지 확인
3. zero/random action `check`로 finite와 항별 부호 확인
4. TensorBoard에서 전체 reward뿐 아니라 `reward/*` 항을 비교
5. 기존 checkpoint fine-tune인지 새 실험인지 명시

observation을 바꿀 때:

1. `_observation()` 순서와 `observation_space.shape`를 같이 수정
2. normalization 단위를 문서화
3. 기존 policy는 원칙적으로 새 학습 대상으로 분리
4. replay adapter가 수학적으로 안전한 prefix/suffix 변화일 때만
   `policy_compat.py`에 명시적으로 추가

action/reference를 바꿀 때:

1. `_apply_action()` 합성 의미와 residual scale 수정
2. reference별 zero-action pose를 검증
3. neutral command에서 residual이 사라지는지 확인
4. 기존 checkpoint와 호환되지 않으면 새 run 이름/metadata 사용

physics/controller/terrain/standing pose를 바꿀 때도 environment version이
바뀐 것으로 취급한다.

---

## 14. RL 조종, 학습과 checkpoint 운영

### 14.1 interactive PPO 조종

[`run_rl_joystick()`](../src/rl/joystick_control.py)은 checkpoint ZIP을 검사하고
`SconeWalkEnv(render_mode="human")`를 만든 뒤 policy를 50 Hz로 실행한다.
terminal input은 mailbox에 최신 command를 쓰고 physics/policy loop가 읽는다.

`NeutralResidualGate`는 command가 neutral이면 새 policy bias를 무시하고 이전
residual을 0.10초로 감쇠한다. `R`을 누르면 `_RLModeRouter`가 PPO Walk를
중지하고 같은 `MuJoCoController`에서 Legacy Drive/Climb을 실행한다. Walk로
돌아오면 heading, reference height, action과 gait state를 다시 맞춘다.

### 14.2 PPO 학습

`walk_learn.py train`은 environment 수가 1이면 `DummyVecEnv`, 2개 이상이면
`SubprocVecEnv`를 사용한다. 새 PPO의 현재 주요 hyperparameter는:

- MLP policy/value `[256, 256]`
- learning rate `3e-4`
- `n_steps=1024`, `batch_size=256`, `n_epochs=10`
- `gamma=0.99`, `gae_lambda=0.95`, `clip_range=0.2`
- `ent_coef=0.005`, `max_grad_norm=1.0`

`PruningCheckpointCallback`은 최근 checkpoint 수를 제한하고 resume pointer를
갱신한다. SIGINT/SIGTERM은 현재 step을 마친 뒤 final model과 pointer를
남기는 graceful stop 경로를 사용한다.

hyperparameter를 수정할 때는 reward/environment 변경과 같은 run에 섞지 않는
것이 비교에 유리하다. `n_steps`는 environment당 rollout 길이이고
`checkpoint_every`는 전체 timestep 기준이므로 `num_envs`와 함께 해석한다.

### 14.3 원격 학습 관리

[`inquiry.py`](../src/rl/inquiry.py)는 다음 흐름을 제공한다.

- standing pose, reference, curriculum, terrain, env 수 선택
- 로컬 환경 check
- SSH project sync와 dependency check/install
- 원격 capacity 확인과 env 수 추천
- background launch/status/pause/resume/reset
- artifact download와 local viewer/watch

reset은 기존 run을 즉시 삭제하지 않고 backup으로 이동한다. pause/resume은
`resume.json` 또는 가장 최신의 정상 checkpoint를 찾는다.

[`remote_watch.py`](../src/rl/remote_watch.py)의 mirror는 다음 순서를 지킨다.

1. 최신 step 후보 선택
2. `.part`로 다운로드
3. PPO ZIP 구조 검사
4. 정상 파일만 atomic replace
5. viewer policy hot-swap

미완성 `.part` 또는 손상 ZIP으로 known-good local checkpoint를 덮어쓰지
않는다.

### 14.4 checkpoint 호환성 체크리스트

학습 재개 또는 성능 비교 전에 다음을 기록한다.

- observation shape와 순서
- action shape, residual scale, target clip
- reference motion
- standing pose
- terrain/preset/seed
- physics timestep와 frame skip
- motor spec/PID/profile
- reward config
- curriculum과 command scale

하나라도 다르면 “같은 policy의 단순 재개”가 아니라 migration 또는 새
실험일 수 있다.

---

## 15. 코드 수정 시나리오별 절차

### 15.1 보행 속도 또는 보폭 변경

1. 대상이 Legacy, non-RL MuJoCo, RL reference 중 무엇인지 구분한다.
2. Legacy면 `MotionProfile`과 `walk.py`, non-RL이면
   `TRIPOD_GAIT_SIMULATION_CONFIG`, RL이면 `SconeWalkEnv`의 reference config를
   수정한다.
3. `v * duty / frequency`와 workspace 상한으로 clipping을 계산한다.
4. IK, profile lag, root displacement, drift/yaw/height를 측정한다.
5. 실물 경로를 바꿨으면 지지대·저속 검증을 별도로 수행한다.

### 15.2 새로운 gait 추가

1. 입력과 출력 계약을 `VelocityCommand → motor target/velocity`로 정의한다.
2. pure planner는 `src/locomotion/`, MuJoCo 전용 actuator adapter는
   `src/simulation/core/`에 둔다.
3. 새 config dataclass에 단위와 범위 validation을 둔다.
4. neutral frame과 `stop()`의 zero command를 먼저 구현한다.
5. CLI enum/picker/route를 연결한다.
6. planner 단위 테스트와 floating-base 동역학 회귀를 분리한다.
7. RL reference로 쓸 경우 position residual과 checkpoint version을 별도로
   설계한다.

### 15.3 계단 알고리즘 변경

1. riser/tread/nosing 가정과 geometry predicate를 명시한다.
2. hardcoded baseline을 유지해 같은 환경에서 비교한다.
3. state transition과 actuator target을 단위 테스트한다.
4. headless benchmark에서 성공 여부뿐 아니라 시간, work, upright, contact,
   assist 횟수를 비교한다.
5. viewer에서 조기 timeout, 전복, lateral escape를 확인한다.
6. 실패 후보도 `docs/11` 또는 `docs/12`에 남긴다.

### 15.4 reward 변경

1. [`06-reward-function-guide.md`](06-reward-function-guide.md)의 현재 수식을
   먼저 갱신한다.
2. 한 reward family만 변경한다.
3. environment check와 reward regression test를 실행한다.
4. old/new run을 분리하고 TensorBoard 항별 변화를 비교한다.
5. checkpoint 재개가 의도된 fine-tune인지 새 baseline인지 적는다.

### 15.5 기존 PPO가 이상하게 움직일 때

코드를 바로 바꾸기 전에 다음 순서로 재현한다.

1. ZIP 무결성 및 observation/action shape
2. 학습 당시 reference motion
3. standing pose
4. terrain/seed와 command
5. current motor profile/PID/physics
6. neutral gate 적용 여부
7. raw policy와 gated replay 비교

특히 학습 당시 `hardcoded` checkpoint에 `tripod-gait`나 `scone-gait`
reference를 선택하지 않는다. 반대 방향 reference와 residual이 합쳐져 보행이
상쇄되거나 뒤틀릴 수 있다.

### 15.6 simulation 기능을 실물로 옮길 때

1. simulation-only type check와 opt-in 경계를 유지한 새 experimental path를
   만든다.
2. actual joint hard stop, 방향, zero, velocity unit을 한 motor씩 측정한다.
3. 전류·온도·통신 timeout·낙상·비상 정지 제한을 구현한다.
4. 지지대에서 낮은 speed와 작은 stroke로 검증한다.
5. 한 tripod contact, 전체 support, 평지, 낮은 장애물 순으로 범위를 넓힌다.
6. MuJoCo 수치를 실물 보증값으로 복사하지 않는다.

---

## 16. 검증 명령과 완료 기준

### 16.1 정적·자동 검증

저장소 root에서 실행한다.

```bash
python -m compileall -q SCONE.py src tests
python -m unittest discover -s tests -v
```

영역별 빠른 테스트:

```bash
python -m unittest tests.test_api tests.test_actuators -v
python -m unittest tests.test_kinematics tests.test_tripod_gait -v
python -m unittest tests.test_scone_gait tests.test_scone_rolling_gait -v
python -m unittest tests.test_stair_geometry tests.test_stair_climber tests.test_stair_demo -v
python -m unittest tests.test_remote_watch tests.test_rl_inquiry tests.test_rl_joystick -v
```

RL environment smoke:

```bash
python -m src.rl.walk_learn \
  --terrain flat \
  --reference-motion tripod-gait \
  check --steps 500 --curriculum easy
```

계단 headless 비교:

```bash
python -m src.simulation.stair_benchmark --all --tuning
```

### 16.2 GUI 검증

macOS viewer는 main-thread 제약 때문에 `mjpython`으로 실행한다.

```bash
mjpython -m src.simulation --control tripod-gait --terrain flat
mjpython -m src.simulation --control scone-gait --terrain flat
mjpython -m src.simulation --control scone-stair --terrain stairs-3
mjpython -m src.simulation --demo compare --terrain stairs-3
```

자동 테스트 통과는 GUI 장시간 접촉 거동을 증명하지 않는다. gait/terrain/
controller를 바꿨다면 다음을 사람이 확인한다.

- 시작 자세에서 폭발적 transient가 없는가
- 명령과 실제 이동 방향이 같은가
- 앞으로 가는 동안 반복적으로 후퇴하지 않는가
- body yaw와 lateral drift가 누적되지 않는가
- 세 다리 지지 전환에서 차체가 과도하게 내려가지 않는가
- C자 opening이 동시에 접지해 지지가 사라지지 않는가
- 종료/Space/Q에서 lower velocity가 0이 되는가

### 16.3 문서 변경 완료 기준

기능을 바꾼 commit에는 최소한 다음을 남긴다.

- 변경한 config와 단위
- 적용 범위: 하드웨어/공용/simulation-only/RL-only
- checkpoint 호환 여부
- 실행한 테스트와 수치
- 실패한 시도와 기각 이유
- 남은 실물/GUI/장기 검증 한계

파일·변수 위치를 찾을 때는
[`05-file-and-folder-map.md`](05-file-and-folder-map.md)와
[`variables/README.md`](variables/README.md), 실행 명령은
[`07-running-testing-and-operations.md`](07-running-testing-and-operations.md)를
함께 사용한다.
