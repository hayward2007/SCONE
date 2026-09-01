# API·CLI·하드웨어·Locomotion 변수 사전

## 1. `src/main.py`

### enum과 상수

| 이름 | 값/형태 | 목적·사용처 |
|---|---|---|
| `RobotCommand.FORWARD/BACKWARD/LEFT/RIGHT` | 문자열 enum | CLI/외부 API의 이동 명령을 `SCONE` 메서드에 연결 |
| `RobotCommand.CHANGE_MODE` | `change_mode` | Walk→Drive→Climb→Walk 전환 |
| `RobotCommand.HOME` | `home` | `initialize()` 별칭 호출 |
| `RobotStatus.IDLE` | 상태 enum | 명령 대기 또는 초기화/동작 종료 상태 |
| `INITIALIZING`, `MOVING`, `SHUTTING_DOWN`, `CLOSED` | 상태 enum | 수명주기와 현재 작업을 외부에 표시 |
| `SCONE.STARTING_MIDDLE_POSITION` | `135°` | 초기화 초기에 중단 관절을 접어 안전 공간 확보 |
| `SCONE.ENDING_MIDDLE_POSITION` | `150°` | shutdown 전에 중단 관절을 더 안전한 종료 자세로 이동 |

### `SCONE` 상태

| 이름 | 목적·사용처 |
|---|---|
| `controller` | 실제 또는 MuJoCo `ControllerProtocol` 구현체 |
| `profile` | 현재 `MotionProfile`; 초기 자세와 모든 동작 속도의 원천 |
| `mode` | 현재 `Walk`, `Drive`, `Climb` 객체 |
| `status` | `RobotStatus`; UI/외부 호출자가 수명주기를 확인 |
| `initialized` | 안전 초기화가 끝나 motion 명령을 받을 수 있는지 표시 |
| `_closed` | backend가 이미 닫혀 재사용할 수 없는지 표시 |
| `_command_lock` | 초기화·동작·종료가 동시에 controller를 변경하지 못하게 하는 reentrant lock |
| `profile` 매개변수 | 문자열이면 `get_profile`, 객체면 그대로 사용 |
| `apply` | `set_profile()` 뒤 이미 초기화된 로봇을 새 자세로 다시 초기화할지 결정 |
| `movement` | 현재 mode에서 찾을 동작 메서드 이름 |

`motor_id`와 position/speed dictionary는 `Actuator.Index` 그룹을 실제 controller batch 호출로 변환하는 짧은 지역 상태다.

## 2. `src/cli.py`

### 키·조이스틱 상수

| 이름 | 값/목적 |
|---|---|
| `KEY_BINDINGS` | `w/s/a/d/r/h`를 legacy `RobotCommand`로 매핑 |
| `_JOYSTICK_KEY_BINDINGS` | `w/s`→`y`, `a/d`→`x`, 좌/우 화살표→`yaw`; 축 값은 `±1` |
| `_TERMINAL_KEY_SEQUENCES` | ANSI/SS3 화살표 escape byte를 정규화된 키 이름으로 매핑 |
| `JoystickLimits.max_vx` | 완전 전진 입력 `0.18 m/s` |
| `JoystickLimits.max_vy` | 완전 측면 입력 `0.12 m/s` |
| `JoystickLimits.max_yaw_rate` | 완전 회전 입력 `0.9 rad/s` |
| `JoystickState.x` | UI의 좌(-)/우(+) 축; body `vy`에는 부호를 반대로 적용 |
| `JoystickState.y` | UI의 뒤(-)/앞(+) 축; body `vx`로 변환 |
| `JoystickState.yaw` | 정규화 회전; 양수는 왼쪽 |

### 키보드와 제어 loop 상태

| 이름 | 목적·사용처 |
|---|---|
| `KeyboardJoystick.release_timeout` | key-up이 없는 terminal에서 반복 입력이 이 시간 끊기면 축을 0으로 복귀; 기본 `0.35 s` |
| `_values` | 현재 `x/y/yaw` 정규화 값 |
| `_deadlines` | 각 축을 언제 neutral로 되돌릴지 monotonic timestamp |
| `timestamp` | 테스트에서 주입 가능하도록 `now` 또는 `time.monotonic()` 사용 |
| `_JoystickTerminal`의 stream/fd/terminal state | raw terminal mode 진입과 원상 복구, nonblocking byte 입력 유지 |
| `period` | velocity joystick loop 주기 `1/50 s` |
| `pending_keys` | 분할되어 읽힌 escape sequence를 다음 frame까지 보존 |
| `last_frame` | 실제 `dt`와 pacing 계산 |
| `stop_event`/`stop` | viewer 종료나 외부 요청을 CLI loop와 공유 |
| `apply_command` | 생성된 `VelocityCommand`와 `dt`를 Non-RL/RL/legacy backend로 전달하는 callback |
| `profile_name`, `control_name`, `control_hint` | terminal HUD 설명용이며 제어 수치에는 직접 영향 없음 |

HUD의 `grid`, `point_column`, `point_row`, `yaw_column`, `yaw_bar`, `motion`은 현재 축 상태를 문자 UI 좌표와 ACTIVE/NEUTRAL 표기로 변환하는 지역변수다.

## 3. `src/hardware/actuator_index.py`

| 이름 | 값 | 목적·사용처 |
|---|---:|---|
| `ALL` | `1..18` | 전체 torque/speed/position batch |
| `UPPER` | `1..6` | 몸체쪽 첫 관절/MX 그룹 |
| `MIDDLE` | `7..12` | 중간 관절/XM430-W350 그룹 |
| `LOWER` | `13..18` | 말단 호형 바퀴/XM430-W210 그룹 |
| `XM` | `MIDDLE + LOWER` | Protocol 2.0 profile acceleration 등 XM 공통 처리 |
| `UPPER_RIGHT` | `(1, 3, 5)` | 오른쪽 상단 관절 |
| `UPPER_LEFT` | `(2, 4, 6)` | 왼쪽 상단 관절 |
| `MIDDLE_RIGHT/LEFT` | upper ID `+6` | 같은 쪽 중단 관절 |
| `LOWER_RIGHT/LEFT` | upper ID `+12` | 같은 쪽 하단 관절 |
| `UPPER_DIAGONAL_RIGHT` | `(1, 4, 5)` | tripod A의 상단 관절. 이름은 기존 코드 호환 |
| `UPPER_DIAGONAL_LEFT` | `(2, 3, 6)` | tripod B의 상단 관절 |
| `MIDDLE_DIAGONAL_*` | 각 upper tripod `+6` | hold/lift하는 중단 관절 |
| `LOWER_DIAGONAL_*` | 각 upper tripod `+12` | 들어 올리거나 wheel position을 바꾸는 하단 관절 |
| `for_leg(leg_number)` | `(leg, leg+6, leg+12)` | 다리 하나의 세 motor ID 반환; 1–6 검증 |

## 4. `src/hardware/actuator_control_table.py`

### 데이터 구조

| 필드 | 목적 |
|---|---|
| `Register.address` | control table의 시작 주소 |
| `Register.size` | read/write byte 수; SDK 메서드와 payload mask 선택 |
| `ControlTable.torque_enable` | torque on/off 주소 |
| `goal_position`, `present_position` | 목표/현재 위치 주소 |
| `moving_speed` | Protocol 1.0 MX 속도 주소; 지원하지 않으면 `None` |
| `operating_mode` | Protocol 2.0 mode 주소 |
| `goal_velocity`, `present_velocity` | velocity mode 목표/상태 주소 |
| `profile_velocity`, `profile_acceleration` | XM position profile 설정 주소 |
| `ActuatorModel.name` | 로그/오류용 모델 이름 |
| `model_number` | ping 결과와 모델 식별에 사용하는 Dynamixel model number |
| `protocol_version` | packet handler와 sync-write 그룹 선택 |
| `position_resolution` | 한 회전 count. 현재 세 모델 모두 4096 |
| `table` | 해당 모델의 `ControlTable` |

### 값

| 이름 | 주요 값/목적 |
|---|---|
| `OperatingMode.VELOCITY/POSITION/EXTENDED_POSITION` | `1/3/4`; XM operating-mode 값 |
| `MX28_AT` | model `29`, protocol `1.0`; torque 24, goal pos 30, speed 32, present pos 36 |
| `_XM430_TABLE` | mode 11, torque 64, goal vel 104, accel 108, profile vel 112, goal pos 116, present vel 128, present pos 132 |
| `XM430_W350T` | model `1020`, protocol `2.0`, XM table |
| `XM430_W210T` | model `1030`, protocol `2.0`, XM table |

## 5. `src/hardware/actuator.py`, `config.py`, `interface.py`

| 이름 | 값/목적 |
|---|---|
| `Torque.OFF/ON` | `0/1`; torque register 값 |
| `Position.START/CENTER/END` | `0/2048/4096`; raw 위치 경계/중앙/한 회전 |
| `Model.*` | control-table의 actuator model을 과거 namespace 방식으로 재노출 |
| `Actuator.Index/Model/OperatingMode/Position/Torque` | 상위 코드가 한 namespace에서 하드웨어 정의에 접근하는 호환 facade |
| `model_for_id(motor_id)` | 1–6 MX, 7–12 W350, 13–18 W210 선택; 범위 검증 지점 |
| `DEFAULT_BAUDRATE` | `1_000_000`; 실제 bus 통신 속도 |
| `DEFAULT_DEVICE_NAME` | `/dev/cu.usbserial-FTBIHSYW`; 환경 변수/탐색이 없을 때 기본 포트 |
| `ControllerProtocol` 메서드 매개변수 | `motor_id`, ID→값 mapping, ID iterable을 실제/시뮬 controller 양쪽이 같은 의미로 구현 |

## 6. `src/hardware/controller.py`

| 상태/상수 | 목적·사용처 |
|---|---|
| `DEFAULT_BAUDRATE`, `DEFAULT_DEVICE_NAME` | config 값을 class public default로 재노출 |
| `device_name` | 인수 → `SCONE_DEVICE` 환경 변수 → 기본 포트 순서로 결정 |
| `baudrate` | `PortHandler.setBaudRate()`에 전달 |
| `verbose` | `[HARDWARE]` 로그 출력 여부 |
| `_closed` | 중복 close와 닫힌 controller 사용 상태 추적 |
| `_port_open` | 실제 포트가 열렸을 때만 close하도록 추적 |
| `port_handler` | Dynamixel SDK serial port 객체 |
| `_packet_handlers` | `{1.0: PacketHandler, 2.0: PacketHandler}`; 모델 protocol별 packet 생성 |
| `grouped` | `(protocol_version, Register)`별 `{motor_id: value}`; 서로 다른 표를 한 sync write에 섞지 않음 |
| `mask` | register byte 길이에 맞춰 signed/oversized Python int의 전송 하위 bit 선택 |
| `payload` | little-endian sync-write bytes |
| `mx`, `xm` | `set_speeds()`에서 moving-speed와 profile-velocity 쓰기를 분리한 mapping |
| `supported` | acceleration register가 있는 motor만 남긴 mapping |
| `wait_until_raw_positions()` | present-position을 20 ms 간격으로 읽어 모든 목표가 tolerance 안에 들 때까지 대기; timeout이면 `False` |
| `verify_drive_stage1_settings()` | ID 7–12의 mode/torque/profile velocity/profile acceleration/goal/present를 read-only 검증 |
| `readings` | stage-1 ID별 live register dictionary |
| `failures` | 기대값과 다른 register/position 메시지; 하나라도 있으면 `ControllerError` |

`set_mode()`는 torque off→register write→torque on 순서다. `degrees_to_raw()`는 `degree/360×4096`으로 변환하며 기계적 범위 제한은 이 계층이 하지 않는다.
초기화는 ID 7–18 모든 XM을 position mode로 명시한다. Drive stage-1 기대값은
position mode 3, torque 1, velocity 50, acceleration 20, goal/present 2048이다.

## 7. `src/hardware/discovery.py`

| 이름 | 목적·사용처 |
|---|---|
| `HardwareProbe.available` | 후보 중 하나에서 정상 Dynamixel 응답을 받았는지 여부 |
| `HardwareProbe.device_name` | 응답에 성공한 serial 경로; 실패 시 `None` |
| `HardwareProbe.detail` | 성공 설명 또는 SDK/import/open/baud/ping/close 실패 원인 문자열 |
| candidate device 목록 | `SCONE_DEVICE`, macOS/Linux USB glob, 기본 경로를 중복 제거한 순서 |
| 대표 ID `(1, 7, 13)` | Protocol 1.0/2.0 handler로 순서대로 ping하며 그중 하나라도 정상 응답하면 SCONE bus 후보로 판정 |
| `mutate=False` 성격 | discovery는 mode/torque/position을 쓰지 않아 기존 자세를 보존 |

## 8. `src/locomotion/profile.py`

| `MotionProfile` 필드 | 목적 |
|---|---|
| `name` | CLI 선택과 표시, `PROFILES` key |
| `upper_initial_position` | ID 1–6의 각 초기 degree |
| `middle_initial_position` | ID 7–12 공통 초기 degree |
| `lower_initial_position` | ID 13–18 공통 초기 degree |
| `boost_speed` | 하단 관절의 큰 전환 동작 속도 |
| `safety_speed` | 초기화/종료/자세 준비의 보수적 속도 |
| `walking_speed` | legacy walk 속도 |
| `driving_speed` | wheel velocity 명령 크기 |
| `climbing_speed` | climb wheel velocity 명령 크기 |

| preset | 자세/속도 차이 |
|---|---|
| `STANDARD` | upper `(135,135,180,180,225,225)`, middle `240`, lower `255`, climb speed `200` |
| `SPORT` | 같은 upper, middle `170`, lower `195`, climb speed `100` |
| `PROFILES` | profile name→immutable object lookup |

## 9. `src/locomotion/mode.py`, `walk.py`, `drive.py`, `climb.py`

| 이름 | 목적·사용처 |
|---|---|
| `Mode.name` | base 표시 이름 `mode`; subclass가 `walk/drive/climb`으로 override |
| `Mode.controller` | 공통 backend |
| `Mode.profile` | 현재 자세/속도 preset |
| `Walk.MOVING_DEGREES` | `20°`; legacy hold와 upper swing offset |
| `transition` | Drive/Climb에서 Walk로 돌아올 때 `_enter_walk_pose()` 수행 여부 |
| `ids`, `value` | `_positions()`가 ID tuple과 scalar/callable 목표를 batch mapping으로 변환 |
| `first_tripod` | 회전 동작에서 먼저 hold할 tripod 선택 |
| `first_is_left`, `hold_*`, `release_*` | 선택된 tripod 순서에 맞춰 bound method를 구성 |
| `sign` | 홀짝 다리의 거울상 회전 offset 부호 |
| `Drive._run(velocity)` | 모든 lower motor에 1초간 적용할 raw velocity; `left` 음수, `right` 양수 |
| Drive stage-1 verifier | physical backend가 제공할 때 ID 7–12 live register를 확인; MuJoCo에는 적용하지 않음 |
| `Climb.middle_ids/lower_ids` | 준비하거나 측면 자세로 만들 tripod의 관절 그룹 |
| `Climb.velocity` | 2.5초 wheel drive 속도; 방향과 profile에 의해 결정 |

## 10. `src/locomotion/legacy_velocity.py`

| 이름 | 목적·사용처 |
|---|---|
| `command` | `VelocityCommand`; yaw 우선, 그다음 vx 부호로 legacy motion 선택. 순수 vy는 지원하지 않아 `None` |
| `robot` | `forward/backward/left/right`를 가진 `LegacyMotionRobot` |
| `_command` | 다음 stride에 사용할 가장 최신 명령 |
| `_lock` | `_command` 읽기/교체 보호 |
| `_updated` | 새 명령 또는 종료가 worker를 깨우는 event |
| `_stop` | worker 종료 요청 |
| `_error` | worker 예외를 호출 thread로 다시 전달 |
| `_worker` | blocking legacy stride를 terminal/viewer와 분리하는 daemon thread |

## 11. `src/locomotion/tripod_gait.py`와 `scone_gait.py`

### 데이터 클래스와 상수

| 이름 | 기본값/목적 |
|---|---|
| `VelocityCommand.vx/vy/yaw_rate` | body 명령 3축; `from_array`는 shape `(3,)` 강제 |
| `GaitConfig.control_frequency` | `50 Hz`; frame 생성/전송 속도 |
| `cycle_frequency` | 공용 기본 `0.8 Hz`; 비-RL MuJoCo 조종 `0.8 Hz`, RL tripod reference만 `0.7 Hz` |
| `duty_factor` | `0.5`; 한 다리가 stance에 있는 cycle 비율 |
| `step_height` | `0.035 m`; swing lift 높이 |
| `max_stride` | `0.070 m`; 전후 발 궤적 한계. 비-RL MuJoCo 조종은 `0.080 m`, RL reference는 `0.060 m` |
| `max_lateral_stride` | 기본 `None`이면 `max_stride`와 동일. 비-RL MuJoCo 조종은 `0.060 m`, RL reference는 `0.050 m` |
| `max_vx/max_vy/max_yaw_rate` | `0.18/0.12/0.9`; 명령 clamp |
| `command_time_constant` | `0.15 s`; low-pass command 응답 시간 |
| `idle_epsilon` | `1e-3`; 명령을 정지로 판단하는 기준 |
| `ik_tolerance` | 기본 `1e-4 m`; IK 수렴 residual. 시뮬레이션/RL은 계산량을 줄이기 위해 `1e-3 m` |
| `ik_max_iterations` | `80`; 다리별 최대 반복 |
| `ik_damping` | `2e-3`; DLS 안정화 |
| `ik_max_step` | `0.15 rad`; 한 IK update 제한 |
| `ik_stride_backoff_attempts` | 기본 `0`; 시뮬레이션/RL은 실패한 foot target을 최대 4회 축소 재시도 |
| `ik_stride_backoff_factor` | `0.8`; 재시도마다 nominal 대비 발 오프셋 배율 |
| `GaitSample.phase` | 생성 frame의 global phase |
| `command` | 필터된 실제 적용 명령 |
| `foot_targets` | 6×3 body-frame 발 목표 |
| `motor_degrees` | actuator ID 순서의 18개 목표 degree |
| `ik_results` | 다리 번호→수렴/잔차/반복 결과 |
| `stance_legs` | 해당 frame에 지면을 지지하는 다리 tuple |
| `cycle_frequency` | 해당 frame에 실제 적용된 cadence |
| `stride_clip_fraction` | 타원형 작업공간에 의해 잘린 다리 비율 |
| `ik_backoff_scale` | IK 재시도 후 실제 foot offset 배율, 미사용 시 `1.0` |
| `TRIPOD_A/B` | `(1,4,5)` / `(2,3,6)` |
| `PHASE_OFFSETS` | A 다리 `0.0`, B 다리 `0.5`로 반 cycle 분리 |

### `TripodGait` 상태

| 이름 | 목적·사용처 |
|---|---|
| `controller` | 선택 사항; offline sample만 만들거나 실제 batch 전송에 사용 |
| `profile` | nominal 18 motor 자세 생성 |
| `config` | gait/IK/안전 제한 |
| `_nominal_motor_degrees` | 현재 gait 중심 자세, actuator 순서 18개 |
| `_nominal_angles` | 위 자세를 `(degree-180)` 후 radian으로 변환한 joint 중심 |
| `kinematics` | support point를 end effector로 쓰는 `RobotKinematics` |
| `_phase` | 현재 `[0,1)` gait phase |
| `_filtered_command` | low-pass 이후 `[vx,vy,yaw]` |
| `_last_update_time` | `update()`에서 실제 `dt` 계산; reset 시 `None` |
| `_last_angles` | IK 실패 시 유지할 마지막 유효 18 joint radian |
| `_nominal_feet` | nominal 자세의 6×3 body-frame support point |
| `_last_cycle_frequency` | 최근 sample에 적용한 고정 cadence |
| `_last_stride_clip_fraction` | 최근 sample의 작업공간 포화 다리 비율 |
| `end_effector_points` | caller calibration 또는 각 tire mesh 말단 최저 0.1 mm 패치 중심의 local 좌표 |
| `SUPPORT_PATCH_DEPTH` | `1e-4 m`; 부채꼴 말단 접촉 폭 중심을 구하는 최저 패치 두께 |
| `world_from_body/world_from_geom/world_from_tire` | support patch를 world↔body↔tire local 좌표로 바꾸는 회전행렬 |
| `point_velocity` | body translation에 yaw 접선 `[-ωy, ωx]`를 더한 각 발의 목표 지면 속도 |

`SconeGaitConfig`는 bounded 학습 reference 설정과 함께
`continuous_rotation`, `point_support_ratio`, `swing_roll_hold_ratio`,
`effective_roll_radius`, `max_roll_rate_degrees`를 추가한다. `SconeGait`는 각 말단
mesh의 접선/극성을 자세마다 보정하고, interactive 고속 route에서는
`_continuous_roll_degrees`를 누적해 IK 2단 움직임과 실제 다회전을 합성한다.
비-RL simulation의 continuous `SconeRollingGait` 변수는
[`02-kinematics-simulation-terrain.md`](02-kinematics-simulation-terrain.md)에
분리해 기록한다.
이전 `non_rl_walk.py`와 `NonRLWalk`는 `TripodGait` 호환 별칭이다.
| `stride` | 속도와 stance 시간을 곱하고 전후/측면 타원형 작업공간으로 제한한 이동 벡터 |
| `ik_backoff_scale` | 실패 target을 nominal 쪽으로 축소하는 누적 배율 |
| `stance_progress/swing_progress` | 각 구간 내부의 0–1 보간 위치 |
| `blend` | endpoint에서 속도/가속도가 부드러운 quintic 보간값 |
| `lift` | swing 중 z축 추가 높이 |
| `positions` | 유효한 18 motor degree를 ID→값으로 만든 batch payload |
