# 기구학·시뮬레이션·지형 변수 사전

## 1. `src/kinematics/types.py`

| 이름 | 목적·사용처 |
|---|---|
| `Vector3` | shape `(3,)`, `float64` 벡터 타입 alias |
| `Matrix3` | shape `(3,3)`, `float64` 회전행렬 alias |
| `vector3(value, name)` | 배열을 정확히 3개 값으로 검증; `name`은 오류 문구 |
| `JointAngles.body` | motor ID `leg`의 관절 radian, raw 2048을 0으로 정의 |
| `JointAngles.stage1` | motor ID `leg+6`의 관절 radian |
| `JointAngles.stage2` | motor ID `leg+12`의 관절 radian |
| `motor_degrees` | `(radian→degree)+180`; controller의 degree 좌표 |
| `raw` | `degree/360×4096`; 정수로 반올림 |
| `EndEffectorPose.position` | 선택 frame에서 타이어 end-effector 위치 |
| `EndEffectorPose.rotation` | 선택 frame에서 타이어 회전행렬 |
| `EndEffectorPose.frame` | `body` 또는 `world` |
| `IKResult.angles` | 가장 좋은/수렴한 관절각 |
| `IKResult.residual` | target과 결과 위치 사이 거리(m) |
| `IKResult.iterations` | 사용한 반복 수 |
| `IKResult.converged` | residual이 tolerance 이하인지 여부 |
| `IKConvergenceError.leg/result` | 실패한 다리 번호와 반환될 수 있었던 best result |

## 2. `src/kinematics/leg.py`

### 상수와 모델 매핑 상태

| 이름 | 목적·사용처 |
|---|---|
| `Frame` | 허용 좌표계를 `Literal["body", "world"]`로 제한 |
| `DEFAULT_MODEL_PATH` | `src/assets/model.xml` |
| `leg` | 1–6 다리 번호; motor/joint/tire 이름 구성 |
| `model_path` | MJCF 경로. 저장된 절대 경로로 정규화 |
| `model`, `data` | 독립 생성하거나 `RobotKinematics`에서 공유하는 MuJoCo 상태 |
| `end_effector_point` | `TIRE_<leg>` body local 좌표의 실제 계산점; 기본 원점, gait는 최저 접촉점 주입 |
| `motor_ids` | `(leg, leg+6, leg+12)` |
| `joint_names` | `Mxx_body_Ln`, `Mxx_stage1_Ln`, `Mxx_stage2_Ln` |
| `joint_ids` | 위 이름의 MuJoCo joint ID 3개 |
| `qpos_addresses` | 세 관절의 generalized position 주소 |
| `dof_addresses` | 세 관절의 generalized velocity/Jacobian column 주소 |
| `end_effector_body_id` | `TIRE_<leg>` body ID |
| `root_body_id` | `root_freejoint`가 연결된 robot root body |
| `_jacobian_position` | `mj_jac`가 채우는 3×`nv` translational buffer |
| `_jacobian_rotation` | `mj_jac`가 요구하는 3×`nv` rotational buffer; 현재 IK 결과에는 미사용 |

### FK/IK 매개변수와 중간값

| 이름 | 기본값/목적 |
|---|---|
| `frame` | `body`; world pose 또는 root-relative pose 선택 |
| `target_position`/`target` | IK가 맞출 3D 점 |
| `initial_angles` | `(0,0,0)`; IK branch와 수렴 경로를 정하므로 실사용은 현재 각도 권장 |
| `tolerance` | `1e-5 m`; 수렴 판정 |
| `max_iterations` | `100`; 최대 update 횟수 |
| `damping` | `1e-3`; `J Jᵀ + λ²I`의 λ |
| `max_step` | `0.25 rad`; update norm 제한 |
| `joint_lower/upper` | 기본 각 관절 `-π/+π`; 실제 기계 한계로 교체 가능 |
| `raise_on_failure` | `False`; 실패 result 반환 대신 예외를 던질지 결정 |
| `angles` | 현재 반복의 clip된 3관절 벡터 |
| `best_angles`, `best_residual` | 중간에 악화/중단되어도 가장 가까운 결과 보존 |
| `pose`, `error`, `residual` | 현재 FK, target-position 차, L2 거리 |
| `jacobian` | 선택 frame의 3×3 translational Jacobian |
| `regularized` | DLS의 `J Jᵀ + damping² I` |
| `delta`, `delta_norm` | 제안 관절 update와 크기 |
| `step_scale` | backtracking 배율, 실패할 때마다 `0.5` |
| `candidate`, `candidate_error` | joint limit로 자른 시험 자세와 오차 |

## 3. `src/kinematics/robot.py`

| 이름 | 목적·사용처 |
|---|---|
| `LegAngleInput` | 다리 mapping, `(6,3)`, actuator-order `(18,)` 입력 union |
| `LegTargetInput` | 다리 mapping 또는 `(6,3)` 발 목표 |
| `model_path`, `model`, `data` | 여섯 `LegKinematics`가 공유하는 하나의 MJCF/상태 |
| `end_effector_points`/`points` | 다리별 tire local 계산점; 없는 다리는 `(0,0,0)` |
| `legs` | `{1..6: LegKinematics}` |
| `missing` | mapping 입력에서 빠진 다리 집합; 오류 검증용 |
| `by_leg`, `target_by_leg`, `initial_by_leg` | 다양한 입력 shape를 다리별 타입으로 정규화한 지역 mapping |
| `solver_options` | 각 다리 `inverse()`로 전달하는 tolerance/damping 등 keyword |
| `output` | IK 결과를 ID 1–18 순서로 재배열한 배열 |
| `SCONEKinematics` | `RobotKinematics`의 호환 alias |

18개 입력에서 index `leg-1`, `leg+5`, `leg+11`은 각각 upper/middle/lower motor ID 위치다.

## 4. `src/simulation/core/model.py`

| 이름 | 목적·사용처 |
|---|---|
| `DEFAULT_MODEL_PATH` | 시뮬레이션 기본 MJCF |
| `assets` | XML mesh `file` 이름→STL bytes; `from_xml_string` compile에 전달 |
| `fixed_model` | 원본 MJCF로 먼저 compile한 모델; contact mesh 최저점 계산 |
| `data` | floor 계산용 초기 MuJoCo 상태 |
| `lowest` | 모든 contact mesh world vertex 중 최저 z |
| `floor_z` | `lowest-0.001 m` 또는 기존 `simulation_floor` z |
| `model_path/path` | 확장·정규화한 MJCF 위치 |
| `floating_base` | `True`면 `root_freejoint` 삽입, `False`면 freejoint 제거 |
| `terrain/selected_terrain` | `TerrainType.parse()`로 정규화한 지형 |
| `terrain_seed` | procedural rough terrain 재현성, 기본 `7` |
| `root`, `worldbody`, `root_body` | 수정 중인 XML element |
| `floor` | 이름이 `simulation_floor`인 plane geom |
| `position` | 기존 floor `pos` 세 값 검증 |
| `xml` | 수정한 MJCF 문자열 |
| `TerrainBuildResult result` | inspectable XML과 함께 반환하는 지형 범위/높이 metadata |

새 floor의 `size="3 3 0.1"`, `friction="1.0 0.005 0.0005"`, `condim="6"`는 접촉 영역과 마찰 모델을 정한다.

## 5. `src/simulation/core/pid.py`

### motor 사양

| `DCMotorSpec` 필드 | 목적 |
|---|---|
| `voltage` | 공급/포화 전압, 현재 12 V |
| `stall_torque` | 정지 시 최대 torque |
| `no_load_speed` | 무부하 각속도(rad/s) |
| `continuous_torque` | controller torque 요구 제한과 XML saturation 기준 |
| `K` | `voltage/no_load_speed`; torque/back-EMF 상수 |
| `R` | `K×voltage/stall_torque`; terminal resistance |

| 사양 | 값 |
|---|---|
| `MX28AT` | 12 V, 2.5 N·m, 55 rpm, continuous 2.5 N·m |
| `XM430_W350` | 12 V, 4.1 N·m, 46 rpm, continuous 4.1 N·m |
| `XM430_W210` | 12 V, 3.0 N·m, 77 rpm, continuous 3.0 N·m |
| `_DEFAULT_GAINS["mx28at"]` | `kp=5.73`, `kd=0.752` |
| `_DEFAULT_GAINS["xm430_w350"]` | `kp=9.40`, `kd=0.792` |
| `_DEFAULT_GAINS["xm430_w210"]` | `kp=6.88`, `kd=0.264` |

XML의 `<dcmotor nominal>`과 Python 사양은 함께 바꿔야 한다.

### `DCMotorPID` 상태와 step 변수

| 이름 | 목적·사용처 |
|---|---|
| `spec` | 해당 motor의 전기·기계 상수 |
| `kp/kd/ki` | position/velocity/integral torque gain |
| `integral_limit` | integral windup 제한; 기본 continuous torque 값 |
| `slew_max` | 내부 target 이동 속도 제한; 기본 무한대 |
| `_integral` | 누적 position error |
| `_slewed_target` | slew limit 적용 이후 target position |
| `dt` | physics step |
| `position`, `velocity` | 현재 joint 상태 |
| `target_position`, `target_velocity` | profile generator의 현재 setpoint |
| `feedforward_torque` | 모델 기반 선행 torque, 기본 0 |
| `position_error`, `velocity_error` | PD/PID 입력 |
| `torque` | gain 합산 torque 요구 |
| `voltage` | `(R/K)×torque + K×velocity`, 이후 `±spec.voltage` clip |

## 6. `src/simulation/core/controller.py`

### 단위/모드 상수

| 이름 | 값/목적 |
|---|---|
| `_POSITION_MODE` | Dynamixel mode `3` |
| `_EXTENDED_POSITION_MODE` | mode `4` |
| `_VELOCITY_MODE` | mode `1` |
| `_CENTER_RAW` | `2048`; 0 rad 기준 |
| `_RAW_PER_REVOLUTION` | `4096` |
| `_MX_SPEED_UNIT_RPM` | raw speed 1당 `0.114 rpm` |
| `_XM_SPEED_UNIT_RPM` | raw speed 1당 `0.229 rpm` |
| `_XM_ACCELERATION_UNIT_RPM_PER_MINUTE` | XM profile acceleration unit `214.577 rev/min²` 환산 기준 |
| `_STANDING_UPPER_DEGREES` | `(135,135,180,180,225,225)` |
| `_STANDING_MIDDLE_DEGREES` | `240` |
| `_STANDING_LOWER_DEGREES` | `255` |
| `_STANDING_POSE_DEGREES` | 위 세 그룹을 합친 18개 startup seed |

### controller 상태 배열

배열은 ID를 그대로 index로 쓰기 위해 길이 19이며 index 0은 사용하지 않는다.

| 이름 | 목적·사용처 |
|---|---|
| `model`, `data` | 공유 MuJoCo model/state |
| `verbose` | 시뮬 controller 로그 |
| `lock` | API thread와 physics update의 상태 경쟁 방지 |
| `_actuator_ids` | motor ID→MuJoCo actuator ID |
| `_joint_ids` | motor ID→연결 joint ID |
| `_qpos_addresses` | motor ID→qpos address |
| `_dof_addresses` | motor ID→qvel address |
| `_pid` | motor ID→`DCMotorPID` |
| `_default_pid_gains` | motor ID별 원래 `(kp,kd)`; model-gait opt-in stiffness 계산 기준 |
| `_stage1_default_kd` | ID 7–12의 원래 `kd`; Drive 종료 시 복구 기준 |
| `_drive_stage1_damping_enabled` | MuJoCo Drive 댐핑 2배 적용 상태 |
| `_torque_enabled` | motor별 전압 출력 활성 여부 |
| `_mode` | motor별 position/velocity mode |
| `_target` | 최종 position target(rad) |
| `_setpoint` | profile velocity/acceleration을 적용해 현재까지 이동한 target(rad) |
| `_setpoint_velocity` | profile generator의 현재 속도(rad/s) |
| `_velocity_command` | velocity mode의 목표 속도(rad/s) |
| `_profile_velocity` | position target 추종 최대 속도; raw 0은 `inf` |
| `_profile_acceleration` | setpoint 속도 변화 제한; raw 0은 `inf` |
| `seed_pose` | caller stance 또는 기본 18개 startup degree |

### startup/update 중간값

| 이름 | 목적·사용처 |
|---|---|
| `prefix` | `A{motor_id:02d}_`; 정확히 한 actuator 이름을 찾는 기준 |
| `freejoint_id`, `qpos_adr` | floating root 높이를 조정할 freejoint와 qpos 시작점 |
| `floor_z`, `lowest`, `clearance` | 최저 contact를 floor+`0.002 m`에 두는 seed 보정 |
| `requested_velocity` | mode에 따라 velocity command 또는 position error/dt |
| `max_velocity` | profile velocity clip |
| `acceleration` | profile acceleration limit |
| `velocity_delta`, `max_velocity_delta` | 한 timestep의 setpoint 속도 변화 |
| `velocity`, `step` | 새 setpoint 속도와 위치 증가량 |
| `remaining`, `reaches_target` | position mode에서 target overshoot 방지 |
| `position`, `actual_velocity` | PID에 넣는 실제 joint state |
| `_DRIVE_STAGE1_DAMPING_MULTIPLIER` | `2.0`; Drive 중 ID 7–12에만 적용하는 시뮬레이션 보정 |
| `arc_wheel_velocities()` | 홀수 말단 ID는 입력 부호, 짝수 ID는 반대 부호로 매핑 |
| `wait_until_raw_positions()` | mode 전환 목표가 tolerance 안에 들 때까지 시뮬레이션 상태 확인 |
| `set_gait_position_stiffness(multiplier)` | 기본 ID 7–12의 `kp`를 배수, `kd`를 배수 제곱근으로 조정; 허용 0.5–4.0, torque cap 불변 |

`degrees_to_raw()`의 `motor_id`는 API 호환 때문에 남아 있지만 현재 변환 방향은 홀짝 ID에 따라 바뀌지 않는다.

## 7. `src/simulation/core/cli_bridge.py`, `simulator_cli.py`, `viewer.py`

| 이름 | 목적·사용처 |
|---|---|
| `TRIPOD_GAIT_SIMULATION_CONFIG` | cadence `1.0 Hz`, lift `0.025 m`, 전후/측면 stride `0.090/0.070 m`, IK 허용오차 `1 mm`, backoff 4회의 비-RL 조종 설정 |
| `SCONE_GAIT_SIMULATION_CONFIG` | RL/position 호환 bounded sector sweep 설정 |
| `SCONE_ROLLING_GAIT_SIMULATION_CONFIG` | lower 연속 회전 비-RL `scone-gait` 설정 |
| `configure_model_gait_controller()` | profile velocity/acceleration 0(무제한), middle stiffness 2배를 비-RL tripod에 opt-in |
| `SimulationControl.OLD/TRIPOD_GAIT/SCONE_GAIT/SCONE_STAIR/RL` | 제어 방식 선택 enum; `NON_RL`은 `TRIPOD_GAIT` 호환 별칭 |
| `profile` | Standard/Sport |
| `floating_base` | 고정형 기구 검사와 실제 동역학 실행 선택 |
| `terrain`, `terrain_seed` | 지형 preset과 난수 seed |
| `control` | old/tripod-gait/scone-gait/rl route |
| `checkpoint`, `rl_device` | RL route의 PPO ZIP과 CPU/CUDA 선택 |
| `rl_reference_motion` | `tripod-gait/scone-gait/hardcoded`; RL residual 기준 모션 |
| `rl_standing_pose_degrees` | RL 환경/controller startup의 18개 stance |
| `stop_event` | CLI worker와 viewer main loop 종료 공유 |
| `cli_errors` | worker 예외를 main thread에서 다시 발생시키는 목록 |
| `worker` | blocking terminal CLI thread |
| `tracking_body_id` | viewer가 따라갈 root body |
| `viewer.opt.geomgroup[0]` | 지형/collision group 0 표시 활성화 |
| `viewer.cam.distance` | `extent×2.2`를 2.2–3.0 사이로 제한 |
| `azimuth/elevation` | `135°/-30°` 초기 시점 |
| `demo` | `hardcoded/improved/compare`; terminal joystick 없는 stair viewer route |

### `SconeRollingGaitConfig`와 자동 데모

| 이름 | 기본값/목적 |
|---|---|
| `roll_velocity` | `175`; lower 연속 회전 raw velocity scale |
| `support_velocity_ratio` | `0.80`; stance tripod의 lower 속도 비율 |
| `tripod_b_phase_offset_degrees` | `60°`; B `(2,3,6)`의 C자 개구 시작 phase |
| `basic_velocity_time_constant` | `0.04 s`; lower 기본 보행 미분 속도 low-pass |
| `basic_lower_motion_blend` | `0.35`; lower 기본 보행 속도 성분의 합성 비율 |
| `max_basic_lower_velocity` | `80`; IK branch 변화 시 lower 기본 속도 상한 |
| `velocity_time_constant` | `0.10 s`; lower 속도 low-pass |
| `profile_velocity/profile_acceleration` | `160/50`; upper/middle 기본 보행 position profile |
| `middle_stiffness_multiplier` | `2.0`; ID 7–12 hold 보정 |
| `cycle_frequency/duty_factor` | `0.8 Hz/0.58`; full-body 기본 보행 phase |
| `step_height/max_stride` | `0.020/0.055 m`; full-body 기본 보행 lift/전후 stroke |
| `max_steering_degrees/steering_blend` | `45°/0.20` |
| `_filtered_roll_velocity` | 여섯 lower의 연속 회전 filter 출력 |
| `_filtered_basic_velocity` | lower 기본 보행 offset 미분의 filter 출력 |
| `_previous_lower_offset` | 다음 frame 미분을 위한 이전 lower 기본 보행 offset |
| `_active` | phase 준비 뒤 velocity mode가 활성화됐는지 여부 |
| `StairDemoStrategy` | `hardcoded`, `improved`, `compare` |
| `HardcodedStairRoller.velocity` | `200`; 1회 동기화 뒤 feedback 없는 계단 baseline |
| `timeout_seconds` | 기본 `16 s`; 자동 상단 판정 최대 시간 |
| `time_to_top_seconds` | root Y/Z가 상단 조건을 처음 동시에 만족한 시각 |

## 8. `src/locomotion/stair_geometry.py`, `src/simulation/core/stair_climber.py`

### 부채꼴 geometry와 분석 함수

| 이름 | 기본값/목적 |
|---|---|
| `SCONE_V2_ARC_WHEEL.inner_radius` | `0.1125 m`; 도면과 `TIRE.stl`의 안쪽 반경 |
| `outer_radius` | `0.1225 m`; 보수적 riser reach와 edge pivot 계산 반경 |
| `width` | `0.044 m`; 축방향 TPU 폭 |
| `occupied_arc_degrees` | `225°`; 현재 contact mesh에서 계산한 점유 호각 |
| `opening_degrees` | `135°`; `360° - occupied_arc_degrees` |
| `opening_chord` | 약 `0.22635 m`; 바깥 반경 양 끝 사이 chord |
| `edge_in_radial_band()` | 계단 모서리 거리가 annular TPU band에 들어오는지 검사 |
| `conservative_riser_limit()` | `R - nosing_radius - clearance` |
| `wheel_edge_offset()` | `sqrt(2Rh-h²)` sharp-edge moment arm |
| `quasi_static_pivot_torque()` | 하중·안전율·효율을 포함한 이상적 edge pivot 토크 |
| `required_friction_coefficient()` | `abs(F_t)/F_n` Coulomb 요구량 |
| `legged_wheel_opening_ratio()` | opening chord / 계단 rise-tread 대각선; 비교용이지 성공 판정은 아님 |
| `support_polygon_margin()` | CoM 투영점과 convex contact hull 경계의 부호 있는 최소 거리 |

### `SconeStairConfig`와 상태

| 이름 | 기본값/목적 |
|---|---|
| `StairControlState` | `idle`, `synchronizing`, `climbing` |
| `max_vy` | `0.12 m/s`; A/D 명령 scale |
| `synchronized_phase_degrees` | `60°`; 100/150 mm 시작 공통 위상 |
| `tall_synchronized_phase_degrees` | `90°`; 200 mm 시작 공통 위상 |
| `neutral/medium/tall_front_stage1_degrees` | `180/184/195°`; improved 앞쪽 1단 `(7,9,11)` 높이별 지지 자세 |
| `legacy_front_stage1_degrees` | `270°`; hardcoded 옛 수직 자세 재현값 |
| `front_stage1_profile_velocity` | `100`; 앞쪽 1단 자세 획득 profile |
| `front_stage1_tolerance_raw/sync_timeout` | `256/4.0 s`; 하중 지지 관절의 준비 허용오차와 timeout |
| `phase_velocity` | `200`; 150/200 mm 공통 위상 속도, DYNAMIXEL velocity 수치 단위 |
| `easy_phase_velocity` | `250`; 100 mm에서 측정상 더 빠르고 일이 적었던 속도 |
| `easy_rise_limit/tall_rise_threshold` | `0.125/0.175 m`; 위 phase/speed 조합 선택 경계 |
| `profile_velocity/profile_acceleration` | `240/80`; extended-position setpoint profile |
| `phase_tolerance_raw/phase_sync_timeout` | `96/4.0 s`; 시작 위상 획득 판정 |
| `phase_degrees` | wrap하지 않는 현재 공통 위상 `θ`; 다회전 누적 |
| `selected_phase_velocity` | 현재 preset rise로 선택한 위상 속도 |
| `phase_sync_entries` | 시작 위상 정렬 횟수 |
| `front_stage1_sync_entries` | 앞쪽 1단 지지 자세 획득 횟수 |
| `maximum_phase_spread_degrees` | 접촉 하중을 포함한 여섯 실제 기하 위상의 최대 순간 spread |

이 값은 현재 MuJoCo preset 튜닝값이며 실물 모터·마찰 안전 한계가 아니다.

## 9. `src/simulation/terrain/types.py`, `presets.py`

| 이름 | 값/목적 |
|---|---|
| `TerrainType` | `flat`, `uneven`, `stairs-1..3`, `slope-1..3`, `mixed` |
| `TERRAIN_CHOICES` | 모든 `TerrainType.value` tuple; argparse `choices`에 재사용 |
| `StairProfile.rises` | 각 step의 증가 높이; 절대 높이가 아님 |
| `tread_depths` | 각 step 진행방향 깊이 |
| `widths` | 각 step 폭 |
| `landing_length` | 정상부 길이 |
| `total_height` | `sum(rises)` |
| `SlopeProfile.angle_degrees` | 0–45° 사이 경사각 |
| `length`, `width` | ramp 진행 길이와 폭 |
| `landing_length` | 정상 평면 길이 |
| `thickness` | box ramp 두께, 기본 `0.06 m` |
| `TerrainBuildResult.terrain` | 실제 생성 preset |
| `geom_names` | 추가된 모든 geom 이름 |
| `start_y/end_y` | 코스 world-y 범위 |
| `max_height` | floor 위 최대 장애물 높이 |
| `STAIR_PRESETS` | 물리 단 rise 난도 1 `.10×3`, 난도 2 `.15×3`, 난도 3 `.20×3 m`; 난도 3은 staged hook 지지를 위해 tread `.35×3 m` |
| `SLOPE_PRESETS` | `8°/15°/25°`, length `1.4/1.2/1.0 m`, width `.9/1.0/1.1 m` |
| `TERRAIN_LABELS` | CLI용 한국어 표시 이름 |

## 10. `src/simulation/terrain/generator.py`

| 이름 | 기본값/목적 |
|---|---|
| `_CONTACT_ATTRIBUTES` | contype/conaffinity 1, condim 6, friction, solver contact 값, viewer group 0 |
| `worldbody` | geom을 추가할 XML element |
| `floor_z` | 장애물 바닥 기준 높이 |
| `center_x` | 코스 좌우 중심, 기본 0 |
| `start_y` | 로봇 앞 코스 시작, 기본 `0.35 m` |
| `cursor_y` | 다음 geom을 놓을 진행 위치 |
| `rng` | `np.random.default_rng(seed)` |
| `geom_names` | 생성 결과 검증과 metadata용 이름 목록 |
| `max_height` | 생성 중 갱신되는 최고 높이 |
| `_box.pos/size/euler/rgba` | MuJoCo box의 중심, half-size, 회전, 색상 |
| `add_gap.length` | 장애물 사이 평지 진행 거리, 기본 `.35 m` |
| uneven `length/width/tile_size` | `1.8/1.2/.2 m`; tile row/column 수 결정 |
| `min_height/max_height` | rough tile `.008–.060 m` 높이 난수 |
| `max_tilt_degrees` | tile roll/pitch `±4°` |
| `gap` | 타일 사이 최대 `.008 m` 간격 |
| `height`, `roll`, `pitch` | seed로 생성되는 각 타일 상태 |
| stairs `height/down_height` | 상승 누적 높이와 하강부 현재 높이 |
| `return_to_floor` | mixed course에서 동일 장애물을 하강시켜 floor로 복귀할지 결정 |
| ramp `center_surface_z/center_z` | 기울어진 box의 윗면/중심 z |
| `top_z` | ramp 끝 표면 높이 |
| mixed `order` | stairs1→slope1→stairs2→slope2→stairs3→slope3 |
