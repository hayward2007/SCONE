# 강화학습 변수와 상수 사전

보상 관련 값은 이 문서와 함께 [보상함수 수정 가이드](../06-reward-function-guide.md)를 확인한다.

## 1. `src/rl/walk_learn.py` 경로와 보상 설정

| 이름 | 값/목적 |
|---|---|
| `PROJECT_ROOT` | `src/` 디렉터리. 이 파일의 모델 자산 경로 기준이며 저장소 루트가 아님 |
| `DEFAULT_MODEL_PATH` | `src/assets/model.xml` |
| `RewardConfig.linear_velocity_sigma` | `0.25 m/s`; xy tracking Gaussian 폭 |
| `yaw_velocity_sigma` | `0.15 rad/s`; yaw-rate tracking 폭 |
| `heading_error_sigma` | `0.60 rad`; heading tracking 폭 |
| `projected_gravity_sigma` | `0.25`; gravity xy 기울기 tracking 폭 |
| `height_sigma` | `0.05 m`; 기준보다 낮아진 높이 penalty 정규화 |
| `vertical_velocity_sigma` | `0.30 m/s`; 수직 진동 정규화 |
| `roll_pitch_rate_sigma` | `0.80 rad/s`; roll/pitch 각속도 정규화 |
| `slip_deadzone` | `0.02 m/s`; 접촉점 접선 속도에서 무시하는 작은 값 |
| `slip_sigma` | `0.20 m/s`; deadzone 초과 slip 정규화 |
| `soft_joint_offset` | 기본 자세에서 `60°`; 이 이후 joint penalty 시작 |
| `hard_joint_offset` | 기본 자세에서 `90°`; 넘으면 episode 종료 |
| `velocity_weight` | `2.0`; 선속도 tracking |
| `yaw_weight` | `1.0`; yaw-rate tracking |
| `heading_weight` | `0.75`; 목표 heading tracking |
| `upright_weight` | `0.5`; projected gravity 자세 |
| `height_weight` | `0.05`; one-sided collapse guard |
| `oscillation_weight` | `0.1`; z속도와 roll/pitch rate |
| `action_rate_weight` | `0.02`; 연속 step action 변화 |
| `action_magnitude_weight` | `0.25`; residual 크기 |
| `idle_velocity_weight` | `1.0`; 정지 구간의 촘촘한 속도 tracking |
| `idle_action_weight` | `0.5`; 정지 구간 residual 억제 |
| `idle_linear_velocity_sigma` | `0.03 m/s`; idle xy 폭 |
| `idle_yaw_velocity_sigma` | `0.05 rad/s`; idle yaw 폭 |
| `idle_activity_threshold` | `0.05`; normalized command activity가 이 값 아래일 때 idle fraction |
| `current_weight` | `0.02`; normalized motor current 제곱 평균 |
| `slip_weight` | `0.1`; 접촉점 미끄러짐 |
| `joint_limit_weight` | `0.2`; soft limit 초과 |
| `collision_weight` | `1.0`; tire 이외 body-ground contact |
| `termination_penalty` | `5.0`; non-finite/fall/collision/hard-limit 즉시 감점 |

## 2. `WalkConfig`, curriculum, neutral gate

| 이름 | 값/목적 |
|---|---|
| `physics_timestep` | `0.002 s`; 500 Hz physics |
| `frame_skip` | `10`; 정책 1 step마다 물리 10 step → 50 Hz |
| `episode_seconds` | `10 s`; 기본 episode 길이 |
| `command_filter_seconds` | `0.35 s`; sampled command 저역통과 응답 |
| `command_hold_seconds_min/max` | `2/4 s`; command target 유지 구간 |
| `idle_command_probability` | `0.20`; 새 구간이 완전 정지일 확률 |
| `gait_frequency_min/max` | `0.6/1.4 Hz`; 명령 activity에 따라 phase 속도 보간 |
| `legacy_stride_degrees` | `20°`; upper reference swing 크기 |
| `legacy_lift_degrees` | `20°`; middle reference lift 크기 |
| `settle_seconds` | `0.20 s`; reset 후 reward 없이 접촉 안정화 |
| `max_height_drop` | `0.12 m`; 기준 아래로 이만큼 내려가면 fall |
| `max_tilt_degrees` | `60°`; upright cosine fall 경계 |
| `contact_force_threshold` | `1 N`; slip/collision으로 인정할 normal force |
| `CURRICULUM_RANGES.easy` | `[0.30, 0, 0]`; 전후진만 |
| `medium` | `[0.40, 0, 0.60]`; 전후진+yaw |
| `full` | `[0.50, 0.25, 0.80]`; lateral 포함 전축 |
| `OBSERVATION_COMMAND_SCALE` | `[0.50, 0.25, 0.80]`; 관측 normalization과 수동 명령 clip 공통 |
| `REFERENCE_MOTION_CHOICES` | `non_rl`, `hardcoded`; CLI 기본은 `non_rl`, 직접 환경 생성 호환 기본은 `hardcoded` |

`NeutralResidualGate`:

| 이름 | 기본값/목적 |
|---|---|
| `command_threshold` | `0.02`; normalized command max가 이를 넘으면 policy action 그대로 통과 |
| `decay_seconds` | `0.10 s`; neutral 전환 시 저장 action의 지수 감쇠 시간 |
| `zero_epsilon` | `5e-3`; 남은 residual 최대값이 작아지면 정확히 0 |
| `_action` | shape `(18,)`, `float32`; 마지막 active residual/감쇠 상태 |
| `activity` | `max(abs(command/OBSERVATION_COMMAND_SCALE))` |
| `parsed_command`, `parsed_action` | shape/단위를 검증한 입력 |

## 3. `SconeWalkEnv` 생성 상태

| 이름 | 목적·사용처 |
|---|---|
| `model_path` | 절대 MJCF 경로 |
| `curriculum` | `easy/medium/full` command sampling 범위 선택 |
| `command_range` | 선택 curriculum array의 복사본 |
| `fixed_command` | 지정되면 random scheduling을 끄고 항상 이 3축 명령 사용 |
| `render_mode` | `None` 또는 `human` |
| `reward_config` | 보상 sigma/weight/limit |
| `walk_config` | timestep, command, episode, reference 설정 |
| `terrain`, `terrain_seed` | 환경 지형과 seed |
| `reference_motion` | residual 아래의 기준 모션 선택 |
| `_non_rl_reference` | `non_rl` 선택 시 공유 Non-RL gait/IK 인스턴스, 아니면 `None` |
| `model`, `data` | terrain까지 compile한 MuJoCo model/state |
| `controller` | reset 때 새로 만드는 `MuJoCoController` |
| `control_dt` | `physics_timestep×frame_skip`, 기본 `0.02 s` |
| `max_episode_steps` | `episode_seconds/control_dt`, 기본 500 |
| `root_joint_id`, `root_body_id`, `root_qpos_address` | freejoint, body state, root position/height 주소 |
| `floor_geom_id` | 기본 plane geom ID |
| `ground_geom_ids` | floor와 이름이 `terrain_`인 모든 geom |
| `tire_geom_ids` | `TIRE_1_geom` … `TIRE_6_geom` |
| `tire_geom_to_body` | tire geom→contact Jacobian에 사용할 body ID |
| `default_degrees` | 검증된 Sport 또는 주입한 18개 standing pose |
| `default_radians` | degree→raw→radian으로 controller와 동일하게 변환한 기준 joint |
| `residual_scale_degrees` | upper 6개 `10°`, middle 6개 `12°`, lower 6개 `15°` |
| `action_space` | shape `(18,)`, 각 `[-1,1]` |
| `observation_space` | shape `(70,)`, unbounded float32 |

## 4. episode 상태와 관측

| 이름 | 목적·사용처 |
|---|---|
| `_phase` | reference gait `[0,1)` 위치 |
| `_episode_step` | 현재 정책 step, truncation과 command schedule 기준 |
| `_next_command_step` | 다음 random target을 뽑을 step |
| `_command` | filter 이후 실제 관측/reward/reference에 쓰는 명령 |
| `_command_target` | 다음 low-pass 목표 명령 |
| `_heading` | reset 시 실제 heading 기록. 현재 별도 reward 계산에는 직접 사용하지 않음 |
| `_target_heading` | yaw command를 적분한 heading 목표 |
| `_last_action` | action-rate penalty와 관측의 이전 residual |
| `_reference_height` | settle 후 root z; height penalty/fall 기준 |
| `_viewer` | lazy passive viewer |
| `_jacobian_position/_rotation` | contact point velocity 계산 buffer |
| `_contact_force` | `mj_contactForce` 6축 buffer; index 0 normal force |
| `_body_velocity` | `mj_objectVelocity` 6축 world vector buffer |
| `np_random` | Gymnasium reset seed로 관리되는 command/phase 난수기 |

70차원 관측의 변수는 `linear_velocity/2`, `angular_velocity/5`, `gravity`, `(joint_position-default)/π`, `joint_velocity/10`, `_last_action`, `_command/scale`, `sin/cos phase`, `sin/cos heading_error` 순서다.

## 5. command와 reference motion 중간값

| 이름 | 목적·사용처 |
|---|---|
| `hold` | 2–4초에서 뽑은 다음 command 유지 시간 |
| `alpha` | `1-exp(-control_dt/filter_seconds)` low-pass 계수 |
| `safe_scale` | command scale이 0인 경우 나눗셈을 보호 |
| `activity` | 0–1 normalized command 크기 |
| `frequency` | min/max gait frequency의 activity 보간 |
| `phase_sine` | `hardcoded` 기준의 `sin(2π phase)` tripod swing/lift 기준 |
| `vx_scale` | `clip(-command.vx/0.50)`; motor convention 때문에 전진 명령 부호 반전 |
| `yaw_scale` | `clip(command.yaw/0.80)` |
| `tripod_a` | legacy `UPPER_DIAGONAL_LEFT`, 실제 set `{2,3,6}` |
| `tripod_sign` | 두 tripod의 phase 부호 |
| `side_sign` | 홀수 오른쪽 `+1`, 짝수 왼쪽 `-1`; yaw 합성 |
| `command_scale` | `vx_scale + yaw_scale×side_sign`를 `[-1,1]` clip |
| `lift_a/lift_b` | sine의 양/음 반주기에 해당 tripod middle lift |
| `reference` | `non_rl` IK 결과 또는 기본 18자세에 분석적 stride/lift를 더한 degree |
| `clipped` | 정책 action `[-1,1]` |
| `targets` | reference + residual scale×action, 이후 기준 `±60°` 임시 clip |

`hardcoded` reference에는 측면 `vy`가 없고 full curriculum의 residual policy가 학습한다. `non_rl` reference는 body-frame `vx`, `vy`, `yaw_rate`를 모두 발 궤적과 IK에 반영한다.

## 6. 접촉·전류·reward 중간값

| 이름 | 목적·사용처 |
|---|---|
| `world_from_body` | world velocity/gravity를 body frame으로 회전 |
| `linear_velocity`, `angular_velocity`, `projected_gravity` | reward/관측의 base state |
| `heading`, `heading_error` | 현재 yaw와 wrap된 `[-π,π]` 목표 오차 |
| `contact_index`, `contact` | MuJoCo active contact 반복 |
| `geom1/geom2`, `tire_geom`, `body_id` | ground-tire 또는 forbidden contact 분류 |
| `point_velocity` | contact position Jacobian×qvel |
| `normal`, `tangential` | contact frame normal과 normal 성분을 제거한 slip 속도 |
| `slip_speed`, `excess`, `values` | deadzone 이후 sigma로 정규화한 접촉별 제곱 |
| `voltage`, `velocity`, `spec` | motor current 역산 입력 |
| `current` | `(voltage-K×velocity)/R` |
| `stall_current` | `stall_torque/K` |
| `normalized_currents` | motor current/stall current; 제곱 평균 penalty |
| `raw_terms` | 각 weighted 세부 보상. 로깅용이며 총합에 다시 전부 더하지 않음 |
| `weighted_terms` | 세부 값에 `direction/stability/damping` aggregate를 추가한 info/log dictionary |
| `finite`, `fallen`, `hard_joint_limit`, `forbidden_collision` | termination 원인 |
| `diagnostics` | 실제 vx/vy/yaw, heading, height, activity, contact 수, 종료 원인 |

`step()`은 `_target_heading += command_yaw×control_dt`를 policy step마다 한 번 실행한다.

## 7. 학습 callback과 PPO hyperparameter

| 이름 | 값/목적 |
|---|---|
| `RewardTermsCallback` | `reward/*`, `state/vx,vy,yaw_rate,height,height_drop,stance_contacts`를 logger에 평균 기록 |
| `GracefulStopCallback.stop_requested` | signal 수신 뒤 현재 step을 끝내고 `learn()` 중지 |
| `resume.checkpoint` | 마지막 완성 checkpoint/final model 경로를 담는 pointer |
| `temporary` | pointer의 `.tmp`; 완성 후 원자적 교체 |
| `PruningCheckpointCallback.keep_last` | 기본 10; step 기준 최신 ZIP만 보존 |
| `save_freq` | global checkpoint interval을 `num_envs`로 나눈 callback 호출 빈도 |
| `factories` | environment index마다 terrain seed를 증가시키는 vector env 생성 함수 |
| `_make_vector_env` | factory 1개는 `DummyVecEnv`, 2개 이상은 `SubprocVecEnv` 선택 |
| `monitor_name` | 새 학습 `monitor.csv`, 재개 시 timestamp suffix |
| `learning_rate` | `3e-4` |
| `n_steps` | env당 rollout `1024` |
| `batch_size` | `256` |
| `n_epochs` | `10` |
| `gamma` | `0.99` |
| `gae_lambda` | `0.95` |
| `clip_range` | `0.2` |
| `ent_coef` | `0.005` |
| `max_grad_norm` | `1.0` |
| `policy_kwargs.net_arch` | policy/value 각각 `[256,256]` MLP |
| `reset_num_timesteps` | 새 모델 `True`, checkpoint 재개 `False` |

## 8. `src/rl/stance.py`, `policy_compat.py`

| 이름 | 값/목적 |
|---|---|
| `UPPER_STANDING_DEGREES` | `(135,135,180,180,225,225)` |
| `SPORT_STANDING_DEGREES` | upper + middle `170×6` + lower `195×6` |
| `STANDARD_STANDING_DEGREES` | upper + middle `240×6` + lower `255×6` |
| `STANCE_PRESETS` | `standard/sport` lookup |
| `validate_standing_pose` | 정확히 18개, finite, 각 0–360° 검증 |
| `LEGACY_OBSERVATION_SHAPE` | `(68,)` |
| `CURRENT_OBSERVATION_SHAPE` | `(70,)` |
| `observation_shape` | checkpoint가 학습된 입력 차원 |
| `supported_shapes` | 현재 env shape 또는 legacy 68만 허용 |
| `expected_shape` | replay 시 policy가 기대하는 shape; 68이면 현재 관측의 첫 68개만 전달 |

구형 policy는 `PPO.load(path, device=device)`로 env 없이 읽는다. 새 학습을 68차원 policy에서 재개하는 기능은 의도적으로 제공하지 않는다.

## 9. `src/rl/joystick_control.py`

| 이름 | 목적·사용처 |
|---|---|
| `_VelocityMailbox._command` | CLI thread가 쓴 최신 `VelocityCommand` |
| `_lock` | command read/update 원자성 |
| `_RLModeRouter.robot/adapter` | 같은 MuJoCo controller를 공유하는 Legacy Walk/Drive/Climb 상태기계 |
| `_transitioning` | 모드 전환 중 PPO target 쓰기를 차단하는 상태 |
| `_resume_pending` | Walk 복귀 후 observation/heading/height 재정렬 요청 |
| `checkpoint_path` | 검증 후 load할 local PPO ZIP |
| `WalkConfig(episode_seconds=24h)` | interactive session이 10초마다 자동 truncation되지 않게 함 |
| `observation` | 현재 70차원 env 관측 |
| `mailbox` | input worker→policy loop 명령 전달 |
| `neutral_gate` | neutral residual 제거 |
| `stop_event` | terminal/viewer 종료 공유 |
| `cli_errors` | worker 예외 전달 |
| `limits` | observation command scale `0.50/0.25/0.80`을 UI 최대 입력으로 사용 |
| `frame_started/remaining` | policy loop를 `env.control_dt`에 맞춤 |
| `policy_observation` | policy shape에 맞춘 70 또는 앞 68개 |

## 10. `src/rl/remote_watch.py`

| 이름 | 목적·사용처 |
|---|---|
| `CHECKPOINT_NAME` | `<prefix>_<steps>_steps.zip`에서 prefix/step 추출 |
| `SAFE_SSH_HOST`, `SAFE_PREFIX` | shell argument에 허용할 문자 제한 |
| `CheckpointCandidate.source_path` | local 또는 remote 원본 파일 경로 |
| `CheckpointCandidate.step` | newest 비교와 hot-swap 순서 |
| `CheckpointSource.label` | 상태 로그의 source 표시 |
| `LocalCheckpointSource.directory` | 감시할 local checkpoint 폴더 |
| `SSHCheckpointSource.host/directory` | 원격 host와 checkpoint 경로 |
| `port`, `identity_file`, `connect_timeout` | SSH 연결 option |
| `destination` | 공개될 완성 cache ZIP |
| `partial` | `<destination>.part`; 검증 전 임시 다운로드 |
| `refresh` | 기존 정상 cache가 있어도 다시 받을지 결정 |
| `CheckpointPoller.source/prefix/cache_dir/poll_interval` | 반복 탐색 설정 |
| `updates` | `(step, Path)` simple queue; viewer가 가장 최신 것만 소비 |
| `_stop`, `_thread` | poller 수명주기 |
| `_mirrored_step` | 이미 배포한 가장 큰 step, 초기 `-1` |
| `_last_error` | 같은 SSH 오류를 반복 출력하지 않기 위한 문자열 |
| `policy`, `active_step` | viewer가 현재 재생하는 PPO와 step |
| `zero_action` | 첫 checkpoint 전 baseline reference만 보여주는 18 zero residual |
| `render_speed` | viewer target dt를 나누는 배속 |

## 11. `src/rl/inquiry.py`

### 경로·검증 상수

| 이름 | 값/목적 |
|---|---|
| `PROJECT_ROOT` | 저장소 루트 |
| `RUNS_DIR` | `<root>/runs` |
| `REMOTE_JOBS_FILE` | `runs/.remote_jobs.json` |
| `DEFAULT_REMOTE_HOST` | `ssh.hayward.kim` |
| `DEFAULT_REMOTE_PROJECT` | `~/Developer/SCONE` |
| `SAFE_NAME` | run 이름의 영문/숫자/`_.-` 제한 |
| `SAFE_SSH_HOST` | SSH alias/user@host 허용 문자 제한 |
| `TERRAIN_OPTIONS` | 내부 terrain 값과 한국어 UI label |
| `REFERENCE_MOTION_OPTIONS` | Non-RL 권장값과 hardcoded 호환값의 UI 순서/label |
| `REMOTE_MEMORY_RESERVE_BYTES` | OS/PPO parent에 남기는 `2 GiB` |
| `ESTIMATED_ENV_MEMORY_BYTES` | MuJoCo worker 하나당 보수적으로 계산하는 `768 MiB` |
| `TRAINING_TASKS["walk"]` | module `src.rl.walk_learn`, prefix `scone_walk` |

### 데이터 클래스 필드

| 클래스/필드 | 목적 |
|---|---|
| `TrainingTask.key` | stable task 식별자 |
| `label` | prompt 표시 |
| `module` | `python -m` target |
| `checkpoint_prefix` | 저장/검색 ZIP prefix |
| `TrainingConfig.task` | 현재 `walk` |
| `run_name` | `runs/<name>`과 원격 state key |
| `curriculum` | easy/medium/full |
| `timesteps` | 이번 실행 학습량 |
| `num_envs` | vector env 수 |
| `checkpoint_every` | global timestep 저장 간격 |
| `keep_checkpoints` | 최신 보존 개수 |
| `terrain/terrain_seed` | 학습 환경 지형 |
| `seed` | PPO/env 재현성 |
| `device` | auto/cpu/cuda 등 |
| `reference_motion` | `non_rl/hardcoded`; 재개·watch·view까지 보존 |
| `standing_pose_name/degrees` | 사람이 읽는 preset명과 실제 18개 값 |
| `RemoteSettings.host/project_dir/port/connect_timeout` | SSH 연결 위치/제한 |
| `RemoteJob.*` | 위 학습 설정에 PID와 `created_at`을 더한 영속 작업 기록 |
| `RemoteCapacity.*` | 물리/논리 코어, 메모리, load와 CPU/메모리/최종 추천 한도 |

### 운영 지역변수

| 이름 | 목적·사용처 |
|---|---|
| `project`, `run_dir`, `resume_path` | shell quoting된 원격 경로 표현 |
| `train_command` | `python -m src.rl.walk_learn train ...` argument list |
| `ssh_transport` | rsync가 사용할 SSH/port option |
| `env/PYTHONPATH` | local subprocess가 저장소 package를 import하게 함 |
| `pid` | nohup background trainer process ID |
| `resume.checkpoint` | 원격 재개 후보 pointer |
| `backup_root/backup_prefix` | reset 시 `runs/.reset_backup/<run>` 보존 위치 |
| `stop_flag` | reset 전에 실행 중 job을 정상 정지할지 shell에 전달 |
| `temporary` | `.remote_jobs.json.tmp` 원자적 저장 |
| `middle/lower/pose` | 사용자 지정 stance의 두 공통 각도와 완성 18개 tuple |
| `timestamp` | 기본 run name과 monitor/resume 구분 |

코드 sync는 가상환경, `runs`, 캐시, archive 등 생성/대용량 영역을 제외한다. Python 3.12 환경이 맞지 않으면 기존 `.venv`를 백업한 뒤 새로 만들며 곧바로 삭제하지 않는다.
