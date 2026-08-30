# 테스트·자산·아카이브 변수 사전

## 1. 테스트의 공통 변수

테스트에서 사용하는 `FakeController`, fake prompt/viewer/policy, temporary directory는 production 변수가 아니라 특정 계약을 관찰하기 위한 test double이다.

| 이름/패턴 | 목적 |
|---|---|
| `controller.calls`/기록 목록 | 호출 순서, motor ID와 값이 예상대로인지 확인 |
| `positions`, `speeds`, `modes`, `torques` | fake backend가 마지막 설정 상태를 보관 |
| `model`, `data` | 실제 `src/assets/model.xml`을 compile한 회귀 fixture |
| `TemporaryDirectory`, `tmp_path` 계열 | checkpoint/remote metadata를 실제 `runs/`와 분리 |
| `mock`, `patch`, fake subprocess result | serial/SSH/menu를 실제 외부 상태 변경 없이 검증 |
| `seed` | terrain/환경 결과가 반복 가능한지 확인 |
| `assert*`의 expected/actual 배열 | shape, 단위, 부호, tolerance 계약 고정 |

## 2. 테스트 파일별 핵심 fixture와 목적

| 파일 | 주요 변수·fixture | 무엇을 고정하는가 |
|---|---|---|
| `test_actuators.py` | ID 1/6/7/12/13/18, `Register`, model objects | ID 경계에서 모델/protocol/control table이 바뀌는 규칙 |
| `test_api.py` | `FakeController`, command call log, fake Inquirer answers, `KeyboardJoystick(now=...)` | 초기화·종료 순서, CLI route, 키 timeout/독립 축/neutral 전송 |
| `test_kinematics.py` | center raw 2048, 180°, 0 rad, nominal/perturbed joint arrays | 단위 변환과 FK→IK round trip, actuator ordering |
| `test_non_rl_walk.py` | `GaitConfig`, phase 0/0.5, nominal feet, fake position batch | tripod alternation, idle, yaw 접선 속도, IK safety, 50 mm sim stride |
| `test_simulation.py` | compiled model/controller, control enum, patched launchers | 18-actuator protocol, mode cycle, wheel sign, target wait, Drive 댐핑과 control route |
| `test_terrain.py` | 모든 `TerrainType`, 동일/다른 seed XML, fake camera | preset compile, visibility group 0, reproducibility, fixed base, camera cap |
| `test_remote_watch.py` | temporary ZIP, local source, reward mock state, 68/70 fake policy | one-sided height, idle action, 합계 중복 방지, atomic checkpoint, replay compatibility |
| `test_rl_inquiry.py` | `TrainingConfig`, `RemoteJob`, mocked subprocess/prompt/capacity | shell quoting, 이름/port 검증, venv, SIGTERM pause, reset backup, stance/reference/병렬 추천 |
| `test_rl_joystick.py` | active/neutral command, residual action, fake mode robot | neutral gate와 RL→Drive→Climb→RL routing |
| `test_rl_reference_motion.py` | hardcoded/Non-RL reference, vx/yaw/vy command | 전진/후진/yaw 부호와 reference별 lateral 처리 |

테스트 tolerance는 기구학/부동소수점 비교의 허용 오차다. 안전 범위나 실제 로봇 허용 오차로 재사용하면 안 된다.

## 3. `src/assets/model.xml` 전역 설정

| XML 이름/속성 | 값/목적 |
|---|---|
| `model="SCONEv2-v19"` | MuJoCo 모델 식별 이름 |
| `<option timestep="0.002" integrator="implicitfast">` | 기본 500 Hz 물리와 안정적인 implicit 계열 적분 |
| `root_freejoint` | floating-base 7 qpos/6 dof |
| `simulation_floor` | 소스에 있거나 loader가 추가하는 지면 이름 |
| `M01…M18` joint | 하드웨어 motor ID와 일치하는 18 관절 |
| `A01_…A18_` actuator | 이름 prefix로 controller가 motor ID를 찾는 18 dcmotor |
| `leg_L1/L3/L5`, `leg_L2/L4/L6` class | 오른쪽/왼쪽 visual 색상과 반복 설정 |
| `motor_mx28at` | 12 V, 2.5 N·m, 55 rpm 계열 actuator class |
| `motor_xm430_w350` | 12 V, 4.1 N·m, 46 rpm |
| `motor_xm430_w210` | 12 V, 3.0 N·m, 77 rpm |
| motor input | voltage; Python `DCMotorPID` output이 `data.ctrl`에 들어감 |
| `TIRE_1_geom…TIRE_6_geom` | support point와 RL 접촉을 찾는 필수 이름 |
| `contype/conaffinity` | tire/필요 body collision은 활성, visual-only geom은 비활성 |
| `friction` | 타이어/바닥 접촉 마찰. sim-to-real tuning 대상 |
| `BODY_FRAME`/root body | body 속도, heading, viewer tracking 기준 |
| payload geoms | battery 3×0.15 kg, Raspberry Pi 0.05 kg, power 0.05 kg, wiring 0.05 kg 등 질량 반영 |

왼쪽 joint 중 `M02/M04/M06`과 `M14/M16/M18`의 axis는 CAD 거울상과 controller convention을 맞추기 위해 모델에서 반전되어 있다. element 순서나 시각적 mesh scale만 보고 방향을 다시 바꾸면 안 된다.

## 4. mesh와 export report

| 자산 | 런타임 목적/관련 변수 |
|---|---|
| `ARC_SHAPED_WHEEL.stl` | 호형 바퀴 visual/geometry |
| `TIRE.stl` | 실제 contact edge와 Non-RL support vertex 추론 |
| `BODY.stl`, `BODY_ACTUATOR.stl` | 중앙 구조와 actuator mount visual/collision 구성 |
| `LEG_ACTUATOR.stl`, `LINK.stl`, `FR07.stl`, `FR12.stl` | 다리 각 링크의 CAD geometry |
| mesh `scale`, mirrored scale | mm 단위 CAD를 meter로 바꾸고 좌우 visual을 재사용 |
| mesh vertex/face/triangle | 렌더링·접촉 정밀도와 compile 비용에 영향; 코드 제어 변수는 아님 |

`acdc4robot-export-report.json`의 의미:

| key | 목적 |
|---|---|
| `design_name`, `plugin_version`, `schema_version` | 어떤 CAD/export 도구 버전에서 생성했는지 추적 |
| `default_length_units` | 원본 CAD 단위(mm) |
| `active_dof_count` | export된 능동 자유도, 현재 18 |
| `joint_count`, `occurrences`, `visible_link_count` | 모델 구조 진단 통계 |
| `joints`, `root_links` | export가 인식한 joint/root 목록 |
| `errors`, `warnings`, `passed` | export 검증 결과. 현재 runtime이 자동 소비하지 않음 |

## 5. `runs/.remote_jobs.json`

각 key는 run 이름이며 value는 `RemoteJob`을 JSON으로 직렬화한 상태다.

| JSON 필드 | 목적 |
|---|---|
| `host`, `project_dir`, `port` | SSH 실행 위치 |
| `run_name`, `task` | 작업 ID와 학습 module 선택 |
| `pid`, `created_at` | 원격 process 확인과 생성 시각 |
| `terrain`, `terrain_seed`, `curriculum`, `seed` | 환경 재현 설정 |
| `num_envs`, `checkpoint_every`, `keep_checkpoints` | 학습/저장 규모 |
| `device` | 학습 accelerator 선택 |
| `standing_pose_name`, `standing_pose_degrees` | 초기 자세 provenance |

PID는 시간이 지나면 재사용될 수 있으므로 run 상태 명령은 PID 하나만 믿지 않고 run 디렉터리/state/log도 함께 확인해야 한다.

## 6. Stable-Baselines3 checkpoint 내부

| ZIP entry 종류 | 목적 |
|---|---|
| `data` | algorithm hyperparameter, observation/action space metadata |
| `policy.pth` | policy/value network weight |
| `policy.optimizer.pth` | optimizer state; 재학습 resume에 필요 |
| `pytorch_variables.pth` | 추가 PyTorch 변수 |
| `_stable_baselines3_version` | 저장 library version |
| `system_info.txt` | Python/OS/library 환경 정보 |

파일 이름의 `<steps>`는 checkpoint 비교·pruning·hot-swap 순서 기준이다. `.part` suffix는 불완전 상태라 loader가 사용해서는 안 된다.

## 7. `archive/codes/SCONEv1.py`

구형 파일의 상수와 변수는 역사적 참고용이다.

| 영역 | 목적/현재와의 차이 |
|---|---|
| ID 1–6, 13–18 AX-18A / 7–12 MX-28AT 구분 | 현재 SCONEv2의 MX+XM 3단계 구성과 다름 |
| Protocol 1.0 고정 주소 | 현재 모델별 `ControlTable` 구조로 교체됨 |
| 재귀 `walk`/방향 함수 | 현재 mode 객체와 연속 gait로 대체 |
| raw position/speed 상수 | 구형 기구 자세에만 유효 |
| MobileNetSSD model/prototxt/confidence/class label | 카메라 객체 탐지 실험; 현재 runtime과 분리 |
| multiprocessing/video stream 상태 | 과거 vision pipeline의 frame 처리 상태 |

이 파일의 변수 값을 현재 controller나 model에 복사하지 않는다.

## 8. 논문·도면·영상 자산의 수치

| 자료 | 주요 값/용도 |
|---|---|
| `SCONEv2 Arc-Shaped Wheel.pdf` | 직경 약 244.94 mm, 폭 44 mm, 반경 R112.5/R122.5, 호각 148.27° 설계 참고 |
| 영문/국문 초기 논문 | 18 actuator, parallel link와 240° 계열 arc wheel 개념, 외부전원 속도/한계의 역사 기록 |
| `SCONEv1.obj` | 전체 구형 mesh; 약 29,208 vertex/58,428 face |
| `SCONEv2.obj` | 전체 신형 mesh; 약 48,008 vertex/96,952 face |
| `.mtl`의 `Kd` | Steel Satin 계열 diffuse 색상; 동역학과 무관 |
| MP4 width/height/fps/duration | 실험 증거를 재구성할 영상 metadata; 제어 파라미터가 아님 |

과거 논문에는 `0.05`, `0.07`, `0.5`, `0.7 m/s`처럼 서로 다른 속도 기록이 있다. 원 영상의 거리/시간, 전원, 자세, 바닥 조건을 재구성하기 전에는 현재 성능 변수로 채택하지 않는다.

## 9. `archive/ICRA/` LaTeX 변수

| 이름/영역 | 목적 |
|---|---|
| `root.tex` title/author/anonymity 설정 | ICRA 제출 top-level metadata |
| section include 순서 | introduction→related/problem→system→control→method→results→discussion |
| `references.bib` citation key | 본문 인용과 참고문헌 연결 |
| `EVIDENCE_PLAN` metric/claim table | 주장마다 필요한 로그, 동일 하드웨어 비교, 반복 횟수 추적 |
| `Makefile` target | Tectonic으로 `root.pdf` 생성/정리 |
| `build/*.aux/.bbl/.blg/.log` | LaTeX 교차참조·참고문헌·진단 임시 상태 |
| `build/root.pdf` | 현재 4쪽 구조 초안; 결과 수치가 확정된 논문이 아님 |

## 10. 환경·생성 변수

| 이름/파일 | 목적 |
|---|---|
| `SCONE_DEVICE` | 기본 serial port를 override |
| `PYTHONPATH` | 원격/로컬 subprocess가 저장소 package를 찾도록 보조 |
| `.venv` | 원격 Python 3.12 RL 환경; 저장소 소스가 아님 |
| `__pycache__/*.pyc` | interpreter cache |
| `.DS_Store` | macOS Finder metadata |
| `.claude/worktrees/*` | 도구가 만든 별도 작업 트리; 현재 문서의 코드 기준이 아님 |
| `tmp/pdfs/*` | 문서 조사 중 PDF 페이지를 시각 검증한 임시 PNG/PDF 추출물 |
