# 파일과 폴더 지도

## 1. 루트

| 경로 | 역할 |
|---|---|
| [`.gitignore`](../.gitignore) | Python 캐시, 가상환경, 학습 산출물, 임시 파일, 미디어/논문 빌드 부산물의 Git 제외 규칙 |
| [`LICENSE`](../LICENSE) | MIT 라이선스 |
| [`README.md`](../README.md) | 사용자용 설치·구조·실행 개요 |
| [`SCONE.py`](../SCONE.py) | 외부 코드가 `SCONE`, 명령/상태 타입, locomotion/kinematics API를 import하는 안정 facade |
| [`example.py`](../example.py) | 실제 장치를 탐색하고 context manager 안에서 전진 동작을 실행하는 최소 예제 |
| [`requirements.txt`](../requirements.txt) | CLI·하드웨어·시뮬레이션·RL 통합 의존성 |
| `docs/` | 이 분석 문서 모음 |
| `runs/` | 로컬/원격 PPO 체크포인트와 작업 상태. 생성 데이터이며 Git 비추적 |
| `tmp/` | PDF 렌더링 등 일시 작업 파일. 런타임 소스가 아님 |

### `docs/`

| 경로 | 역할 |
|---|---|
| [`README.md`](README.md) | 문서 색인, 범위, 현재 상태 주의사항 |
| [`01-project-overview.md`](01-project-overview.md) | 목표, 하드웨어 개념, 제어 경로, 완성/미완성 범위 |
| [`02-features-and-technologies.md`](02-features-and-technologies.md) | 구현 기능, library/tool, 설계 선택 |
| [`03-architecture-and-data-flow.md`](03-architecture-and-data-flow.md) | 계층, 실제/시뮬레이션/Non-RL/RL 데이터 흐름 |
| [`04-problems-and-solutions.md`](04-problems-and-solutions.md) | 실패 증상, 진단, 해결책, 남은 기술 부채 |
| [`05-file-and-folder-map.md`](05-file-and-folder-map.md) | 저장소 전체 파일·폴더 역할(현재 문서) |
| [`06-reward-function-guide.md`](06-reward-function-guide.md) | reward 수식, 값, 수정·검증·호환성 절차 |
| [`07-running-testing-and-operations.md`](07-running-testing-and-operations.md) | 설치, 실행, 학습, 원격 운영, 테스트 |
| [`08-rl-development-log.md`](08-rl-development-log.md) | RL/시뮬레이션 시행착오와 변경 이력 |
| [`09-gait-performance-analysis.md`](09-gait-performance-analysis.md) | 하드코드·Non-RL·policy 수치 비교와 개선 우선순위 |
| [`variables/README.md`](variables/README.md) | 변수 사전 색인과 포함 규칙 |
| [`variables/01-api-hardware-locomotion.md`](variables/01-api-hardware-locomotion.md) | API/CLI/hardware/locomotion 변수 |
| [`variables/02-kinematics-simulation-terrain.md`](variables/02-kinematics-simulation-terrain.md) | kinematics/simulation/terrain 변수 |
| [`variables/03-reinforcement-learning.md`](variables/03-reinforcement-learning.md) | RL 환경, reward, checkpoint, inquiry 변수 |
| [`variables/04-tests-assets-archive.md`](variables/04-tests-assets-archive.md) | 테스트 fixture, MJCF/assets, runs, archive 값 |

## 2. `src/` public API와 수명주기

| 경로 | 역할 |
|---|---|
| [`src/__init__.py`](../src/__init__.py) | 패키지 public symbol과 지연 import 제공 |
| [`src/main.py`](../src/main.py) | `SCONE` facade, 초기화/프로필/모드 전환/종료, `RobotCommand`, `RobotStatus` |
| [`src/cli.py`](../src/cli.py) | 장치·시뮬레이션·RL 통합 메뉴, raw terminal 키보드 조이스틱, legacy/Non-RL adapter |

## 3. `src/hardware/`

| 경로 | 역할 |
|---|---|
| [`__init__.py`](../src/hardware/__init__.py) | 하드웨어 public export |
| [`actuator_index.py`](../src/hardware/actuator_index.py) | 1–18 ID, 관절 단계, 좌우/대각 tripod, 다리→세 모터 인덱스 정의 |
| [`actuator_control_table.py`](../src/hardware/actuator_control_table.py) | register, control table, actuator model 데이터 구조와 MX/XM 주소·protocol 정의 |
| [`actuator.py`](../src/hardware/actuator.py) | torque/position namespace, 모델 선택, 호환용 `Actuator` 묶음 |
| [`config.py`](../src/hardware/config.py) | 기본 baudrate와 serial device 경로 |
| [`interface.py`](../src/hardware/interface.py) | 실제/시뮬레이션 controller가 구현해야 하는 runtime-checkable protocol |
| [`controller.py`](../src/hardware/controller.py) | Dynamixel SDK 포트 관리, 모델별 sync write, 위치/속도/mode/torque API |
| [`discovery.py`](../src/hardware/discovery.py) | 환경 변수·glob·기본 장치 후보를 ping해 실제 포트를 비파괴 탐색 |

## 4. `src/locomotion/`

| 경로 | 역할 |
|---|---|
| [`__init__.py`](../src/locomotion/__init__.py) | locomotion public export |
| [`mode.py`](../src/locomotion/mode.py) | 모드 객체의 공통 기반, 시뮬레이션 선택 기능(목표 도달 대기·바퀴 부호·Drive 댐핑) adapter |
| [`profile.py`](../src/locomotion/profile.py) | `MotionProfile`, Standard/Sport 자세·속도·가속도 preset |
| [`walk.py`](../src/locomotion/walk.py) | blocking legacy tripod walk, hold/release, Drive 전환 |
| [`drive.py`](../src/locomotion/drive.py) | 하단 바퀴 velocity mode 주행과 Climb 전환 |
| [`climb.py`](../src/locomotion/climb.py) | 등반 준비 자세, tripod 교대 동작, Walk 복귀 |
| [`legacy_velocity.py`](../src/locomotion/legacy_velocity.py) | 연속 명령을 legacy discrete 동작으로 바꾸는 background latest-command adapter |
| [`non_rl_walk.py`](../src/locomotion/non_rl_walk.py) | Phoenix식 연속 gait, support point, phase 궤적, IK, batch 전송 |

## 5. `src/kinematics/`

| 경로 | 역할 |
|---|---|
| [`__init__.py`](../src/kinematics/__init__.py) | kinematics public export |
| [`types.py`](../src/kinematics/types.py) | vector/matrix alias, 관절각·발 pose·IK 결과/오류 타입, 단위 변환 |
| [`leg.py`](../src/kinematics/leg.py) | 한 다리의 모델 이름 해석, FK, translational Jacobian, DLS IK |
| [`robot.py`](../src/kinematics/robot.py) | 공유 MuJoCo model/data 위의 6개 다리 FK/IK와 actuator-order 변환 |

## 6. `src/simulation/`

### public/호환 모듈

| 경로 | 역할 |
|---|---|
| [`__init__.py`](../src/simulation/__init__.py) | 시뮬레이션 public export |
| [`__main__.py`](../src/simulation/__main__.py) | `python -m src.simulation` 진입점 |
| [`cli_bridge.py`](../src/simulation/cli_bridge.py) | 과거 import 경로를 `core.cli_bridge`로 전달하는 shim |
| [`controller.py`](../src/simulation/controller.py) | `core.controller` 호환 import |
| [`model.py`](../src/simulation/model.py) | `core.model` 호환 import |
| [`pid.py`](../src/simulation/pid.py) | `core.pid` 호환 import |
| [`simulator_cli.py`](../src/simulation/simulator_cli.py) | `core.simulator_cli` 호환 import |

### `src/simulation/core/`

| 경로 | 역할 |
|---|---|
| [`__init__.py`](../src/simulation/core/__init__.py) | core public export |
| [`model.py`](../src/simulation/core/model.py) | MJCF 수정, freejoint/floor/terrain 삽입, STL asset compile |
| [`pid.py`](../src/simulation/core/pid.py) | 액추에이터 물리 사양, PID, 전압-토크 변환과 포화 |
| [`controller.py`](../src/simulation/core/controller.py) | 이름 기반 18 motor 매핑, profile setpoint, mode/torque/position/velocity API |
| [`cli_bridge.py`](../src/simulation/core/cli_bridge.py) | Old/Non-RL/RL control을 viewer와 물리 loop에 연결 |
| [`simulator_cli.py`](../src/simulation/core/simulator_cli.py) | argparse와 대화형 시뮬레이션 선택 UI |
| [`viewer.py`](../src/simulation/core/viewer.py) | passive viewer 카메라 추적·가시 그룹 설정 |

### `src/simulation/terrain/`

| 경로 | 역할 |
|---|---|
| [`README.md`](../src/simulation/terrain/README.md) | 지형 종류와 사용법 |
| [`__init__.py`](../src/simulation/terrain/__init__.py) | terrain public export |
| [`types.py`](../src/simulation/terrain/types.py) | 지형 enum/data type과 입력 검증 |
| [`presets.py`](../src/simulation/terrain/presets.py) | 계단 난도·경사 각도 preset |
| [`generator.py`](../src/simulation/terrain/generator.py) | 평지/불규칙/계단/경사/혼합 지형 MJCF 생성 |

## 7. `src/rl/`

| 경로 | 역할 |
|---|---|
| [`__init__.py`](../src/rl/__init__.py) | RL public export |
| [`__main__.py`](../src/rl/__main__.py) | `python -m src.rl`에서 inquiry 실행 |
| [`walk_learn.py`](../src/rl/walk_learn.py) | RL 환경, 관측, reference gait, reward, PPO 구성, 학습/재개/저장 |
| [`stance.py`](../src/rl/stance.py) | Standard/Sport standing pose 정의와 검증 |
| [`motion_profile.py`](../src/rl/motion_profile.py) | RL standing pose를 가장 가까운 Legacy 모션 프로필과 결합해 모드 전환에 재사용 |
| [`policy_compat.py`](../src/rl/policy_compat.py) | checkpoint observation 차원 판별과 68→70 replay adapter |
| [`joystick_control.py`](../src/rl/joystick_control.py) | 로컬 checkpoint 실시간 제어, command mailbox, neutral residual gate, RL/Legacy 모드 router |
| [`remote_watch.py`](../src/rl/remote_watch.py) | 로컬/SSH checkpoint source, atomic download, ZIP 검증, policy hot swap |
| [`inquiry.py`](../src/rl/inquiry.py) | 학습 설정 prompt, 로컬/원격 작업 생성·pause·resume·reset·watch·view |

## 8. `src/assets/`

| 경로 | 역할 |
|---|---|
| [`model.xml`](../src/assets/model.xml) | SCONEv2 MuJoCo 모델: 18 joint/motor, freejoint, body/geom/material, 접촉, payload |
| [`acdc4robot-export-report.json`](../src/assets/acdc4robot-export-report.json) | Fusion export 결과·경고·joint/occurrence 통계. 런타임 입력이 아닌 provenance 자료 |
| `meshes/ARC_SHAPED_WHEEL.stl` | 호형 바퀴 시각/기하 메시 |
| `meshes/BODY.stl` | 중앙 몸체 메시 |
| `meshes/BODY_ACTUATOR.stl` | 몸체측 actuator mount 메시 |
| `meshes/FR07.stl` | Fusion 부품 FR07 메시 |
| `meshes/FR12.stl` | Fusion 부품 FR12 메시 |
| `meshes/LEG_ACTUATOR.stl` | 다리 actuator assembly 메시 |
| `meshes/LINK.stl` | 링크 메시 |
| `meshes/TIRE.stl` | 지면 접촉과 support point 계산에 사용되는 타이어 메시 |

## 9. 테스트

| 경로 | 검증 내용 |
|---|---|
| [`test_actuators.py`](../tests/test_actuators.py) | ID 단계, 다리 매핑, 모델/레지스터 선택 |
| [`test_api.py`](../tests/test_api.py) | facade 수명주기, fake controller, 메뉴 dispatch, 키보드 조이스틱, neutral 종료 |
| [`test_kinematics.py`](../tests/test_kinematics.py) | 단위 변환, FK/IK round trip, 전체 다리 순서 |
| [`test_non_rl_walk.py`](../tests/test_non_rl_walk.py) | idle stance, support height, tripod phase, yaw 접선, IK/batch/simulation stride |
| [`test_simulation.py`](../tests/test_simulation.py) | simulator protocol, Non-RL 설정, RL route |
| [`test_terrain.py`](../tests/test_terrain.py) | 모든 지형 compile, seed 재현성, camera/fixed base/group 설정 |
| [`test_remote_watch.py`](../tests/test_remote_watch.py) | reward 회귀, idle/height, checkpoint 원자성, legacy policy, graceful stop |
| [`test_rl_inquiry.py`](../tests/test_rl_inquiry.py) | 원격 shell command, 입력 안전성, venv, reset backup, standing/reference prompt, SSH 자원 추천 |
| [`test_rl_joystick.py`](../tests/test_rl_joystick.py) | neutral residual gate와 RL Walk→Drive→Climb mode router |
| [`test_rl_reference_motion.py`](../tests/test_rl_reference_motion.py) | hardcoded와 Non-RL 기준 모션의 전진/후진/yaw 부호, lateral 처리 |

## 10. `runs/` 생성 파일

| 경로/패턴 | 역할 |
|---|---|
| `runs/.remote_jobs.json` | 원격 학습 작업 이름, PID, host/path, 생성 시각, curriculum·terrain·stance·reference 설정 |
| `runs/<run>/checkpoints/scone_walk_<steps>_steps.zip` | Stable-Baselines3 PPO checkpoint |
| `runs/remote_watch/*.zip` | watcher가 원격에서 받은 검증 완료 checkpoint |
| `runs/remote_watch/*.zip.part` | 다운로드 중이거나 미완성인 checkpoint; 정책 로드 대상이 아님 |
| `resume.json` 또는 pointer 파일 | 마지막 정상 checkpoint를 원자적으로 가리키는 학습 재개 정보 |

조사 시점에는 다음 run이 있다.

- `walk_easy_20260830_175540`: 6.3M, 6.4M, 6.6M step checkpoint
- `walk_full_20260830_213245`: 100K, 500K step checkpoint
- `walk_full_standard`: 300K, 500K, 6.1M step checkpoint
- `remote_watch`: 20K–240K 구간, 300K–700K 일부, 1.0M–1.26M 구간의 snapshot과 미완성 `260000_steps.zip.part`

각 PPO ZIP에는 policy/optimizer PyTorch state, Stable-Baselines3 metadata, Python 변수, version/system 정보가 포함된다. 대용량 바이너리이므로 문서·소스와 함께 Git에 넣지 않는다.

## 11. `archive/`

### 설계·발표 자산

| 경로 | 역할 |
|---|---|
| `assets/Eco-friendly deliver SCONE Poster.jpg` | 초기 프로젝트 포스터(12000×10000) |
| `assets/Eco-friendly deliver SCONE Quad Chart.png` | 초기 프로젝트 quad chart(3960×3060) |
| `assets/SCONEv2 Arc-Shaped Wheel.pdf` | 호형 바퀴 치수 도면. 직경·폭·반경·호각 참고 |

### 논문과 영상

| 경로 | 역할 |
|---|---|
| `papers/Eco-friendly deliver SCONE Paper.pdf` | 6쪽 영문 초기 프로젝트 논문. 구형 하드웨어 결과와 한계 |
| `papers/대한민국_한국디지털미디어고등학교_김형석_논문.pdf` | 10쪽 국문 초기 프로젝트 논문 |
| `videos/SCONEv1.mp4` | SCONEv1 실험 영상 |
| `videos/SCONEv2.mp4` | SCONEv2 일반 동작 영상 |
| `videos/SCONEv2_stairs.mp4` | SCONEv2 계단 동작 영상 |

### 구형 코드와 메시

| 경로 | 역할 |
|---|---|
| [`codes/SCONEv1.py`](../archive/codes/SCONEv1.py) | 초기 단일 파일 controller/gait와 MobileNetSSD 카메라 탐지. 현재 runtime에는 사용하지 않음 |
| `meshes/SCONEv1.obj`, `SCONEv1.mtl` | 구형 전체 모델과 재질 |
| `meshes/SCONEv2.obj`, `SCONEv2.mtl` | SCONEv2 전체 모델과 재질. 런타임은 분할 STL 사용 |

### `archive/ICRA/`

| 경로 | 역할 |
|---|---|
| [`README.md`](../archive/ICRA/README.md) | ICRA 2027 익명 8쪽 초안 작업 원칙 |
| [`EVIDENCE_PLAN.md`](../archive/ICRA/EVIDENCE_PLAN.md) | 주장별 필요한 로그·실험 증거, 역사 속도 수치 충돌 기록 |
| [`root.tex`](../archive/ICRA/root.tex) | 논문 top-level LaTeX |
| `sections/01_introduction.tex` | 문제·기여 초안 |
| `sections/02_related_work_problem.tex` | 관련 연구와 문제 정의 |
| `sections/03_system_design.tex` | 하드웨어/소프트웨어 설계 |
| `sections/04_locomotion_control.tex` | legacy, IK, residual RL 제어 |
| `sections/05_experimental_method.tex` | 같은 하드웨어 실험·ablation 계획 |
| `sections/06_results.tex` | 결과 표/수치 자리. 현재 TODO 중심 |
| `sections/07_discussion_conclusion.tex` | 한계와 결론 초안 |
| [`references.bib`](../archive/ICRA/references.bib) | 참고문헌 BibTeX |
| [`Makefile`](../archive/ICRA/Makefile) | Tectonic 기반 빌드 명령 |
| `ieeeconf.cls`, `IEEEtran.bst` | IEEE 형식 제3자 템플릿/참고문헌 스타일 |
| `figures/README.md` | 향후 figure 파일 규칙 |
| `build/root.{pdf,aux,bbl,blg,log}` | LaTeX 생성 결과와 빌드 로그. 원본이 아닌 재생성 가능 파일 |

## 12. 생성·캐시·도구 폴더

| 경로/패턴 | 의미와 처리 |
|---|---|
| `**/__pycache__/*.pyc` | Python bytecode cache; 삭제해도 재생성됨 |
| `**/.DS_Store` | macOS Finder metadata; 소스 아님 |
| `.claude/worktrees/` | 도구가 만든 다른 checkout 사본; 현재 작업 트리의 기준으로 분석하지 않음 |
| `tmp/pdfs/` | 문서 조사 중 PDF를 페이지 이미지로 렌더링한 임시 산출물 |
| `archive/ICRA/build/` | LaTeX 빌드 산출물 |
