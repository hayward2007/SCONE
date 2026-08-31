# 기능과 사용 기술

## 1. 기능 목록

### 하드웨어 제어

- Dynamixel SDK를 런타임에 로드하고 장치 포트를 연다.
- MX-28AT(Protocol 1.0)와 XM430 계열(Protocol 2.0)을 모델/ID별로 분리한다.
- torque, operating mode, goal position, goal velocity, profile velocity/acceleration을 동기 쓰기 한다.
- 서로 다른 프로토콜과 레지스터를 `(protocol, register)` 그룹으로 묶어 동일 API로 전송한다.
- 대표 ID 1, 7, 13을 ping해 포트를 찾으며 탐색 단계에서는 torque나 position을 변경하지 않는다.
- context manager와 명시적 `shutdown()`으로 안전 자세와 torque-off를 보장한다.

### 로봇 동작

- `Standard`와 `Sport` 모션 프로필을 제공한다.
- tripod 기반 전진·후진·좌회전·우회전 동작을 지원한다.
- 하단 관절을 velocity mode로 바꾸는 바퀴 주행을 지원한다.
- 등반 준비 자세와 좌·우 교대 등반 동작을 제공한다.
- 연속 velocity command를 legacy discrete gait 또는 IK gait에 연결한다.
- 명령 필터링, stride 제한, swing lift, tripod phase로 연속 보행을 생성한다.
- 시뮬레이션 Legacy/RL 조종 중 `R`로 Walk→Drive→Climb→Walk 상태를 전환한다.

### 기구학

- MuJoCo MJCF에서 18개 관절, qpos/dof 주소, 다리 root body를 이름으로 찾는다.
- 한 다리와 전체 로봇에 대해 forward kinematics를 계산한다.
- 3×3 translational Jacobian과 damped least-squares IK를 제공한다.
- 최대 관절 step, joint limit, backtracking으로 불안정한 IK update를 억제한다.
- radian, degree, Dynamixel raw 위치 사이를 변환한다.

### 시뮬레이션

- fixed-base/ floating-base 모델을 선택해 MuJoCo 모델을 빌드한다.
- 모델의 최저 contact mesh를 기준으로 floor를 자동 배치한다.
- 외부 STL을 메모리 asset으로 주입해 임시 XML을 디스크에 만들지 않고 compile한다.
- position actuator 대신 motor actuator와 DC motor/PID를 조합해 전압·토크 포화를 반영한다.
- 실제 제어기와 같은 position/velocity/torque/mode API를 제공한다.
- passive viewer, 로봇 추적 카메라, collision group 표시를 지원한다.
- 평지, 불규칙 블록, 계단, 경사, 혼합 지형을 XML로 생성한다.
- Drive에서만 1단 관절 댐핑을 2배로 높이고, 모드를 벗어나면 원래 값으로 복구한다. 이 보정은 MuJoCo controller에만 있다.
- legacy 모드 전환 시 고정 sleep 뒤 바로 다음 명령을 보내지 않고, 시뮬레이션 관절이 허용 오차 안에 도달했는지 기다린다.

### 강화학습

- Stable-Baselines3 PPO와 Gymnasium 환경을 사용한다.
- 70차원 관측과 18차원 residual action을 사용한다.
- 연속 IK 기반 `non_rl`과 기존 사인파 `hardcoded` 중 residual reference를 선택한다. 새 CLI 실행은 `non_rl`을 권장 기본값으로 사용한다.
- reference gait, 명령 curriculum, idle 구간, randomized gait frequency를 사용한다.
- 실제 접촉점 Jacobian으로 tire slip을 측정한다.
- motor voltage/back-EMF/resistance에서 normalized current penalty를 계산한다.
- forbidden body collision, 넘어짐, hard joint limit을 episode 종료 조건으로 사용한다.
- 68차원 구형 정책은 관측을 변환해 재생만 허용하고 재학습에는 사용하지 않는다.
- 체크포인트를 원자적으로 저장·검증하고 오래된 파일을 정리한다.
- neutral 명령에서 residual을 억제/감쇠해 정지 안정성을 높인다.
- `num_envs=1`은 `DummyVecEnv`, 2개 이상은 `SubprocVecEnv`를 사용해 실제 프로세스 병렬 학습을 수행한다.

### 운영 도구

- InquirerPy 기반 메뉴와 argparse 직접 명령을 함께 제공한다.
- 키보드 escape sequence를 속도 벡터로 바꾸고 키가 풀리면 자동 중립 복귀한다.
- SSH/rsync로 원격 학습을 시작, 상태 확인, 일시정지, 재개, 다운로드한다.
- SSH 머신의 코어·가용 메모리·load를 읽어 병렬 환경 수를 추천하며 사용자가 최종 값을 수정할 수 있다.
- 원격 run reset은 삭제 대신 `.reset_backup`으로 이동한다.
- 다운로드 중인 `.part` 파일은 완전한 ZIP 검증 후 원자적으로 교체한다.
- 로컬 viewer 실행 중 새 체크포인트를 감지해 policy를 hot-swap한다.

## 2. 주요 기술과 역할

| 기술 | 이 프로젝트에서의 역할 |
|---|---|
| Python 3 | 전체 API, 제어, 시뮬레이션, 학습, CLI |
| Dynamixel SDK | 실제 serial bus, ping, sync write/read |
| MuJoCo | 강체 동역학, 접촉, 센서 상태, Jacobian, viewer |
| NumPy | 벡터/행렬 계산, 필터, gait와 reward 계산 |
| Gymnasium | RL 환경의 observation/action/episode 계약 |
| Stable-Baselines3 | PPO 정책 학습, 저장, 로드, 재개 |
| PyTorch | Stable-Baselines3 정책/가치망 backend |
| InquirerPy | 프로필, 장치, 지형, 학습 작업 대화형 선택 |
| POSIX terminal/select | raw terminal byte 입력과 key-repeat 기반 self-centering joystick |
| OpenCV/imutils | 구형 SCONEv1 카메라 객체 탐지 코드에만 사용하며 현재 요구사항에는 없음 |
| XML/MJCF | 로봇 링크, 관절, actuator, contact 모델 정의 |
| STL/OBJ/MTL | 시뮬레이션 메시와 설계/논문 시각 자료 |
| SSH/rsync/nohup | 원격 학습 배포와 장시간 작업 관리 |
| unittest | 모듈·통합·회귀 테스트 |
| LaTeX/IEEEtran/Tectonic | ICRA 논문 초안과 PDF 빌드 |

## 3. 의존성 파일

| 파일 | 목적 |
|---|---|
| [`requirements.txt`](../requirements.txt) | 기본 terminal launcher용 `InquirerPy` |
| [`requirements.txt`](../requirements.txt) | CLI, Dynamixel SDK, Gymnasium, MuJoCo, NumPy, Stable-Baselines3, TensorBoard를 한 파일에서 설치 |

Python 패키지 import는 일부 영역에서 의도적으로 지연된다. 예를 들어 루트 facade와 하드웨어 controller는 MuJoCo나 Dynamixel SDK가 필요하지 않은 작업에서 선택 의존성 때문에 전체 import가 실패하지 않도록 lazy import를 사용한다.

실제 하드웨어 controller가 요구하는 `dynamixel-sdk`는 현재 두 requirements 파일에 포함되어 있지 않다. 새 장치 환경에서는 별도 설치해야 하며, 재현 가능한 배포를 위해 manifest에 명시하는 것이 남은 작업이다.

## 4. 설계상 중요한 기술 선택

### 하나의 controller protocol

상위 계층은 실제 포트나 MuJoCo joint를 직접 알지 않는다. 양쪽 controller가 position, velocity, mode, torque, profile API를 동일하게 제공하므로 gait와 `SCONE` facade가 교체 없이 재사용된다.

### 이름 기반 모델 연결

현재 시뮬레이터와 기구학은 actuator·joint의 배열 순서를 가정하지 않고 `A01_`, `M01` 같은 이름으로 ID를 해석한다. XML 편집으로 element 순서가 바뀌어도 연결이 유지된다.

### 분석적 기준 + 학습 residual

정책 action 전체를 motor target으로 쓰지 않고 이미 작동하는 tripod reference에 제한된 residual을 더한다. 학습 초기의 무작위 정책이 로봇을 즉시 무너뜨릴 위험을 줄이고, action 의미를 각 관절의 보정량으로 제한한다.

### 원자적 체크포인트 처리

학습 저장과 원격 다운로드는 임시 파일을 완성한 뒤 교체한다. 뷰어는 ZIP 구조 검증을 통과한 파일만 읽으므로 저장 중인 checkpoint를 읽는 경쟁 조건을 줄인다.
