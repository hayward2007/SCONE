# 아키텍처와 데이터 흐름

## 1. 계층 구조

```text
사용자 / CLI / 외부 Python 코드
            │
     SCONE facade · mode objects
            │
  ControllerProtocol (공통 계약)
       ┌────┴───────────┐
       │                │
DynamixelController  MuJoCoController
       │                │
 serial bus        DC motor/PID + physics
```

`src/main.py`는 로봇 수명주기와 모드 전환을 관리하고, locomotion 객체는 `ControllerProtocol`만 호출한다. 따라서 실제 장치와 시뮬레이터의 차이는 controller 경계 아래에 머문다.

## 2. 실제 로봇 실행 흐름

1. CLI가 `discover_device()`로 후보 serial port를 조사한다.
2. 대표 액추에이터 ID를 ping한 포트를 선택한다.
3. `DynamixelController`가 포트와 Protocol 1.0/2.0 packet handler를 준비한다.
4. `SCONE.initialize()`가 torque-off, operating mode, acceleration, 초기 위치, 속도, torque-on 순서로 설정한다.
5. `Walk`, `Drive`, `Climb` 또는 `NonRLWalkController`가 목표 위치/속도를 전송한다.
6. 종료 시 안전한 중간 자세로 이동하고 모든 torque를 해제한 뒤 포트를 닫는다.

장치별 주소 차이는 `ActuatorModel`과 `ControlTable`이 캡슐화한다. 동기 쓰기는 같은 protocol/register/byte length인 motor를 묶기 때문에 MX와 XM 명령이 잘못된 레지스터로 섞이지 않는다.

## 3. 시뮬레이션 실행 흐름

1. `build_model()`이 MJCF를 파싱해 floating-base 여부, floor, terrain XML을 반영한다.
2. STL 파일을 MuJoCo in-memory assets로 전달하고 모델을 compile한다.
3. `MuJoCoController`가 `A01_`부터 `A18_`까지 actuator 이름을 찾아 연결된 joint의 qpos/dof 주소를 저장한다.
4. controller가 안정 자세로 joint를 seed하고 floating body를 floor 위로 들어 올린다.
5. locomotion 또는 RL이 controller의 목표값을 변경한다.
6. 매 physics step마다 profile generator → position/velocity setpoint → DC motor/PID → voltage → MuJoCo control 순으로 갱신한다.
7. `mujoco.mj_step()`이 물리 상태를 전진시키고 passive viewer가 이를 표시한다.

시뮬레이션 loop는 MuJoCo timestep에 맞춰 pace한다. viewer/CLI thread가 있어도 메인 물리 loop가 lock을 독점하는 tight loop가 되지 않게 하는 것이 중요하다.

## 4. Non-RL 보행 데이터 흐름

```text
(vx, vy, yaw_rate)
   → clamp / low-pass filter
   → 모터 속도 한계에 맞춘 고정 cadence와 다리별 tripod phase
   → stance 또는 swing 발 목표점
   → body yaw가 만드는 접선 속도 합성
   → 전후/측면 타원형 보폭 제한
   → 각 다리 DLS inverse kinematics
   → 실패 시 nominal 발 위치 쪽으로 adaptive backoff 후 재시도
   → radian → degree → Dynamixel raw
   → 18개 목표를 batch 전송
```

각 다리의 neutral support point는 tire contact mesh의 부채꼴 끝단 중 가장 낮은 0.1 mm 패치의 중심에서 추론한다. 한 모서리 vertex를 고르면 44 mm 폭의 한쪽으로 IK가 치우쳐 접지 모멘트와 slip이 생긴다. stance에서는 명령 반대 방향으로 발이 지면을 민다. swing에서는 quintic 보간과 lift를 사용해 다음 접촉점으로 이동한다. 실물 공용 cadence는 0.8 Hz를 유지하고, 시뮬레이션/RL은 actuator 속도 profile sweep에서 선택한 0.7 Hz와 전후 60 mm·측면 50 mm 작업공간, 최대 4회 IK backoff를 사용한다. 그래도 수렴하지 않으면 마지막 유효 관절각을 유지하고 frame을 실패로 보고한다.

## 5. RL 환경 데이터 흐름

### 한 control step

1. 외부 velocity command를 안전 범위로 자르고 필터링한다.
2. 선택한 reference가 `non_rl`이면 Phoenix식 발 궤적과 IK로, `hardcoded`이면 사인파 tripod로 기준 관절 목표를 계산한다.
3. reference가 관리하는 gait phase를 정책 관측과 맞춘다.
4. 정책의 18차원 `[-1, 1]` action을 관절별 residual degree로 스케일한다.
5. reference + residual을 임시 joint range로 자르고 controller에 전송한다.
6. `frame_skip`번 physics step을 실행한다.
7. world-frame 속도를 body frame으로 회전해 관측을 만든다.
8. 접촉·자세·전류·명령 추종을 측정해 reward와 종료 여부를 계산한다.

### 70차원 관측

| 구간 | 차원 | 내용 |
|---|---:|---|
| body linear velocity | 3 | body frame, `/2` |
| body angular velocity | 3 | body frame, `/5` |
| projected gravity | 3 | body frame의 중력 방향 |
| joint position error | 18 | 기준 자세 대비, `/π` |
| joint velocity | 18 | `/10` |
| previous action | 18 | 직전 residual action |
| command | 3 | 각 최대 명령으로 정규화 |
| gait phase | 2 | `sin`, `cos` |
| heading error | 2 | `sin`, `cos` |

합계는 `3+3+3+18+18+18+3+2+2 = 70`이다. 구형 68차원 정책은 마지막 heading 두 항이 없으며 호환 adapter가 재생 시에만 두 항을 제거한다.

## 6. CLI와 스레드 경계

- 터미널 입력은 공통 CLI 계층만 소유한다. viewer 또는 controller가 별도로 키보드를 읽지 않는다.
- `KeyboardJoystick`는 key press/release 상태를 속도 벡터로 변환하고 일정 timeout 뒤 neutral로 복귀한다.
- legacy velocity adapter는 background worker가 최신 명령만 소비한다.
- 시뮬레이터에서는 물리 update가 주 thread, CLI가 worker thread다.
- RL 조종에서 `R`을 누르면 PPO 목표 출력을 멈추고 같은 MuJoCo controller로 Legacy Drive/Climb 전환을 수행한다. Walk로 돌아오면 heading·높이·residual 상태를 재정렬한 뒤 정책을 재개한다.
- 원격 감시에서는 checkpoint poller가 다운로드·검증하고 viewer/control loop가 안전한 시점에 policy를 바꾼다.
- 공유 controller 상태는 lock으로 보호하지만, 긴 I/O나 sleep을 lock 안에서 수행하지 않는 것이 원칙이다.

## 7. 병렬 학습과 SSH 자원 추천

`num_envs=1`은 현재 프로세스의 `DummyVecEnv`를 사용한다. 2개 이상이면 환경마다 별도 프로세스를 만드는 `SubprocVecEnv`를 사용하고, 환경 index만큼 terrain seed를 증가시킨다. PPO parent가 각 worker의 rollout을 모아 업데이트한다.

원격 CLI는 학습 설정을 묻기 전에 SSH 머신의 물리/논리 코어, 가용 메모리, 1분 load를 조회한다. 추천값은 `min(물리 코어-1, (가용 메모리-2 GiB)/768 MiB)`의 정수 하한이며 최소 1이다. 이는 안전한 출발값이지 성능 보장은 아니므로, 다른 작업 부하와 실제 FPS를 보고 조정한다.

## 8. 파일 의존 방향

```text
hardware.interface
    ↑
hardware.controller / simulation.core.controller
    ↑
locomotion.* ─────── kinematics.*
    ↑                    ↑
main / cli       non_rl_walk / rl.walk_learn
                         ↑
                 rl.joystick_control
                 rl.remote_watch
                 rl.inquiry
```

루트 `SCONE.py`와 `src/__init__.py`는 사용자가 내부 패키지 경로를 알 필요가 없게 public symbol을 다시 노출한다. `src/simulation`의 얇은 shim 모듈도 과거 import 경로를 보존한다.
