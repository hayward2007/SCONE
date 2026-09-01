# 문제와 해결 과정

이 문서는 코드, 테스트, [`08-rl-development-log.md`](08-rl-development-log.md), Git 기록, 논문 증거 계획을 함께 비교해 정리했다. 과거 진단이 현재 구현과 다르면 현재 코드 상태를 별도로 적었다.

## 1. floating-base 모델이 바닥으로 무너지는 문제

### 증상

- floating base를 켜면 초기화 직후 몸체가 바닥으로 내려앉거나 관절이 목표를 유지하지 못했다.
- 겉으로는 PID gain 문제처럼 보였지만 gain만 높여도 안정적으로 해결되지 않았다.

### 진단

- visual mesh와 collision mesh가 섞여 불필요한 접촉이 발생했다.
- 모델의 초기 qpos가 서 있는 자세가 아니었다.
- position actuator가 물리적인 motor 한계를 우회하거나, 반대로 motor torque가 현재 중량을 버티지 못했다.
- floor 높이를 하드코딩하면 메시 원점과 실제 접촉 최저점이 맞지 않았다.

### 해결

- tire와 필요한 body geom만 collision group으로 분리하고 visual geom의 contact를 끈다.
- controller 생성 시 Standard 계열 안정 자세를 qpos에 seed한다.
- floating freejoint 높이를 최저 contact mesh와 floor 기준으로 조정한다.
- position actuator 대신 motor actuator + `DCMotorPID`를 사용해 12 V, stall torque, no-load speed를 반영한다.
- floor를 contact mesh의 최저 정점에서 자동 계산한다.

### 현재 상태

시뮬레이션의 초기 안정성은 개선되었지만 collision geometry와 질량/마찰의 실제 측정값은 더 검증해야 한다. gain을 올리는 것보다 actuator saturation, payload, 접촉 형상을 함께 확인해야 한다.

## 2. 왼쪽 다리의 방향과 대칭 문제

### 증상

같은 raw 목표를 주었는데 좌우 다리가 같은 기하학적 방향으로 움직이지 않거나, 특정 관절 단계만 거울상과 반대로 움직였다.

### 실패했던 접근

과거에는 actuator ID 배열을 mirror pair로 통째로 재배열하는 방법을 검토했다. 그러나 좌우 반전은 모든 관절에 동일하지 않고 stage별 joint axis가 다르므로, 전역 ID swap은 일부 관절을 고치는 대신 다른 관절을 틀리게 만든다.

### 해결

- Fusion/MJCF의 실제 joint axis를 단계별로 검사한다.
- 왼쪽의 필요한 joint(`M02`, `M04`, `M06`, `M14`, `M16`, `M18`)만 XML axis에서 보정한다.
- 시뮬레이터는 배열 위치가 아니라 actuator 이름과 연결 joint로 매핑한다.
- 실제 Dynamixel controller의 ID/명령 변환은 변경하지 않는다.

### 현재 상태

현재 코드는 과거의 고정 mirror-pair remap을 사용하지 않는다. 좌우 보정은 모델 계층에 있으며, 최종 실물 캘리브레이션 전까지 하드웨어 방향 변환을 성급히 추가하면 안 된다.

## 3. Fusion 부품 번호와 모터 ID의 불일치

### 문제

CAD export 이름, 다리 번호, actuator ID가 서로 다른 규칙을 사용하면 메시와 joint를 잘못 연결하기 쉽다.

### 해결

- MJCF joint는 `M01` … `M18`, actuator는 `A01_...` prefix를 사용한다.
- 코드의 다리→모터 규칙은 `(leg, leg+6, leg+12)`로 한곳에 둔다.
- model/kinematics/controller 모두 이름 조회를 사용하고 XML element 순서에는 의존하지 않는다.
- export report는 provenance 진단에만 쓰고 런타임 모델 입력으로 취급하지 않는다.

## 4. 물리 loop가 멈춘 것처럼 보이는 문제

### 증상

시뮬레이션 viewer와 키보드 CLI를 함께 실행할 때 화면이 멈추거나 입력 반응이 크게 늦었다.

### 원인

물리 update의 tight loop가 lock/CPU를 계속 점유해 viewer와 CLI thread가 실행될 기회를 잃었다.

### 해결

- MuJoCo timestep을 기준으로 실시간 pacing한다.
- terminal 입력은 공통 CLI thread 한 곳만 소유한다.
- controller lock은 상태 복사와 짧은 update 구간에만 사용한다.
- 종료할 때 neutral command를 명시적으로 전송한다.

## 5. Non-RL gait의 IK 불안정과 stride 범위

### 문제

큰 stride에서 일부 다리 IK가 수렴하지 않거나 관절 목표가 0–360 degree를 벗어났다. 실제 프로필과 시뮬레이션의 안전 범위도 달랐다.

### 해결

- damped least-squares, joint step 제한, backtracking을 사용한다.
- 모든 18개 결과가 finite, converged, 유효 범위일 때만 batch 전송한다.
- 실패한 다리는 마지막 유효 관절각을 유지한다.
- RL reference는 checkpoint 호환을 위해 0.7 Hz와 전후 `0.060 m`·측면
  `0.050 m`를 유지한다. 비-RL MuJoCo 조종은 후속 sweep에서 0.8 Hz와
  `0.080/0.060 m`로 분리했다.
- 복합 명령의 특정 다리 IK가 실패하면 nominal 발 위치 쪽으로 0.8배씩 최대 4회 backoff해 다시 푼다.
- 회전은 각 발의 위치에 `(-yaw*y, yaw*x)` 접선 속도를 더해 만든다.

초기 sweep는 RL 환경의 무제한 관절 profile 속도를 사용해 1.4 Hz를 잘못 채택했다. 실제 CLI와 같이 `walking_speed=100`으로 초기화한 별도 controller에서 6초간 최대 전진을 비교하면 1.4 Hz는 `0.0143 m/s`, 0.8 Hz는 `0.0502 m/s`였다. 또한 support point를 말단 패치 중심으로 교정하고 0.7 Hz를 적용했다. 같은 물리 profile의 Standard zero-action 500-step에서 Non-RL은 `0.05602 m/s`, hardcoded는 `0.01633 m/s`였고 slip penalty도 Non-RL이 약 42% 작았다. 다만 이 profile을 기존 PPO 재생에 뒤늦게 강제하면 학습 동역학이 달라져 15.4M policy가 전진 대신 후진했다. RL reset은 기존 checkpoint 호환을 위해 무제한 profile을 유지하고, 물리 profile 학습은 별도 버전으로 0 step부터 시작해야 한다. Sport는 느려 Standard를 권장한다.

## 6. body velocity 좌표계 오류

### 문제

MuJoCo/Fusion 모델의 local 축을 그대로 body velocity라고 가정하면 로봇이 회전했을 때 전진·측면 속도 부호가 틀어진다. 이 오류는 reward가 실제 이동 방향과 반대로 정책을 강화할 수 있어 특히 위험하다.

### 해결

- freejoint/world velocity를 명시적으로 구한다.
- body orientation의 역회전을 적용해 world vector를 body frame으로 변환한다.
- forward/reverse/yaw reference motion의 부호를 별도 테스트로 고정한다.

## 7. RL 보상의 정지·높이·중복 계산 문제

### 과거 문제

- 높이 오차의 양쪽을 벌주면 약간 높아지는 동작도 불필요하게 감점됐다.
- 일반 velocity reward만으로는 zero command에서 작은 진동과 residual action을 충분히 억제하지 못했다.
- 세부 term과 aggregate term을 모두 합하면 같은 reward가 두 번 계산될 위험이 있었다.
- heading 정보가 관측에 없어 yaw-rate가 0인 경우 장기 heading drift를 직접 보정하기 어려웠다.

### 해결

- height penalty는 기준 높이 아래로 내려간 양만 제곱한다.
- idle 판정, 작은 sigma의 idle velocity tracking, idle action penalty를 추가한다.
- reward 합계는 `velocity`, `direction`, `stability`, `damping` aggregate 네 값만 한 번씩 더한다. 세부 term은 진단 로그용이다.
- 관측에 heading error의 `sin/cos` 두 항을 추가해 68→70차원으로 확장했다.
- 구형 68차원 policy는 replay adapter만 제공하고 새 환경 학습에는 연결하지 않는다.
- neutral command에서는 `NeutralResidualGate`가 residual을 즉시 억제하거나 감쇠한다.

현재 정확한 수식과 가중치는 [보상함수 수정 가이드](06-reward-function-guide.md)를 기준으로 한다.

## 8. 접지 미끄러짐 측정의 부정확성

### 문제

body center velocity나 단순 foot body velocity는 실제 타이어-지면 접촉점의 미끄러짐을 나타내지 못한다.

### 해결

- MuJoCo contact에서 tire contact point와 normal force를 찾는다.
- contact point Jacobian으로 해당 점의 world velocity를 계산한다.
- normal 성분을 빼고 접선 속도만 남긴다.
- `1 N` 이상의 접촉만 사용하고 작은 deadzone 뒤 slip penalty를 계산한다.

## 9. checkpoint 손상과 동시 접근

### 문제

학습이 ZIP을 쓰는 중 원격 watcher가 읽거나, 네트워크 다운로드가 끊기면 불완전한 checkpoint가 정상 파일 이름으로 남을 수 있다.

### 해결

- 임시/`.part` 파일로 먼저 저장·다운로드한다.
- ZIP 필수 entry를 검사한다.
- 검증 뒤 `os.replace` 계열의 원자적 교체를 사용한다.
- resume pointer도 임시 파일 뒤 교체한다.
- reset은 기존 run을 삭제하지 않고 `.reset_backup`으로 이동한다.
- 원격 pause는 우선 SIGTERM으로 정상 저장 기회를 주며 즉시 SIGKILL하지 않는다.

## 10. 역사 문서와 현재 코드의 불일치

### 발견된 예

- 개발 기록의 일부 reward weight는 현재 `RewardConfig` 기본값과 다르다.
- 구형 논문과 아카이브에는 속도가 `0.05`, `0.07`, `0.5`, `0.7 m/s` 등 서로 다르게 기록되어 있다.
- ICRA 초안의 정량 결과는 아직 `TODO`이고 evidence matrix가 완성되지 않았다.
- 구형 SCONEv1 코드는 AX/MX 혼합, 재귀 gait, MobileNetSSD 카메라를 포함하지만 현재 SCONEv2 runtime 구조가 아니다.

### 문서화 원칙

- 실행 동작은 현재 코드와 테스트를 기준으로 설명한다.
- 논문 수치는 원 로그·영상·실험 설정으로 재구성되기 전에는 확정 주장으로 쓰지 않는다.
- history는 “왜 현재 구조가 되었는가”를 설명하는 근거로만 사용한다.

## 11. 시뮬레이션 모드 전환과 Drive 흔들림

### 문제

- 고정 sleep만 사용한 Walk→Drive→Climb 전환은 물리 관절이 목표에 도달하기 전에 다음 단계로 넘어갈 수 있었다.
- MuJoCo의 좌우 말단 관절축은 같은 raw velocity가 같은 지면 이동을 뜻하지 않아 Drive가 서로 밀어내는 방향으로 돌 수 있었다.
- Drive 중 하중을 받는 1단 관절(ID 7–12)이 목표 주변에서 흔들렸다.
- Standard 프로필의 기존 Climb 준비각은 Drive 중앙 자세에서 발을 들어 올리는 방향과 맞지 않았다.

### 해결

- 실물 controller API는 유지하고, MuJoCo controller가 제공할 때만 목표 도달 대기를 사용한다.
- 시뮬레이션의 짝수 말단 ID 속도 부호를 뒤집어 좌우 바퀴의 지면 이동 방향을 맞춘다.
- Drive에 들어간 동안만 ID 7–12의 `kd`를 2배로 적용하고 Walk/Climb에서 복구한다.
- 시뮬레이션 Climb 준비각만 160°로 보정하고, 실물의 검증된 프로필 명령은 변경하지 않는다.

측정 sweep에서 2배 댐핑은 기본값 대비 1단 RMS 속도를 약 18%, 최대 각도 오차를 약 13% 줄였다. 4배는 진동 영역으로 넘어갔으므로 채택하지 않았다. 이 수치는 해당 MuJoCo 설정의 비교값이며 실물 성능 주장으로 사용하지 않는다.

## 12. 병렬 환경 수가 실제 병렬이 아니었던 문제

기존 `num_envs`는 `DummyVecEnv` 안에서 환경을 순서대로 실행해 CPU 코어를 실질적으로 활용하지 못했다. 한 환경은 기존 동작을 유지하고, 두 개 이상은 `SubprocVecEnv` worker로 분리했다. SSH CLI도 물리 코어 하나와 메모리 2 GiB를 남긴 보수적 추천값을 계산하되 사용자가 수정할 수 있게 했다.

## 13. 계단 연속 회전과 후킹 assist 선택 문제

아래 120 mm까지의 비교는 최초 controller 선택 당시 기록이다. 현재
100/150/200 mm 재검증과 shallow-tread 실패 sweep는 이 절 끝과
11번 문서 12절을 기준으로 본다.

### 비교한 접근

- 여섯 C-sector를 같은 속도로 계속 돌리는 pure rolling
- 기존 Legacy `Climb`
- 항상 큰 middle 관절 동작을 쓰는 tripod hook
- 항상 작은 tripod 보조를 쓰는 hybrid
- 쉬운 단에서는 rolling을 유지하고 높은 단/정체에만 hybrid를 켜는 adaptive
  `scone-stair`

### 발견한 문제와 해결

- pure rolling은 `stairs-1/2`에서 가장 빠르고 적은 일을 사용했지만 120 mm
  rise가 있는 `stairs-3`에서 정체됐다.
- 기존 Legacy `Climb`은 현재 side-on procedural stair의 진행 phase와 맞지
  않아 세 preset 모두 제한 시간에 정상부 판정을 통과하지 못했다.
- 상시 tripod/hybrid는 높은 단을 통과했지만 쉬운 계단에서도 middle 관절을
  불필요하게 움직여 시간과 절대 기계 일이 증가했다.
- rolling 중인 여섯 하단 관절에 곧바로 서로 다른 tripod 속도를 섞은 초기
  prototype은 첫 지지 교대에서 옆으로 넘어졌다. assist 진입 전에 하단
  속도를 0으로 보내 phase를 동기화하고 0.18초 smoothstep을 적용했다.
- 알려진 높은 단 조건을 assist 종료 뒤에도 계속 평가하면 같은 구간에서
  assist가 반복됐다. known pre-hook은 한 번만 사용하고 이후에는 정체
  detector만 재진입을 허용한다.

현재 높이에서는 100 mm만 pure/adaptive가 모두 4.920초에 통과했다. pure는
150/200 mm에서 실패했고 adaptive는 각각 12.682초/assist 2회,
14.394초/assist 3회로 통과했다. 200 mm는 기존 170--240 mm tread에서
실패해 350 mm support tread로 바꾼 조건이다. 이 결과는 현재 MuJoCo preset
한 번씩의 결정론적 비교로, 실물 성공률이나 일반 계단 보장이 아니다.
수식·전체 표·실패 로그·실물 진입 조건은
[`11-scone-stair-climbing.md`](11-scone-stair-climbing.md)에 있다.

## 14. 비-RL 보행의 처짐·속도·가짜 rolling 문제

### 증상

- 세 다리 support가 바뀔 때 MuJoCo 차체가 실물보다 크게 내려가거나 출렁였다.
- `tripod-gait`가 PPO보다 답답하고 최대 명령을 올려도 보폭이 거의 같았다.
- 기존 `scone-gait`가 lower를 약 30° 왕복할 뿐이라 일반 tripod와 외형·성능이
  비슷했고 C자 말단을 바퀴처럼 사용하지 못했다.

### 실패와 원인

- `qfrc_bias` 중력 feed-forward는 contact constraint와 PID 보상을 중복해 root
  높이와 middle tracking error를 악화시켜 제거했다.
- speed 175/200과 cadence 0.85/0.9는 더 높은 명령인데도 slip과 20–28 mm
  하방 drop 때문에 느려졌다.
- lower 여섯 개를 같은 phase로 연속 회전하면 135° 개구부가 동시에 지면을
  향해 root Z가 63.5 mm 빠졌다.
- 여섯 다리 arbitrary phase는 0.1835 m/s로 빨랐지만 44.4 mm lateral drift와
  upright 0.964로 기각했다.
- 후속 직진 진단에서 0.8 Hz/80 mm 경로는 최대 명령의 약 97% frame에서
  stride가 잘렸고 lower 목표가 nominal 대비 최대 약 94°까지 다른 IK branch로
  이동했다. 8초 동안 역방향 이동 24.5 mm, 측면 편향 51.8 mm, 최대 yaw
  3.38°가 측정되어 “평균 전진”만으로는 문제를 발견할 수 없었다.
- 최초 continuous-roll 구현은 18관절 `SconeGait` 결과 중 1–12번만 보내고
  13–18번 기본 보행 position을 버렸다. 그래서 상·중단 진폭도 25/4 mm로
  작고 화면에서는 사실상 말단 회전만 보였다.

### 해결

- 비-RL tripod를 1.0 Hz, 90/70 mm, lift 25 mm로 바꾸고 simulation profile
  limiter를 해제했다. 모터 PID·전압·토크 한계와 middle stiffness 2배는
  유지한다. PPO replay와 실물은 그대로다.
- interactive `scone-gait`를 `SconeRollingGait`로 route해 lower를 velocity mode로
  연속 회전시켰다.
- `scone-gait` 기본 보행을 stride/lift 55/20 mm로 키우고, lower bounded 목표의
  시간 미분을 0.35배로 연속 회전 속도에 합성했다. 상단·1단·2단 기본 보행과
  회전이 동시에 남는다.
- tripod B `(2,3,6)` lower 시작각을 +60° 벌려 개구부 support phase를
  de-synchronize했다.
- RL의 bounded `SconeGait` reference는 checkpoint action 의미 때문에 보존했다.
- 자동 계단 데모를 조종 메뉴와 분리해 hardcoded/improved/compare를 입력 없이
  실행하게 했다.

수정 후 tripod는 8초 0.9469 m(0.1184 m/s), 역방향 누적 3.7 mm, 측면
0.7 mm, 최대 yaw 1.17°였고 20초에도 측면 5.0 mm/IK 실패 0이었다.
full-body continuous roll은 6초 1.2556 m(0.2093 m/s), lower 평균 3.09회,
최소 upright 0.9847, IK 실패 0이었다. 전체 sweep과 식은
[`12-automatic-stair-demo-and-continuous-roll-rework.md`](12-automatic-stair-demo-and-continuous-roll-rework.md)에 있다.

## 15. 남아 있는 기술 부채와 권장 순서

1. 실제 관절별 mechanical range와 zero offset을 측정해 임시 `±60°/90°` 제한을 교체한다.
2. link별 collision geom과 self-collision filter를 실제 형상에 맞게 검증한다.
3. 실물 IMU/odometry 기반으로 RL 관측과 reward에 대응하는 state estimator를 만든다.
4. motor 전류·전압·온도 안전 제한과 비상정지를 RL 배포 계층 앞에 둔다.
5. latency, sensor noise, friction, payload, motor strength randomization을 추가한다.
6. 통합 requirements의 버전 lock과 새 macOS/Linux 환경 설치 smoke를 자동화한다.
7. reward ablation과 같은-hardware 반복 실험으로 각 논문 주장을 검증한다.
8. 실물 stair rise/tread/nosing/friction과 모터 전류를 계측한 뒤
   `scone-stair`의 torque·support margin·비상정지를 재튜닝한다.
