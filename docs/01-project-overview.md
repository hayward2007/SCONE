# 프로젝트 개요

## 1. 목적

SCONE은 18개의 Dynamixel 액추에이터를 사용하는 6족 로봇을 하나의 Python 코드베이스에서 제어하고, 같은 상위 제어 인터페이스를 MuJoCo 시뮬레이션과 강화학습에 재사용하기 위한 프로젝트다. 각 다리는 세 관절을 가지며, 보행·바퀴 주행·등반의 세 이동 모드를 지원하는 구조다.

핵심 설계 목표는 다음과 같다.

- 실제 Dynamixel 버스와 MuJoCo 액추에이터를 같은 `ControllerProtocol`로 교체한다.
- 구형 tripod 동작, 역기구학 기반 연속 보행, Residual RL을 함께 유지한다.
- Fusion 360에서 내보낸 모델과 메시를 시뮬레이션·기구학 계산에 재사용한다.
- 키보드/조이스틱 명령, 학습 명령, 원격 체크포인트 감시를 하나의 CLI에서 연결한다.
- 실물 제어기의 ID·프로토콜·레지스터 의미를 시뮬레이션에서도 가능한 한 동일하게 유지한다.

## 2. 하드웨어 개념

로봇은 다리 번호 1–6과 세 관절 단계로 구성된다.

| 단계 | 액추에이터 ID | 역할 | 기본 모델 계열 |
|---|---:|---|---|
| 상단 | 1–6 | 몸체와 다리의 수평/방향 관절 | MX-28AT |
| 중단 | 7–12 | 다리 높이와 중간 링크 관절 | XM430-W350 |
| 하단 | 13–18 | 말단/호형 바퀴 구동 관절 | XM430-W210 |

홀수 다리(1, 3, 5)는 오른쪽, 짝수 다리(2, 4, 6)는 왼쪽이다. 보행 tripod A는 `(1, 4, 5)`, tripod B는 `(2, 3, 6)`이다. 이 규칙은 [액추에이터 인덱스](../src/hardware/actuator_index.py), legacy gait, Non-RL gait, RL reference gait에서 공통으로 사용한다.

## 3. 제공하는 제어 경로

### Legacy 동작

미리 정한 각도와 속도를 순서대로 전송하는 blocking 동작이다. `Walk`, `Drive`, `Climb` 객체가 상태 전환을 담당하며 초기 로봇 동작과 하드웨어 검증에 유용하다. 연속 속도 입력은 `LegacyVelocityController`가 최신 명령을 짧은 동작으로 변환한다.

### Non-RL 연속 보행

`NonRLWalkController`는 명령 `(vx, vy, yaw_rate)`을 받아 50 Hz로 보행 궤적을 만든다. stance와 swing을 부드러운 quintic 곡선으로 잇고, 각 발의 목표점을 3축 damped-least-squares IK로 18개 관절각으로 바꾼 뒤 한 번에 전송한다.

### Residual RL

RL 환경은 선택한 기준 모션 위에 18차원 정책 residual을 더한다. 기본 권장값은 Non-RL과 같은 연속 발 궤적·IK 기준이고, 비교와 구형 실행 호환을 위해 사인파 tripod 기준도 남겨 두었다. 정책이 기준 보행을 완전히 새로 만들기보다 기준 자세의 오차와 동역학 차이를 보정하도록 설계했다. 명령 추종, 방향, 자세 안정, 미끄러짐, 전류, 관절 한계, 충돌, idle 안정성을 보상으로 사용한다.

## 4. 실행 영역

| 영역 | 진입점 | 설명 |
|---|---|---|
| 고수준 API | `SCONE.py`, `src/main.py` | 초기화, 프로필, 동작 모드, 종료 수순 |
| 통합 CLI | `python -m src.cli` | 실제 장치 탐색, 시뮬레이션, RL 메뉴 |
| 시뮬레이터 | `python -m src.simulation` | 제어 방식·지형·체크포인트 선택 |
| RL 학습 | `python -m src.rl.walk_learn` | PPO 학습/재개/체크포인트 저장 |
| RL 재생 | `python -m src.rl.joystick_control` | 로컬 정책을 실시간 명령으로 재생 |
| 원격 감시 | `python -m src.rl.remote_watch` | SSH 체크포인트 동기화와 hot swap |
| 테스트 | `python -m unittest discover -s tests` | 하드웨어 추상화, IK, gait, 시뮬레이션, RL 운영 검증 |

## 5. 현재 완성 범위

구현되어 테스트되는 범위:

- 실제 Dynamixel 모델별 Protocol 1.0/2.0 레지스터 처리
- 실제 포트 탐색과 안전한 초기화·종료
- 6족 FK/IK와 actuator-order 변환
- legacy/Non-RL/RL 제어 경로
- 평지, 계단, 경사, 혼합 지형 생성
- MuJoCo motor + 자체 DC motor/PID 모델
- PPO 학습, 체크포인트 재개·정리, legacy 정책 재생 호환
- 원격 학습 작업 생성·일시정지·재개·다운로드·실시간 감시
- 실제 subprocess 병렬 환경과 SSH 코어/메모리 기반 환경 수 추천
- 키보드 기반 속도 명령, neutral residual gate, RL/Legacy 모드 순환

추가 개발이 필요한 범위:

- 실물 로봇용 body velocity/IMU 상태 추정과 RL 관측 생성
- 실제 링크의 정확한 기계적 관절 한계 적용
- 모든 링크에 대한 정밀 collision geometry와 self-collision 검증
- 실제 하드웨어 ID, 축 방향, 정·역회전, 영점의 최종 캘리브레이션
- sim-to-real 안전 제한, 비상정지, 지연·노이즈·마찰 randomization
- ICRA 논문의 정량 결과와 재현 가능한 실험 증거 채우기
