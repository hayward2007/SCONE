# SCONE 프로젝트 문서

이 문서는 2026-09-02 기준 SCONE 저장소 전체를 설명한다. 현재 RL·시뮬레이션 코드, `runs/`의 학습 산출물 구조, `archive/`의 논문·도면·구형 코드도 함께 조사했다. 코드와 과거 문서의 설명이 다를 때는 현재 코드와 테스트를 우선한다.

## 문서 읽는 순서

1. [프로젝트 개요](01-project-overview.md) — 목표, 주요 기능, 현재 범위
2. [기능과 기술](02-features-and-technologies.md) — 구현 기능과 사용 기술
3. [아키텍처와 데이터 흐름](03-architecture-and-data-flow.md) — 실제 장치·시뮬레이션·RL의 연결
4. [문제와 해결 과정](04-problems-and-solutions.md) — 원인, 진단, 적용한 해결책, 남은 위험
5. [파일과 폴더 지도](05-file-and-folder-map.md) — 저장소의 파일별 역할
6. [변수와 상수 사전](variables/README.md) — 영역별 변수·상수·상태 필드
7. [보상함수 수정 가이드](06-reward-function-guide.md) — 현재 수식, 수정 절차, 검증법
8. [실행·학습·운영·검증](07-running-testing-and-operations.md) — 설치, CLI, 테스트, 체크포인트 운영
9. [RL·시뮬레이션 개발 기록](08-rl-development-log.md) — 초기 설계부터 실패·수정·검증까지의 이력
10. [보행 성능 분석과 개선 로드맵](09-gait-performance-analysis.md) — 하드코드/Non-RL/RL 수치 비교와 우선순위
11. [`tripod-gait`와 `scone-gait` 상세 가이드](10-tripod-gait-and-scone-gait.md) — 이름·수식·IK·부채꼴 rolling/creep·CLI/RL·검증·튜닝 전체 설명
12. [SCONE 부채꼴 후킹과 계단 알고리즘](11-scone-stair-climbing.md) — 후킹 기하·토크·마찰·안정 조건, 여섯 프레임 공통 위상, 옛 앞 1단 270° 재현과 partial-brace sweep·최종 수치
13. [자동 계단 데모와 연속 회전형 `roll-gait`의 역사](12-automatic-stair-demo-and-continuous-roll-rework.md) — 입력 없는 synchronized open/closed-loop 비교, 세 다리 지지 처짐, motor profile·보폭 sweep, C자 말단 연속 회전 phase 가설
14. [기능 구현 및 코드 수정 가이드](13-feature-implementation-and-modification-guide.md) — launcher·하드웨어·Legacy·FK/IK·MuJoCo·terrain·두 gait·계단·RL·원격 학습의 구현 흐름, 수정 절차, 호환 경계와 검증 기준
15. [`roll-gait` 분리와 PPO/점접지 하이브리드 `scone-gait`](14-roll-gait-and-hybrid-scone-gait.md) — 현재 이름, 저속/yaw PPO와 고속 multi-turn 전환식, point-support/누적회전 공식, 계단 Drive→Climb 준비, stage-1 live read-back, 15.4M checkpoint 검증
16. [보행·계단·PPO 통합 활동 기록](15-complete-development-activity-log.md) — 전체 요청·가설·실행·채택/기각·수치·코드 위치·검증·남은 한계를 한 문서에서 추적
17. [ICRA 시뮬레이션 벤치마크 구현과 실행 기록](16-icra-simulation-benchmark-implementation-and-results.md) — 평지·계단 A/B/C, 강건성·모드 전환 벤치마크, 실제 실행 수치, 통계·검증·논문 사용 조건
18. [구형 PPO 진단과 개선 계획](17-ppo-diagnosis-and-fix-plan.md) — `walk_learn` 70차원 정책의 역사적 진단과 V2 이전 개선 근거
19. [액추에이터 모델·좌표계·PPO V2 설계](18-actuator-model-and-frame-convention.md) — DC motor/armature 검증, 정규 좌표계, `walk_v2` 초기 설계와 CLI
20. [공개된 다이나믹셀 MuJoCo 모델과 설정 대조](19-actuator-settings-vs-published-models.md) — OP3·Open Duck 모델과 SCONE dcmotor/armature를 같은 벤치에서 비교한 결과
21. [백래시 적용과 다이나믹셀 패키지 분리](20-backlash-and-dynamixel-package.md) — 직렬 유격 모델, 단위 버그, 민감도 측정과 독립 패키지 구조
22. [`walk_v2` PPO 실제 학습 분석](21-walk-v2-ppo-training-analysis.md) — 35.4M checkpoint, TensorBoard, 고정 명령 평가, 행동 포화·누적 motor randomization 원인과 재학습 gate

## 문서 범위와 표기

- **현재 코드**: `src/`, 루트 API, 테스트가 정의하는 실제 동작이다.
- **역사 자료**: `archive/`, [`08-rl-development-log.md`](08-rl-development-log.md), 구형 영상과 논문은 설계 배경을 설명하지만 현재 API를 정의하지 않는다.
- **생성 파일**: `runs/`, `tmp/`, 캐시, 논문 빌드 결과는 프로그램 산출물이므로 파일 형식과 보존 목적을 묶어서 설명한다.
- **변수 사전**: 모듈 상수, 열거형, 데이터 클래스 필드, 장기 상태, 외부 동작에 영향을 주는 매개변수를 개별적으로 기록한다. `i`, `item`, `error`처럼 의미가 자명하고 일회성인 반복·예외 지역변수는 해당 함수의 흐름으로 묶어 설명한다.
- 단위는 별도 표기가 없으면 각 코드의 원 단위를 따른다. Dynamixel 위치는 raw 값 또는 degree, MuJoCo 관절은 radian, 시간은 second이다.

## 현재 상태에서 특히 주의할 점

- 구형 70차원 정책의 기준은 `src/rl/walk_learn.py`, 새 82차원 정책의 기준은 `src/rl/walk_v2.py`다. 두 환경의 checkpoint는 서로 호환되지 않는다. V2의 현재 보상·관측·학습 상태는 [`21-walk-v2-ppo-training-analysis.md`](21-walk-v2-ppo-training-analysis.md)를 우선한다.
- 현재 시뮬레이터는 액추에이터 이름(`A01_` … `A18_`)을 통해 관절을 찾는다. 과거의 고정 mirror-pair 재배열 방식은 현재 구현이 아니다.
- 실물 장치 코드는 보존되어 있으며 시뮬레이션의 좌우 축 보정은 MJCF 모델에서 처리한다.
- RL 정책은 시뮬레이션에서 학습·재생한다. 실제 로봇에 바로 배포하는 상태 추정·안전 계층은 아직 완성된 기능이 아니다.
- Residual RL 기준 모션은 새 실행에서 `tripod-gait`가 기본값이며 `scone-gait`는 실험 선택지다. 기존 작업 기록에는 `hardcoded` 또는 `non_rl`이 저장돼 있을 수 있으므로 재개 전에 설정을 확인한다.
- 두 gait를 수정하거나 새 checkpoint를 학습하기 전에는 [`10-tripod-gait-and-scone-gait.md`](10-tripod-gait-and-scone-gait.md)의 reference 호환성, 동역학 검증값, 실물 진입 조건을 먼저 확인한다.
- `scone-stair`는 현재 MuJoCo 전용이다. 실물 계단 적용 전에는 [`11-scone-stair-climbing.md`](11-scone-stair-climbing.md)의 마찰·모터 전류·nosing·지지다각형 실측 항목을 먼저 검증한다.
- lower velocity-mode 자유 회전은 `roll-gait`다. 현재 `scone-gait`는 저속/제자리 yaw PPO와 고속 점접지 tripod+위상 동기 multi-turn 말단 회전을 합치는 checkpoint 필수 supervisor다. [`14-roll-gait-and-hybrid-scone-gait.md`](14-roll-gait-and-hybrid-scone-gait.md)를 우선한다.
- 계단 준비는 실제 상태기와 같은 Walk→Drive→Climb 순서를 완료한 뒤 공통 위상 제어로 넘어간다. 물리 Drive 진입 시 stage-1 mode/profile/goal/present register를 read-back한다. MuJoCo의 Drive 1단 댐핑 2배는 여전히 시뮬레이션 전용이다.
- `benchmark/` 결과는 현재 MuJoCo와 controller 구현의 개발 증거다. dirty worktree의 단일 명목 실행을 논문 최종 성능으로 사용하지 말고, [벤치마크 실행 기록](16-icra-simulation-benchmark-implementation-and-results.md)의 보정·반복·실물 교차 검증 조건을 따른다.
