# 변수와 상수 사전

이 디렉터리는 “값을 어디서 바꾸면 무엇이 달라지는가”를 빠르게 찾기 위한 사전이다.

`tripod-gait`와 `scone-gait`의 수식·처리 순서·설정 간 관계는
[`10-tripod-gait-and-scone-gait.md`](../10-tripod-gait-and-scone-gait.md)를
먼저 읽고, 이 디렉터리에서는 개별 필드와 상태값을 찾는다.
계단 후킹 수식과 `scone-stair` 상태 전환의 근거는
[`11-scone-stair-climbing.md`](../11-scone-stair-climbing.md)를 먼저 읽는다.
비-RL 연속 회전 설정, support 처짐 보정과 자동 데모 변수의 채택 근거는
[`12-automatic-stair-demo-and-continuous-roll-rework.md`](../12-automatic-stair-demo-and-continuous-roll-rework.md)를 따른다.

## 문서 구성

- [API·CLI·하드웨어·locomotion](01-api-hardware-locomotion.md)
- [기구학·시뮬레이션·지형](02-kinematics-simulation-terrain.md)
- [강화학습](03-reinforcement-learning.md)
- [테스트·자산·아카이브](04-tests-assets-archive.md)

## 포함 기준

개별 행으로 기록하는 대상:

- 모듈 또는 클래스 상수
- `Enum`/`IntEnum` 값
- 데이터 클래스의 모든 필드
- 인스턴스에 유지되는 상태 변수
- 단위, 좌표계, 안전 범위, 외부 I/O를 바꾸는 함수 매개변수
- 의미가 이름만으로 드러나지 않는 계산 중간값

묶어서 기록하는 대상:

- 단순 반복자의 `i`, `item`, `motor_id`, `leg`
- 예외 객체 `error`
- return 직전 한 번만 사용되는 `result`, `value`
- 동일한 의미의 dictionary comprehension 지역변수

이런 짧은 지역변수도 무시한 것은 아니다. 각 파일의 “주요 지역변수” 행에서 실행 흐름 단위로 묶었다.

## 공통 규칙

| 이름/형태 | 공통 의미 |
|---|---|
| `motor_id` | Dynamixel ID 1–18. `leg`, `leg+6`, `leg+12`가 한 다리의 세 관절 |
| `leg` | 다리 번호 1–6 |
| `raw` | Dynamixel 위치 단위. 4096 count/revolution, 중심 2048 |
| `degrees` | motor position을 degree로 표현. 중립 관절각은 일반적으로 180° 기준 |
| `angles`, `qpos` | MuJoCo 관절 radian |
| `dof`, `qvel` | MuJoCo generalized velocity 주소/값 |
| `vx`, `vy` | body frame 선속도. `+vx` 전진, 모델의 `+vy` 왼쪽 |
| `yaw_rate` | body z축 회전속도. 양수는 왼쪽/counter-clockwise |
| `phase` | gait cycle 위치, 범위 `[0, 1)` |
| `dt` | 한 물리 또는 제어 step의 시간 |
| `seed` | 재현 가능한 난수 지형/학습을 위한 seed |
| leading `_` | 외부 public API가 아닌 내부 상태/도우미 |
