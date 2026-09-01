# SCONE 보행·계단·PPO 통합 활동 기록

이 문서는 2026-08-31부터 2026-09-01까지 이어진 SCONE 보행, Residual RL,
평지 주행, 계단 이동, 실물 준비 상태, 자동 시뮬레이션 작업을 한 곳에서 추적하기
위한 통합 기록이다. 기능별 세부 공식은 기존 전문 문서에 남겨 두고, 여기서는
**어떤 증상에서 출발해 무엇을 시험했으며, 무엇을 채택·기각했고, 현재 코드가
어디까지 검증됐는지**를 시간 순서와 요구사항 순서로 함께 정리한다.

기록의 우선순위는 다음과 같다.

1. 현재 `src/` 코드와 자동 테스트
2. 동일 source에서 실행한 headless MuJoCo 수치
3. macOS `mjpython` viewer smoke 결과
4. Git 이력과 과거 코드
5. 설계 가설과 아직 실험하지 않은 항목

결정론적 MuJoCo 단일 실행값은 회귀 자료이지 실물 성능 보증이나 통계적 평균이
아니다. 특히 `roll-gait`, `scone-gait`, `scone-stair`의 새 경로는 현재
simulation-first이며 실물 기본 `Walk/Drive/Climb`을 자동으로 대체하지 않는다.

---

## 1. 최종 상태 한눈에 보기

현재 이름과 역할은 다음과 같다.

| 이름 | 현재 역할 | 제어 방식 | checkpoint | 적용 범위 |
|---|---|---|---|---|
| `tripod-gait` | SCONE에 맞춘 고전 교대 삼각보 | 18관절 position+IK | 불필요 | MuJoCo 조종 |
| `roll-gait` | 기존 연속 회전형 gait | 상·중단 보행 + lower velocity 다회전 | 불필요 | MuJoCo 조종 |
| `scone-gait` | 저속 PPO와 고속 SCONE hybrid supervisor | PPO + point-support 보행 + lower 누적 position 회전 | 필수 | MuJoCo 조종 |
| `scone-stair` | 별도 계단 모션 | 앞 stage-1 brace + lower 6개 공통 기하 위상 | 불필요 | MuJoCo 계단 |
| Legacy `Walk/Drive/Climb` | 실제 controller의 기존 상태기 | blocking position/velocity sequence | 불필요 | 실물·MuJoCo 공용 기반 |

가장 중요한 현재 경계는 다음 세 가지다.

- Residual PPO는 학습할 때 사용한 reference와 함께 재생해야 한다. reference를
  임의로 바꾸면 policy action과 기준 모션이 상쇄될 수 있다.
- 자유 velocity-mode 다회전은 `roll-gait`다. 현재 `scone-gait`는 PPO가 필요한
  별도 hybrid이고, 고속에서 실제 multi-turn lower position target을 사용한다.
- 계단 이동은 Drive형 free rolling이 아니다. 여섯 부채꼴 프레임을 같은 물리
  위상으로 정렬하고 함께 움직이는 별도 `scone-stair` 모션이다.

---

## 2. 요청과 조치 추적표

| 접수한 문제/요구 | 확인한 직접 원인 | 수행한 조치 | 현재 판정 |
|---|---|---|---|
| Residual 기준 모션을 고른 뒤 PPO가 꼬임 | Residual policy는 학습 reference에 종속됨 | reference 선택·저장·재생 경로 분리 | 해결, 호환 경계 유지 필요 |
| 하드코딩 방향이 학습 때와 반대 같음 | 방향 의심과 별도로 재생 동역학이 학습 시점과 달라짐 | 15.4M checkpoint를 학습 당시 profile로 대조 | 실제 회귀 원인 확정 |
| PPO가 예전보다 심하게 이상함 | RL reset에 뒤늦게 넣은 `walking_speed=100`/acceleration 제한 | 기존 PPO replay에서 제한 제거, 기본 reference `hardcoded` 복구 | 기존 checkpoint 전진 복구 |
| `non-rl`을 실제 알고리즘 이름으로 변경 | 구현이 alternating tripod+IK임 | 정식 이름 `tripod-gait`, 호환 alias 보존 | 완료 |
| SCONE 전용 rolling 보행 필요 | bounded lower ±30° 왕복은 순회전 0 | 별도 연속 회전 controller 구현 | 현재 이름 `roll-gait` |
| 입력 없는 계단 비교 시뮬레이션 필요 | 기존에는 조이스틱 route만 있음 | `hardcoded/improved/compare` 자동 viewer 추가 | 완료 |
| 세 다리 지지 시 몸체가 과도하게 처짐 | middle position tracking lag와 support 전환 | model gait에서만 middle stiffness 2배 | 하방 처짐 감소, 실물 gain은 불변 |
| 알고리즘 보행이 느리고 보폭이 작음 | profile lag와 높은 stride clipping | speed/stride/cadence sweep | 1차 개선 후 추가 직진 교정 |
| `tripod-gait`가 앞뒤로 휘청이고 yaw 누적 | 0.8 Hz에서 필요한 112.5 mm를 80 mm로 97% clipping | MuJoCo gait를 1.0 Hz/90 mm로 맞추고 profile lag 제거 | 20초 직진 회귀 통과 |
| 기존 `scone-gait`가 tripod와 비슷함 | lower가 한 cycle마다 되감김 | 실제 연속 회전 경로 분리 | `roll-gait`로 이동 |
| 연속 회전 gait가 회전만 하고 다리 보행이 작음 | lower 기본 보행 성분 폐기, stride/lift 25/4 mm | 상·중단 55/20 mm, lower 기본속도 0.35 합성 | full-body `roll-gait` 완료 |
| 계단 10/15/20 cm 재시험 | 15/20 cm에서 단순 rolling 실패 | tread/phase/brace 후보 sweep | 100/150/200 mm preset 재구성 |
| 계단 이동이 주행처럼 보임 | lower 여섯 축을 free-run하고 assist 중 위상을 깨뜨림 | odd/even 축을 같은 물리 위상으로 변환, 공통 `θ` 적분 | 현재 계단 기준으로 교체 |
| 옛 코드의 앞 1단 수직 자세 재현 | Legacy 상승 앞쪽 IDs 7/9/11이 270° | 270° baseline과 180–270° sweep 실행 | improved는 180/184/195° 채택 |
| 주행에서 계단으로 바로 바뀌어 무리 가능 | custom route가 Drive→Climb 준비를 건너뜀 | Walk→Drive→Climb을 모두 거친 뒤 brace/phase 시작 | 완료 |
| Drive 중 1단 모터 흔들림 설정 확인 | mode/profile/goal/present를 실제로 검증하지 않음 | 초기 position mode 명시, 실물 read-back fail-closed 추가 | 코드/가짜 controller 검증, 실기 연결 미검증 |
| 옛 연속 controller를 `roll-gait`로 변경 | 이름과 새 hybrid 요구가 충돌 | 클래스/CLI 이름 분리, 과거 import alias 보존 | 완료 |
| 새 `scone-gait`: 저속/yaw PPO, 고속 보행+회전 | bounded high-speed reference가 화면상 회전하지 않음 | point-support 보행과 실제 누적 lower 회전 구현 | checkpoint headless/GUI 검증 완료 |
| 모든 구현·공식·실패·수정법 문서화 | 정보가 기능별 문서에 분산됨 | 08–14번 상세 문서와 현재 통합 기록 연결 | 완료 |

---

## 3. 시간 순 활동 기록

### 3.1 Residual RL 기준 모션과 PPO 회귀 진단

처음에는 “원래 RL 모델은 기준 모션이 필요 없는 것 아닌가”라는 의문에서
시작했다. 일반적인 end-to-end PPO라면 policy가 최종 관절 명령을 모두 만들 수
있지만, 이 프로젝트의 `SconeWalkEnv`는 다음 residual 구조다.

```text
최종 관절 목표 = reference motion + policy residual
```

따라서 학습에 사용한 reference는 policy 의미의 일부다. checkpoint가
`hardcoded`로 학습됐는데 replay에서 `tripod-gait`나 `scone-gait` reference를
넣으면 같은 action도 다른 최종 자세가 된다. 이 문제를 막기 위해 학습, check,
enjoy, 원격 실행, checkpoint 다운로드 후 재생까지 reference 값이 전달되도록
정리했다. 이전 기록의 `non_rl`은 현재 `tripod-gait`로 normalize한다.

다만 `scone_walk_15410928_steps.zip`의 실제 후진 회귀는 단순한 부호 오류가
아니었다. 학습 후 RL reset에 추가된 `walking_speed=100`과 XM acceleration 20
제한이 관절 목표 추종을 늦춰 동역학을 바꿨다.

| 재생 조건 | `vx=0.18` 평균 진행 |
|---|---:|
| 뒤늦게 추가된 유한 profile | `-0.0225 m/s` |
| 학습 당시 무제한 simulation profile | `+0.0509 m/s` |

따라서 기존 checkpoint replay에서는 이 제한을 제거하고 기본 reference를
학습 당시 `hardcoded`로 복구했다. 물리 profile을 포함한 새 환경이 필요하면
기존 checkpoint를 resume하지 않고 새 policy를 0 step부터 학습해야 한다.

### 3.2 이름과 호환성 정리

초기 `non-rl`이라는 이름은 제어 원리를 설명하지 못했다. 실제 구현은 고전적인
교대 tripod와 Cartesian 발 궤적, 수치 IK를 사용하므로 `tripod-gait`로
변경했다. 과거 명령과 저장 설정을 깨지 않도록 `non_rl` 입력은 호환 alias로만
남겼다.

그 뒤 연속 lower velocity 회전 controller가 `scone-gait`라는 이름을 사용했지만,
새 요구사항은 “저속/yaw PPO + 고속 보행/부채꼴 회전”이었다. 의미 충돌을 없애기
위해 자유 연속 회전 controller를 `roll-gait`/`RollGait`로 옮기고,
`SconeRollingGait*`는 import 호환 alias로 남겼다. `scone-gait`는 현재 PPO가
필수인 hybrid supervisor만 의미한다.

### 3.3 `tripod-gait` 속도, 처짐, 방향 안정화

#### 첫 번째 속도·지지 sweep

초기 MuJoCo 조종 route는 speed 100, acceleration 20, 0.7 Hz, 전후 60 mm라
PPO보다 답답했다. `vx=0.18`, duty 0.5에서 필요한 stroke는

```text
0.18 × 0.5 / 0.7 = 0.1286 m
```

인데 상한은 0.060 m여서 평균 stride clipping이 98.67%였다. speed/cadence/
stride/stiffness 8개 후보를 6초씩 실행했다.

세 다리 지지 처짐에는 먼저 MuJoCo `qfrc_bias` 중력 feed-forward를 시험했지만,
settle root Z가 약 0.1518→0.1454 m로 더 낮아지고 middle 오차가 3.6→6.8°로
커져 제거했다. 채택한 방법은 torque cap을 올리지 않고 model gait route의
ID 7–12 position stiffness만 2배, damping은 `sqrt(2)`배로 올리는 것이었다.
이는 PPO와 실물 controller gain을 바꾸지 않는다.

1차 채택값 speed 160/acceleration 50, 0.8 Hz, 80/60 mm는 평균 속도를
0.0639→0.1058 m/s로 올리고 최저 Z를 -0.10 mm로 유지했다. 하지만 viewer에서
앞뒤 접촉 혼합과 방향 틀어짐이 남아 평균 속도만으로는 충분하지 않다는 것이
확인됐다.

#### 두 번째 직진 안정화

0.8 Hz에서 최대 전진에 필요한 stroke는 112.5 mm인데 80 mm로 제한돼 8초
평균 clipping이 약 97%였다. 큰 IK branch와 profile lag가 겹쳐 8초 동안 실제
역방향 누적 24.50 mm, 측면 51.77 mm, 최대 yaw 3.38°가 발생했다.

최종 MuJoCo `tripod-gait`는 1.0 Hz, duty 0.5, 90 mm stride, 25 mm lift로
필요 stroke와 workspace 상한을 일치시켰다. 이 simulation gait route에서는
profile limiter를 제거했지만 PPO reset과 실물 profile은 바꾸지 않았다.

| 측정 | 최종 결과 |
|---|---:|
| 8초 전진 | `0.9469 m` |
| 8초 평균 | `0.1184 m/s` |
| 20초 전진 | `2.4263 m` |
| 역방향 누적 | `3.67 mm` |
| 8초 측면 최종 | `-0.66 mm` |
| 최대 yaw | `1.17°` |
| 최저 ΔZ | `-0.02 mm` |
| IK 실패 | `0` |

lower IK 비율을 0.75/0.50/0.25로 줄이는 후보와 duty 0.60 double-support 후보는
추진을 상쇄하거나 yaw/역방향 누적을 키워 기각했다.

### 3.4 `roll-gait`: 자유 회전과 full-body 보행 합성

초기 bounded `SconeGait`는 lower를 최대 30° 움직였다가 swing에서 되감아
순회전이 0이었다. RL position reference에는 유용하지만 C자 프레임을 바퀴처럼
쓰는 주행에는 맞지 않았다. 별도 `RollGait`는 lower 6개를 velocity mode로
여러 바퀴 회전시킨다.

처음에는 C자 개구가 동시에 지면을 향해 root Z가 63.52 mm 내려갔다. 여섯 축
임의 위상도 lateral 44.4 mm와 upright 0.9642로 불안정했다. tripod A/B의
개구 phase를 분리해 이 문제를 줄였다.

이후 “회전만 하고 몸통과 다리가 움직이지 않는다”는 관찰을 계측했다. 실제로
planner의 ID 13–18 기본 position 성분을 버리고 있었고, stride/lift도
25/4 mm라 middle 진폭이 2.31°뿐이었다. lower position과 velocity mode를
동시에 쓸 수 없으므로 기본 lower offset의 미분을 회전속도에 합성했다.

```text
Δq_basic,i(t) = q_planner,i(t) - q_nominal,i

v_lower,i = LPF(v_roll,i, 0.10 s)
          + 0.35 × LPF(dΔq_basic,i/dt, 0.04 s)
```

상·중단 planner는 stride/lift 55/20 mm로 키웠다. full-body 조건에서 phase를
다시 sweep해 과거 +72° 대신 tripod B +60°, steering 0.20을 채택했다.

| 최종 6초 결과 | 값 |
|---|---:|
| 전진/평균 | `1.2556 m / 0.2093 m/s` |
| 측면 | `-52.5 mm` |
| 최대 yaw | `5.56°` |
| 최저 ΔZ | `-20.68 mm` |
| minimum upright | `0.9811` |
| lower 평균 회전 | `3.09회` |
| upper/middle 명령 진폭 | `17.28° / 9.59°` |
| IK 실패 | `0` |

측정 yaw로 좌우 회전속도를 차등하는 heading 보정 gain도 시험했지만 lateral
0.16–0.30 m 또는 root Z 하강을 키워 모두 제거했다.

### 3.5 자동 계단 시뮬레이션과 100/150/200 mm 환경

루트 메뉴를 사람이 계속 조종하는 `시뮬레이션 조종`과 입력 없는
`시뮬레이션 (자동 데모)`로 분리했다. 자동 데모는 `hardcoded`, `improved`,
두 창을 순서대로 여는 `compare`를 제공한다. 물리 timeout과 control scheduling은
벽시계가 아니라 `data.time` 기준으로 바꿔 렌더링 속도 때문에 16초보다 일찍
끝나는 오류도 수정했다.

계단 preset은 각 단 rise를 다음과 같이 고정했다.

| preset | rise | 현재 용도 |
|---|---:|---|
| `stairs-1` | `100 mm` | 낮은 계단/직접 회전 기준 |
| `stairs-2` | `150 mm` | 중간 높이/접촉 충격 비교 |
| `stairs-3` | `200 mm` | 높은 계단, 현재 tread `350 mm` |

200 mm에서 middle/upper, bank, phase, reverse, tripod 조합, tread
170/200/240/280/300/350/400 mm 등을 실행했다. shallow tread의 가장 좋은
후보도 두 번째 riser 부근에서 정체했고 350 mm에서만 당시 adaptive assist가
통과했다. 이는 200 mm가 기구학적으로 절대 불가능하다는 뜻이 아니라, 현재
MJCF와 후보 제어 범위에서 검증된 조합이 제한적이라는 뜻이다.

### 3.6 계단 모션을 주행에서 공통 위상으로 교정

초기 improved 계단 제어는 lower free rolling과 tripod assist를 섞었다. 그러나
Legacy `Climb`의 계단 모션은 여섯 부채꼴 프레임을 같은 접촉 위상으로 맞추고
함께 움직인다. 따라서 이전 H0–H4/assist 결과는 시행 이력으로만 남기고 현재
controller를 synchronized phase 방식으로 교체했다.

MJCF에서 홀수와 짝수 lower joint 축 방향이 반대이므로 같은 숫자 각도가 아니라
다음 target이 같은 물리 C-frame 위상을 만든다.

```text
q_odd(θ)  = θ
q_even(θ) = 360° - θ

θ[k+1] = θ[k]
       - velocity_unit × 0.229 × 6 × |vy|/max_vy × Δt
```

`θ`는 wrap하지 않고 extended-position mode에서 계속 적분한다. command가 0이면
velocity 0으로 free stop하지 않고 마지막 공통 target을 hold한다. open-loop도
비교 전에 같은 위상을 획득하고 velocity mode로 전환하므로 비교 조건이 같다.

공통 위상/속도 sweep 결과는 다음과 같다.

| rise | improved 시작 위상 | phase velocity |
|---:|---:|---:|
| 100 mm | `60°` | `250` |
| 150 mm | `60°` | `200` |
| 200 mm | `90°` | `200` |

### 3.7 옛 앞 stage-1 270° 자세 재현과 partial brace

Git 이력 `a6b45ca`, `b353bc5`에서 상승 방향 앞쪽 stage-1
`(7,9,11)`을 270°로 내린 뒤 lower 6개를 함께 회전하는 Legacy 자세를
확인했다. 이를 hardcoded baseline에 그대로 복원했다.

그러나 270° 고정은 100/150 mm에서 크게 기울고 200 mm에서 실패했다.
180–270° coarse sweep, 접촉 전환부 1° sweep, 270°에서 선택각으로 되돌리는
후보, Drive damping 해제 대조를 모두 실행했다. 270° 시작 후 회수는 transient,
기계일, 접촉 충격만 늘려 기각했다.

최종 improved brace는 높이별 `180/184/195°`다.

| terrain | hardcoded 270° | improved partial brace |
|---|---:|---:|
| 100 mm | `3.804 s / 50.104 J / upright 0.608` | `2.594 s / 42.424 J / 0.912` |
| 150 mm | `4.268 s / 64.369 J / upright 0.438` | `4.194 s / 65.464 J / 0.752` |
| 200 mm | `16초 내 실패` | `5.996 s / 86.295 J / upright 0.760` |

150 mm는 improved의 일이 약 1.7% 더 크고 시간도 거의 같다. 개선의 핵심은
속도가 아니라 upright `0.438→0.752`다. 200 mm에서는 hardcoded 실패와
improved 통과가 실제 viewer에서도 재현됐다.

### 3.8 계단 준비 상태기와 Drive stage-1 점검

custom 계단 route가 네 번 좌회전 뒤 Walk→Drive까지만 실행하고 바로 lower를
덮어써 실제 Drive→Climb 준비를 건너뛰는 문제가 있었다. 현재 공통 준비 순서는
다음과 같다.

```text
Walk
  → 네 번 left, course +Y로 side-on 정렬
  → Walk→Drive 준비와 settle
  → Drive→Climb 준비와 settle
  → 앞 stage-1 brace
  → lower 6개 공통 위상 획득
  → synchronized stair motion 활성화
```

두 mode 전환이 예상 상태를 반환하지 않으면 custom 계단 제어를 시작하지 않는다.
자동 데모, 조이스틱, benchmark가 같은 준비 함수를 사용한다.

Drive에서 흔들린다는 보고를 확인하기 위해 ID 7–12의 expected setting도 코드로
검증했다.

| 상태/register | 기대값 |
|---|---:|
| operating mode | `POSITION (3)` |
| torque | `ON (1)` |
| profile acceleration | `20` |
| profile velocity | `safety_speed=50` |
| goal position | `2048 raw = 180°` |
| present position | `2048 ± 64 raw` |

초기화는 ID 7–18의 position mode를 명시하고, 물리 Drive 진입 시
`verify_drive_stage1_settings()`가 mode/profile/goal/present를 read-back한다.
하나라도 다르면 `ControllerError`로 중단한다. 물리 position 도달 대기도
present-position 기반으로 확장했다.

이 작업에서 실제 로봇의 serial port에 연결해 register를 읽은 것은 아니다.
정상/오류 read-back은 fake controller 자동 테스트로 검증했다. 따라서 실기에서
read-back이 통과한 뒤에도 흔들림이 남으면 horn/볼트/프레임 유격, 케이블 장력,
전류, 온도, 전압, hardware error, position gain을 추가 측정해야 한다. MuJoCo의
Drive `kd` 2배는 실물 gain 변경이 아니다.

### 3.9 새 PPO/점접지 hybrid `scone-gait`

새 supervisor는 translation 속력

```text
s = sqrt(vx² + vy²)
```

을 기준으로 동작을 나눈다.

- `s <= 0.10 m/s`: PPO 100%
- `0.10 < s < 0.18 m/s`: smoothstep 전환
- `s >= 0.18 m/s`: full-body hybrid reference 100%
- 제자리 yaw: translation이 0이므로 PPO 100%

reference 비율이 커질수록 PPO residual을 같은 비율로 줄여 학습하지 않은
reference와 action이 서로 상쇄되는 문제를 막았다. 저속 PPO reference는 반드시
checkpoint 학습 설정과 같아야 한다.

최초 고속 hybrid는 stance 앞 55%를 point-support로 두고 late stance에서
bounded 30° 회전했지만, swing에서 같은 각도를 되감아 화면상 순회전이 0이었다.
목표각 계측으로 한 cycle의 lower 왕복이 약 20–42°이고 누적 회전은 없음을
확인했다.

interactive high-speed route만 실제 누적 회전으로 변경했다. 다리 위치
`(x_i,y_i)`에서 몸체 명령에 따른 국소 접선속도와 말단 회전속도는 다음과 같다.

```text
v_i = [vx - yaw_rate × y_i,
       vy + yaw_rate × x_i]

roll_rate_i = min(
    (180/π) × ||v_i|| / effective_roll_radius,
    max_roll_rate_degrees
)
```

현재 `effective_roll_radius=0.1225 m`, 최대 `360°/s`다. 위상 gate는 stance 앞
55%에서 0, late stance에서 quintic 가속, swing 앞 70%에서 1, 착지 전 30%에서
quintic 감속한다.

```text
Θ_i[k+1] = Θ_i[k]
         - polarity_i × roll_rate_i × gate_i × alignment_i × Δt

q_lower,i = q_IK,lower,i + Θ_i
```

첫 multi-turn checkpoint 실행은 0.6초에 hard joint limit로 종료됐다. 실제
충돌이 아니라 누적 qpos를 기본각과 단순 비교한 판정 오류였다. 실제 MuJoCo
qpos/target은 unwrapped로 유지하고 PPO 관측과 진단만 360° 동등 위상으로 접었다.
interactive hybrid의 lower에만 periodic joint-limit 의미를 적용하고 상·중단
안전 범위는 유지했다. checkpoint 기준각도 현재 목표에서 가장 가까운 360°
branch로 옮겨 전환 중 여러 바퀴 되감지 않게 했다.

---

## 4. 실패·기각한 가설 모음

| 가설/시도 | 기대 | 실제 문제 | 결론 |
|---|---|---|---|
| Residual reference 방향만 뒤집기 | PPO 전진 복구 | 실제 직접 원인은 profile 동역학 회귀 | 단독 수정 기각 |
| RL reset에 실물 profile 적용 | sim-to-real에 가까워짐 | 기존 checkpoint가 후진 | 기존 replay에서 제거 |
| cadence 1.4 Hz adaptive | 더 빠른 Non-RL | profile 추종 실패로 속도 약 72% 감소 | 제거 |
| `qfrc_bias` 중력 보상 | 세 다리 지지 처짐 감소 | root Z와 middle 오차 악화 | 제거 |
| tripod speed 175/200 | 속도 향상 | slip, 하방 붕괴, yaw 증가 | 기각 |
| lower IK를 nominal 쪽으로 혼합 | 전후 흔들림 감소 | 추진 상쇄와 역방향 증가 | 기각 |
| duty 0.60 double-support | 안정성 향상 | 느려지고 yaw/역방향 증가 | 기각 |
| C-frame 6개 동기 free rolling | 단순하고 빠른 주행 | 개구 동시 접지로 Z 63.52 mm 하강 | 기각 |
| arbitrary six-way phase | 개구 분산 | lateral/tilt 증가 | 기각 |
| yaw 기반 roll 속도 차등 | heading drift 보정 | lateral 또는 Z 하강 악화 | 제거 |
| 낮은 tread에서 200 mm 등반 | 짧은 계단 대응 | 두 번째 riser 정체/전복 | 현재 미검증 |
| 계단 tripod assist | 높은 단 후킹 | 실제 계단 모션의 공통 위상을 깨뜨림 | 역사 기록으로만 보존 |
| 앞 stage-1 270° 고정 | 옛 하드코드 재현 및 효율 | 20 cm 실패, upright 악화 | baseline에만 유지 |
| 270° 시작 후 partial angle 회수 | 옛 후킹+안정 절충 | transient/일/충격 증가 | 기각 |
| bounded high-speed lower sweep | 점접지+회전 hybrid | swing에서 되감아 순회전 0 | multi-turn으로 교체 |
| multi-turn을 일반 hard limit로 판정 | 관절 안전 유지 | 360° 동등 자세도 limit로 오판 | periodic lower 처리 |

---

## 5. 최종 시뮬레이션 결과

### 5.1 `tripod-gait`와 `roll-gait`

| route/조건 | 핵심 결과 | 해석 |
|---|---|---|
| `tripod-gait`, 8초 | 0.9469 m, 역방향 3.67 mm, yaw 1.17° | clipping/방향 상쇄 해결 |
| `tripod-gait`, 20초 | 2.4263 m, IK 실패 0 | 시작 transient 뒤 역방향 누적 없음 |
| `roll-gait`, 6초 | 1.2556 m, 0.2093 m/s, lower 3.09회 | full-body 보행과 자유 회전 동시 발생 |

### 5.2 계단 자동 데모 최종값

| terrain | hardcoded | improved |
|---|---:|---:|
| 100 mm | 3.804 s / 50.104 J / upright 0.608 | **2.594 s / 42.424 J / 0.912** |
| 150 mm | 4.268 s / 64.369 J / upright 0.438 | **4.194 s / 65.464 J / 0.752** |
| 200 mm | 16초 내 실패 | **5.996 s / 86.295 J / 0.760** |

macOS viewer에서 `stairs-2`는 두 전략 모두 통과했고, `stairs-3`는 hardcoded가
16초 내 실패하고 improved가 5.94 simulation s에 통과했다. GUI 시간은 렌더
스케줄링이 포함되므로 headless 성능표와 섞지 않는다.

### 5.3 15.4M PPO checkpoint의 현재 `scone-gait`

공통 조건은 flat, Standard stance, hardcoded reference, seed 7, 50 Hz policy,
500 Hz physics, 4초 deterministic 1회다.

| 명령/route | body X | body Y | yaw | 종료 |
|---|---:|---:|---:|---|
| PPO 저속 `vx=0.06` | `+0.0704 m` | `+0.1154 m` | `-11.25°` | 없음 |
| PPO 제자리 `yaw=0.6` | `+0.1290 m` | `-0.1977 m` | `+119.96°` | 없음 |
| bounded hybrid 최대 `vx=0.5` | `+0.456 m` | `-0.013 m` | `-2.2°` | 없음 |
| 현재 multi-turn hybrid `vx=0.5` | `+0.298 m` | `+0.0119 m` | `-4.11°` | 없음 |
| 기존 순수 PPO `vx=0.5` 비교 | `+0.918 m` | `+0.103 m` | `-11.7°` | 없음 |

multi-turn hybrid의 실제 lower ID 13–18은
`+436.8/+434.1/+342.4/+344.8/-432.7/-460.3°` 회전했다. 동시에 ID 1–12
peak-to-peak가 14.0–24.1°이므로 회전만 한 것이 아니라 상·중단 보행도 함께
발생했다. 역방향 `vx=-0.5`는 body X `-0.407 m`이고 lower 부호가 반전됐다.

`mjpython` human viewer 200 frame도 비정상 종료 없이 완료됐고 마지막 HUD는
`scone-gait/hybrid/roll-1.3turn`이었다. 현재 hybrid는 순수 PPO보다 빠르지
않다. 장점은 고속에서 실제 point-support 보행과 multi-turn 회전을 함께 쓰며
lateral/yaw drift가 비교적 작다는 것이다. 저속 PPO와 제자리 yaw의 translation
drift는 checkpoint 자체의 후속 재학습 과제다.

---

## 6. 코드 변경 지도

| 기능 | 기준 코드 | 주요 수정 내용 |
|---|---|---|
| launcher/이름/route | `src/cli.py`, `src/simulation/core/simulator_cli.py` | 자동 데모와 조종 분리, gait 이름과 checkpoint 선택 |
| 공통 viewer 연결 | `src/simulation/core/cli_bridge.py` | `tripod-gait`, `roll-gait`, `scone-gait`, `scone-stair`, RL route |
| 고전 보행 | `src/locomotion/tripod_gait.py` | phase, workspace, IK/backoff, 방향 안정화 |
| bounded/multi-turn planner | `src/locomotion/scone_gait.py` | point-support gate, rolling rate, 누적 lower angle |
| 자유 연속 회전 | `src/simulation/core/scone_rolling_gait.py` | `RollGait`, full-body+velocity 합성, B phase stagger |
| PPO hybrid supervisor | `src/rl/joystick_control.py` | 저속/yaw PPO, speed blend, nearest-turn branch, HUD |
| PPO 환경 | `src/rl/walk_learn.py` | reference override, lower periodic observation/limit 처리 |
| 계단 기하 | `src/locomotion/stair_geometry.py` | 후킹 reach, torque, friction, support 조건 |
| 계단 controller | `src/simulation/core/stair_climber.py` | 앞 brace, odd/even 공통 위상, extended target |
| 자동 계단 | `src/simulation/core/stair_demo.py` | 270° open-loop와 partial-brace closed-loop 비교 |
| 계단 측정 | `src/simulation/stair_benchmark.py` | 동일 조건 시간/일/upright/contact/phase 측정 |
| 준비 상태 | `src/locomotion/drive.py`, `src/locomotion/mode.py` | settle hook, simulation Drive damping, Climb 전환 |
| 실물 register 검증 | `src/hardware/controller.py` | stage-1 goal/present/mode/profile read-back |

각 기능의 수정 절차와 함께 확인할 파일은
[`13-feature-implementation-and-modification-guide.md`](13-feature-implementation-and-modification-guide.md)가
기준이다. 이 문서는 변경 이유와 활동 이력을, 13번 문서는 실제 수정 방법을
담당한다.

---

## 7. 검증 활동

### 7.1 자동 회귀

최종 multi-turn 수정 상태에서 다음 명령을 실행해 137개 테스트가 통과했다.

```bash
python -m compileall -q src tests
python -m unittest discover -s tests -p 'test_*.py'
git diff --check
```

중요 회귀 항목은 다음과 같다.

- gait 이름과 route가 서로 섞이지 않음
- `tripod-gait` 방향·IK·장기 직진 기준
- `roll-gait`의 full-body 성분과 lower 다회전
- `scone-gait` 저속/yaw PPO-only, 전환 smoothstep, 고속 multi-turn
- PPO 관측의 lower 360° 동등 위상
- multi-turn lower가 거짓 hard-limit를 만들지 않음
- 계단 odd/even이 같은 물리 위상을 명령함
- 100/150/200 mm에서 synchronized controller가 상단 조건을 통과함
- Walk→Drive→Climb 준비 순서
- stage-1 정상/오류 register read-back
- 자동 demo와 root launcher routing

### 7.2 직접 viewer

실제로 실행한 viewer smoke는 다음과 같다.

- `mjpython -m src.simulation --demo compare --terrain stairs-2`
- `mjpython -m src.simulation --demo compare --terrain stairs-3`
- `scone_walk_15410928_steps.zip`, `scone-gait`, `vx=0.5`, 200 frame

viewer smoke는 창이 열리고 route가 완료되는지, hardcoded/improved의 성공
판정이 headless 정의와 같은지, lower 누적 회전이 실제 HUD와 qpos에서 유지되는지
확인한다. 여러 표면·장시간·복합 조작을 모두 시각 검증했다는 뜻은 아니다.

---

## 8. 재현 명령

### 8.1 launcher

```bash
mjpython SCONE.py
```

메뉴에서 `시뮬레이션 조종`을 선택한 뒤 `tripod-gait`, `roll-gait`,
checkpoint가 있는 `scone-gait`, 또는 `scone-stair`를 고른다.

### 8.2 자동 계단 비교

```bash
mjpython -m src.simulation --demo compare --terrain stairs-2
mjpython -m src.simulation --demo compare --terrain stairs-3
```

### 8.3 계단 headless benchmark

```bash
python -m src.simulation.stair_benchmark \
  --terrain stairs-1 --terrain stairs-2 --terrain stairs-3 \
  --strategy synchronized-open-loop --strategy adaptive
```

### 8.4 PPO hybrid

```bash
mjpython -m src.simulation \
  --control scone-gait \
  --profile standard \
  --terrain flat \
  --checkpoint runs/walk_full_standard/checkpoints/scone_walk_15410928_steps.zip \
  --rl-reference-motion hardcoded
```

checkpoint가 다른 reference로 학습됐다면 마지막 값을 반드시 해당 설정으로
바꾼다.

---

## 9. 이후 수정할 때의 규칙

1. **한 번에 한 계층만 바꾼다.** gait phase, actuator profile, contact friction,
   PPO reference를 동시에 바꾸면 원인을 분리할 수 없다.
2. **checkpoint reference를 먼저 확인한다.** reference가 달라지는 변경은 기존
   policy resume가 아니라 새 environment version과 새 학습 대상이다.
3. **평균 전진거리만 보지 않는다.** 역방향 누적, lateral, yaw, root Z,
   upright, IK failure, stride clipping을 함께 기록한다.
4. **계단은 공통 위상을 보존한다.** 새 assist를 넣더라도 여섯 C-frame의 물리
   phase 정의를 깨지 않아야 한다.
5. **실물과 MuJoCo adapter를 구분한다.** simulation stiffness/damping 또는
   unlimited hinge 예외를 실물 controller에 자동 복사하지 않는다.
6. **headless 뒤 viewer를 실행한다.** 자동 테스트 성공만으로 장시간 접촉과
   화면상 움직임을 검증했다고 쓰지 않는다.
7. **실물은 fail-closed로 진입한다.** mode/profile/goal/present와 current,
   temperature, voltage, hardware error를 확인하고 e-stop/tether를 준비한다.

---

## 10. 남은 한계와 다음 활동

- 현재 모든 성능 수치는 deterministic MuJoCo 단일 run이다. seed, 표면 마찰,
  payload, 관절 오차에 대한 성공률 분포가 없다.
- `roll-gait`는 최대 yaw와 측면 excursion이 남아 있다. 표면별 TPU 변형,
  복합 x/y/yaw, 20초 이상 GUI 측정이 필요하다.
- 현재 PPO checkpoint는 저속 전진과 제자리 yaw 중 translation drift가 크다.
  기존 checkpoint를 억지로 수정하기보다 현재 reference/dynamics를 고정한 새
  학습과 평가가 필요하다.
- multi-turn lower는 MuJoCo unlimited hinge에서 검증됐다. 실물 XM extended
  position range, 케이블 감김, hard stop, current/temperature 조건을 확인하지
  않았다.
- 200 mm 계단은 현재 350 mm tread에서 검증됐다. 170–300 mm, 400 mm,
  nosing/overhang, 낮은 마찰, payload, 시작 yaw 오차, 하강은 미검증이다.
- 계단의 절대 관절일 `J`는 배터리 소비량이 아니다. 실물 전류 적분과 열 측정이
  필요하다.
- Drive stage-1 read-back 코드는 추가됐지만 이 작업 중 실제 hardware 연결은
  하지 않았다. 실기 흔들림의 기계·전기 원인은 아직 측정 대상이다.

실물 승격 순서는 한 다리 무부하 lower 회전, 세 다리 지그, 평지 tether,
100 mm 단차, 150 mm, 마지막으로 200 mm가 안전하다. 각 단계에서 current,
temperature, goal/present 오차, 접촉 위치, emergency stop 결과를 기록해야 한다.

---

## 11. Git과 문서 이력

이번 활동과 직접 연결된 최근 commit은 다음과 같다.

| commit | 내용 |
|---|---|
| `49c3660` | PPO replay 동역학과 reference 정렬 |
| `7276652` | continuous gait와 자동 계단 demo |
| `ad7009f` | SCONE gait 튜닝과 높은 계단 검증 |
| `0c1df25` | 기능 구현·수정 가이드 추가 |
| `bb32236` | SCONE 계단 brace 공통 위상 동기화 |

`roll-gait` 이름 분리, 새 PPO/multi-turn `scone-gait`, Drive→Climb 복구,
stage-1 read-back과 관련 문서 변경은 이 통합 기록 작성 시점의 작업 트리에 있다.
커밋 전에 전체 diff와 사용자 소유 변경을 다시 분리해야 한다.

후속 작업으로 논문용 `benchmark/` 패키지와 평지·계단 A/B/C, 강건성,
Walk↔Roll 전환, JSONL/CSV 통계 경로를 추가하고 실제 명목·스모크 실험을
실행했다. 이 후속 작업의 조건, 수치, 143개 전체 회귀 테스트와 논문 사용 제한은
[`16-icra-simulation-benchmark-implementation-and-results.md`](16-icra-simulation-benchmark-implementation-and-results.md)에 분리해 기록한다.

세부 문서의 역할은 다음과 같다.

| 문서 | 읽어야 할 때 |
|---|---|
| [`08-rl-development-log.md`](08-rl-development-log.md) | RL 환경의 전체 역사와 날짜별 시행착오가 필요할 때 |
| [`09-gait-performance-analysis.md`](09-gait-performance-analysis.md) | 하드코드/Non-RL/PPO 속도와 clipping 원인을 비교할 때 |
| [`10-tripod-gait-and-scone-gait.md`](10-tripod-gait-and-scone-gait.md) | gait 수식, 좌표계, IK, reference 호환성을 수정할 때 |
| [`11-scone-stair-climbing.md`](11-scone-stair-climbing.md) | 후킹 공식, 100/150/200 mm sweep, brace 결과가 필요할 때 |
| [`12-automatic-stair-demo-and-continuous-roll-rework.md`](12-automatic-stair-demo-and-continuous-roll-rework.md) | 처짐, 속도, `roll-gait`, 자동 viewer의 모든 후보를 볼 때 |
| [`13-feature-implementation-and-modification-guide.md`](13-feature-implementation-and-modification-guide.md) | 실제 코드와 설정을 안전하게 수정할 때 |
| [`14-roll-gait-and-hybrid-scone-gait.md`](14-roll-gait-and-hybrid-scone-gait.md) | 현재 이름, PPO/hybrid 공식, multi-turn과 준비 상태를 볼 때 |
| [`16-icra-simulation-benchmark-implementation-and-results.md`](16-icra-simulation-benchmark-implementation-and-results.md) | 평지·계단 A/B/C, 강건성, 전환 실험의 조건·원본 결과·통계를 볼 때 |
| 현재 문서 | 전체 활동의 시작점과 최종 판정을 빠르게 추적할 때 |
