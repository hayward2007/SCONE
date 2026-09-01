# ICRA 시뮬레이션 벤치마크 구현과 실행 기록

## 1. 문서 목적

이 문서는 2026-09-01에 ICRA 제출용 실험 기반을 만들기 위해 수행한 SCONE
시뮬레이션 벤치마크 작업을 정리한다. 구현한 코드, 실험 조건, 실제 실행 명령,
측정 결과, 검증 상태와 아직 논문 근거로 사용할 수 없는 부분을 한곳에 기록한다.

이번 작업의 목표는 다음 질문을 재현 가능한 코드로 검사하는 것이었다.

1. 같은 SCONE MuJoCo 모델에서 관절 보행, 말단만 회전, 전체 SCONE 회전은
   전진 속도와 안정성·에너지·미끄러짐에서 어떻게 다른가?
2. 계단에서 말단 회전만 사용하거나 고정 open-loop 동작을 사용하는 것보다
   SCONE의 능동 관절과 부채꼴 말단을 함께 사용하는 방식이 더 높은 형상을
   통과하는가?
3. 질량·마찰·액추에이터 출력과 초기 조건이 달라져도 결과가 유지되는가?
4. 보행과 주행 사이 모드 전환은 어느 정도 시간이 걸리고 자세를 유지하는가?

이 벤치마크는 현재 MuJoCo 모델과 현재 controller 구현에 대한 증거다. 실물
SCONE의 성능을 자동으로 입증하지 않으며, 시뮬레이션-실물 보정과 반복 실험을
대체하지 않는다.

## 2. 추가한 파일

| 경로 | 역할 |
|---|---|
| [`benchmark/__main__.py`](../benchmark/__main__.py) | `python -m benchmark`의 통합 subcommand 진입점 |
| [`benchmark/common.py`](../benchmark/common.py) | fresh MuJoCo trial, 모델 perturbation, 공통 지표, JSONL/CSV 출력 |
| [`benchmark/controllers.py`](../benchmark/controllers.py) | 평지 A/B/C controller adapter |
| [`benchmark/flat.py`](../benchmark/flat.py) | 평지 명령 추종 및 A/B/C 비교 |
| [`benchmark/robustness.py`](../benchmark/robustness.py) | 질량·마찰·출력·초기 자세·지형 seed Monte Carlo |
| [`benchmark/stairs.py`](../benchmark/stairs.py) | 계단 A/B/C와 riser/tread Cartesian sweep |
| [`benchmark/transitions.py`](../benchmark/transitions.py) | Walk→Roll, Roll→Walk 전환 측정 |
| [`benchmark/report.py`](../benchmark/report.py) | JSONL/CSV 집계, 평균·표준편차·95% CI 생성 |
| [`benchmark/README.md`](../benchmark/README.md) | 각 suite의 실행법과 해석 규칙 |
| [`benchmark/results/.gitignore`](../benchmark/results/.gitignore) | 생성된 실험 결과를 Git 소스와 분리 |
| [`tests/test_benchmark.py`](../tests/test_benchmark.py) | 입력 검증, 짧은 동역학 실행, 통계·파일 출력 회귀 테스트 |

기존 하드웨어 controller와 공용 locomotion 파일은 이 작업에서 변경하지 않았다.
벤치마크는 기존 구현을 adapter로 호출하므로 실물 제어 경로에 별도 동작을
추가하지 않는다.

## 3. 공통 실행 구조

평지·강건성·전환 trial은 다음 순서로 실행된다.

```text
MJCF + terrain 로드
  → 질량/마찰/actuator strength perturbation
  → 새 MjData·MuJoCoController·SCONE 생성
  → 기존 robot.initialize() 실행
  → controller별 준비 자세
  → 50 Hz 명령, MuJoCo physics step 반복
  → 상태·접촉·토크·에너지 누적
  → JSONL 한 줄 저장
```

각 trial은 모델, `MjData`, controller와 robot을 새로 생성한다. 이전 trial의
적분기·관절 상태·접촉 상태가 다음 trial에 남지 않는다. 결과에는 Git revision과
dirty-worktree 여부도 함께 저장한다.

## 4. 실험 조건

### 4.1 평지 A/B/C

| 논문용 이름 | 상·중단 관절 | 하단 부채꼴 말단 |
|---|---|---|
| `articulated-walk` | 기존 alternating-tripod IK 보행 | bounded position motion |
| `distal-only-roll` | 초기 목표 자세에 고정 | velocity mode 연속 회전 |
| `full-roll` | 현재 SCONE stabilizing gait | 연속 회전과 phase stagger |

명목 실행은 평지에서 `vx=0.18 m/s`, 1초 settling, 6초 측정, gait phase 0으로
진행했다. `completed=True`는 목표 속도 달성을 의미하지 않는다. 비정상 수치가
발생하지 않고 root tilt가 60° 한계를 넘지 않은 채 측정 창을 완료했다는 뜻이다.

세 조건은 같은 MJCF에서 현재 구현된 controller를 비교하지만, controller별
motion profile·gain·bandwidth가 완전히 동일한 구동 예산으로 정규화된 것은
아니다. 따라서 현재 결과는 **as-implemented controller ablation**이다. 구조
자체의 우월성을 주장하려면 토크·회전속도·제어주기·명령 saturation을 맞춘
추가 ablation이 필요하다.

### 4.2 계단 A/B/C

| 논문용 이름 | 기존 계단 구현 | 의미 |
|---|---|---|
| `distal-only` | `pure-rolling` | 상·중단의 능동 계단 자세 없이 말단 회전 |
| `synchronized-open-loop` | 동명의 기존 전략 | 앞 1단 270° brace와 고정 distal velocity |
| `full-scone` | `adaptive` | riser별 partial brace와 closed-loop shared phase |

계단 성공은 기존 `stair_benchmark`의 `top_reached` 판정으로 결정한다. preset뿐
아니라 사용자가 지정한 riser와 tread 조합을 임시 `StairProfile`로 만들어
Cartesian sweep할 수 있다. 임시 profile은 trial 종료 후 원래 preset으로
복구한다.

### 4.3 강건성

기본 Monte Carlo 범위는 다음과 같다.

- 전체 body mass/inertia: 현재 모델의 `0.9..1.1`배
- sliding friction: `0.4..1.2`배
- actuator gain과 force range: `0.85..1.15`배
- 초기 x/y: 각각 `±30 mm`
- 초기 yaw: `±5°`
- gait phase: `0..1`
- procedural terrain seed: trial마다 분리

현재 perturbation은 모든 body·actuator에 같은 scale을 적용한다. 부품별 질량
오차, 배터리 위치 오차, 타이어 이방성·속도 의존 마찰은 아직 포함하지 않는다.

### 4.4 모드 전환

- `walk-to-roll`: 보행 → neutralization → Roll 준비 자세 → velocity mode → 회복
- `roll-to-walk`: Roll → neutralization → position mode → Standard 18관절 목표
  재획득 → 보행 회복

`roll-to-walk`의 `mode_switch_duration_s`는 모든 18개 관절이 Standard 목표의
96 raw 이내에 들어올 때까지 포함한다. 전체 recorder는 전환 직전 neutralization과
전환 뒤 recovery도 측정한다.

## 5. 기록하는 지표와 계산법

### 이동과 명령 추종

- 시작 body frame 기준 `x/y/z` 변위와 평균 `vx/vy`
- 누적 yaw와 평균 yaw rate
- 매 physics step의 body velocity 및 yaw-rate RMSE
- 횡방향 drift와 명령 반대 방향으로 이동한 거리

### 자세 안정성

- root 높이의 최저·최고·RMS 변화
- roll/pitch RMS와 최대 절댓값
- body z축과 world z축 내적으로 계산한 `minimum_upright`
- non-finite state 또는 최대 tilt 초과 termination

### 구동과 에너지

- 절대 기계일:

  ```text
  W_mech = integral(sum(abs(tau_i * qdot_i))) dt
  ```

- 기계적 Cost of Transport:

  ```text
  COT = W_mech / (total_mass * g * horizontal_distance)
  ```

- motor resistance·back-EMF와 simulation terminal voltage로 복원한 절대 전기
  에너지 추정치
- peak actuator torque와 추정 motor current

전기 에너지는 배터리에서 측정한 값이 아니다. 회생과 driver 손실을 실제 전원계와
동일하게 모델링한 결과도 아니므로 논문에서는 반드시 **estimated electrical
energy**로 표기한다.

### 접촉과 제어 품질

- peak contact force
- contact-point Jacobian으로 계산한 타이어-지면 접선 slip distance
- 1 N 이상의 normal force를 받는 서로 다른 타이어 body 수의 시간 평균
- 타이어 외 부품이 지면과 충돌한 physics step 수
- IK 실패 frame, 평균 stride clipping, 최소 IK backoff scale

하나의 타이어에서 MuJoCo contact point가 여러 개 생겨도 지지 다리는 하나로
계산한다. 다만 slip은 접촉점 속도의 평균을 적분하므로 실물 타이어 변형량을
직접 뜻하지 않는다.

## 6. 실제 실행 명령

### 코드와 단위 테스트

```bash
python -m compileall -q benchmark tests/test_benchmark.py
python -m unittest tests.test_benchmark -v
git diff --check -- benchmark tests/test_benchmark.py
```

### 평지 명목 A/B/C

```bash
python -m benchmark flat \
  --all \
  --command forward \
  --trials 1 \
  --duration 6 \
  --settle 1 \
  --no-random-phase \
  --output benchmark/results/flat-nominal.jsonl
```

### 세 preset의 계단 A/B/C

```bash
python -m benchmark stairs \
  --all \
  --trials 1 \
  --output benchmark/results/stairs-nominal.jsonl
```

### 모드 전환

```bash
python -m benchmark transitions \
  --all \
  --trials 1 \
  --output benchmark/results/transitions-nominal.jsonl
```

### 짧은 강건성 실행

```bash
python -m benchmark robustness \
  --all \
  --terrain uneven \
  --trials 3 \
  --duration 2 \
  --output benchmark/results/robustness-smoke.jsonl
```

### 통계 리포트

```bash
python -m benchmark report benchmark/results/flat-nominal.jsonl \
  --output benchmark/results/flat-nominal-summary.csv

python -m benchmark report benchmark/results/stairs-nominal.jsonl \
  --output benchmark/results/stairs-nominal-summary.csv

python -m benchmark report benchmark/results/robustness-smoke.jsonl \
  --output benchmark/results/robustness-smoke-summary.csv

python -m benchmark report benchmark/results/transitions-nominal.jsonl \
  --output benchmark/results/transitions-nominal-summary.csv
```

### 전체 회귀 테스트

```bash
python -m unittest discover -s tests -v
```

## 7. 실제 결과

### 7.1 평지 명목 실행

모든 조건은 6초 측정 창을 완료했다.

| Controller | 평균 vx (m/s) | 속도 RMSE (m/s) | 기계일 (J) | 추정 전기에너지 (J) | COT | slip (m) | 평균 지지 다리 | minimum upright |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| `articulated-walk` | 0.1171 | 0.0971 | 18.95 | 27.46 | 0.661 | 0.212 | 3.668 | 0.9999 |
| `distal-only-roll` | 0.0897 | 0.2679 | 57.01 | 171.57 | 2.576 | 1.603 | 2.797 | 0.9759 |
| `full-roll` | 0.2001 | 0.2529 | 60.24 | 139.76 | 1.227 | 1.323 | 3.114 | 0.9769 |

현재 결과에서는 `full-roll`이 가장 빨랐고 `distal-only-roll`보다 slip과 추정
전기에너지가 작았다. 그러나 `articulated-walk`보다 기계일과 slip이 크다.
또한 목표 `0.18 m/s`를 넘는 controller의 순간 진동 때문에 RMSE가 커질 수
있으므로 평균 속도 하나만으로 추종 성능을 판단하면 안 된다.

`distal-only-roll`과 `full-roll`의 stride clipping 약 0.993은 두 adapter가
공유하는 planning 진단값이다. 말단만 회전하는 조건에서 실제 상·중단 관절의
clipping으로 그대로 해석하지 않는다.

- 원본: [`benchmark/results/flat-nominal.jsonl`](../benchmark/results/flat-nominal.jsonl)
- 요약: [`benchmark/results/flat-nominal-summary.csv`](../benchmark/results/flat-nominal-summary.csv)

### 7.2 계단 명목 실행

| Riser / tread | `distal-only` | `synchronized-open-loop` | `full-scone` |
|---|---:|---:|---:|
| 100 / 240 mm | 성공, 4.886초 | 성공, 3.818초 | 성공, 2.646초 |
| 150 / 200 mm | 성공, 14.960초 | 성공, 5.546초 | 성공, 6.736초 |
| 200 / 350 mm | 실패 | 실패 | 성공, 5.964초 |

`full-scone`의 성공 trial 기계일은 각 형상에서 45.50 J, 95.48 J, 88.18 J였다.
200 mm 조건에서만 `full-scone`이 성공한 결과는 논문 가설을 지지할 가능성이
있지만 현재는 결정론적 simulation 1회다. seed·마찰·출력 perturbation 반복과
실물 계단 교차 검증 전에는 일반적인 등반 우월성으로 주장하지 않는다.

- 원본: [`benchmark/results/stairs-nominal.jsonl`](../benchmark/results/stairs-nominal.jsonl)
- 요약: [`benchmark/results/stairs-nominal-summary.csv`](../benchmark/results/stairs-nominal-summary.csv)

### 7.3 강건성 스모크 실행

불규칙 지형, 각 controller 3회, trial당 2초 결과다.

| Controller | 완료 | 평균 vx (m/s) | vx 범위 (m/s) | 평균 기계일 (J) | 최저 upright |
|---|---:|---:|---:|---:|---:|
| `articulated-walk` | 3/3 | 0.0768 | 0.0732–0.0815 | 6.98 | 0.9999 |
| `distal-only-roll` | 3/3 | 0.1025 | 0.0668–0.1380 | 12.89 | 0.9922 |
| `full-roll` | 3/3 | 0.2009 | 0.1893–0.2143 | 22.16 | 0.9924 |

세 번의 짧은 실행은 Monte Carlo 파이프라인과 seed 분리를 확인한 smoke test다.
성공률이나 신뢰구간을 논문에 쓰기에는 표본과 시간이 부족하다.

- 원본: [`benchmark/results/robustness-smoke.jsonl`](../benchmark/results/robustness-smoke.jsonl)
- 요약: [`benchmark/results/robustness-smoke-summary.csv`](../benchmark/results/robustness-smoke-summary.csv)

### 7.4 모드 전환 실행

| 전환 | 전환 시간 (s) | 전체 기록 시간 (s) | 전체 평균 vx (m/s) | minimum upright | 완료 |
|---|---:|---:|---:|---:|---:|
| Walk→Roll | 0.342 | 2.542 | 0.1451 | 0.9907 | 성공 |
| Roll→Walk | 0.660 | 2.860 | 0.0138 | 0.9904 | 성공 |

Roll→Walk 평균 속도가 낮은 것은 18관절 pose 재획득 시간이 전체 기록 창에
포함되기 때문이다. 전환 1회 결과이므로 평균 전환 시간이나 신뢰구간으로
보고하지 않는다.

- 원본: [`benchmark/results/transitions-nominal.jsonl`](../benchmark/results/transitions-nominal.jsonl)
- 요약: [`benchmark/results/transitions-nominal-summary.csv`](../benchmark/results/transitions-nominal-summary.csv)

### 7.5 추가 스모크 확인

- controller별 1초 평지 A/B/C 실행
- 120 mm riser / 300 mm tread custom 계단에서 `full-scone` 성공
- 짧은 Walk↔Roll 양방향 전환
- 단일 무작위 강건성 trial
- `python -m benchmark --help` subcommand 노출

이 파일들은 개발 중 pipeline 확인용이며 `benchmark/results/` 아래에서 Git에
추적하지 않는다.

### 7.6 저용량 시뮬레이션 영상과 사진

평지 A/B/C 3개, 200 mm 계단 A/B/C 3개, Walk↔Roll 전환 2개, 고정
perturbation 불규칙 지형 A/B/C 3개를 동일한 캡처 설정으로 저장한다.

```bash
mjpython -m benchmark capture --suite all
```

기본값은 640×360, 15 fps, H.264 CRF 34와 JPEG quality 72다. frame은 FFmpeg
stdin으로 직접 전달하므로 무압축 중간 영상은 남지 않는다. 파일과 장면별 결과는
[`archive/simulation_media/README.md`](../archive/simulation_media/README.md)와
[`manifest.json`](../archive/simulation_media/manifest.json)에 기록한다. 영상은
화면상 동작 확인 자료이며 정량 성능값은 원본 JSONL을 우선한다.

## 8. 통계 처리

`benchmark report`는 기본적으로 `benchmark`, `controller`, `command_name`으로
그룹화한다.

- 이진 성공률: Wilson 95% confidence interval
- 연속값: 표본 평균, sample standard deviation, normal-approximation 95% CI
- `N=1`: 평균은 기록하지만 연속값 CI는 `NaN`
- 실패 trial에서 존재하지 않는 `time_to_top_s` 등은 해당 연속 지표의 N에서 제외

결정론적 nominal trial을 같은 조건으로 여러 번 복제해도 독립 표본이 아니다.
성공률을 추정하려면 seed나 식별된 물리 파라미터를 사전에 정한 분포로 바꿔야
한다.

## 9. 검증 결과

| 검증 | 결과 |
|---|---|
| `compileall` | 통과 |
| 벤치마크 단위 테스트 | 5/5 통과 |
| 전체 프로젝트 회귀 테스트 | 143/143 통과, 약 20.7초 |
| `git diff --check` | 통과 |
| JSONL→CSV report | 평지 3그룹, 계단 9그룹, 강건성 3그룹, 전환 2그룹 생성 |
| Ruff 정적 검사 | 현재 Python 환경에 Ruff가 설치되지 않아 실행하지 못함 |

단위 테스트는 perturbation의 비물리 입력 거부, 짧은 MuJoCo trial의 finite/JSON
직렬화, custom stair profile 복구, Wilson 구간과 JSONL/CSV 출력을 검증한다.

전체 테스트 통과는 headless 동역학과 API 회귀를 뜻한다. 장시간 GUI에서 실제
접촉·다리 배치·카메라를 눈으로 검증한 결과는 아니며, 필요하면 macOS에서
`mjpython`으로 별도 viewer 실험을 수행해야 한다.

## 10. 현재 결과를 논문에 사용하는 기준

현재 수치 자체를 최종 ICRA 표로 사용하면 안 된다. 실행 당시 revision은
`bb32236`이지만 worktree가 dirty였고, 결과 JSON의 `git_dirty`도 `true`다.
현재 결과는 코드 경로와 가설을 검증하는 개발 기록이다.

논문용 최종 실험은 다음 조건을 만족해야 한다.

1. MJCF, controller, benchmark를 하나의 깨끗한 Git revision으로 고정한다.
2. 질량·관성·joint zero·속도·stall/continuous torque·TPU 마찰을 실측 보정한다.
3. controller별 torque, maximum speed, control bandwidth와 duration을 맞춘다.
4. tuning seed와 evaluation seed를 분리하고 실험 전에 목록을 고정한다.
5. 평지 명령별 최소 20회 이상 또는 power analysis로 정한 표본 수를 수행한다.
6. 계단은 각 riser/tread에서 성공과 실패를 모두 남기고 반복 성공률을 보고한다.
7. 실물에서도 동일 명령·동일 형상으로 속도, 전류/전력, slip, 자세를 계측한다.
8. 시뮬레이션과 실물의 차이를 별도 sim-to-real 표로 보고한다.
9. 무편집 영상과 원본 로그를 trial ID로 연결한다.
10. traversal-only 시간과 준비·모드 전환을 포함한 end-to-end 시간을 분리한다.

특히 200 mm 계단의 `full-scone` 단독 성공은 후속 실험의 중심 가설로 삼을 수
있지만, 현재 단계에서 “SCONE 구조가 일반 hexapod/RHex보다 우수하다”는 결론은
도출할 수 없다.

## 11. 다음 실행 권장 순서

1. **모델 식별**: 실물 질량 중심, joint 방향·zero, actuator 속도/토크, TPU
   접촉 파라미터를 측정한다.
2. **평지 명령 grid**: idle, 전후, 좌우, 양방향 yaw, forward-turn을 조건당
   20회 이상 실행한다.
3. **동일 구동예산 ablation**: 세 controller의 제한을 일치시킨 별도 조건을
   추가한다.
4. **계단 경계 탐색**: riser/tread Cartesian grid에서 성공 경계와 95% CI를
   구한다.
5. **전환 반복**: 양방향 각 20회 이상 실행하고 peak tilt와 pose reacquisition
   실패를 함께 보고한다.
6. **실물 교차 검증**: 배터리 단독 운용, 2.0 m/s 주행, 0.7 m/s 보행, 15단
   연속 계단 영상과 원본 시간·거리·전력 로그를 benchmark trial ID와 연결한다.
7. **논문 표 생성**: 고정 revision의 JSONL만 모아 `benchmark report`로 표를
   만들고, 수기로 옮기지 않는다.

## 12. 결과 파일 보존 규칙

`benchmark/results/`는 생성 산출물이므로 기본적으로 Git에서 제외한다. 논문에
사용한 최종 결과는 다음 정보를 포함해 별도 immutable archive로 보존하는 것이
좋다.

- Git commit과 dirty 여부
- 실행 명령 전체
- Python, MuJoCo와 운영체제 버전
- MJCF checksum
- seed와 perturbation 값
- trial별 JSONL, 집계 CSV, 무편집 영상
- 실패 원인과 중단 여부

논문에는 집계 CSV만 남기지 말고 원본 JSONL과 실물 영상/센서 로그의 대응 관계를
함께 보존해야 한다.
