# ICRA 제출 실험 계획과 추가 정리 항목 (2026-09-03)

원고 초안은 [`archive/ICRA/`](../archive/ICRA/)에 있고, 벤치마크 구현과 개발용
1회 실행 결과는 [`16-icra-simulation-benchmark-implementation-and-results.md`](16-icra-simulation-benchmark-implementation-and-results.md)에
기록돼 있다. 이 문서는 **제출까지 남은 실험과 정리 항목**을 우선순위, 실행 명령,
합격 기준, 소요 시간과 함께 정한다.

---

## 0. 가장 먼저: 논문의 검증 실험이 현재 동작하지 않는다

원고 §5는 개발용 ablation과 별도로 **matched(공통 파라미터·공통 난수) 프로토콜**을
"required validation"으로 약속한다. 그 프로토콜을 실제로 실행해 보니 세 조건 모두
사실상 이동하지 않는다. 아래는 `0.18 m/s` 전진, 1초 settle, 6초 측정, 명목 모델
직접 측정값이다.

| 조건 | 현재 | 개발용 대응 조건 | 비율 |
| --- | ---: | ---: | ---: |
| `matched-articulated` | **+0.0013 m/s** | `articulated-walk` +0.101~0.111 | 1/80 |
| `matched-distal-only` | **+0.0050 m/s** | `distal-only-roll` +0.090 | 1/18 |
| `matched-coordinated` | **+0.0539 m/s** (위상 0)<br>**-0.003~-0.027 m/s** (무작위 위상) | `full-roll` +0.199~0.220 | 1/4 ~ 역주행 |

### 0.1 원인 (단일 변수로 격리됨)

`MATCHED_ROLL_CONFIG`는 `RollGaitConfig()`와 **완전히 동일한 객체**다. 따라서
`matched-distal-only`와 `matched-coordinated`가 개발 조건과 다른 점은
`recalibrate_phase_pose=True` 하나뿐이다. 이 플래그는 tripod B의 하단 관절에
`+60°`를 더한 자세를 **gait planner의 nominal stance로 다시 심는다**.

```python
# benchmark/controllers.py — 세 곳에 같은 4행이 복제돼 있다
pose = planner.nominal_motor_degrees
for leg in planner.TRIPOD_B:
    pose[11 + leg] += config.tripod_b_phase_offset_degrees
planner.reset(phase=phase, motor_degrees=pose)   # <-- 여기
```

`60°` stagger 자체는 필요하다. 여섯 개구부가 동시에 바닥을 향해 차체가 주저앉는
것을 막는 **물리적 초기 조건**이다(`RollGaitConfig` 주석). 문제는 그 자세를 IK가
기준으로 삼는 **nominal**로도 써 버린 것이다. planner는 회전된 부채꼴을 기준
접촉점으로 가정하고 발 위치를 계획하므로, 실제 접촉 기하와 계획이 어긋난다.

격리 실험(`matched-articulated`, 위상 0, 6초):

| 변형 | 전진 속도 |
| --- | ---: |
| 현재 그대로 | +0.0013 m/s |
| `steering_blend 0.20 -> 0.0` | -0.0006 m/s (무효) |
| **nominal 재심기 제거** | **+0.0588 m/s** |
| 둘 다 | +0.0655 m/s |

`profile_velocity/acceleration`(160/50)도 부분 원인이다. 같은 조건에서 프로파일을
`0/0`(무제한, 개발 조건이 쓰는 값)으로 바꾸면 기계일이 `29.5 J -> 12.7 J`로
줄었다. 다만 속도는 여전히 0.005 m/s여서 주원인은 아니다. 참고로
`configure_model_gait_controller()`의 주석은 160/50 프로파일이 clipped Cartesian
gait를 최대 52° 지연시켜 **접촉이 전진·후진을 번갈아 밀게 만든다**고 이미 기록하고
있다.

### 0.2 수정 후 예상 결과 (측정)

nominal 재심기만 제거하고 물리적 stagger는 유지한 상태에서 세 조건을 다시 측정한
결과다.

| matched 조건 | 전진 | RMSE | 기계일 | COT | slip | min upright |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| articulated | +0.0588 | 0.132 | 26.5 J | 1.838 | 0.252 m | 1.0000 |
| distal-only | +0.0007 | 0.267 | 64.3 J | 33.59 | 1.710 m | 0.9899 |
| coordinated | **+0.2013** | 0.248 | 64.7 J | 1.308 | 1.238 m | 0.9776 |

**공통 파라미터에서도 원고의 순서(coordinated > articulated > distal-only)가
유지되고, 속도 배율은 오히려 커진다(1.7배 -> 3.4배).** 즉 이 버그는 논문의 결론을
바꾸지 않지만, 고치지 않으면 검증 실험이 아무것도 말해 주지 못한다.

> `distal-only`는 위 실험에서 재심기 제거 대상이 아니었다(이 조건은 helper를 쓰지
> 않고 같은 4행을 인라인으로 복제한다). 수정 시 세 곳을 모두 고쳐야 하며,
> `distal-only`는 물리적 stagger를 **유지한 채** nominal만 원복해야 한다.

### 0.3 조치 (E0, 최우선)

1. `benchmark/controllers.py`의 세 복제본을 한 함수로 합치고, "물리 목표"와
   "planner nominal"을 분리한다. stagger는 목표로만 보낸다.
2. matched 조건의 프로파일을 개발 조건과 같은 `0/0`으로 통일하거나, 160/50을
   유지할 근거(52° 지연 재측정)를 문서화한다.
3. 세 조건 각각이 `>= 0.03 m/s`로 전진하는 것을 확인한 뒤 프로토콜을 잠근다.
   이 확인 없이 `--profile evaluation`을 돌리면 2시간을 버린다.
4. 회귀 테스트를 추가한다: 세 matched 조건이 2초 창에서 양의 전진 변위를 낼 것.

---

## 1. 주장과 증거 대응표

| 주장 | 증거 상태 | 남은 작업 |
| --- | --- | --- |
| H1. 자세와 회전을 **동시에** 쓰면 평지 속도가 가장 높다 | 개발 1회 실행(0.200 vs 0.117 vs 0.090), matched 1회(0.201 vs 0.059 vs 0.001) | E0 후 `evaluation` 프로파일 20 trial, 신뢰구간 |
| H1'. 그 속도는 효율이 아니라 **slip과 일**로 산다 | 개발 실행에서 COT 1.227 vs 0.661, slip 6.2배. **초과 기계일 41.3 J이 Coulomb slip 소산 45.3 J로 설명됨**(§4.2) | 그대로 사용 가능. no-slip 제어 비교(E5)를 추가하면 주장이 강해진다 |
| H2. 열린 부채꼴 + 능동 관절이 더 높은 단을 통과한다 | 개발 1회: 200 mm에서 `full-scone`만 성공 | E2 20 trial × 3 형상 × 2 기하(닫힌 바퀴 대조) |
| H2'. 성공은 **개구부** 때문이다 | 미검증. 닫힌 바퀴 대조군이 프로토콜에 구현돼 있으나 실행되지 않았다 | E2에 포함 |
| H3. 모드 전환이 자세를 유지한다 | 1회씩(0.342 s, 0.660 s, min upright 0.991) | E3 양방향 20회 |
| H4. 불규칙 지형에서도 순서가 유지된다 | 3 seed × 2초 smoke | E4 seed 20개 × 6초 |
| 수치 안정성 | 미실행 | E6 (`--suite sensitivity`, 6 trial) |
| 실물 대응 | **없음.** 원고는 "no physical measurement"를 명시 | §3의 결정 필요 |

---

## 2. 실행할 실험

소요 시간은 이 저장소에서 측정한 **9초 trial당 약 4.8초 wall**(단일 프로세스)을
기준으로 계산했다.

| ID | 실험 | 규모 | 예상 시간 | 합격 기준 |
| --- | --- | --- | ---: | --- |
| **E0** | matched 조건 수정과 스모크 | 3 조건 × 2초 | 1분 | 세 조건 모두 전진 `>= 0.03 m/s` |
| **E1** | 평지 `pilot` -> `evaluation` | 20 trial × 9 명령 × 2 기하 × 3 조건 = 1,080 | **~1.4 h** | 조건별 95% CI가 겹치지 않거나, 겹치면 그대로 보고 |
| **E2** | 계단 `evaluation` | 20 × 3 형상 × 2 기하 × 2 전략 = 240 | **~0.5 h** | 200 mm에서 open-arc `full-scone`의 Wilson 하한 > closed-wheel 상한 |
| **E3** | 전환 반복 | 양방향 20회 | ~10분 | 전환 시간 CI와 최저 upright 보고, 실패 0건 |
| **E4** | 불규칙 지형 | 3 조건 × 20 seed × 6초 | ~10분 | 순서 유지 여부를 CI로 보고 |
| **E5** | **no-slip 회전율 ablation** (신규) | 3 회전율 × 20 trial | ~20분 | slip과 COT가 예측대로 감소([`26`](26-stair-and-hybrid-locomotion-theory.md) §4.4) |
| **E6** | 수치 민감도 | 2 기하 × 3 timestep | 2분 | `dt` 0.001/0.002 사이 속도 차 < 5% |
| **E7** | 영상·사진 재캡처 | 11 장면 | ~20분 | 동결 revision과 trial ID 연결 |

전체 약 **3시간**이며 하루 안에 끝낼 수 있는 규모다. 병렬화는 suite 단위로만 하고
(seed 분리가 이미 되어 있다), 같은 revision·같은 manifest 안에서 실행한다.

```bash
# E0 후 파일럿으로 파이프라인 확인
PYTHONPATH=. python -m benchmark icra --profile pilot --suite all --seed 20260903 \
    --output-dir benchmark/results/icra/pilot

# 본 실행 (manifest는 덮어쓰기를 거부하므로 디렉터리를 새로 준다)
PYTHONPATH=. python -m benchmark icra --profile evaluation --suite all --seed 20260903 \
    --output-dir benchmark/results/icra/eval-20260903

# 표 생성 (수기 전사 금지)
PYTHONPATH=. python -m benchmark report --input benchmark/results/icra/eval-20260903/flat.jsonl \
    --output benchmark/results/icra/eval-20260903/flat-summary.csv
```

---

## 3. 결정이 필요한 두 가지

### 3.1 강화학습(`walk_v3`)을 논문에 넣는가

현재 원고에는 RL이 **전혀 없다**. 벤치마크에도 PPO 조건이 없다(`benchmark/`에
checkpoint를 읽는 경로가 없다).

| 선택지 | 장점 | 비용/위험 |
| --- | --- | --- |
| **A. 넣지 않는다 (권고)** | 원고의 주장이 형태-제어 결합에 집중된다. 추가 실험 0 | "학습 기반과 비교했나"라는 리뷰 질문에 future work로 답해야 한다 |
| B. 짧은 절로 넣는다 | 같은 형태에서 학습 제어도 성립함을 보인다 | `walk_v3`가 zero-residual을 이기는 것이 **선행 조건**이다. 아직 미검증이며 장기 학습 + `benchmark`에 PPO adapter 신설이 필요하다 |
| C. 주요 조건으로 넣는다 | 기여가 하나 늘어난다 | ICRA 마감 안에 학습·튜닝·통계까지 끝낼 여유가 없다. 실패하면 원고 구조를 다시 써야 한다 |

**권고는 A**다. 단, B로 갈 수 있는 문을 열어 두려면 지금 필요한 것은 하나뿐이다:
`walk_v3`의 원격 장기 학습을 시작하고 `eval/beats_zero_residual`을 지켜보는 것.
승격된 `best_model.zip`이 나오면 그때 B를 재검토한다(판단 시점: E1 완료 시).

### 3.2 실물 측정을 포함하는가

원고는 시뮬레이션 전용임을 명시하고 있어 내부적으로 일관되다. 그래도 ICRA
리뷰어는 대개 실물 증거를 요구한다.

| 선택지 | 필요한 것 | 판단 |
| --- | --- | --- |
| 시뮬레이션 전용 유지 | 없음 | 형태·제어 **가능성** 주장까지만. 지금 상태에서 가장 안전 |
| 최소 실물 검증 추가 | 평지 속도 1개 명령 × 5회, 계단 100 mm 성공/실패, 전류 로그 | 하드웨어 가용성과 안전 확인이 선행. 있다면 리뷰 방어력이 가장 크게 오른다 |
| 전체 sim-to-real 표 | 질량·관성·joint zero·마찰·토크 실측 보정 | 이번 마감에는 비현실적 |

**권고**: 하드웨어가 동작 가능하면 "최소 실물 검증"만 추가하고, 표는 별도
sim-to-real 절로 분리해 시뮬레이션 표와 섞지 않는다. 불가능하면 시뮬레이션 전용을
유지하고 §7 한계에 그 이유를 명시한다.

---

## 4. 원고에 추가로 정리할 내용

### 4.1 §0의 버그와 그 수정은 방법론 절에 한 문장으로 남긴다

"matched 조건의 planner nominal 재심기 결함을 수정한 뒤 프로토콜을 잠갔다"는
사실은 재현성 서술이다. 결과가 바뀌지 않았으므로 숨길 이유가 없고, 숨기면
개발 결과와 matched 결과의 불일치를 설명할 수 없다.

### 4.2 slip 에너지 수지를 결과로 승격한다 (신규, 이미 계산됨)

원고 §6은 slip 예측식의 4.2% 일치를 이미 보고한다. 여기에 **에너지 수지**를
추가하면 "속도를 slip으로 산다"는 주장이 정량적으로 닫힌다. 모델 질량
4.161 kg(무게 40.82 N), 타이어/지형 마찰 1.0에서:

| 조건 | slip 적분 | `mu W D_slip` | 실제 기계일 | 비 |
| --- | ---: | ---: | ---: | ---: |
| articulated-walk | 0.212 m | 8.67 J | 18.95 J | 45.8% |
| distal-only-roll | 1.603 m | 65.44 J | 57.01 J | 114.8% |
| full-roll | 1.323 m | 53.99 J | 60.24 J | 89.6% |

그리고 **`full-roll`이 walk보다 더 쓴 41.29 J은 두 조건의 Coulomb slip 소산 차
45.31 J로 설명된다(비 1.097).** 가정(전체 수직 하중이 미끄러지는 접촉에 실린다,
마찰 1.0)을 명시하면 한 문단으로 쓸 수 있다. 이 값은 §2의 E5(no-slip ablation)의
예측을 만들어 주기도 한다.

### 4.3 계단 이론을 단계 모델로 다시 쓴다

현재 §3~4는 조건들의 목록이다. [`26`](26-stair-and-hybrid-locomotion-theory.md) §3의
**5단계 모델(reach → engage → pivot → transport → settle)**로 재구성하면 "왜
200 mm에서 관절이 있어야만 성공하는가"가 수식으로 설명된다. 핵심 한 줄은
다음과 같다: `h > R_o`이면 순수 pivot의 수평 push 요구가 발산하므로
(`F_x = W x_Q/(R-h)`), 성공은 **swing 다리의 기구학적 도달**에서 와야 한다.
측정된 도달 범위는 Standard 자세에서 **307 mm**, Sport 자세에서 **114 mm**다.
후자는 200 mm를 통과할 수 없다. 이것이 계단 실험이 Standard 자세를 쓰는 이유이자,
`distal-only`가 200 mm에서 실패하는 이유의 기구학적 설명이다.

### 4.4 표·그림 목록과 생성 스크립트를 고정한다

| 산출물 | 내용 | 생성 |
| --- | --- | --- |
| Table I | 평지 matched 3조건 × 9명령, 평균 ± 95% CI | `benchmark report` (E1) |
| Table II | 계단 성공률 Wilson CI, open-arc vs closed-wheel | `benchmark report` (E2) |
| Table III | 전환 시간·최저 upright | `benchmark report` (E3) |
| Table IV (신규) | slip 에너지 수지 (§4.2) | E1 JSONL에서 계산 |
| Fig. flat_metrics | 속도/일/slip 막대 + CI | E1 |
| Fig. stair_results | 형상별 성공/시간 | E2 |
| Fig. teaser | 3장면 캡처 | E7 |
| Fig. (신규) slip-rate | 회전율 대비 slip·COT (E5) | E5 |

모든 표는 JSONL에서 스크립트로 생성하고 **수기 전사를 금지**한다(현재 원고의
표는 수기 값이다).

### 4.5 원고 문구 점검 목록

- §5의 "A and C do not share the walking parameter set" 각주는 matched 실행 후
  갱신한다. matched 결과가 본문에 들어가면 이 caveat의 위치가 바뀐다.
- §6의 `distal-only` 150 mm 성공 서술은 `q_open`(개구 chord)만으로 개구부 효과를
  주장하지 않도록 닫힌 바퀴 대조 결과를 함께 쓴다.
- "no physical measurement" 문장은 §3.2 결정에 따라 갱신한다.

---

## 5. 동결과 재현성 체크리스트

```text
[ ] E0 수정과 회귀 테스트 통과, 전체 206개 테스트 통과
[ ] git worktree clean, revision 기록 (현재 개발 결과는 git_dirty=true 라 사용 불가)
[ ] MJCF checksum, Python/MuJoCo/OS 버전을 manifest에 포함 (icra.py가 이미 기록)
[ ] seed 목록을 실행 전에 고정 (tuning seed와 evaluation seed 분리)
[ ] E1~E7을 같은 revision에서 실행
[ ] 원본 JSONL + 집계 CSV + 무편집 영상을 trial ID로 연결해 아카이브
[ ] 표·그림을 스크립트로 재생성해 원고에 삽입
[ ] 실패 trial도 함께 보존 (성공률 분모)
```

## 6. 위험

| 위험 | 영향 | 완화 |
| --- | --- | --- |
| E0 수정이 개발 결과와 다른 순서를 만든다 | 원고 주장 재작성 | 이미 측정했다(§0.2). 순서는 유지된다 |
| `matched-distal-only`가 수정 후에도 0에 가깝다 | 3조건 비교가 2조건 비교로 축소 | 그 자체를 결과로 보고한다. "회전만으로는 이 모델에서 전진하지 못한다"는 명확한 음성 결과다 |
| 20 trial로도 CI가 겹친다 | H1의 강한 주장 불가 | 그대로 보고하고 효과 크기만 주장한다. 표본을 늘려 유의성을 만들지 않는다 |
| 계단 성공률이 형상마다 0 또는 1로 포화 | CI가 무의미 | riser/tread 경계 sweep으로 전이 구간을 찾는다(`benchmark stairs`의 Cartesian sweep) |
| 실물 검증 불가 | 리뷰 지적 | 시뮬레이션 전용임을 초록과 결론에 명시(현재 원고가 이미 그렇게 함) |
