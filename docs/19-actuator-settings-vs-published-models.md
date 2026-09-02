# 공개된 다이나믹셀 MuJoCo 모델과의 설정 대조 (2026-09-02)

`src/assets/model.xml`의 액추에이터 설정이 타당한지, 실제로 MuJoCo에서 학습해
실물에 배포하는 프로젝트들과 비교해 검증한 기록이다.

인용으로 끝내지 않고, 찾은 설정을 **같은 벤치에 올려 직접 측정**했다.

---

## 1. 찾은 것

| 출처 | 로봇 | 액추에이터 | 비고 |
| --- | --- | --- | --- |
| [google-deepmind/mujoco_menagerie](https://github.com/google-deepmind/mujoco_menagerie/blob/main/robotis_op3/README.md) `robotis_op3` | ROBOTIS OP3 | **XM430-W350** ×20 | DeepMind 공식 컬렉션 |
| [ROBOTIS-GIT/robotis_mujoco_menagerie](https://github.com/ROBOTIS-GIT/robotis_mujoco_menagerie) `robotis_op3` | 동일 | 동일 | **제조사 배포판** |
| [mujoco_menagerie `aloha`](https://github.com/google-deepmind/mujoco_menagerie) | ALOHA / ViperX | XM430·XM540 계열 | 관절마다 감속·벨트가 달라 직접 비교 불가 |

**OP3가 결정적이다.** 우리 중간 관절(ID 7–12)과 **완전히 같은 XM430-W350**을 쓴다.

그리고 DeepMind판과 ROBOTIS 제조사판의 값이 **완전히 동일**했다. 즉 독립적인 두
출처가 아니라 하나가 다른 하나에서 파생된 것이므로, 데이터 포인트는 하나로 세야
한다.

### OP3의 설정 (양쪽 동일)

```xml
<joint  armature="0.045" damping="1.084" frictionloss="0.03"/>
<position kp="21.1" forcerange="-5 5" ctrlrange="-3.141592 3.141592"/>
```

우리 설정 (XM430-W350, ID 7–12):

```xml
<joint armature="0.01749"/>              <!-- damping / frictionloss 없음 -->
<dcmotor nominal="12 4.1 4.817108735504349" saturation="4.1 0 0"
         ctrllimited="true" ctrlrange="-12 12"/>
```

---

## 2. 직접 벤치에 올린 결과

두 모델을 같은 단일 관절 리그에 올려 스톨 토크와 무부하 속도를 측정했다.
데이터시트: XM430-W350 @12 V — 스톨 4.1 N·m, 무부하 46 rev/min = 4.817 rad/s.

| 모델 | 스톨 토크 | 사양 대비 | 무부하 속도 | 사양 대비 |
| --- | ---: | ---: | ---: | ---: |
| 데이터시트 | 4.100 N·m | — | 4.817 rad/s | — |
| **SCONE** (dcmotor+nominal) | **4.100** | **+0.0 %** | **4.817** | **−0.0 %** |
| Menagerie OP3 (position) | 5.000 | **+22.0 %** | 4.581 | **−4.9 %** |

우리 파라미터화가 데이터시트 두 끝점을 정확히 재현하고, OP3판은 스톨을 22 % 높게
잡는다. OP3판이 틀렸다기보다 **목적이 다르다**: 안정적인 위치 제어가 목표이지
데이터시트 충실도가 목표가 아니다.

---

## 3. 가장 중요한 발견 — 어제 수정한 판단이 옳았음을 뒷받침한다

OP3의 `damping="1.084"`이 무엇을 하는지 계산해 보면:

$$\frac{5 - 0.03}{1.084} = 4.585\ \mathrm{rad/s}$$

**이 값이 곧 XM430-W350의 무부하 속도다.** 즉 OP3의 damping은 임의의 감쇠가 아니라
**토크-속도 직선 그 자체를 인코딩한 것**이다.

이유는 액추에이터 종류가 다르기 때문이다.

| | SCONE | Menagerie OP3 |
| --- | --- | --- |
| 액추에이터 | `dcmotor` (전압 입력) | `position` (위치 소스) |
| 역기전력 항 | **모델 내장** ($\tau = K(V-K\omega)/R$) | 없음 |
| 토크-속도 직선 | `nominal` 삼중항에 포함 | **`damping`으로 별도 공급** |

정리하면 **토크-속도 특성은 정확히 한 번만 인코딩해야 한다.** OP3는 `damping`으로,
우리는 `nominal`로 넣는다. 우리 모델에 damping을 추가로 넣으면 이중 계산이 되고,
실제로 무부하 속도가 3.0–3.7 % 미달했다(어제 측정, `docs/18` §1.4). **어제의 제거
결정이 공개 모델의 구조와도 일치한다.**

부수적으로: 우리 쪽이 구조적으로는 실물에 더 가깝다. 실제 다이나믹셀은 내부 PID를
가진 전압 구동 DC 모터이고, 우리 모델이 바로 그 구조다. OP3판은 모터 모델이 아예
없는 순수 위치 소스다.

---

## 4. 유일하게 남은 실질적 이견 — armature

| | armature | 함의 $J_{\rm rotor}$ | 기계 시상수 $\tau_m = JR/K^2$ |
| --- | ---: | ---: | ---: |
| SCONE | 0.01749 | $1.40\times10^{-7}$ | **21 ms** |
| Menagerie OP3 | 0.045 | $3.60\times10^{-7}$ | **53 ms** |

**2.6배 차이**다. 어느 쪽이 맞는가:

- 같은 크기 코어리스 모터의 기계 시상수는 통상 5–20 ms다. **21 ms인 우리 값이
  교과서 범위에 가깝고 53 ms는 느리다.**
- 반대로 OP3판은 널리 쓰이는 공개 값이고, `position` 액추에이터에서 `armature`는
  뻣뻣한 위치 서보를 큰 timestep에서 안정화하는 **수치 안정성 손잡이**를 겸한다.
  즉 순수한 물리 추정치가 아닐 수 있다.

**결론: 값을 바꾸지 않는다.** 대신 이 2.6배를 **정직한 불확실성 폭**으로 받아들이고,
벤치마크 결론이 그 폭 안에서 뒤집히는지 확인해야 한다.

> **다음 실험(권장).** `benchmark/icra.py`의 `_run_sensitivity`는 지금 물리
> timestep(0.001/0.002/0.004)만 훑는다. 여기에 **armature 축을 추가**해
> 0.0175 / 0.028 / 0.045(= 우리 값 / 기하평균 / Menagerie 값)로 A/B/C를 돌린다.
> 순서가 유지되면 "결론은 미공개 로터 관성 추정에 둔감하다"는 한 문장을 논문에
> 넣을 수 있고, 뒤집히면 실물 동정이 논문의 선결 조건이 된다.

---

## 5. Coulomb 마찰을 넣고 싶다면

OP3는 `frictionloss="0.03"`을 갖고 우리는 0이다. 우리 모델에서 이걸 넣으려면
**같은 편집에서 `nominal`의 무부하 속도를 모터측 값으로 올려야** 한다. 그러지
않으면 §3의 이중 계산이 재발한다.

원하는 Coulomb 항 $f$에 대해, 순 무부하 속도가 데이터시트 $\omega_{\rm nl}$이 되도록
`nominal`에 넣을 값 $\omega^{*}$는

$$\omega^{*} = \omega_{\rm nl}\left(1 - \frac{f}{\tau_{\rm stall}}\right)^{-1}$$

예: XM430-W350에 $f=0.03$을 넣으려면 $\omega^{*} = 4.817/(1-0.03/4.1) = 4.852$
rad/s = 46.3 rev/min을 `nominal`에 적어야 한다.

현재는 마찰 분해를 측정하지 않았으므로 0으로 두고, **역구동 저항이 모델에 없다**는
한계를 문서화해 둔 상태다(`docs/18` §1.4).

---

## 6. 학습·배포 관행과의 비교

MuJoCo Playground 기술 보고서는 sim-to-real에서 마찰·질량·센서 잡음 무작위화와
**하드웨어 지연을 모사하는 확률적 지연**을 공통 요소로 든다. 구체적 수치 범위는
본문에 표로 제공되지 않는다.

`walk_v2`가 이미 갖춘 것과 대조하면:

| 항목 | 공개 관행 | `walk_v2` |
| --- | --- | --- |
| 마찰 무작위화 | 표준 | ○ 0.70–1.30 |
| 질량 무작위화 | 표준 | ○ 0.90–1.10 |
| 구동력 스케일 | 표준 | ○ 0.85–1.15 |
| 관측 잡음 | 표준 | ○ σ=0.01 |
| 행동 지연 | 표준(확률적) | ○ 1 스텝, p=0.5 |
| 외란 push | 표준 | ○ 2–5 s 간격 |
| 초기 자세·헤딩 | 표준 | ○ |
| **좌우 미러 증강** | 드묾 | ○ (기구 대칭 검증 후) |

즉 무작위화 축은 공개 관행을 이미 충족한다. **빠진 것은 지연의 크기를 실측하지
않았다는 점**이다. 실물 버스 왕복 지연을 측정해 1 스텝(20 ms) 가정이 맞는지
확인해야 한다.

---

## 7. 종합

1. 우리 액추에이터 파라미터화는 **데이터시트 두 끝점을 정확히 재현**하고, 공개
   모델보다 오히려 충실하다.
2. damping/frictionloss를 뺀 어제의 수정은 **공개 모델의 구조가 뒷받침한다.**
   토크-속도 특성은 한 번만 인코딩한다.
3. **armature 2.6배 이견만 남는다.** 값은 유지하되 민감도 축으로 검증할 것.
4. 무작위화 축은 공개 관행 수준이며, **지연의 실측**이 남았다.

### 참고

- [mujoco_menagerie robotis_op3](https://github.com/google-deepmind/mujoco_menagerie/blob/main/robotis_op3/README.md)
- [ROBOTIS-GIT/robotis_mujoco_menagerie](https://github.com/ROBOTIS-GIT/robotis_mujoco_menagerie)
- [XM430-W350 e-Manual](https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/)
- [MuJoCo issue #1075 — Dynamixel XM430 액추에이터 파라미터 설정](https://github.com/google-deepmind/mujoco/issues/1075)
- [MuJoCo Playground 기술 보고서](https://arxiv.org/html/2502.08844v1)

### 재현

이 문서의 모든 표는 저장소 안의 벤치로 재생성된다.

```bash
python -m benchmark.actuator_bench            # 세 검사 모두
python -m benchmark.actuator_bench datasheet  # §2 데이터시트 재현
python -m benchmark.actuator_bench damping    # PD 임계감쇠 확인
python -m benchmark.actuator_bench compare    # §3, §4 Menagerie 대조
```

`datasheet`와 `damping`은 회귀 검사로 쓸 수 있다. 데이터시트 편차가 0.5 %를
넘거나 armature 적용 후 오버슈트가 1 %를 넘으면 0이 아닌 코드로 종료한다.
