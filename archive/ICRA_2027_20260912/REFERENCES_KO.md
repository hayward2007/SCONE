# 참고문헌과 주장 연결

확인일: 2026-09-12. 영문 원고의 `references.bib`에는 15개 항목을 두었다. 아래의 외부 연구 수치는 SCONE의 실험값에 합치지 않았다. 출판사·저자 자료로 내용의 범위를 확인하고 DOI 등록 메타데이터로 저자·권호·쪽을 대조했다. 원문 접근 수준이 다른 항목은 명시했다.

## 가장 가까운 선행연구

| 문헌 | 본문에서 인용하는 이유 | SCONE이 추가로 입증해야 할 차이 |
|---|---|---|
| [Quattroped, 2014](https://doi.org/10.1109/TMECH.2013.2253615) | 동일 구동계를 바퀴/다리 형태에서 공유하는 선행 플랫폼 | 모터 공유 자체를 독창성으로 주장할 수 없음 |
| [TurboQuad, 2017](https://doi.org/10.1109/TRO.2017.2696022) | 변형과 보행/주행 전환의 선행 사례 | 고정된 열린 호와 다리별 역할 할당에 한정한 차이 |
| [Rolling vs. Swing, 2025](https://doi.org/10.3390/biomimetics10070435) | R-Taichi에서 굴림·스윙과 혼합 동작을 물리 실험으로 비교 | 단순 혼합 동작을 넘어서 이동량 분배·재인덱싱의 필요성 입증 |
| [Stride-Level Hybrid Locomotion, 2026](https://doi.org/10.1115/1.4070596) | 한 보폭 안의 굴림/다리 이동과 공중 재배치, 에너지 최적화를 다룸 | 유한 호의 역할 분배가 기존 보폭 수준 접근보다 무엇을 더 설명하는지 입증 |

Quattroped/TurboQuad의 구조·동작 설명은 [저자 연구실의 플랫폼 설명](https://biorola.me.ntu.edu.tw/research_legged_quadruped.html)과 [출판 목록](https://biorola.me.ntu.edu.tw/publication_journals.html)으로 확인했다. R-Taichi는 [PMC의 출판 원문](https://pmc.ncbi.nlm.nih.gov/articles/PMC12292866/)을 Europe PMC의 동일 논문 XML로 읽었다. 새로운 이미지나 성능 비교를 그 논문에서 복제하지 않았다.

**Chang & Lin의 2026 논문은 저자 출판 목록과 ASME가 Crossref에 등록한 초록·서지정보까지 확인했다. 유료 원문 전체를 확보하지 못했으므로 구현·수식의 세부 차이를 검증 완료라고 하지 않는다.** 원고에서는 초록이 직접 뒷받침하는 수준만 서술했다. 제출 전 우선 읽어야 할 가장 가까운 추가 원문이다. 2025 온라인 공개로 소개되는 곳이 있지만 최종 권호는 2026, 18(3), 031003으로 맞췄다.

## 곡선 다리의 기하·에너지

| 문헌 | 원고에서의 역할 | 확인 자료 |
|---|---|---|
| Saranli, Buehler, Koditschek, RHex, IJRR 20(7), 616–631, 2001 | 곡선 다리와 tripod는 기존 개념임 | [저자 기관 자료](https://publications.ri.cmu.edu/rhex-a-simple-and-highly-mobile-hexapod-robot), [DOI](https://doi.org/10.1177/02783640122067570) |
| Zhang et al., Q-Whex, JFR 40(6), 1444–1459, 2023 | quasi-wheel 방식의 가까운 6족 선행연구 | [출판사](https://onlinelibrary.wiley.com/doi/full/10.1002/rob.22186) |
| Vina & Barrientos, Applied Sciences 11(6), 2513, 2021 | C-leg 기하와 에너지 분석 | [출판 원문](https://www.mdpi.com/2076-3417/11/6/2513) |
| Burzyński et al., Sensors 24(5), 1636, 2024 | RHex 보행 매개변수·변위의 기구학 및 시험대 검증 | [출판 원문](https://www.mdpi.com/1424-8220/24/5/1636) |

이 그룹을 ‘SCONE보다 계단을 못 오르는 방식’으로 단정하지 않았다. 다른 기구와 실험 조건의 최고속도/에너지 수치로 순위를 매기지 않았다.

## 바퀴-다리 제어와 최근 연구

| 문헌 | 인용 범위 | 확인 자료 |
|---|---|---|
| Yoshioka et al., ASTERISK H, JRM 20(3), 403–412, 2008 | 6족의 보행·굴림과 센서 기반 전환 | [출판사 DOI](https://doi.org/10.20965/jrm.2008.p0403) |
| Bjelonic et al., Keep Rollin', RA-L 4(2), 2116–2123, 2019 | 바퀴와 몸체 운동의 전신 결합 | [저자 공개본](https://arxiv.org/abs/1809.03557), [DOI](https://doi.org/10.1109/LRA.2019.2899750) |
| Bjelonic et al., Whole-Body MPC, IROS 2021, 8388–8395 | 이동 접촉과 비주기 gait 선택 | [저자 PDF](https://www.markobjelonic.com/publications/files/2021_iros_bjelonic.pdf), [DOI](https://doi.org/10.1109/IROS51168.2021.9636371) |
| Sun et al., ATRos, 2025 | 학습 기반 hybrid locomotion의 관련 방향 | [arXiv:2510.09980](https://arxiv.org/abs/2510.09980); 워크숍 제출 preprint임을 명시 |
| Patrizi et al., RL-Augmented MPC, RA-L 11(5), 5797–5804, 2026 | RL과 MPC로 접촉 시점을 다루는 최근 접근 | [저자 공개본](https://arxiv.org/abs/2603.10878), [최종 저널 DOI](https://doi.org/10.1109/LRA.2026.3675839) |

Patrizi 논문은 arXiv만 보고 preprint라고 남기지 않고 연결된 RA-L DOI와 출판 정보를 확인해 저널 항목으로 고쳤다. SCONE의 이번 제어에는 RL/MPC를 사용하지 않았으며 이 논문들보다 우수하다는 비교 결과는 없다.

## 시뮬레이션 근거

- Todorov, Erez, Tassa, MuJoCo, IROS 2012, 5026–5033: [저자 원문](https://homes.cs.washington.edu/~todorov/papers/TodorovIROS12.pdf), [DOI](https://doi.org/10.1109/IROS.2012.6386109). 물리 엔진 출처에만 사용한다.
- Google DeepMind, [MuJoCo Collision Detection](https://mujoco.readthedocs.io/en/stable/computation/index.html): 단일 mesh의 볼록 충돌 경로와 비볼록 표현의 필요성을 확인했다. SCONE 타이어의 실제 문제는 별도로 실행한 hull 계산과 sphere probe에서 확인했다.

## 내부 원본 자료의 사용 규칙

- `archive/papers/Eco-friendly deliver SCONE Paper.pdf`: KSEF 계열의 과거 연구 보고서. 평행 링크, 배터리/외부 전원 실패, 모터 과부하가 적혀 있다. 기존 보고서의 0.5/0.7 m/s를 현 제어기의 성능으로 사용하지 않는다.
- `archive/assets/Eco-friendly deliver SCONE Poster.jpg`: 실물 제작·타이어·v1/v2 개선 사진과 과거 성능 주장이 있다. 포스터의 v2 0.7/2.0 m/s, 계단 기록 등은 동기화된 원시 측정·조건이 확인되지 않아 새 결과 표에 쓰지 않는다. 이름·학교·포스터 번호를 포함하므로 전체 이미지를 익명 원고에 넣지 않는다.
- `archive/assets/SCONEv2 Arc-Shaped Wheel.pdf`: 2024-06-19 도면. 112.5/122.5 mm 반경·44 mm 폭 및 148.27° 표기가 있다. docs/26의 135° 개구부 설명과 버전/각도 정의를 바로 동일시하지 않는다. 원고에서는 치수 없는 부품 상세만 기구 맥락으로 사용했다.
- `archive/videos/SCONEv2.mp4`, `SCONEv2_stairs.mp4`: 실물 존재·정성 동작 근거. 현재 controller의 실물 시험이라고 하지 않는다. 원본 해시와 프레임 시점은 `evidence/media_manifest.json`에 있다.

KSEF 보고서의 정식 출판·DOI·심사된 proceedings 여부는 저장소만으로 확정할 수 없다. 원고에 외부 공개 이력이 있는 저자 자신의 선행 발표를 누락할지 단정하지 말고, 실제 저자가 이력을 확인한 뒤 ICRA의 중복 출판 및 익명 자기 인용 규칙에 맞춰 처리해야 한다. 현재 내부 보고서를 저널 논문처럼 꾸민 서지 항목은 만들지 않았다.

## 제출 정책과 내부 자료

[ICRA 2027 공식 CFP](https://2027.ieee-icra.org/contribute/call-for-icra-2027-papers-now-accepting-submissions/), [공식 reviewer 지침](https://www.ieee-ras.org/conferences-workshops/fully-sponsored/icra/information-for-icra-reviewers/), [익명 심사 규칙](https://www.ieee-ras.org/publications/rules-for-the-double-anonymous-review-process/), [AI 지침](https://www.ieee-ras.org/publications/guidelines-for-generative-ai-usage/)은 한국어 검토 보고서의 정책 출처다. 기술 논문의 참고문헌 수를 늘리기 위한 항목으로 넣지 않았다.

참고문헌 해설과 내부 검토 보고서는 저자용 작업 자료다. 일반 논문에서 별도의 긴 PDF supplement를 제출할 수 있다고 가정하지 않는다. 필수 주장은 8쪽 본문 안에 포함하고, 동반 영상만 공식 규격에 맞춰 별도로 준비한다.
