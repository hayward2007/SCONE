# SCONE ICRA 2027 revision 2

SCONE 6족 중심의 영문 원고와 내부 심사 검토. 작업은 2026-09-12 시작했고 최종 문서 생성·검증은 한국 시각 2026-09-13에 완료했다. 기존 `archive/ICRA`와 `archive/ICRA_2027_20260912`는 보존했다. 외부 제출이나 공개는 하지 않았다.

## 먼저 읽을 파일

- `output/pdf/SCONE_ICRA2027_Manuscript_EN.pdf`: 8쪽 영문 원고. 7개 그림, 3개 표, 본문 인용 참고문헌 15개.
- `output/pdf/SCONE_ICRA2027_Review_KO.pdf`: 4쪽 한국어 내부 모의 심사와 개선 기록.
- `media/companion_video.mp4`: 74초, 1280×720, 25 fps, 약 6.19 MB. 과거 실물 장면과 새 N/U/B 시뮬레이션을 구분한 정상 속도 영상.
- `paper.tex`, `references.bib`, `results_text.tex`, `numbers.tex`: 수정 가능한 논문 원본과 수치 생성 결과.
- `REVIEW_KO.md`: 한글 검토서 원본.
- `STYLE_AND_REFERENCES_KO.md`: 실제 ICRA 논문 세 편의 문체 적용 및 근접 연구 검토 기록.
- `HARDWARE_VALIDATION_KO.md`: 새 제어기의 실물 검증 계획. 수행한 실험으로 간주하지 않는다.

## 결과의 출처

논문 표의 모든 수치는 `evidence/confirmation/raw.jsonl`의 268개 시험에서 나온다. B는 68/68개를 완료했다. 193개 전체 완료, 75개 IK 목표 거부는 모든 대조군을 합한 수이며 B의 완료율과 구분한다. 고정된 위상 집합의 관측 범위를 확률 신뢰구간으로 해석하지 않는다.

최종 실험 소스 revision은 `2267f3a48b9ad2ee0221abe128729936dcad7bad`이다. `evidence/confirmation/protocol.json`은 실행 전 조건·설정·파일 해시를, `source_stability.json`은 실행 후 변경 없음과 깨끗한 Git 상태를 기록한다. 원래 사용자의 작업 폴더는 커밋하거나 초기화하지 않았다.

- `evidence/confirmation_source.bundle`: 최종 실험 소스의 Git 번들.
- `evidence/confirmation_source.tar.gz`: 같은 소스의 파일 아카이브.
- `evidence/summary.json`, `all_trials.csv`: 원자료 기반 집계.
- `evidence/confirmation_tests.log`: 최종 전체 279개 테스트 통과 기록.
- `evidence/decomposition_validation.json`: 질량·관성, 내부 공간, 외부 지지 함수 확인.
- `evidence/replay_validation.json`: 세 시각 재실행의 속도가 원자료와 일치함을 확인.
- `evidence/velocity_validation.json`: 몸체 원점 Jacobian 속도 적분으로 20개 대표 시험을 재실행한 독립 확인. 최대 차이 0.00009324 m/s.

`raw.jsonl`의 `velocity_integral_forward_mps`는 MuJoCo `mjOBJ_BODY`의 질량중심 속도 적분이다. 주지표인 `mean_vx_mps`는 몸체 원점 순이동 거리/시간이므로, 회전 중 두 지표가 달라질 수 있다. 새 `validate_velocity.py`에서는 정확히 같은 몸체 원점에서 Jacobian으로 속도를 계산해 비교했다. 이 재실행도 원자료의 순이동 속도를 수치 허용오차 안에서 재현한다. 원자료 필드를 사후에 덮어쓰지 않았다.

`evidence/evaluation`은 선제적 재인덱싱을 도입하기 전의 252개 개발 시험이다. `aborted_instrumentation_run`은 IK 목표 거부를 시험 실패로 기록하기 전 중단된 세 행을 보존한다. 이 결과들은 최종 268개와 합쳐 집계하지 않는다. 수정 동기는 `protocol_amendment.json`, `confirmation_rationale.json`에 기록되어 있다. 최종 결과를 미공개 테스트 집합의 성능이라고 부르지 않는다.

## 재실행

별도의 폴더에 번들을 복제하고 해당 폴더에서 실행한다. 기존 연구 기록을 덮어쓰지 않도록 출력은 새 폴더여야 한다.

```sh
git clone /absolute/path/to/evidence/confirmation_source.bundle /tmp/scone-confirmation
cd /tmp/scone-confirmation
python3 -m benchmark.revision_study --output /tmp/scone-confirmation-results --workers 4
```

실험은 깨끗한 소스 상태를 요구하며 하드웨어 명령을 보내지 않는다. 실행 환경은 Python 3.12.13, MuJoCo 3.10.0, NumPy 2.5.0이며 나머지 환경 정보는 `evidence/reproduction_environment.txt`와 프로토콜에 있다. 타이어 조각은 이미 `benchmark/assets/tire_coacd_2mm.json`에 포함되어 있어 실험 재실행에 CoACD를 새로 실행할 필요는 없다. 분해를 다시 생성할 때만 CoACD 1.0.14 및 trimesh 5.1.0이 필요하다.

원고와 그림은 이 패키지 폴더에서 재생성한다.

```sh
python3 analyze_results.py
python3 build_schematic.py
tectonic --keep-logs --keep-intermediates --outdir build paper.tex
```

`render_review.py`는 ReportLab과 AppleGothic 글꼴을 사용한다. 영문 원고는 Times New Roman, Arial을 사용하므로 동일한 빌드 환경이 필요하다. 완성된 PDF에는 글꼴이 포함되어 있다. 렌더링과 영상 생성 경로는 macOS의 `mjpython` 및 FFmpeg를 사용한다. 다른 OS에서는 폰트·렌더링 실행 경로만 환경에 맞게 바꾸고, 수치 재실행과 문서 빌드를 분리한다.

## 코드 변경 범위

공유 제어기의 새 제약은 기본값으로 꺼져 있으며, 시뮬레이션 비교 경로에서 명시적으로 켠다. `rewind-budget-scone`은 B, `rewind-distance-scone`은 G이다. 기존 작업 중이던 `scone_gait_v2.py`와 `tests/test_scone_gait_v2.py`는 변경 전 사본 및 해시와 함께 보존했다. 기본 하드웨어 경로를 새 제어기로 전환하지 않았다.

## 제출 판단

내부 모의 평가는 C(3.0)를 중심으로 보며, 개별 채택 확률을 계산한 것은 아니다. 남는 가장 큰 과제는 B 제어기의 정량 실물 데이터, 최종 관절 궤적의 물리적 제약, Chang & Lin의 근접 연구 전문 대조다. 과거 계단 영상은 새 제어기의 실험이 아니다. ICRA 업로드에는 영문 원고와 허용된 영상만 사용하며, 이 내부 검토서나 소스 번들을 별도 심사 보충 PDF로 제출하지 않는다.
