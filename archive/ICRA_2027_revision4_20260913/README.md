# SCONE ICRA 2027 Revision 4

2026-09-13. 6족 SCONE의 심사상 부족한 근거를 정리하고, 18관절 목표 제한과 새로운 176개 시뮬레이션을 적용한 개정이다. 이전 Revision 2/3와 기존 268개 원자료는 보존했다. 자체 모의 평가는 64/100 → 68/100이며 권고는 C(3.0), 거절 쪽 경계선이다. 이 수치는 채택 확률이 아니다.

## 바로 볼 결과물

- `output/pdf/SCONE_ICRA2027_Manuscript_EN.pdf`: 8쪽 익명 영문 원고, 그림 6개, 표 3개, 인용 문헌 19개.
- `output/pdf/SCONE_ICRA2027_Review_KO.pdf`: 4쪽 한국어 재평가서. 점수 근거와 적용 결과, 남은 채택 근거.
- `media/companion_video.mp4`: 114초, 1280×720, 25fps, progressive, 약 10MB. 기존 출처 표시 자료 74초 + 새 B/P 제한 조건 시뮬레이션 20초씩.
- `ACCEPTANCE_REQUIREMENTS_KO.md`: 보강 우선순위, 이번 적용 범위, 실물 측정과 비교 기준.
- `REVIEW_KO.md`, `paper.tex`, `joint_method.tex`, `results_text.tex`, `joint_table.tex`, `references.bib`: 수정 가능한 문서 소스.

## 실제 결과와 범위

새 176/176 시험이 지정 시간을 완료했다. 제한을 적용한 128개 시험의 18관절 발행 목표 및 내부 보간 속도 위반은 0개다. 실제 관절 속도는 명령 제한의 1.232배까지 관측됐다. 명령 제한과 물리적 실행 가능성은 다르다.

고속 명령에서 B의 평균 속도는 무제한 0.289m/s, H 80% 0.227m/s, F 80% 0.316m/s다. H는 이전 목표를 완전히 발행한 뒤 보행 계산을 진행하며, F는 계산을 계속해 요청 궤적을 바꿀 수 있다. P는 같은 조건에서 0.289/0.228/0.317m/s로 경쟁력이 있다. 적응형 B가 더 좋은 방법이라고 결론내리지 않았다.

무부하 제조사 사양에 근거한 속도 제한은 실물 부하 상태에서 식별한 한계가 아니다. 60초 실행, 전환 완료, 코드 테스트 통과는 하드웨어 신뢰도나 지형 일반화를 입증하지 않는다. 새 제한기는 `benchmark/joint_governor.py`와 `benchmark/joint_limit_study.py`의 선택적 시뮬레이션 경로에만 적용했다. 기존 하드웨어·기본 제어 경로는 변경하지 않았다.

## 검증 자료

- `evidence/joint_limit/protocol.json`: 실행 전에 고정한 176개 설계, 속도 사양, 코드·모델 해시, 종료 조건.
- `evidence/joint_limit/raw.jsonl`: 실패를 포함하도록 설계한 전체 원자료. 이번 행렬에서는 176개 모두 완료.
- `evidence/joint_limit/source_stability.json`: 깨끗한 고정 소스에서 실행 중 변화 없음.
- `evidence/joint_summary.json`: 원자료에서 계산한 요약과 교차 검사.
- `evidence/joint_limit_source.bundle`, `evidence/joint_limit_source.tar.gz`: 실행 소스 `49e6851abd4864c7d4c5606a3c87d70b53780e18`.
- `evidence/tests_283.log`: 이번에 실행한 관련 전체 테스트 283개 통과. 성능 시험 수와 구분.
- `evidence/preflight.json`: 계측 확인용 짧은 B/P 실행 2개. 본 결과에 포함하지 않음.
- `evidence/joint_replay_validation.json`: B/P 시각 재생의 수치 일치. 이미지·영상 재생성은 추가 성능 표본이 아님.
- `evidence/confirmation/`: 이전 268개 기록. 새 176개와 프로토콜·측정창을 섞지 않음.
- `evidence/preserved_inputs_sha256.json`: 이전 원자료와 Revision 3 PDF의 보존 확인 해시.
- `evidence/reference_verification.json`: 새 ICRA 2024 참고논문 출처와 아직 확보되지 않은 전문.
- `evidence/final_validation.json`: 최종 PDF 페이지·폰트·숫자·출처·보존 점검.

제출 원고는 핵심 증명·조건·결과를 8쪽 안에 담는다. 내부 심사서와 소스 번들은 별도 심사 PDF로 제출하는 자료가 아니다. 소스 번들에는 과거 Git 이력이 있어 익명 외부 공개 패키지로 검토 없이 업로드하지 않는다.

## 재생성

문서만 재생성하려면 이 폴더에서 `python3 build_packet.py`를 실행한다. Tectonic, NumPy, Matplotlib, pypdf, ReportLab 및 현재 사용한 macOS 글꼴이 필요하다. 기존 원자료를 읽으며 로봇 시뮬레이션은 다시 실행하지 않는다.

시뮬레이션 재현은 다른 폴더에서 `git clone /ABSOLUTE/PATH/evidence/joint_limit_source.bundle frozen`으로 복원하고, 그 깨끗한 체크아웃에서 `python3 -m benchmark.joint_limit_study --output /ABSOLUTE/NEW/OUTPUT --workers 4`를 실행한다. 출력은 체크아웃 밖의 새 폴더를 사용해야 한다. Python 3.12.13, NumPy 2.5.0, MuJoCo 3.10.0으로 실행했으며 전체 조건은 protocol.json에 있다. 기본 제어 업데이트는 20ms, 물리는 2ms다.

영상 재생은 같은 체크아웃에서 `mjpython /ABSOLUTE/PATH/evidence/render_joint.py`를 실행한 뒤 문서 폴더의 `assemble_media.py`를 실행한다. 렌더링에서만 지면 표시 범위를 확장했으며 모델 접촉 기하는 변경하지 않는다. 같은 시험 ID의 수치 일치를 확인한 후 이미지를 사용한다.

## 제출 일정과 남은 일

[공식 ICRA 2027 안내](https://2027.ieee-icra.org/contribute/call-for-icra-2027-papers-now-accepting-submissions/)에서 원고 마감은 2026-09-15 23:59 PST, 영상의 다음 접수 기간은 9월 17~22일로 안내한다. 실제 시스템의 마감 표시를 최종 확인한다. 작성한 원고와 영상의 외부 제출은 수행하지 않았다.

채택 쪽 권고로 바꾸려면 가까운 선행연구 전문의 대응 수식 비교, 주기형 대비 필요한 조건의 증명, 부하 상태의 구동 한계와 동기화된 U/B/P 실물 검증이 가장 중요하다. 계획만으로 추가 점수를 미리 부여하지 않았다.
