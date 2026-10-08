# SCONE · ICRA 2027 · 2026-09-12 검토 묶음

사용자가 선택한 **SCONE 6족** 기준이다. 기존 `archive/ICRA/` 원고와 수정 중인 제어 코드를 보존하고 새로 작성했다.

## 읽을 자료

- `output/pdf/SCONE_ICRA2027_Manuscript_EN.pdf`: IEEE 2단 영문 연구 초안. 심사용 본문은 고유 이름·저자·기관을 익명화했다.
- `output/pdf/SCONE_ICRA2027_Review_KO.pdf`: 한국어 전체 프로젝트 검토, 공식 제출 요건, 모의 심사, 수정 우선순위.
- `REVIEW_KO.md`: 위 검토 보고서의 편집 가능한 원문.
- `REVISION_NOTES_KO.md`: 채택 가능성 판단, 모의 심사 후 개선점, 실제 반영 내용과 남은 실험.
- `REFERENCES_KO.md`: 15개 참고문헌의 역할, DOI/출판 정보 확인, 가까운 연구와의 차이, 내부 자료 사용 원칙.
- `media/companion_video.mp4`: 74초, 1280×720, 25 fps, 약 6.2 MB의 동반 영상 초안. 실물 archive와 새 시뮬레이션을 구분한다.
- `paper.tex`, `results_text.tex`, `numbers.tex`, `references.bib`, `figures/`: 논문 원문과 데이터 기반 도표.
- `evidence/`: 원시 실험, source manifest, 소스 스냅샷, 전체 테스트 로그, 접촉 형상 진단.

**문서 작성은 완료했지만 학술 제출 준비가 완료된 것은 아니다.** 현재 모의 심사 판정은 C- / 2.5 수준이다. 실제 심사 결과나 채택 확률을 뜻하지 않는다. 남은 공백은 충돌 모델의 내부 오목 형상, 최신 후보의 실행 연결, 동결된 외부 평가와 실물 검증이다. 업로드나 제출은 수행하지 않았다.

수정 후에도 채택 가능성은 낮다고 판단했다. 실물 사진의 추가는 기체의 존재를 확인하지만 이번 제어기의 실물 정량 비교를 대체하지 않는다. 수락 확률을 계산할 근거가 없어 임의의 퍼센트를 만들지 않았다. 핵심은 더 많은 사진 자체보다 구성요소별 제거 실험과 2025/2026년 가까운 선행연구에 대한 실질적 추가 기여다.

## 이번에 확인한 결과

- Python 소스 107개, 33,072줄의 구문/구조 목록화와 핵심 구현 검토.
- 전체 테스트: 260개 실행 항목, 259개 통과와 후보 테스트 모듈 import 오류 1개. 미수집된 후보 모듈 내부 테스트는 실행 수에 포함하지 않는다.
- 새 전진 비교: 동일 설정에서 5개 조건 × 명령 2개 × 초기 위상 8개 = 80회. 모두 8초 구간을 완료했다.
- 별도 기존 adapter 6종을 6초씩 다시 실행하여 문서/초기화 문제를 확인했다.
- TIRE convex hull 및 1 mm sphere probe로 내부 공간의 충돌 재현 문제 확인.
- 126개 기하 접점 표본으로 sector addition의 근사 한계 측정.
- 1·2·4 ms 시간 간격 × N/P12/C × 초기 위상 0의 9회 추가 진단. 2 ms 대비 속도 차이 최대 0.00085 m/s 미만.
- N/P12/C의 3회 영상 재생은 원래 위상 0의 수치와 1e-9 m/s 이내 일치. 재생 및 시간 간격 진단을 80회의 독립 표본 수에 더하지 않았다.
- 실제 v1/v2/계단 영상, 과거 KSEF 보고서, 포스터와 제조 도면 검토. 사진·도면·계단 4개 프레임·시뮬레이션 9개 프레임을 6개 논문 그림에 구성했다.
- 실행 전/후 입력 소스 해시 변화 없음. 실험은 dirty worktree의 명시적 개발 스냅샷이며 held-out/clean release evaluation이 아니다.

`phase_grid.jsonl`과 `legacy_adapter_audit.json`은 서로 다른 제어 계열의 데이터다. 이전 `benchmark/results/*nominal*`과도 합치지 않는다. 위상 8개에 대해 보고하는 범위는 관측된 최솟값~최댓값이다. 신뢰구간이나 실물 성공 확률이 아니다.

## 빌드

원래 실험 환경: Python 3.12.13, MuJoCo 3.10.0, NumPy 2.5.0. 다른 의존성은 `evidence/audit_protocol.json`과 현재 프로젝트 요구사항을 함께 확인한다. 문서 생성은 Matplotlib, SciPy, Tectonic, ReportLab, 시스템 폰트를 사용한다. 기본 제어 패키지를 설치하거나 변경하는 작업은 포함하지 않았다.

저장소 루트에서 실행:

```bash
MPLCONFIGDIR=/private/tmp/scone-icra-mpl python3 archive/ICRA_2027_20260912/build_outputs.py
```

패킷 폴더에서:

```bash
tectonic --keep-logs --keep-intermediates --outdir build paper.tex
```

한국어 보고서는 ReportLab이 있는 Python에서 `render_review.py`를 실행한다. macOS `AppleGothic.ttf`를 사용하므로 다른 OS에서는 해당 폰트 경로를 적절한 한글 TrueType 폰트로 바꿔야 한다. 글꼴은 배포 묶음에 복제하지 않았다. 영문 최종본은 `build/paper.pdf`에서 식별 메타데이터를 정리해 `output/pdf/`로 내보냈다.

실물/재생 패널은 `build_media_figures.py`, 영상 구성은 `build_video.py`로 재생성할 수 있다. 원본 archive 영상이 필요하다. 새 시뮬레이션 화면은 `evidence/render_replays.py`를 macOS의 `mjpython`으로 촬영했다. `capture_and_sensitivity.py`는 기록 덮어쓰기를 거부하며, 원래 source manifest와 다르면 실행을 중단한다. 영상 생성에는 FFmpeg와 시스템 Arial 폰트가 필요하다.

영상 순서: 0–6초 안내, 6–30초 실물 floor archive(원본 0–24초), 30–38초 N, 38–46초 P12, 46–54초 C, 54–66초 실물 stair archive(원본 18–30초), 66–74초 한계. 모든 동작은 1배 재생이다. 원본 실물 영상 30 fps를 25 fps로 변환하면서 재생 시간은 유지했고 음성은 제외했다. `evidence/video_validation.json`에 프레임·크기·형식 확인이 있다.

## 수치 재현과 보존

`evidence/run_audit.py`는 기존 원시 결과를 덮어쓰지 않도록 `open('x')`를 사용한다. 기존 evidence 폴더에서 다시 실행하면 중단하는 것이 정상이다.

독립 재현은 새 작업 폴더에 `evidence/source_snapshot.tar.gz`를 풀고, 그 안에 `archive/ICRA_2027_20260912/evidence/`를 만든 뒤 **runner 파일만** 복사해 실행한다. 같은 디렉터리 깊이를 유지해야 runner가 소스 루트를 찾는다. 원본 결과를 복사하지 않으면 새 원시 기록을 독립적으로 만들 수 있다. `.git` 없는 재현 사본의 revision은 null일 수 있으므로 원본 해시 manifest로 소스를 비교한다. 이것이 정식 clean revision 평가를 대체하지는 않는다.

실물 controller를 열지 않는 headless simulation만 실행한다. 새로운 결과를 본 뒤 설정을 바꾸면 별도 실험 이름과 manifest로 관리해야 한다. 패킷 전체에는 내부 경로·원본 코드·실제 프로젝트 이름이 포함되어 있으므로 **그대로 익명 supplement로 업로드하지 않는다.** 심사 제출용 익명 재현 묶음은 별도 작성해야 한다.

## 제출 전 확인

논문 마감은 공식 안내상 2026-09-15 23:59 PST이며 시간대 해석 주의점은 한국어 보고서에 있다. 영문 PDF가 8쪽 이내라는 것은 분량 확인일 뿐 PaperPlaza의 전체 형식 검사를 통과했다는 뜻이 아니다. 최신 원고·영상·익명 자료를 실제 제출 시스템에서 다시 검증해야 한다.

한국어 보고서·참고문헌 해설·원시 archive는 내부 작업 자료다. 추가 PDF supplement가 허용된다고 가정하지 않는다. KSEF 계열 과거 보고서의 정식 출판/DOI 여부와 자기 선행 발표 인용은 실제 저자가 확인해야 한다. 익명 영상에도 원본 장면에서 개인·기관을 식별할 정보가 남는지 최종 검토해야 한다.
