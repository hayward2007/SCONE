# SCONE ICRA 2027 revision 3

2026-09-13. 심사위원의 빠른 읽기를 가정한 재평가, 기존 실험 기록의 사후 분석, 원고·서식 개정이다. Revision 2와 로봇 제어·실험 코드는 보존했다. 새 시뮬레이션이나 실물 실험은 실행하지 않았다.

## 결과물

- `output/pdf/SCONE_ICRA2027_Manuscript_EN.pdf`: 8쪽 영문 원고, 그림 7개, 표 3개, 본문에서 인용한 참고문헌 15개.
- `output/pdf/SCONE_ICRA2027_Review_KO.pdf`: 한국어 모의 심사와 점수 근거, 변경 내역, 서식·문체 적용 기록.
- `REVIEW_KO.md`: 수정 가능한 한국어 검토서.
- `paper.tex`, `results_text.tex`, `references.bib`: 수정 가능한 영문 원고와 참고문헌.
- `evidence/reviewer_reanalysis.json`: 이전 268개 기록의 전체 말단 목표 변화율과 위상별 비교. 사후 분석이며 새 성능 시험이 아니다.
- `evidence/template_validation.json`: PaperCept 공식 템플릿과 로컬 클래스의 동일성 확인.
- `evidence/final_validation.json`: 최종 페이지 수·폰트·수치·원자료 보존 검사.

자체 가중 평가: 58/100에서 64/100. 공식 ICRA 공개 척도에 대응한 모의 권고는 전후 모두 C(3.0), Low Borderline이다. 64는 채택 확률이 아니며, 공식 ICRA가 사용하는 100점 척도도 아니다. 실제 심사자는 평가하지 않았다.

## 무엇을 바꿨는가

제목·초록·첫 그림을 되감기 시간과 직접 비교 결과 중심으로 정리했다. 전체 말단 목표 변화율을 주요 결과 표에 추가했고, 원고는 이 결과가 기존 기록의 사후 분석임을 설명한다. N은 내부 기능 제한 비교군이고, P는 경쟁력 있는 주기형 대안이라는 점을 명확히 했다. 보간법의 가정, 시간 제약 활성 조건, 전체 관절 변화율 조건을 구분했다. 실물 사진은 플랫폼 설명에 유지하고, 과거 계단 연속 그림은 기존 보조 영상에서 제공한다.

8쪽 제한과 이중 익명 심사를 기준으로 공식 ieeeconf 클래스의 치수는 수정하지 않았다. Pedipulate, SpaceHopper, Weng 등의 ICRA 2024 저자 원고에서 문제 제기·능동형 서술·실험 범위 구분을 참고했다. 새 참고문헌을 수만 늘리기 위해 추가하지 않았다. 자세한 출처와 적용 근거는 한국어 검토서에 있다.

## 원자료와 이전 실행

`evidence/confirmation/`은 Revision 2에서 복사한 원자료·실행 프로토콜이다. 실행 소스는 `2267f3a48b9ad2ee0221abe128729936dcad7bad`이며 원자료 해시를 보존한다. 소스 번들, 개발 이력, 기존 279개 테스트 통과 기록, 실물 검증 계획과 74초 영상은 [이전 패키지](../ICRA_2027_revision2_20260912/README.md)에 있다. 이번 수정에 그 279개를 새로 실행했다고 주장하지 않는다.

- [기존 보조 영상](../ICRA_2027_revision2_20260912/media/companion_video.mp4)
- [실물 검증 계획](../ICRA_2027_revision2_20260912/HARDWARE_VALIDATION_KO.md)

다음 실물·구동 한계 비교에는 B/U뿐 아니라 주기형 P를 포함해야 한다. 가장 가까운 Chang와 Lin 논문의 전문은 이번에도 확보하지 못했으며, 특정 제약이 그 논문에 없다고 단정하지 않았다.

## 문서 재생성

이 폴더에서 `python3 build_packet.py`를 실행한다. Tectonic, NumPy, Matplotlib, pypdf, ReportLab 및 macOS 글꼴이 필요하다. 번들 Python이 없는 환경은 ReportLab을 설치한 현재 Python을 사용한다. 문서 재생성은 시뮬레이션을 실행하지 않는다. `reviewer_analysis.py`는 읽기 전용 원자료로 수치와 첫 그림을 재생성한다. `results_text.tex`의 본문 편집은 이전 패키지의 생성기로 덮어쓰지 않는다.

ICRA에는 영문 원고와 허용된 영상을 제출 대상으로 검토한다. 내부 심사서·실험 번들을 추가 심사 PDF로 취급하지 않는다. 외부 업로드나 논문 제출은 수행하지 않았다.
