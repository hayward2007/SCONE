# SCONE / MARC 프로젝트 종료 및 아카이브

종료일: **2026-10-09 (Asia/Seoul)** · 저장소: [hayward2007/SCONE](https://github.com/hayward2007/SCONE)

개발을 종료하고 소스·설계·실험·논문·영상·학습 체크포인트를 보존한다.
최종 커밋 메시지는 **`get hyped for the new project 🚀`**이다.
아카이브는 현재 상태를 보관하는 절차이며, 남아 있던 미완료 설계와 검증도 그대로 기록한다.

## 프로젝트와 최종 구현 범위

SCONE은 DYNAMIXEL 실물 제어와 MuJoCo 시뮬레이션을 같은 고수준 API로 다루는
6족 로봇 프로젝트다. `SCONE.py`와 `src/main.py`가 진입점이며, `src/cli.py`가
터미널 입력을 해석한다. 하드웨어 통신, 보행, 운동학, 시뮬레이션과 RL을 모듈로 나눴다.

- **실물 제어:** `src/hardware/`의 액추에이터 ID·레지스터·장치 탐색·통신 계층과
  `src/locomotion/`의 Walk/Drive/Climb 동작.
- **보행 및 계단:** tripod, 연속 roll, PPO/점접지 하이브리드, 역할을 나눈 `scone-gait-v2`,
  동기 위상 계단 제어. 최신 역할 분리·두 다리 구동 근거는 문서 28·29에 있다.
- **시뮬레이션:** `src/simulation/`, `src/assets/`의 MuJoCo 모델·메시, 지형 생성,
  가상 DYNAMIXEL, DC motor/PID·백래시 모델과 자동 계단 데모.
- **RL:** `src/rl/`의 기존 residual, V2, V3, 다리 상실 failsafe 환경과 학습·재생·원격 관찰 도구.
  각각 70/82/76/85차원 관측을 사용하므로 체크포인트를 혼용하지 않는다.
- **실험:** `benchmark/`의 평지·계단·강건성·전환 및 ICRA 프로토콜,
  관절 제한·다리별 사용량·접촉 기하 비교 도구와 기록.
- **독립 패키지:** `packages/dynamixel-mujoco/`의 액추에이터 모델 및 사용 예제.
- **기구 계보:** SCONE v1/v2의 실물 기록, v3 설계안, 4족 MARC v4와 이후 housing·팔·전장 CAD.
  이름과 계보는 문서 30에 기록되어 있다. 기존 소스와 자산의 파일 이름은 유지했다.

## 전체 자료 색인

| 자료 | 보관 위치 | 읽는 목적 |
|---|---|---|
| 소스·실행 예제 | [`../SCONE.py`](../SCONE.py), [`../src/`](../src/), [`../example.py`](../example.py) | 최종 구현과 공개 API |
| 기능·구조·운영 문서 | [문서 색인](README.md), 문서 01–27 | 아키텍처, 문제 해결, 학습·검증 및 개발 이력 |
| 최신 역할 분리 보행 | [문서 28](28-scone-gait-v2-role-split-rolling.md), [문서 29](29-two-leg-sector-drive.md) | 굴림/보행 분담과 다리별 측정 |
| MARC 설계 명세 | [문서 30](30-scone-v3-design-plan.md), [`reviews/`](reviews/) | 설계 선택, 계산 근거와 제작 관문 |
| 벤치마크·시험 | [`../benchmark/README.md`](../benchmark/README.md), [`../tests/`](../tests/) | 실험 프로토콜과 소프트웨어 계약 |
| 액추에이터 패키지 | [`../packages/dynamixel-mujoco/README.md`](../packages/dynamixel-mujoco/README.md) | 분리된 동역학 모델 |
| 초기 MARC D0 및 수정 검토 | [`../artifacts/marc/`](../artifacts/marc/), [`../artifacts/scone_v3/`](../artifacts/scone_v3/) | CAD·STEP·매니페스트·실패 근거·인수인계 |
| 하우징 R03–R09 | [`../artifacts/housing/20260915_MARC_v10/`](../artifacts/housing/20260915_MARC_v10/) | 버전별 원본과 출력/조립 자료 |
| 본체42 / MX28 체결 | [인수 문서](../artifacts/housing/20260927_body42/README_KO.md) | 조립본·단품·치수·자세·가동 검사 기록 |
| C1/L1 및 보강 착수 | [`../artifacts/housing/20260930_c1_l1/`](../artifacts/housing/20260930_c1_l1/), [`../artifacts/housing/20261001_c1_l1_reinforced/`](../artifacts/housing/20261001_c1_l1_reinforced/) | 납품본과 보강 전 상태를 구분 |
| 팔 검토 자료 | [`../artifacts/arm/20260929_refinement/`](../artifacts/arm/20260929_refinement/) | 원본 CAD와 작업 기록 |
| Jetson·U2D2·캐리어 | [`../artifacts/electronics/`](../artifacts/electronics/) | 부품 배치·원본 CAD·검증 기록 |
| 배터리·P1 전원 검토 | [최종 PDF](../output/pdf/MARC_v4_battery_P1_review_20261007.pdf), [`../artifacts/electronics/20261007_power_review/`](../artifacts/electronics/20261007_power_review/) | 2026-10-07의 측정·회로 위험·부품 자료 |
| P1 rev A 입력 원문 | [원래 첨부 문서 복사본](../artifacts/electronics/20261007_power_review/sources/P1_rev_A_original_20261007.txt) | 검토에 사용한 설계 입력 보존 |
| ICRA 원고·검토 이력 | [`../archive/ICRA/`](../archive/ICRA/), [`../archive/ICRA_2027_revision5_20260913/`](../archive/ICRA_2027_revision5_20260913/) 및 이전 revision 폴더 | 버전별 원고·소스·참고문헌·실험 근거 |
| ICRA 제출 파일 묶음 | [시작 안내](../archive/ICRA_2027_submission_20260914/00_START_HERE_KO.md) | PDF·영상·입력 양식·내부 점검 자료 |
| 실물 중심 영상 수정본 | [영상 수정 설명](../archive/ICRA_2027_video_revision2_20260920/README_KO.md) | 원본 프레임 보존 및 편집·검증 기록 |
| 과거 실물·미디어·코드 | [`../archive/videos/`](../archive/videos/), [`../archive/simulation_media/README.md`](../archive/simulation_media/README.md), [`../archive/assets/`](../archive/assets/), [`../archive/papers/`](../archive/papers/), [`../archive/codes/`](../archive/codes/) | 원본과 역사 자료 |
| CAD·계산 자동화 | [`../tools/`](../tools/) | Fusion RPC, housing 생성/검증, 기하·질량·토크 계산 |
| 추가 개발 상태·학습 자료 | [보존 설명](../archive/development_snapshots/20261009/README.md) | 별도 worktree 변경, stash 패치, 학습 checkpoint |
| 최종 파일·검증 기록 | [`archive/2026-10-09/`](archive/2026-10-09/) | 커밋 파일 목록·해시, 소프트웨어 검사 및 보존 범위 |

## 종료 시점의 한계와 미완료 사항

다음은 보관된 파일에 기록된 상태다. 이번 종료 작업에서 실물 시험·Fusion 재검증·논문 접수는 수행하지 않았다.

1. **실물과 시뮬레이션의 구분:** `scone-stair`와 RL 정책의 성능은 시뮬레이션 기록이다.
   실제 로봇 적용에 필요한 상태 추정·안전 계층과 동일 조건 실물 성능은 별도 검증 대상이다.
2. **실험 동결:** 과거 개발 실행과 smoke 결과를 최종 matched-controller 평가로 대체하지 않는다.
   이번 커밋은 보관 리비전이며 새 ICRA 정량 평가를 실행한 리비전이라는 뜻이 아니다.
3. **MARC 제작 상태:** 초기 D0 인수 문서에 간섭·관문 실패와 RELEASE 미완료가 남아 있다.
   후속 housing CAD도 각 파일의 검증 범위만 갖는다. CAD 검사는 출력·강도·피로·실물 운전 검증을 대신하지 않는다.
4. **C1/L1 보강 착수:** `20261001_c1_l1_reinforced/`에는 source·baseline·inspection 기록이 있다.
   이 폴더를 보강 완료 납품본으로 표시하지 않는다.
5. **최신 Fusion 클라우드 원본:** 전원 검토의 `fusion_inventory.json`은 `MARC v4 Body v5 BOX v63`을 조사한 기록이다.
   같은 v63의 완전한 F3D 조립본이 로컬에 있다는 근거는 확인되지 않았다.
   종료 시 Fusion 로컬 RPC 서비스도 연결을 닫아 새 내보내기를 만들지 못했다.
   저장소의 F3D 내보내기와 Fusion 클라우드 문서를 동일한 최신본으로 취급하지 않는다.
6. **P1 전원 PCB:** 2026-10-07 PDF에 회생 에너지 처리, 공차를 포함한 Jetson 과전압 차단,
   최대 부하 전원 용량, ESTOP 단선 동작 및 참조 원본 부족이 남아 있다. 제조 승인 자료가 아니다.
7. **논문과 제출:** ICRA 파일 묶음은 로컬 준비 자료다. 접수·심사·게재 상태를 이번 아카이브로 확정하지 않는다.
   실물 영상은 과거 하드웨어 기록이며 현재 평가 제어기의 동일 조건 검증으로 확대 해석하지 않는다.
8. **외부 장치와 파일:** 원격 학습 서버, 실제 로봇·Jetson 저장장치, Fusion 클라우드,
   저장소 밖의 첨부 문서는 이 Git 저장소의 복사 범위에 포함되지 않는다.

최종 소프트웨어 검사 결과와 실행 환경은 [`archive/2026-10-09/validation.json`](archive/2026-10-09/validation.json)에 기록한다.
실패가 있는 경우 실패 상태와 로그를 보존하고, 아카이브를 위해 판정 기준이나 구현을 바꾸지 않는다.

## 보관 방식과 원본 보존

- 주 작업 트리의 기존 변경 전체와 미추적 코드·설계·논문·영상 자료를 한 번의 최종 커밋에 담는다.
- 최종 소스·자료와 기존 원격 `main`의 PR 병합 이력을 **`main` 하나**에 통합했다.
  기존 원격 `main`의 별도 4개 커밋에는 고유한 코드 변경이 없음을 확인한 뒤 정상 병합했다.
  강제 푸시 없이 `main`을 기본 브랜치로 지정하고 나머지 개발 브랜치를 정리했다.
- SCONE v2는 [`archive/scone_v2/`](../archive/scone_v2/README.md)에 원래 소스·모델·논문·포스터를 보존했다.
  영상은 사용자가 이미 압축한 `archive/videos/`를 연결하고, 대용량 원본과 정리 전 영상이 담긴 bundle은 로컬에만 남겼다.
  이전 보행 실험은 [별도 스냅샷](../archive/experiments/scone_gait_v2/README.md)에 보존했다.
  [브랜치별 보관 안내](../archive/branch_history/README.md)에 원래 커밋, 태그와 bundle 복원 방법이 있다.
  `archive/` 태그는 이전 이력을 가리키며 추가 개발 브랜치는 아니다.
- 큰 CAD·메시·영상·압축파일·체크포인트는 Git LFS로 보관한다.
  `.gitattributes`가 포인터와 실제 원본의 관계를 정의한다. 일반 ZIP 다운로드에 바이너리 원본이 포함되는지 추정하지 않는다.
- 별도 worktree의 미커밋 파일과 stash는 `archive/development_snapshots/20261009/`에 패치와 압축본으로 저장한다.
  주 구현에 합치거나 원래 worktree·stash를 삭제하지 않는다.
  별도 worktree는 같은 커밋의 detached HEAD로 유지하여 파일 변경 없이 브랜치 이름만 정리했다.
- 학습 checkpoint와 평가 파일은 `training/runs.tar.gz` 및 해시 목록으로 보존한다.
  개인 원격 작업 연결 정보와 미완성 `.part`는 로컬 복구본에만 보존한다.
- 가상환경·캐시·개인 실행 연결 정보와 원본 경로가 필요한 임시 기록은 Git 커밋 대상에서 제외한다.
  로컬 복구 묶음은 `archive/project-closeout/20261009/`에 따로 만든다.
- 원본 CAD·실물 영상·논문 수치·학습 정책·제어 코드는 종료 정리를 위해 수정하지 않았다.
  정리는 상태 표시, 자료 색인, 보존 규칙과 복구 자료 생성으로 수행했다.

## GitHub 보관본 복원

Git과 Git LFS를 설치한 환경에서 다음 순서로 복원한다.

```bash
git lfs install
git clone --branch main https://github.com/hayward2007/SCONE.git
cd SCONE
git lfs pull
git lfs fsck
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
python -m unittest discover -s tests -v
```

학습 checkpoint가 필요하면 저장소 루트에서 다음 파일을 풀어 원래 경로를 복원한다.

```bash
tar -xzf archive/development_snapshots/20261009/training/runs.tar.gz
```

실행과 학습 재개 조건은 [운영 문서](07-running-testing-and-operations.md)를 확인한다.
`requirements.txt`는 허용 버전을 정의하고, 종료 시 설치 버전 목록은 검증 기록 폴더에 보관했다.
같은 환경을 재현하려면 운영체제·Python·MuJoCo 버전과 관측 차원도 맞춰야 한다.
macOS의 시뮬레이션 뷰어는 `mjpython SCONE.py`로 실행한다.

GitHub 아카이브 해제 후 변경하거나 복제본에서 새 개발을 시작할 수 있다.
후속 프로젝트에서는 이 종료 기록과 당시 검증 한계를 함께 가져간다.

## 로컬 복구 묶음

`archive/project-closeout/20261009/`의 안내와 검증 기록을 따른다.
최종 파일 스냅샷, 전체 Git refs의 bundle, 변경 전 상태 및 해시를 보관한다.
로컬 자료에는 원격 작업 연결 정보와 절대 경로가 포함될 수 있으므로 이 묶음을 공개 GitHub 파일로 올리지 않았다.
복원은 빈 폴더에서 수행하고 파일별 해시 및 Git bundle 검증을 먼저 확인한다.

브랜치 정리 후의 상태와 추가 복구 bundle은 `archive/project-closeout/20261009-branch-cleanup/`에 별도로 보관했다.
최초 종료 기록과 파일 목록은 당시 커밋의 기록으로 유지하며, 이후 README와 브랜치 정리를 반영한
검증 결과는 [브랜치 정리 기록](archive/2026-10-09-branch-cleanup/README.md)에서 확인한다.
