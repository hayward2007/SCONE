# 2026-10-09 최종 보관 검증 기록

[프로젝트 종료 문서](../../31-project-archive.md)에 전체 자료 색인과 복원 방법이 있다.

- [validation.json](validation.json): 종료 시 실행한 검사와 실제 미실행 항목.
- [tests.log](tests.log): `python3 -m unittest discover -s tests -v`의 원본 출력. **287 tests / OK**.
- [environment.json](environment.json): Python·운영체제·설치 패키지 버전. 의존성 lockfile은 아니다.
- [files.json](files.json): 최종 커밋 대상의 실제 파일 크기·SHA-256·Git blob·LFS OID.
  자기 참조를 피하기 위해 파일 목록 자체와 최종 검증 JSON은 해시 대상에서 제외했다.

Git LFS 포인터의 OID·크기는 실제 바이너리와 대조했다. 과거에 보관된 검증 결과를
이번에 재실행한 결과와 합치지 않는다. 하드웨어 시험, Fusion 형상 재검증, 새 ICRA 평가와
논문 접수 상태 확인은 이번 종료 작업의 실행 항목이 아니다.

최종 커밋·원격 확인·아카이브 후 상태·로컬 복구 묶음의 검증 기록은
Git 커밋 밖의 `archive/project-closeout/20261009/`에 보관한다.
