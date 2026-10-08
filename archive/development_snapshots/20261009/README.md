# 2026-10-09 개발 상태 보존

주 작업 트리 이외의 미커밋 개발 코드와 학습 산출물이다. 현재 `src/` 구현에 합치지 않고 원래 상태를 보존했다.

- `wf_*/metadata.json`: 원래 작업 트리 위치, 기준 커밋, 변경 목록.
- `wf_*/tracked_changes.patch`: 기준 커밋에 적용할 추적 파일 변경.
- `wf_*/untracked_files.tar.gz`: 해당 작업 트리의 미추적 소스·모델·메시.
- `stash_0.patch`: 기존 stash의 변경 내용. 원래 stash도 로컬 Git 보관본에 남아 있다.
- `training/runs.tar.gz`: 학습 체크포인트·평가 기록. 저장소 루트에서 풀면 원래 `runs/` 경로를 복원한다.
- `training/inventory.json`: 학습 파일 목록·SHA-256. 원격 작업 연결 정보와 미완성 `.part` 다운로드는 로컬 보관본에만 남겼다.

복원 시 Git LFS 파일을 먼저 내려받는다. 체크포인트는 학습기별 관측 차원이 다르므로 [학습·검증 문서](../../../docs/07-running-testing-and-operations.md)를 확인한다.
