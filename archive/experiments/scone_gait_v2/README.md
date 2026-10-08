# Role-split gait experiment

기존 `scone-gait-v2` 브랜치의 9개 실험 커밋에서 변경한 **13개 파일**을 원본 그대로 보존했다.
기준 커밋: `ad0fdae54db814c0ebb071f17b95997ccc72236b`.
전체 소스와 이력은 [`archive/role_split_gait` 태그](https://github.com/hayward2007/SCONE/tree/archive/role_split_gait)에 있다.

This snapshot preserves the files changed by the nine role-split gait experiments,
including rear/front pair rolling and their walking controls. It is a partial source
snapshot; the tag provides the complete checkout. The final implementation in `main`
contains later contact-geometry and rewind-budget revisions.

- [실험 설명](docs/28-scone-gait-v2-role-split-rolling.md)
- [두 다리 구동 분석](docs/29-two-leg-sector-drive.md)
- [실험 제어기](benchmark/controllers.py)
- [보행 구현](src/locomotion/scone_gait_v2.py)
- [파일 목록과 해시](source_manifest.json)

이전 실험 API를 최신 구현에 덮어쓰지 않고 별도 보관했다. 당시 시스템을 실행하려면 전체 태그를 복원한다.

```bash
git worktree add --detach ../SCONE-gait-experiment archive/role_split_gait
```
