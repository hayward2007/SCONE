# Branch history archive — 2026-10-09

최종 개발 브랜치는 **`main` 하나**다. 이전 시스템과 실험은 폴더·태그·Git bundle로 보존했다.
원래 브랜치 이름과 커밋의 전체 대응은 [branches.json](branches.json)에 기록했다.

The final development branch is `main`. Historical systems and experiments remain
available through source snapshots, annotated tags and local recovery bundles. No force push
was needed to consolidate the published `main` history.

| 이전 브랜치 / Previous branch | 보관 방법 / Preservation |
| --- | --- |
| `feature/scone-gait-v2` | 최종 `main`의 기존 소스·자료 및 커밋 이력 / Final source, materials and commit history in `main` |
| 원격 `main` / Published `main` | PR 병합 기록을 `main`에 병합; `archive/main_before_cleanup` 태그 / Merged history and archive tag |
| `rl` | `main`에 이미 포함; `archive/rl` 태그 / Already included in `main`, plus archive tag |
| `SCONEv2_old`, 이전 `develop` | [원래 시스템](../scone_v2/README.md), `archive/scone_v2` 태그 / Original system and archive tag |
| `scone-gait-v2` | [실험 파일](../experiments/scone_gait_v2/README.md), `archive/role_split_gait` 태그 / Experimental snapshot and archive tag |
| `dynamixel-mujoco` | [최종 패키지](../../packages/dynamixel-mujoco/README.md), `archive/dynamixel_mujoco` 태그 / Byte-identical package and standalone history tag |
| `backup/pre-media-cleanup-20260830` | 로컬 복구 bundle; 압축 전 영상은 추가 업로드하지 않음 / Local recovery bundle; original videos are not uploaded again |
| `worktree-wf_619614a6-1b7-5`, `worktree-wf_619614a6-1b7-6` | [미커밋 변경 보존본](../development_snapshots/20261009/README.md); 실제 작업 폴더는 같은 커밋의 detached HEAD로 유지 / Saved uncommitted changes and preserved detached worktrees |

`backup/pre-media-cleanup-20260830`에는 GitHub의 일반 Git 파일 크기 한도를 넘는
원본 영상이 있다. 따라서 그 브랜치만의 2개 커밋과 객체는 **로컬 전용 bundle**로 보존했다.
위치: `archive/project-closeout/20261009-branch-cleanup/pre-media-cleanup-20260830.bundle`.
이 파일은 GitHub에 올리지 않는다. 공개 저장소의 영상은 사용자가 이미 압축한 파일을 유지한다.
bundle의 선행 커밋 `444f7442195c3eaaeb17f68da881d20cf121c7e7`은 최종 `main` 이력에 포함되어 있다.

The pre-cleanup backup contains an original video exceeding GitHub's regular Git
blob limit. Its two unique commits and their objects are preserved in a local-only bundle,
with the prerequisite commit retained in `main`.
The bundle is excluded from GitHub uploads; the existing compressed videos are retained.

로컬 bundle을 가진 복제본에서 복원할 때는 다음 명령을 사용한다. 보관된 원래 커밋은
`f2801d24ad865311ad5041c3312cca4649c9a441`이다.

```bash
git bundle verify archive/project-closeout/20261009-branch-cleanup/pre-media-cleanup-20260830.bundle
git fetch archive/project-closeout/20261009-branch-cleanup/pre-media-cleanup-20260830.bundle \
  refs/heads/backup/pre-media-cleanup-20260830:refs/tags/restored/pre-media-cleanup-20260830
git worktree add --detach ../SCONE-pre-media-cleanup restored/pre-media-cleanup-20260830
```

`archive/`로 시작하는 이름은 **태그**이며 추가 개발 브랜치가 아니다.
태그를 확인하려면 `git tag --list 'archive/*'`를 사용한다.

[전체 종료 및 복원 안내](../../docs/31-project-archive.md)
