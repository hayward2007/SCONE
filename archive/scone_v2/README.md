# SCONE v2 — original hardware control system

`SCONEv2_old` 브랜치의 최종 상태를 원래 파일 내용 그대로 보존한 폴더다.
기준 커밋: `86c3a0fa789a1975f3c7f5229c0613187a04eeb9`.
전체 Git 이력은 [`archive/scone_v2` 태그](https://github.com/hayward2007/SCONE/tree/archive/scone_v2)에 남겼다.

This is the original SCONE v2 hardware-control system, preserved before the later
simulation and reinforcement-learning refactors. The source, papers, poster and
models retain their original bytes. Videos link to the existing compressed files;
the tag preserves the original history without uploading another video copy.

| 자료 / Material | 위치 / Location |
| --- | --- |
| 원래 진입점 / Original entry point | [main.py](main.py) |
| 로봇 API / Robot API | [src/SCONE.py](src/SCONE.py) |
| 액추에이터·통신 / Actuators and communication | [src/core/](src/core/) |
| 보행·굴림·계단 / Walking, driving and climbing | [src/provider/](src/provider/) |
| 포스터 / Poster | [Original poster](docs/Eco-friendly%20deliver%20SCONE%20Poster.jpg) |
| 논문 / Paper | [Original paper](docs/Eco-friendly%20deliver%20SCONE%20Paper.pdf) |
| 압축한 실물 영상 / Compressed hardware footage | [SCONE v1](../videos/SCONEv1.mp4) · [SCONE v2](../videos/SCONEv2.mp4) |
| 원래 README / Original README | [README.original.md](README.original.md) |
| 파일별 원본 해시 / Original file hashes | [source_manifest.json](source_manifest.json) |

원래 소스·문서 경로를 유지했다. macOS의 `docs/.DS_Store`와 압축 전 원본 영상 두 개는 스냅샷에서 제외했다.
실물 영상은 이미 압축해 보관한 `archive/videos/`를 사용하며, 원래 이력은 기존 Git 커밋과 로컬 복구본에 남아 있다.
원래 README는 `README.original.md`로 보존했다. 대용량 원본은 복제 후 `git lfs pull`로 받는다.

이 폴더는 역사 자료이며 현재 저장소 루트의 실행 환경을 사용하는 진입점이 아니다.
당시 전체 저장소 구조로 작업하려면 별도 작업 폴더에서 태그를 체크아웃한다.

```bash
git worktree add --detach ../SCONE-v2 archive/scone_v2
```

[최종 프로젝트 및 복원 안내](../../docs/31-project-archive.md)
