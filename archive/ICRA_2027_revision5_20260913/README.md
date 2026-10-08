# SCONE ICRA 2027 — Revision 5

2026-09-13. Six-legged SCONE; previous manuscript revisions and the user's existing source changes are preserved.

This revision studies **which control choices can be removed while retaining flat transport and stair ascent**. It uses periodic flat reindexing and a fixed stair posture with one common phase. The modes are evaluated separately; it does not demonstrate autonomous mode selection or an uninterrupted floor–stair–floor trial.

## Outputs

- `output/pdf/SCONE_ICRA2027_Manuscript_EN.pdf`: 6-page anonymous English manuscript, including references and AI disclosure.
- `output/pdf/SCONE_ICRA2027_Review_KO.pdf`: Korean review, revised claims, score, and evidence needed next.
- `media/companion_video.mp4`: labeled archival hardware, previous flat P replay, and new stair replays. It is a sequence of separate demonstrations.
- `paper.tex`, `references.bib`, `REVIEW_KO.md`: editable sources.
- `SOURCE_NOTES.md`: reference mapping and evidence interpretation.

## Results and interpretation

At a 2 ms physics step, C195 and C180 complete 6/6 conditions at each of 100 and 150 mm and 0/6 at 200 mm. The height lookup has the same success set. The matched wheel succeeds in 5/6 and 0/6 at 100 and 150 mm. Its nominal 150 mm outcome changes at 1 ms, so no wheel-impossibility claim is justified. All 30 main-grid 200 mm trials fail the full rear-clearance endpoint. All 102 planned trials remain in the denominator, including the 12 timestep checks.

The 195-degree setting was selected after an exploratory nominal pilot. Neither the pilot nor visual replays count as additional evidence. The grid was frozen before the full run, but it is not an independent hardware or population validation. The internal score is 71/100 (previously 68), **not a 71% acceptance probability**.

## Reproduce

The stair snapshot is `14d677ab07969898ee3bee69214a30fac16be6f6` and the old flat snapshot is `49e6851abd4864c7d4c5606a3c87d70b53780e18`. The self-contained stair Git bundle contains both. All 102 original stair records show the stair revision and `git_dirty=false`.

From a fresh clone of `evidence/stair_source.bundle`, run:

```sh
python3 -m benchmark.stair_simplification --output /absolute/path/outside-checkout --workers 4
```

Use Python 3.12, MuJoCo 3.10.0, NumPy, SciPy, and the repository dependencies. The exact model fingerprint and job design are in `evidence/stairs/protocol.json`. The source tarball provides a second code snapshot; run from a Git checkout to retain provenance. `evidence/pilot.json` and `evidence/strict-pilot/` are development records only.

The retained `evidence/joint_limit/` directory contains the 176-trial flat design and measurements used in the manuscript. Do not pool those trials with the stairs: their preparation, horizon, and endpoints differ. The two new root source files are `benchmark/stair_simplification.py` and `tests/test_stair_simplification.py`; they add an opt-in simulation experiment without modifying physical controllers.

Rebuild the documents with `python3 build_packet.py` from this packet directory. Tectonic, macOS Times New Roman/Arial, and the bundled ReportLab runtime are used; identical layout requires equivalent fonts. `analyze_stairs.py` derives the new tables and charts from all records. To regenerate the three new visual replays, invoke `mjpython` on the absolute path to `evidence/render_stairs.py` **with the clean stair snapshot as the working directory**, then run `assemble_media.py`. Renderer camera edits change presentation only and are checked against the frozen numeric outputs.

## Verification and remaining work

- 287 tests passed in the root workspace, including four new protocol checks (`evidence/tests.log`).
- Three independent stair replays match the recorded numeric endpoints and work.
- PDFs are rendered page by page; all fonts are embedded; no paper margin or template font-size changes are used.
- Current evidence does not identify loaded actuators, compliant contact, non-tire collision clearance, physical multi-turn feasibility, or end-to-end mode transitions.
- The next decisive experiments are identical-start-state timestep convergence, support-aware rear-bank transfer at 200 mm, and matched physical trials.

The official ICRA 2027 page was checked on 2026-09-13. The manuscript deadline is listed as September 15, 2026, 23:59 PST. The video window reopens September 17–22. No submission or external communication was performed.
