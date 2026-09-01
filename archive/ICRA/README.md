# SCONE ICRA manuscript

This directory is a local ICRA 2027 working manuscript built from PaperCept's
official `ieeeconf.cls` and `IEEEtran.bst` files.

## Build

```bash
cd archive/ICRA
make          # builds both manuscripts
make check    # page count, page size and embedded fonts
```

Two manuscripts are kept in step from one set of numbers:

| Source | Sections | Output |
| --- | --- | --- |
| `root.tex` | `sections/` | `output/SCONE_ICRA_English.pdf` |
| `root_ko.tex` | `sections_ko/` | `output/SCONE_ICRA_Korean.pdf` |

`build.sh` selects the compiler. It prefers Tectonic when it is on `PATH`
(hermetic, fetches missing packages itself) and otherwise falls back to a local
`xelatex` + `bibtex` chain so the manuscripts also build offline. Both documents
therefore avoid packages that only one of those toolchains carries:

- `\boldsymbol` from `amsmath` instead of `bm`.
- `root_ko.tex` uses `kotex` when the distribution provides it and otherwise
  falls back to `xeCJK` with the system Noto CJK KR faces (`CJKspace=true`,
  because Korean is space-delimited).
- `ieeeconf.cls` sets `\rmdefault` to `ptm`, which XeLaTeX cannot resolve in its
  Unicode encoding; both roots load `fontspec` with TeX Gyre Termes, the
  metric-compatible Times clone, so bold and italic runs are not silently lost.

Figures are generated from the benchmark JSONL files by
`figures/generate_figures.py`; `make` reruns it whenever those results change.
`stair_results.pdf` and `arc_geometry.pdf` are authored at column width because
they are included at `\columnwidth`, and `flat_metrics.pdf` /
`control_architecture.pdf` are authored at text width because they sit in
`figure*` floats. `arc_geometry.pdf` is pure geometry derived from the symbols in
the text, so it cannot drift away from the equations it illustrates.

## ICRA 2027 submission constraints

- US Letter, 10 pt, IEEE two-column format.
- Eight pages total, including figures, tables, acknowledgments, and references.
- Initial submission is double-anonymous.
- Remove names, affiliations, lab names, logos, faces, identifying PDF metadata,
  and identity-revealing external links from the manuscript and video.
- A unique robot may appear visually, but its individual name should be hidden in
  the review version under the IEEE RAS double-anonymous rules. The `\robotname`
  macro handles this in the text.
- Accompanying video: at most 180 s and 20 MB; MPEG/MP4/MPG; minimum 480 px
  height, 20 fps, progressive scan.
- No `[TODO: ...]` marker may remain in a submitted build.

## Page budget

| Content | Target pages |
| --- | ---: |
| Title, abstract, introduction, teaser | 1.0 |
| Related work and problem formulation | 0.8 |
| Robot and arc-leg design | 1.2 |
| Locomotion and mode transitions | 0.9 |
| Experimental methodology | 1.0 |
| Results and ablations | 1.6 |
| Discussion and conclusion | 0.5 |
| References | 1.0 |
| **Total** | **8.0** |

Current build: English 8 pages, Korean 7 pages, zero overfull boxes, zero
undefined references, and the two languages contain an identical multiset of
numeric literals (checked by extracting `\d+\.\d+` from both PDFs).

This is a budget, not a reason to pad the paper. Results and figures take priority
over application background, implementation narration, and future work.

## Working rules

1. Do not insert a number until its raw video/log, units, protocol, and trial count
   are known.
2. Keep v1--v2 history as design motivation; use same-hardware restrictions for
   causal ablation.
3. Build all result plots from raw data with scripts rather than editing chart
   values manually.
4. Keep the paper self-contained. Reviewers are not required to visit GitHub or
   other external links.
5. Sec. IV must describe the controller that produced the numbers in Sec. VI.
   `full-roll` is `RollGait`; `bounded-scone` (`SconeGait`) and the PPO speed
   blend are design/deployment paths that no reported trial exercises, and the
   text says so explicitly. If the benchmark controller changes, Sec. IV changes
   with it.
6. Bibliographic fields added on 2026-09-01 carry verified authors, titles,
   venues and years; volume and page fields still need one publisher-record pass
   before submission.
6. Follow the ICRA AI-use policy for the final wording and disclosure. Formatting
   and grammar assistance are treated differently from generated technical
   content; retain an internal record of how assistance was used.
