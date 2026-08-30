# SCONE ICRA manuscript

This directory is a local ICRA 2027 working manuscript built from PaperCept's
official `ieeeconf.cls` and `IEEEtran.bst` files.

## Build

```bash
cd archive/ICRA
make
make check
```

The output PDF is `build/root.pdf`. The installed local compiler is Tectonic.

## ICRA 2027 submission constraints

- US Letter, 10 pt, IEEE two-column format.
- Eight pages total, including figures, tables, acknowledgments, and references.
- Initial submission is double-anonymous.
- Keep `\icraanonymoustrue` in `root.tex` for review.
- Remove names, affiliations, lab names, logos, faces, identifying PDF metadata,
  and identity-revealing external links from the manuscript and video.
- A unique robot may appear visually, but its individual name should be hidden in
  the review version under the IEEE RAS double-anonymous rules. The `\robotname`
  macro handles this in the text.
- Accompanying video: at most 180 s and 20 MB; MPEG/MP4/MPG; minimum 480 px
  height, 20 fps, progressive scan.
- Remove every visible `[TODO: ...]` by setting `\icradraftfalse` only after each
  item has been resolved.

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
5. The current text is a structural draft. Every technical sentence must be
   checked by the authors against hardware, data, and primary literature.
6. Follow the ICRA AI-use policy for the final wording and disclosure. Formatting
   and grammar assistance are treated differently from generated technical
   content; retain an internal record of how assistance was used.
