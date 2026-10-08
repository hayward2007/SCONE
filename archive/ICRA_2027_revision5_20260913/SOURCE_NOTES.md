# Reference and claim audit

Checked 2026-09-13; inherited bibliographic fields were verified in the preserved earlier packets. Fifteen sources are actually cited in the new manuscript; unused BibTeX entries remain available but do not appear in the bibliography.

- [ICRA 2027 call](https://2027.ieee-icra.org/contribute/call-for-icra-2027-papers-now-accepting-submissions/): 8-page inclusive limit, double-column PDF, double anonymity, video rules, and AI disclosure. The page prints the deadline timezone as PST; no silent substitution with PDT was made. Its reviewer-irrelevant FAQ contains legacy year fragments, so the explicit submission instructions are the basis used here.
- [Q-Whex, Journal of Field Robotics 2023](https://onlinelibrary.wiley.com/doi/10.1002/rob.22186): publisher full text, especially Introduction and Section 4.4. Six actuators, quasi-wheel geometry, and phase-based physical stair trials make this a close prior art reference. SCONE cannot claim novelty from curved legs, simple phases, or stair ascent alone. We do not compare cross-platform success percentages or claim lower mechanical complexity.
- [Mane and Hubicki, ICRA 2024](https://doi.org/10.1109/ICRA57147.2024.10610524): explicit rolling-contact assumptions provide a useful model for writing the scope of a mechanics claim. The SCONE study does not implement that paper's curved-terrain dynamics or claim its guarantees.
- [Weng et al., ICRA 2024](https://doi.org/10.1109/ICRA57147.2024.10610064): controlled experimental conditions motivate paired comparisons. This does not make our deterministic grid a standardized reliability test.
- [Quattroped](https://doi.org/10.1109/TMECH.2013.2253615), [TurboQuad](https://doi.org/10.1109/TRO.2017.2696022), and [ASTERISK H](https://doi.org/10.20965/jrm.2008.p0403): shared actuation, transformable morphology, and hybrid motion are established. SCONE uses a fixed open arc; its two modes are not evidence of a novel autonomous transition.
- [Keep Rollin'](https://doi.org/10.1109/LRA.2019.2899750) and [whole-body MPC](https://doi.org/10.1109/IROS51168.2021.9636371): broader dynamics-aware control context. No cross-robot performance superiority or lower computational runtime was measured.
- [CoACD](https://doi.org/10.1145/3528223.3530103) and [MuJoCo collision documentation](https://mujoco.readthedocs.io/en/stable/computation/index.html): motivate the decomposed contact representation. Cavity probes and support-function checks are repository-derived measurements, not claims copied from those publications.
- ROBOTIS e-manual references give **12 V no-load speeds**, not calibrated loaded-joint constraints. The flat target governor enforces command/profile limits only.

## Corrections anchored in source and experiment

1. The historical `adaptive` stair controller selects brace/phase/rate from a known maximum riser before moving. It does not update these choices from online terrain or contact sensing.
2. Stair targets are unwrapped extended positions. The flat mode's bounded reindexing rationale must not be applied to every mode merely because the terminal material is an arc.
3. The single-wheel horizontal-axle-push equation is not a multibody powered-wheel impossibility theorem. The wheel's 1 ms ascent explicitly rules out that interpretation here.
4. The current collision comparison changes the entire arc/cylinder contact geometry. It does not isolate an inner-hook mechanism; identify actual contact locations before making that claim.
5. `prepare_side_on()` plus two 4 s acquisition windows costs about 31 s. Both ascent-only and preparation-inclusive times are retained.
6. The old body-height endpoint could accept partial 200 mm ascent. The new criterion requires rear-bank clearance plus support and stopping, so those partial ascents no longer count as task success.
7. The project theory documents and old videos remain historical records; this packet's prospective protocol and raw records define the new manuscript numbers.

## Numerical evidence

- Stair raw records: `evidence/stairs/trials.jsonl`; every planned ID 0–101 occurs once.
- Nominal sensitivity flips: W195, 150 mm, tread 350 mm, pose 0; failure at 2/4 ms, success at 1 ms. The preparation sequence is also reintegrated, so use the phrase end-to-end numerical sensitivity.
- Geometry/posture comparisons use identical commands and body inertias; geometry-dependent acquired positions and contact states are not forced equal.
- C195 selection used earlier exploratory trials. Do not call the full grid independent, preregistered externally, or a hardware reliability sample.
- Visual replay records are excluded from counts and match numeric outputs. Camera framing affects only rendered images.
