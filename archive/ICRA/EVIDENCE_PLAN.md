# Evidence plan

## Claim-to-evidence matrix

| Paper claim | Minimum evidence | Current source to locate | Status |
| --- | --- | --- | --- |
| Untethered operation | continuous battery-only run; battery log | battery demo video | pending audit |
| 2.0 m/s quasi-rolling | measured distance, frame timestamps, repeated runs | raw speed video/data | pending audit |
| 0.7 m/s walking | identical protocol to rolling | raw walking video/data | pending audit |
| 15-step ascent | uncut full ascent and stair dimensions | stair video | pending audit |
| 23 s per step | explicit start/end rule and repeated timings | timing notes/raw video | pending audit |
| Irregular terrain | defined obstacle dimensions and trial outcomes | outdoor videos | pending audit |
| Mode transition | uncut walk-roll-climb-walk sequence | SCONEv2 video | available; quantify |
| Morphology benefit | same-hardware A/B/C restrictions | new or archived trials | needs protocol |

## Data fields for every trial

- trial ID, date, robot configuration, controller commit;
- battery voltage before/after, payload, total mass;
- surface or stair riser/tread dimensions;
- start and finish timestamps;
- success/failure and failure category;
- distance, elapsed time, voltage/current log, body attitude if available;
- raw video filename with no speed change or cut during the measured interval.

## Historical inconsistency to resolve

The archived documents report multiple v1 speed pairs: 0.05/0.7 m/s,
0.5/0.7 m/s, and 0.07/0.5 m/s for walking/driving. Do not cite a v1 speed until
the original measurement is reconstructed or repeated. The v1--v2 comparison is
supporting engineering history, not the main causal experiment.
