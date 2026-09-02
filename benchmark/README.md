# SCONE simulation benchmarks

This package runs paper-oriented, headless MuJoCo experiments without changing
the physical controller or the existing simulation routes. Every trial creates
a fresh model, data, virtual controller, and robot, then writes one JSON object.

The benchmark is evidence about the current MuJoCo model. It is not a substitute
for repeated physical-robot tests or simulation-to-real calibration.

## Locked ICRA protocol

Use the `icra` suite for publication-facing runs. It is intentionally separate
from the historical A/B/C commands below.

```bash
# Fast end-to-end validation of both contact geometries and all output files.
python -m benchmark icra \
  --profile smoke \
  --suite all \
  --seed 2027 \
  --output-dir benchmark/results/icra/smoke-seed-2027

# Small tuning/debug set. Do not report it as the held-out evaluation.
python -m benchmark icra --profile pilot --suite all --seed 12027

# Locked publication set: 20 paired trials, a nine-command grid and all stairs.
python -m benchmark icra --profile evaluation --suite all --seed 22027
```

The protocol crosses two contact geometries with matched controllers:

| Factor | Levels |
| --- | --- |
| Contact | exported `open-arc`; same-radius/width `closed-wheel` cylinder |
| Flat control | `matched-articulated`; `matched-distal-only`; `matched-coordinated` |
| Stair control | `distal-only`; `full-scone` |

The closed cylinder replaces collision geometry only. The explicit body mass,
inertia, actuator, friction, contact solver parameters, and outer envelope are
preserved. All matched flat controllers use the same `RollGaitConfig`, actuator
limits, 60-degree tripod-B distal phase pose, and gait-phase calibration. For a
given `pair_id`, every geometry/controller condition receives the same sampled
mass, friction, actuator-strength, initial-pose and gait-phase perturbation plus
the same terrain seed.

Each output directory starts with `protocol.json` and refuses to overwrite an
existing manifest. Records include the model-plus-mesh SHA-256, MuJoCo version,
Git revision/dirty flag, solver settings, physics/control time steps, geometry,
pair ID and every sampled perturbation. The suite writes raw JSONL, ordinary
group summaries, and candidate-minus-reference paired-difference CSV files.
Binary outcomes use Wilson intervals; continuous means and paired differences
use deterministic percentile-bootstrap intervals.

`sensitivity` reruns a fixed nominal condition at 1, 2 and 4 ms physics steps.
This checks numerical convergence separately from controller/morphology effects.
The mass/friction/strength ranges are sensitivity ranges, not measured hardware
uncertainty distributions. Before submission they must be replaced or justified
using assembled-mass, motor and TPU/contact identification data.

## Suites

### Flat A/B/C ablation

The main same-model comparison is:

| Name | Proximal stages | Distal C-frames |
| --- | --- | --- |
| `articulated-walk` | alternating-tripod IK | bounded position motion |
| `distal-only-roll` | held at the initialized pose | continuous rotation |
| `full-roll` | SCONE stabilizing gait | continuous rotation plus bounded lower-rate term |

Run one quick comparison:

```bash
python -m benchmark flat \
  --all \
  --command forward \
  --trials 1 \
  --duration 8 \
  --output benchmark/results/flat-nominal.jsonl
```

Run a command grid with ten randomized initial gait phases:

```bash
python -m benchmark flat \
  --all \
  --command idle \
  --command forward \
  --command reverse \
  --command left \
  --command right \
  --command yaw-left \
  --command yaw-right \
  --command forward-turn \
  --trials 10 \
  --duration 8 \
  --output benchmark/results/flat-grid.jsonl
```

`bounded-scone` is available as an engineering comparison, but it is not one of
the paper's primary A/B/C conditions. It is the model-based bounded reference,
not PPO.

These conditions compare the controllers currently implemented in SCONE on the
same MJCF. Their motion profiles and gains are not yet normalized to an equal
actuation budget, so report them as an as-implemented controller ablation. A
paper claim about morphology alone additionally requires a torque-, speed-, and
control-bandwidth-matched ablation.

### Stair A/B/C and geometry sweep

Paper-facing names are mapped onto the existing deterministic stair engine:

| Name | Existing implementation |
| --- | --- |
| `distal-only` | `pure-rolling` |
| `synchronized-open-loop` | fixed 270-degree leading brace and open-loop distal velocity |
| `full-scone` | rise-conditioned partial brace and closed-loop shared phase |

Run all three preset heights:

```bash
python -m benchmark stairs \
  --all \
  --trials 1 \
  --output benchmark/results/stairs-nominal.jsonl
```

Run a custom riser/tread Cartesian sweep:

```bash
python -m benchmark stairs \
  --all \
  --riser-mm 80 --riser-mm 100 --riser-mm 120 --riser-mm 150 --riser-mm 200 \
  --tread-mm 200 --tread-mm 270 --tread-mm 350 \
  --trials 1 \
  --output benchmark/results/stairs-geometry.jsonl
```

Repeated nominal stair runs are exactly deterministic. Use `--randomize` to
sample mass, friction, and actuator-strength scales when estimating a simulated
success rate:

```bash
python -m benchmark stairs --all --preset stairs-3 --trials 20 --randomize
```

### Robustness Monte Carlo

The robustness suite samples mass, sliding friction, actuator strength, initial
position, yaw, gait phase, and procedural terrain seed:

```bash
python -m benchmark robustness \
  --all \
  --terrain uneven \
  --trials 100 \
  --output benchmark/results/robustness.jsonl
```

The default ranges are mass `0.9..1.1`, friction `0.4..1.2`, and actuator
strength `0.85..1.15` relative to the current MJCF.

### Mode transitions

This suite records the whole neutralization, mode change, pose acquisition, and
recovery window. For `roll-to-walk`, `mode_switch_duration_s` ends only after all
18 joints reacquire the Standard walk pose within 96 raw units.

```bash
python -m benchmark transitions \
  --all \
  --trials 20 \
  --output benchmark/results/transitions.jsonl
```

### Statistical summary

Create a CSV grouped by benchmark, controller, and command. Continuous metrics
include mean, sample standard deviation, and deterministic percentile-bootstrap
95% intervals. Binary success uses a Wilson 95% interval.

```bash
python -m benchmark report \
  benchmark/results/flat-grid.jsonl \
  --output benchmark/results/flat-grid-summary.csv
```

### Compact simulation media

Capture the paper-facing flat A/B/C, 200 mm stair A/B/C, Walk/Roll
transitions, and one reproducible uneven-terrain sensitivity example. Use
`mjpython` on macOS so MuJoCo owns the graphics context on the main thread.

```bash
mjpython -m benchmark capture --suite all
```

The default output is `archive/simulation_media/`: 640x360, 15 fps H.264 MP4
at CRF 34 plus one progressive JPEG per scene. Frames are streamed directly to
FFmpeg, so no uncompressed source video is written. `manifest.json` records the
scene parameters, outcomes, frame counts, durations, and file sizes.

## Recorded flat/transition metrics

- body-frame displacement, average `vx/vy`, yaw change and tracking RMSE;
- root-height variation, roll/pitch RMS and extrema, minimum upright value;
- absolute mechanical work `integral(abs(tau*qdot)) dt` and mechanical COT;
- estimated absolute terminal electrical energy `integral(abs(V*I)) dt`;
- peak actuator torque, estimated motor current, and contact force;
- tire-ground tangential slip distance and mean loaded contact count;
- non-tire ground-collision steps;
- IK failures, stride clipping, and minimum IK backoff scale;
- Git revision and dirty-worktree flag.

Electrical energy is reconstructed from the simulated dcmotor terminal voltage,
back-EMF, and resistance. It is an estimate, not a measured battery-energy
result. Stair `work_to_top_j` is also simulated absolute mechanical work.
The loaded contact count is the number of distinct tire bodies carrying at
least 1 N normal contact force, not MuJoCo's raw number of contact points.

## Experimental rules

1. Freeze a Git revision before producing paper numbers.
2. Do not combine results from different MJCF/controller revisions.
3. Tune on one set of terrain seeds and report a disjoint evaluation set.
4. Do not treat identical deterministic repeats as independent trials.
5. Report traversal-only and end-to-end stair timing separately when preparation
   time is part of the research question.
6. Never mix legacy A/B/C records with `icra` matched-controller records.
7. Run the pilot and held-out evaluation with different pre-registered seeds;
   do not retune after inspecting evaluation outcomes.
8. Treat the open-vs-closed result as a frozen-controller morphology effect.
   If either geometry is retuned, report that as a separate tuned comparison.
9. The current MJCF has no mechanical joint limits and its TPU/friction model is
   not identified from hardware; disclose this until calibration is complete.

Generated result files under `benchmark/results/` are ignored by Git.
