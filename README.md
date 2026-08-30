# SCONE

SCONE is a six-legged robot project with one high-level control API and two
interchangeable backends: physical DYNAMIXEL hardware and MuJoCo simulation.

## Quick start

Run the launcher on macOS with `mjpython` so the MuJoCo viewer can own the main
thread:

```bash
python -m pip install -r requirements-rl.txt
mjpython SCONE.py
```

The launcher searches for a physical controller without changing torque or
position, then displays:

```text
? SCONE 실행 메뉴
❯ 시뮬레이션 조종
  하드웨어 조종 (/dev/...) or 하드웨어 조종 (현재 불가)
  하드웨어 다시 탐색
  강화학습 관리
  종료
```

Use the arrow keys and Enter to choose launcher, profile, terrain, and RL
options. Press Ctrl-C to leave a selection menu safely.

The detailed Korean documentation starts at [`docs/README.md`](docs/README.md),
and the complete learning/checkpoint runbook is in
[`docs/07-running-testing-and-operations.md`](docs/07-running-testing-and-operations.md).

After choosing simulation control, select one locomotion implementation:

- `Legacy mode control`: adapts terminal input to the original blocking
  Walk/Drive/Climb state machine. Press `R` to cycle modes. Walk uses W/S and
  yaw; Drive/Climb use A/D for their left/right motion.
- `Non-RL control`: sends all three axes to the model-based `NonRLWalk` gait.
- `RL control`: asks for a local `runs/**/*.zip` PPO checkpoint, standing pose,
  and residual reference. It runs PPO Walk at 50 Hz and also accepts `R` to
  cycle through legacy Drive/Climb without replacing the shared controller.

Simulation opens a self-centering velocity joystick in the terminal:

```text
W/S       joystick Y (forward/backward)
A/D       joystick X (left/right strafe)
Left/Right arrow  yaw
Space     immediate neutral
R         Walk -> Drive -> Climb -> Walk (Legacy/RL control)
H         return to home pose (Legacy control)
Q         return to the launcher
```

The dashboard displays normalized `x`, `y`, and `yaw` values together with the
scaled body-frame `vx`, `vy`, and `yaw_rate` command. A held key is maintained by
normal keyboard repeat; shortly after release its axis automatically returns to
zero. Multiple live axes are combined, so translation and yaw can be commanded
together.

The MuJoCo window has no SCONE-specific key callback. All joystick keys are read
by the terminal, so robot commands do not alter viewer rendering controls. The
physical-hardware launcher retains the proven discrete W/A/S/D command surface;
the continuous gait is not enabled on real hardware automatically.

## Python API

`import SCONE` is the stable public entry point. Object construction does not
open a serial port and does not start an interactive CLI.

```python
import SCONE
from src.hardware import Controller, discover_hardware

probe = discover_hardware()
if probe.available:
    with SCONE.SCONE(Controller(probe.device_name), profile="sport") as robot:
        robot.forward()
        robot.left()
        robot.change_mode()
```

See `example.py` for a runnable hardware example. A custom backend can be used
by implementing `src.hardware.ControllerProtocol` and passing it to
`SCONE.SCONE`.

## Command flow

```text
terminal key
  -> src.cli normalized x/y/yaw joystick
       -> LegacyVelocityAdapter -> src.main.SCONE old motions
       -> NonRLWalk -> ControllerProtocol -> MuJoCoController
       -> PPO policy -> SconeWalkEnv residual action -> MuJoCoController
```

The terminal owns the only keyboard map. Each locomotion implementation consumes
the same body command `[vx, vy, yaw_rate]`; the MuJoCo viewer has no robot key
callback of its own.

## Folder responsibilities

```text
SCONE.py                         stable `import SCONE` facade and CLI launcher
example.py                       minimal API usage example
src/main.py                      high-level robot lifecycle and command API
src/cli.py                       the only interactive key/launcher interpreter
src/hardware/
  actuator_index.py              physical IDs and leg/tripod groups
  actuator_control_table.py      register address + byte width per motor model
  actuator.py                    ID-to-model catalogue and shared constants
  interface.py                   ControllerProtocol backend contract
  controller.py                  physical DYNAMIXEL transport only
  discovery.py                   non-mutating serial/DYNAMIXEL probe
src/locomotion/
  profile.py                     Standard/Sport posture and speed values
  walk.py, drive.py, climb.py    backend-independent motion sequences
  legacy_velocity.py             x/y/yaw adapter for blocking old motions
  non_rl_walk.py                 Phoenix-style velocity gait + model-based IK
src/kinematics/
  leg.py                         model-derived FK/Jacobian/numerical IK per leg
  robot.py                       six-leg FK/IK and actuator-order conversion
  types.py                       joint-angle, end-pose, and IK result types
src/simulation/
  core/
    model.py                     MJCF loading, terrain injection, base setup
    controller.py                virtual DYNAMIXEL implementation
    pid.py                       voltage-input DC motor position loop
    cli_bridge.py                one viewer + common terminal CLI integration
    simulator_cli.py             direct simulation entry point and terrain menu
  terrain/
    types.py                     terrain names and validated parameter types
    presets.py                   explicit stair/slope difficulty dimensions
    generator.py                 rough/stair/ramp/mixed MJCF algorithms
  controller.py, model.py, ...   backward-compatible import shims only
src/rl/
  inquiry.py                     InquirerPy local/SSH training launcher
  joystick_control.py            live PPO policy + x/y/yaw simulation runner
  walk_learn.py                  Gym environment, observations, rewards, PPO CLI
  remote_watch.py                SSH checkpoint mirroring and local replay
src/assets/                       MJCF and meshes used by simulation/RL
runs/                             generated training/checkpoint data (gitignored)
tests/                            API, actuator-map, and simulation contract tests
```

Dependency direction is one-way: locomotion knows only the controller protocol;
hardware and simulation never own key mappings or gait decisions; reinforcement
learning consumes the simulation backend without being imported by the core API.

## Simulation terrain

Terrain is injected into the robot MJCF at load time from explicit primitive
definitions in `src/simulation/terrain`. No generated terrain is written into
`src/assets`: boxes and ramps need no external mesh, parameters remain easy to
review, and the same generator can produce deterministic or randomized courses.

Available names are `flat`, `uneven`, `stairs-1`, `stairs-2`, `stairs-3`,
`slope-1`, `slope-2`, `slope-3`, and `mixed`. The root launcher displays the
same list when simulation control is selected.

```python
from src.simulation import load_model

model = load_model(terrain="mixed", terrain_seed=42)
```

The three stair presets use different per-step rise, tread depth, and width;
the public `TerrainGenerator.add_stairs()` algorithm also accepts a custom
`StairProfile`. Slopes use 8°, 15°, and 25°. The mixed course contains the
rough patch and all six difficulty variants with equal gaps, and generates
matching descents so every section returns to the base floor.

See `src/simulation/terrain/README.md` for the complete dimensions.

## Actuator metadata

The physical map is centralized in `Actuator.Index`:

```text
IDs 1..6    upper/body stage     MX-28AT
IDs 7..12   middle/leg stage     XM430-W350-T
IDs 13..18  distal wheel stage   XM430-W210-T
```

For leg `n`, `Actuator.Index.for_leg(n)` returns `(n, n+6, n+12)`. All models
use 4096 position units per revolution. Register access in the hardware
controller goes through the model control tables. Multi-motor target poses use
DYNAMIXEL GroupSyncWrite, while MuJoCo implements the same batch API locally.

## Kinematics

Kinematics load `src/assets/model.xml` directly. Joint locations, axes,
left/right axis reversal, body transforms, and tire frames therefore come from
the same MJCF used by simulation rather than separately hard-coded link lengths.

Angles are radians around raw position 2048: `0 rad = 180 motor degrees`. The
default end effector is the origin of each `TIRE_1` through `TIRE_6` body.

```python
import numpy as np
import SCONE

# One leg: motor degrees -> FK, then position IK.
leg = SCONE.LegKinematics(leg=1)
pose = leg.forward_motor_degrees([135, 170, 195], frame="body")
result = leg.ik(
    pose.position,
    initial_angles=SCONE.JointAngles.from_motor_degrees([140, 168, 192]),
)
print(result.converged, result.angles.as_motor_degrees())

# Whole robot: input/output actuator order is motor ID 1..18.
kinematics = SCONE.RobotKinematics()
motor_degrees = np.array(
    [135, 135, 180, 180, 225, 225] + [170] * 6 + [195] * 6
)
poses = kinematics.forward_motor_degrees(motor_degrees)
targets = np.stack([poses[leg].position for leg in range(1, 7)])
results = kinematics.ik(
    targets,
    initial_angles=np.radians(motor_degrees - 180.0),
)
solved_motor_degrees = kinematics.results_as_motor_degrees(results)
```

IK solves the 3D position with a damped-least-squares MuJoCo Jacobian. Three
joints cannot independently constrain both position and tire orientation, so
orientation is returned by FK but is not an IK target. Use current measured
joint angles as the IK initial value to select the nearest solution branch.

For a calibrated tire/contact point instead of the tire-frame origin, pass its
local coordinate with `LegKinematics(..., end_effector_point=[x, y, z])` or use
the `end_effector_points` mapping on `RobotKinematics`.

## Non-RL Phoenix-style gait

`NonRLWalk` is a continuous, model-based gait engine separate from the legacy
discrete `Walk` motions. It accepts body-frame `[vx, vy, yaw_rate]`, generates
alternating tripod foot trajectories, solves all six legs through the MJCF IK,
and produces one batch of 18 motor positions.

```python
import SCONE

gait = SCONE.NonRLWalk(controller, profile="sport")

# Call at 50 Hz. Units: m/s, m/s, rad/s.
sample = gait.update(
    SCONE.VelocityCommand(vx=0.04, vy=0.00, yaw_rate=0.20),
    dt=0.02,
    send=True,
)
print(sample.stance_legs, sample.failed_legs)
```

The support tripods are `(1, 4, 5)` and `(2, 3, 6)`. During stance, a foot
moves opposite the requested body twist; during swing, minimum-jerk horizontal
interpolation and a zero-touchdown-velocity lift arc return it to the front.
Yaw uses `omega × r` at each nominal foot position, so turning is not a shared
sideways offset. Command filtering, velocity/stride limits, and an all-or-none
IK send guard are enabled by default in `GaitConfig`.

The gait does not treat the centre of `TIRE_n` as a foot. At startup it loads
the collision mesh from `model.xml`, applies the selected profile pose, and
uses each TPU mesh's lowest vertex as a fixed local support point. Supplying
`end_effector_points` overrides this automatic selection with measured points.

Before physical use, call `gait.reset_from_controller()` to centre the stroke
on the measured 18 positions, calibrate each tire's actual support point via
`end_effector_points`, and tune the conservative defaults in MuJoCo. The module
does not automatically enable torque or initialize hardware.

The gait architecture is based on the public
[Lynxmotion Phoenix implementation](https://github.com/KurtE/Phantom_Phoenix)
by Jeroen Janssen, Kurt Eckhardt, and Kåre Halvorsen (Zenta), together with the
[Phoenix PEP description](https://wiki.lynxmotion.com/info/wiki/lynxmotion/view/ses-v1/ses-v1-robots/ses-v1-3-4-dof-hexapods/phoenix-excel-kinematic-seq-01/).
Its phase-offset, stance-translation, swing-lift, and IK structure was adapted
to SCONE's `model.xml`; the trajectory is not a byte-for-byte port of its
discrete Arduino servo loop.

## Reinforcement-learning launcher

Install the RL dependencies and open the interactive launcher:

```bash
python -m pip install -r requirements-rl.txt
python -m src.rl
```

The same menu is available as `4. 강화학습 관리` from `mjpython SCONE.py`. It
asks for the residual reference (`non_rl` is recommended and shown first, while
`hardcoded` preserves the older sinusoidal tripod), curriculum, terrain,
standing pose, timestep count, checkpoint interval, and local/SSH destination.
For SSH training it probes physical/logical CPU cores, available memory, and
load before asking for `num_envs`. The editable recommendation reserves one
physical core, 2 GiB for the OS/PPO parent, and estimates 768 MiB per MuJoCo
environment. Runs with more than one environment use `SubprocVecEnv`, so this is
real process parallelism rather than sequential interleaving.

For a remote run, the launcher can synchronize the current local source first
(excluding `.git`, virtual environments, `runs`, and archives), then starts the
trainer with `nohup`. If the remote RL packages are missing, it can create a
Python 3.12 project-local `.venv` and install `requirements-rl.txt`
automatically before creating the run directory. A pre-existing virtualenv
using another Python version is renamed to `.venv.python-old_<timestamp>` rather
than deleted. Remote artifacts remain on the training machine at:

```text
~/Developer/SCONE/runs/<run-name>/
  train.log
  train.pid
  checkpoints/scone_walk_<steps>_steps.zip
  final_model.zip
```

The launcher can run a local environment/reward smoke test, show the remote PID
and recent log output, atomically download and ZIP-validate the newest
checkpoint/final model into the matching local `runs/<run-name>/` directory,
continuously mirror checkpoints while a local MuJoCo viewer is open, or replay
any downloaded `.zip` policy.

If reward or observation semantics change and a run must restart from zero,
choose `원격 실행/체크포인트 초기화`. The launcher refuses to touch a running
trainer without a separate confirmation and requires the exact run name. It
then moves the complete remote run (checkpoints, final model, PID, and log) to
`runs/.reset_backup/<run-name>_<timestamp>` instead of permanently deleting it.
The original run name is then free for a clean training run.

## Direct commands

```bash
# Old and Non-RL joystick control
mjpython -m src.simulation --control old --profile standard
mjpython -m src.simulation --control non_rl --profile sport --terrain mixed

# RL joystick control with a downloaded/trained checkpoint
mjpython -m src.simulation --control rl \
  --checkpoint runs/remote_watch/scone_walk_700000_steps.zip \
  --rl-reference-motion non_rl \
  --terrain flat

# RL environment smoke check
PYTHONPATH=. python -m src.rl.walk_learn --reference-motion non_rl check \
  --steps 5 --curriculum easy

# PPO training
PYTHONPATH=. python -m src.rl.walk_learn --reference-motion non_rl train \
  --curriculum easy --timesteps 1000000 --num-envs 4

# Unit and integration-contract tests
python -m unittest discover -s tests -v
```

The locomotion reward configuration is defined by `RewardConfig` in
`src/rl/walk_learn.py`; reward calculation and per-term logging
are implemented in `SconeWalkEnv` in the same file.
