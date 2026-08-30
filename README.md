# SCONE

SCONE is a six-legged robot project with one high-level control API and two
interchangeable backends: physical DYNAMIXEL hardware and MuJoCo simulation.

## Quick start

Run the launcher on macOS with `mjpython` so the MuJoCo viewer can own the main
thread:

```bash
mjpython SCONE.py
```

The launcher searches for a physical controller without changing torque or
position, then displays:

```text
1. 시뮬레이션 조종
2. 하드웨어 조종 (/dev/...)
   or 하드웨어 조종 (현재 불가)
3. 하드웨어 다시 탐색
```

Simulation and hardware use exactly the same terminal command interpreter:

```text
W/S  forward/backward
A/D  left/right
R    Walk -> Drive -> Climb
H    return to the selected profile's home pose
?    help
Q    return to the launcher
```

The MuJoCo window has no SCONE-specific key callback. W/A/S/D are read by the
terminal only, so robot commands do not alter viewer rendering controls.

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
  -> src.cli RobotCommand interpreter
  -> src.main.SCONE API
  -> src.locomotion Walk / Drive / Climb
  -> ControllerProtocol
       -> src.hardware.Controller       (DYNAMIXEL)
       -> src.simulation.MuJoCoController (MuJoCo)
```

There is no second simulation key map or duplicated simulation gait logic.

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
src/simulation/
  model.py                       MJCF loading and runtime floor/freejoint utility
  controller.py                  virtual DYNAMIXEL implementation
  pid.py                         voltage-input DC motor position loop
  cli_bridge.py                  one viewer + common terminal CLI integration
  simulator_cli.py               direct simulation entry point
src/reinforce_learning/
  walk_learn.py                  Gym environment, observations, rewards, PPO CLI
  remote_watch.py                SSH checkpoint mirroring and local replay
src/assets/                       MJCF and meshes used by simulation/RL
runs/                             generated training/checkpoint data (gitignored)
tests/                            API, actuator-map, and simulation contract tests
```

Dependency direction is one-way: locomotion knows only the controller protocol;
hardware and simulation never own key mappings or gait decisions; reinforcement
learning consumes the simulation backend without being imported by the core API.

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

## Direct commands

```bash
# Same common terminal control, directly in simulation
mjpython -m src.simulation --profile sport

# RL environment smoke check
PYTHONPATH=. python -m src.reinforce_learning.walk_learn check \
  --steps 5 --curriculum easy

# PPO training
PYTHONPATH=. python -m src.reinforce_learning.walk_learn train \
  --curriculum easy --timesteps 1000000

# Unit and integration-contract tests
python -m unittest discover -s tests -v
```

The locomotion reward configuration is defined by `RewardConfig` in
`src/reinforce_learning/walk_learn.py`; reward calculation and per-term logging
are implemented in `SconeWalkEnv` in the same file.
