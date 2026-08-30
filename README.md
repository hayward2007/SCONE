# SCONE

SCONE is a six-legged robot project that combines real hardware control, MuJoCo simulation, gait generation, and reinforcement learning in a single codebase.

The repository is organized around a single design idea:

- hardware constants and actuator metadata live in one place
- gait and locomotion logic are separated from the control interface
- simulation behaves like the same robot interface as the real hardware
- RL training consumes the robot state and command, not raw actuator quirks

---

## Repository structure

```text
SCONE/
├── LICENSE
├── README.md
├── RL_Log.md
├── main.py
├── archive/
│   ├── assets/
│   ├── codes/
│   ├── ICRA/
│   ├── meshes/
│   ├── papers/
│   └── videos/
├── src/
│   ├── __init__.py
│   ├── SCONE.py
│   ├── assets/
│   │   ├── model.xml
│   │   └── meshes/
│   ├── hardware/
│   │   ├── __init__.py
│   │   ├── actuator.py
│   │   ├── actuator_index.py
│   │   └── actuator_control_table.py
│   ├── locomotion/
│   │   ├── __init__.py
│   │   ├── climb.py
│   │   ├── drive.py
│   │   ├── mode.py
│   │   └── walk.py
│   ├── reinforce_learning/
│   │   ├── __init__.py
│   │   ├── remote_watch.py
│   │   ├── walk_learn.py
│   │   └── remote_watch/
│   └── simulation/
│       ├── __init__.py
│       ├── __main__.py
│       ├── app.py
│       ├── cli_bridge.py
│       ├── controller.py
│       ├── pid.py
│       ├── runner.py
│       └── simulator_cli.py
└── .gitignore
```

---

## Layer responsibilities

### Hardware layer

Location: src/hardware/

Contains actuator definitions, side/leg groupings, and DYNAMIXEL register maps.

Examples:

- actuator IDs and leg grouping
- robot-specific constants
- protocol/version metadata

This layer should stay static and avoid motion logic.

### Locomotion layer

Location: src/locomotion/

Defines motion profiles and gait behavior, such as:

- Walk
- Drive
- Climb
- mode switching logic

This layer defines how the robot should move, independent of RL training.

### Simulation layer

Location: src/simulation/

Provides a MuJoCo-backed runtime that mirrors the same robot interface used by the real system.

This is used for:

- debugging motion sequences
- validating controller behavior offline
- testing control loops before hardware use

### Reinforcement learning layer

Location: src/reinforce_learning/

Handles policy training, observation construction, reward design, and model checkpoints.

This layer consumes robot state and commands, not hardware-specific magic numbers.

---

## Main entry points

### Python package entry

```bash
python main.py
```

### Environment check for RL

```bash
PYTHONPATH=. python -m src.reinforce_learning.walk_learn check --steps 5 --curriculum easy
```

### Training

```bash
PYTHONPATH=. python -m src.reinforce_learning.walk_learn train --curriculum easy --timesteps 1000000
```

### Simulated preview

```bash
PYTHONPATH=. python -m src.simulation.simulator_cli
```

---

## Architectural principles

1. Hardware constants live only in src/hardware
2. Motion logic lives only in src/locomotion
3. Simulation is a test environment for the same control abstractions
4. RL is optimized over robot state and commands, not raw actuator values
5. No magic numbers should leak across modules when a shared constant exists

---

## Notes

- The project intentionally keeps the real-robot and simulation paths close so they can share the same logic and behaviors.
- The model used by the simulator and RL stack is under src/assets/model.xml.
- Older historical experiments and archived material remain under archive/.

This repository is currently organized around a cleaner refactor model: robot hardware metadata, locomotion logic, simulation runtime, and RL learning are separated by responsibility rather than mixed together.
