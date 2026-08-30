# SCONE v2
Six-legged robot Capable Of rotating motioN ( Enhanced )  
회전운동이 가능한 6족 로봇, 스콘

![Eco-friendly deliver SCONE Poster](./docs/Eco-friendly%20deliver%20SCONE%20Poster.jpg)

Please check the [reference folder](./docs/) for more information.  
made by & all copyrights to **Kim Hyoung Seok** a.k.a **Hayward Kim**

## MuJoCo motion preview

The simulator reuses the existing `Walk`, `Drive`, and `Climb` providers through
a MuJoCo-backed controller, so it never opens the physical DYNAMIXEL serial port.

On macOS, launch the interactive viewer with `mjpython`:

```bash
mjpython simulator.py
```

Keyboard controls in the MuJoCo window:

```text
W/S  forward/backward
A/D  left/right
R    Walk -> Drive -> Climb
H    return to the profile's home pose
I    toggle all raw 2048 / profile initial pose
Q    quit
```

The default fixed-base view is useful for checking joint directions and motion
sequences. To add a floor and a floating root at runtime without changing
`model.xml`, use:

```bash
mjpython simulator.py --floating-base
```

All actuators use the same direction convention: raw `0` is the CCW limit,
`2048` is center, and `4096` is the CW limit. Motors mounted the same physical
way rotate the same way for the same command, so no left/right ID remapping
is needed; the model's own mirrored leg geometry already accounts for any
handedness difference between sides (see SCONE_RL.md).

Other examples:

```bash
mjpython simulator.py --profile sport --demo forward
python simulator.py --headless --duration 2 --quiet
```
