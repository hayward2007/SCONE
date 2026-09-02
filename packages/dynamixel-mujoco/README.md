# dynamixel-mujoco

Model a DYNAMIXEL actuator in MuJoCo without double-counting gearbox loss.

Small, dependency-light, and verified by simulation rather than by assertion:
the test suite loads the emitted MJCF and checks it reproduces the ROBOTIS
e-Manual.

```bash
pip install -e .
dynamixel-mujoco-bench          # datasheet, damping and backlash checks
```

## The rule

**Encode the torque-speed line exactly once.**

The e-Manual stall torque and no-load speed are measured at the **output shaft
of the assembled actuator**, so gearbox and motor loss already lives inside
them. A `dcmotor` built from that pair reaches zero net torque exactly at the
rated no-load speed:

```
actuator            stall [N m]          no-load [rad/s]
                sheet        sim      sheet        sim     err
MX-28AT         2.500      2.500      5.760      5.760  -0.00%
XM430-W350-T    4.100      4.100      4.817      4.817  -0.00%
XM430-W210-T    3.000      3.000      8.063      8.063  -0.00%
```

Adding `damping` or `frictionloss` on top double-counts the loss. In one
measured case a plausible-looking `frictionloss` of 0.06-0.09 N m cost 3.0-3.7%
of no-load speed across the three actuators.

Models built on a `position` actuator must do the opposite, because a position
source has no back-EMF term: their `damping` *is* the torque-speed line. Two
independent examples, both of which reproduce the servo's rated no-load speed
to within a few percent:

| model | arithmetic | result | rated |
| --- | --- | ---: | ---: |
| MuJoCo Menagerie `robotis_op3` (XM430-W350) | `(5 - 0.03) / 1.084` | 4.59 rad/s | 4.82 |
| Open Duck Mini v2 (Feetech STS3215) | `(3.23 - 0.068) / 0.56` | 5.65 rad/s | ~5.5 |

Neither convention is wrong. Mixing them is.

## What the e-Manual does not contain

**Inertia.** A torque-speed pair is a steady-state characteristic, so reflected
rotor inertia has to be supplied separately, and leaving it out is the largest
single sim-to-real error in a geared servo model. At 193-353:1 it is 0.2x to
2.5x the link inertia of a small leg.

```python
from dynamixel_mujoco import spec
item = spec("XM430-W350-T")
item.armature                      # 0.01749 kg m^2  = J_rotor * N^2
item.mechanical_time_constant      # 0.0206 s, invariant under gearing
```

ROBOTIS does not publish rotor inertia, so `DEFAULT_ROTOR_INERTIA` is an
estimate chosen to put `tau_m` in the 10-25 ms band such motors occupy.
**Identify it per actuator** by fitting a no-load step response to
`w(t) = w_inf (1 - exp(-t/tau_m))`; that needs no knowledge of `J_rotor`.

Published armature values for the same XM430-W350 span 0.01749 (this package)
to 0.045 (Menagerie). Treat that 2.6x as the uncertainty band and check that
your conclusions survive it.

### Adding armature invalidates your PD gains

`kd` tuned for critical damping at the link inertia is no longer critical once
armature is added. Measured step overshoot on a 20-degree step:

| armature | kd | overshoot |
| --- | ---: | ---: |
| ignored | `2*sqrt(kp*J_link)` | up to 13.6% |
| included | `2*sqrt(kp*(J_link + armature))` | 0.0% |

```python
item.critical_damping(kp=9.40, link_inertia=0.01668)
```

## Backlash

The e-Manual publishes it (20 arcmin on the MX-28AT, 15 on the XM430) and it is
not negligible: at a 0.12 m contact radius, 15 arcmin is 0.5 mm of contact
position. Model it as a limited free joint in series with the driven joint;
MuJoCo composes the joints of one body in order, so a second hinge on the same
axis makes the body angle the sum.

```python
import xml.etree.ElementTree as ET
from dynamixel_mujoco import add_backlash, by_pattern

root = ET.parse("robot.xml").getroot()
add_backlash(root, by_pattern(
    (r"M0[1-6]_", "MX-28AT"),
    (r"M(0[7-9]|1[0-2])_", "XM430-W350-T"),
    (r"M1[3-8]_", "XM430-W210-T"),
))
```

Two things that bite:

* **Units.** MuJoCo defaults to *degrees* when `<compiler angle=...>` is absent,
  so a radian range emitted into such a model shrinks the dead band 57-fold.
  `add_backlash` reads the setting; `backlash_joint` takes it explicitly. There
  is a regression test for this.
* **Stiffness.** MuJoCo's default joint limit is soft enough that a working
  torque pushes several times past the stop. The defaults here hold the stop to
  within 4% of the limit up to stall torque.

A real X-series encoder sits on the output shaft, so **whatever reads joint
position must sum the driven angle and the play**, otherwise the controller
sees a joint that is stiffer than the hardware.

## Catalog

| actuator | 12 V stall | no-load | gear | backlash | armature | tau_m |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| MX-28AT | 2.5 N m | 55 rpm | 193:1 | 20' | 0.00521 | 12 ms |
| XM430-W350-T | 4.1 N m | 46 rpm | 353.5:1 | 15' | 0.01749 | 21 ms |
| XM430-W210-T | 3.0 N m | 77 rpm | 212.6:1 | 15' | 0.00633 | 17 ms |

Add an entry only from the e-Manual itself; every other field is derived.

## Not modelled

Continuous-torque derating (`saturation` is the *instantaneous* stall limit and
a real unit overheats within seconds of holding it), thermal effects, supply
sag, bus latency, and backdrive resistance. With no Coulomb term a de-energized
joint here is frictionless. If you measure a friction split and want to add
`frictionloss`, you must raise the `nominal` no-load speed to the motor-side
value in the same edit:

```
w_nominal = w_sheet / (1 - frictionloss / stall_torque)
```

## Origin

Extracted from the [SCONE](https://github.com/hayward2007/SCONE) hexapod
project, where the double-counting mistake was made, measured, and corrected.

## Licence

MIT.
