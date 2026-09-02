"""Check the actuator model against the ROBOTIS e-Manual and published models.

Three questions, each answered by measurement rather than by argument:

``datasheet``   does ``model.xml`` reproduce the 12 V stall torque and no-load
                speed of each DYNAMIXEL?
``damping``     does the closed loop still critically damp now that the joints
                carry reflected rotor inertia?
``compare``     how does our parameterisation differ from the MuJoCo Menagerie
                ROBOTIS OP3 model, which drives the same XM430-W350?

See docs/18-actuator-model-and-frame-convention.md and
docs/19-actuator-settings-vs-published-models.md for the conclusions.
"""

from __future__ import annotations

import argparse
import math
from typing import Sequence

import mujoco
import numpy as np


# ROBOTIS e-Manual, 12 V column.
#   name: (volts, stall N m, no-load rev/min, stall A, gear, armature)
DATASHEET = {
    "MX-28AT": (12.0, 2.5, 55.0, 1.4, 193.0, 0.00521),
    "XM430-W350-T": (12.0, 4.1, 46.0, 2.3, 353.5, 0.01749),
    "XM430-W210-T": (12.0, 3.0, 77.0, 2.3, 212.6, 0.00633),
}

# src/simulation/core/pid.py, and the link inertia each kd was critical for.
GAINS = {
    "MX-28AT": (5.73, 0.828, 0.02467),
    "XM430-W350-T": (9.40, 1.134, 0.01668),
    "XM430-W210-T": (6.88, 0.494, 0.00253),
}

_ONE_JOINT = """
<mujoco>
  <option timestep="0.0002" gravity="0 0 0"/>
  <worldbody><body>
    <joint name="j" type="hinge" axis="0 0 1" armature="{arm}"
           damping="{damp}" frictionloss="{fric}"/>
    <geom type="cylinder" size="{radius} 0.01" mass="{mass}"/>
  </body></worldbody>
  <actuator>{actuator}</actuator>
</mujoco>
"""
_DCMOTOR = ('<dcmotor name="a" joint="j" nominal="{V} {tau} {wnl}" '
            'saturation="{tau} 0 0" ctrllimited="true" ctrlrange="-{V} {V}"/>')
# google-deepmind/mujoco_menagerie robotis_op3 (ROBOTIS-GIT ships the same file).
_POSITION = ('<position name="a" joint="j" kp="21.1" forcerange="-5 5" '
             'ctrlrange="-3.141592 3.141592"/>')


def _rig(actuator: str, *, arm: float, damp: float = 0.0, fric: float = 0.0,
         inertia: float = 0.0):
    radius = 0.05
    mass = max(1e-6, 2.0 * inertia / (radius * radius))
    xml = _ONE_JOINT.format(arm=arm, damp=damp, fric=fric, radius=radius,
                            mass=mass, actuator=actuator)
    model = mujoco.MjModel.from_xml_string(xml)
    return model, mujoco.MjData(model)


def _stall(model, data, drive: float) -> float:
    mujoco.mj_resetData(model, data)
    data.ctrl[0] = drive
    data.qvel[0] = 0.0
    mujoco.mj_forward(model, data)
    return float(data.qfrc_actuator[0])


def _free_speed(model, data, drive: float, seconds: float = 20.0) -> float:
    mujoco.mj_resetData(model, data)
    data.ctrl[0] = drive
    for _ in range(int(seconds / model.opt.timestep)):
        mujoco.mj_step(model, data)
        if abs(data.qpos[0]) > 3.0:      # keep a position servo saturated
            data.qpos[0] = 0.0
    return float(data.qvel[0])


def run_datasheet() -> int:
    print(f"{'actuator':15s} {'stall [N m]':>22s} {'no-load [rad/s]':>24s}")
    print(f"{'':15s} {'sheet':>10s} {'model':>10s} {'sheet':>10s} {'model':>10s} {'err':>7s}")
    worst = 0.0
    for name, (volts, tau, rpm, _amp, _gear, arm) in DATASHEET.items():
        wnl = rpm * 2 * math.pi / 60
        model, data = _rig(_DCMOTOR.format(V=volts, tau=tau, wnl=wnl), arm=arm)
        stall = _stall(model, data, volts)
        speed = _free_speed(model, data, volts)
        error = 100 * (speed / wnl - 1)
        worst = max(worst, abs(error), abs(100 * (stall / tau - 1)))
        print(f"{name:15s} {tau:10.3f} {stall:10.3f} {wnl:10.3f} {speed:10.3f} "
              f"{error:+6.1f}%")
    print(f"\nworst deviation from the datasheet: {worst:.2f}%")
    return 0 if worst < 0.5 else 1


def run_damping(step_degrees: float = 20.0) -> int:
    print(f"{'joint group':15s} {'armature':>10s} {'overshoot':>10s} {'settle 2%':>10s}")
    ok = True
    for name, (volts, tau, rpm, _amp, _gear, arm) in DATASHEET.items():
        wnl = rpm * 2 * math.pi / 60
        kp, kd, link = GAINS[name]
        K = volts / wnl
        resistance = K * volts / tau
        for label, armature in (("off", 0.0), ("on", arm)):
            model, data = _rig(_DCMOTOR.format(V=volts, tau=tau, wnl=wnl),
                               arm=armature, inertia=link)
            mujoco.mj_resetData(model, data)
            target = math.radians(step_degrees)
            trace = []
            for _ in range(int(1.5 / model.opt.timestep)):
                position, velocity = float(data.qpos[0]), float(data.qvel[0])
                torque = kp * (target - position) - kd * velocity
                data.ctrl[0] = float(np.clip(
                    resistance / K * torque + K * velocity, -volts, volts))
                mujoco.mj_step(model, data)
                trace.append(float(data.qpos[0]))
            trace = np.array(trace)
            overshoot = 100.0 * (trace.max() - target) / target
            outside = np.where(np.abs(trace - target) > 0.02 * target)[0]
            settle = (outside[-1] + 1) * model.opt.timestep if len(outside) else 0.0
            if label == "on" and overshoot > 1.0:
                ok = False
            print(f"{name if label == 'off' else '':15s} {label:>10s} "
                  f"{overshoot:9.1f}% {settle:9.3f}s")
    print("\ncritically damped with armature" if ok else "\nUNDERDAMPED: recompute kd")
    return 0 if ok else 1


def run_compare() -> int:
    volts, tau, rpm, _amp, gear, arm = DATASHEET["XM430-W350-T"]
    wnl = rpm * 2 * math.pi / 60
    K = volts / wnl
    resistance = K * volts / tau

    ours = _rig(_DCMOTOR.format(V=volts, tau=tau, wnl=wnl), arm=arm)
    theirs = _rig(_POSITION, arm=0.045, damp=1.084, fric=0.03)

    print("both models claim to represent a DYNAMIXEL XM430-W350 at 12 V\n")
    print(f"{'model':28s} {'stall':>10s} {'err':>8s} {'no-load':>10s} {'err':>8s}")
    print(f"{'datasheet':28s} {tau:10.3f} {'--':>8s} {wnl:10.3f} {'--':>8s}")
    for label, (model, data), drive in (
        ("SCONE (dcmotor+nominal)", ours, volts),
        ("Menagerie OP3 (position)", theirs, math.pi),
    ):
        stall = _stall(model, data, drive)
        speed = _free_speed(model, data, drive)
        print(f"{label:28s} {stall:10.3f} {100 * (stall / tau - 1):+7.1f}% "
              f"{speed:10.3f} {100 * (speed / wnl - 1):+7.1f}%")

    print("\nMenagerie's damping IS its torque-speed line: "
          f"(5 - 0.03) / 1.084 = {(5 - 0.03) / 1.084:.3f} rad/s, "
          "which is the no-load speed. A position actuator has no back-EMF, so\n"
          "the velocity term must be supplied by damping. Our dcmotor already\n"
          "contains it, which is why adding damping on top double-counts.\n")
    for label, armature in (("SCONE", arm), ("Menagerie", 0.045)):
        print(f"{label:10s} armature {armature:.5f} -> J_rotor "
              f"{armature / gear ** 2:.3e} kg m^2, "
              f"tau_m = {armature * resistance / K ** 2 * 1000:.0f} ms")
    print("\ncoreless motors this size are typically 5-20 ms; the 2.6x spread is\n"
          "the honest uncertainty band on the unpublished rotor inertia.")
    return 0


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "check", nargs="?", default="all",
        choices=("all", "datasheet", "damping", "compare"),
    )
    args = parser.parse_args(argv)
    status = 0
    for name, runner in (("datasheet", run_datasheet),
                         ("damping", run_damping),
                         ("compare", run_compare)):
        if args.check in ("all", name):
            print(f"\n=== {name} ===")
            status |= runner()
    return status


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
