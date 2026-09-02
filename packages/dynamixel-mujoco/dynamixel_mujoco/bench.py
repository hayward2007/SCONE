"""Verify an actuator model by simulating it, not by trusting the table.

``datasheet``  does the emitted MJCF reproduce the e-Manual stall torque and
               no-load speed?
``damping``    is a torque-space PD still critically damped once the joint
               carries reflected rotor inertia?
``backlash``   does the play joint hold its dead-band under working torque?

``datasheet`` and ``damping`` are usable as regression tests: they return a
non-zero exit status when the model drifts.
"""

from __future__ import annotations

import argparse
import math
import xml.etree.ElementTree as ET
from typing import Sequence

import mujoco
import numpy as np

from .mjcf import backlash_joint, dcmotor, joint_attributes
from .specs import CATALOG, DynamixelSpec


def _single_joint_xml(item: DynamixelSpec, *, inertia: float = 0.0,
                      backlash: bool = False, timestep: float = 2e-4) -> str:
    radius = 0.05
    mass = max(1e-6, 2.0 * inertia / (radius * radius))
    model = ET.Element("mujoco")
    ET.SubElement(model, "compiler", {"angle": "radian"})
    ET.SubElement(model, "option", {"timestep": f"{timestep:g}",
                                    "gravity": "0 0 0",
                                    "integrator": "implicitfast"})
    world = ET.SubElement(model, "worldbody")
    body = ET.SubElement(world, "body")
    joint = ET.SubElement(body, "joint", {
        "name": "j", "type": "hinge", "axis": "0 0 1", **joint_attributes(item)
    })
    if backlash:
        body.append(backlash_joint(joint, item))
    ET.SubElement(body, "geom", {"type": "cylinder",
                                 "size": f"{radius:g} 0.01", "mass": f"{mass:g}"})
    ET.SubElement(model, "actuator").append(dcmotor("a", "j", item))
    return ET.tostring(model, encoding="unicode")


def _rig(item: DynamixelSpec, **kwargs):
    model = mujoco.MjModel.from_xml_string(_single_joint_xml(item, **kwargs))
    return model, mujoco.MjData(model)


def run_datasheet(tolerance: float = 0.5) -> int:
    print(f"{'actuator':15s} {'stall [N m]':>21s} {'no-load [rad/s]':>23s}")
    print(f"{'':15s} {'sheet':>10s} {'sim':>10s} {'sheet':>10s} {'sim':>10s} {'err':>7s}")
    worst = 0.0
    for item in CATALOG.values():
        model, data = _rig(item)
        data.ctrl[0] = item.volts
        data.qvel[0] = 0.0
        mujoco.mj_forward(model, data)
        stall = float(data.qfrc_actuator[0])
        mujoco.mj_resetData(model, data)
        data.ctrl[0] = item.volts
        for _ in range(int(20.0 / model.opt.timestep)):
            mujoco.mj_step(model, data)
        speed = float(data.qvel[0])
        error = 100.0 * (speed / item.no_load_speed - 1.0)
        worst = max(worst, abs(error), abs(100.0 * (stall / item.stall_torque - 1.0)))
        print(f"{item.name:15s} {item.stall_torque:10.3f} {stall:10.3f} "
              f"{item.no_load_speed:10.3f} {speed:10.3f} {error:+6.2f}%")
    print(f"\nworst deviation {worst:.2f}% (tolerance {tolerance}%)")
    return 0 if worst < tolerance else 1


def run_damping(link_inertia: float = 0.02, kp: float = 6.0,
                step_degrees: float = 20.0, tolerance: float = 1.0) -> int:
    print(f"{'actuator':15s} {'armature':>10s} {'kd':>8s} {'overshoot':>10s}")
    ok = True
    for item in CATALOG.values():
        for label, armature in (("ignored", 0.0), ("included", item.armature)):
            kd = (2.0 * math.sqrt(kp * link_inertia) if armature == 0.0
                  else item.critical_damping(kp, link_inertia))
            model, data = _rig(item, inertia=link_inertia)
            model.dof_armature[0] = armature
            mujoco.mj_resetData(model, data)
            target = math.radians(step_degrees)
            peak = 0.0
            for _ in range(int(2.0 / model.opt.timestep)):
                position, velocity = float(data.qpos[0]), float(data.qvel[0])
                torque = kp * (target - position) - kd * velocity
                data.ctrl[0] = float(np.clip(
                    item.resistance / item.torque_constant * torque
                    + item.torque_constant * velocity, -item.volts, item.volts))
                mujoco.mj_step(model, data)
                peak = max(peak, float(data.qpos[0]))
            overshoot = 100.0 * (peak - target) / target
            if label == "included" and overshoot > tolerance:
                ok = False
            print(f"{item.name if label == 'ignored' else '':15s} {label:>10s} "
                  f"{kd:8.3f} {overshoot:9.2f}%")
    # The point: reuse a kd tuned without armature and the joint rings.
    print("\nkd must be recomputed once armature is added; "
          f"{'it was' if ok else 'IT WAS NOT'}")
    return 0 if ok else 1


def run_backlash() -> int:
    print(f"{'actuator':15s} {'play [arcmin]':>14s} {'held under torque':>28s}")
    for item in CATALOG.values():
        model, data = _rig(item, inertia=0.02, backlash=True)
        limit = model.jnt_range[1][1]
        travel = []
        for torque in (0.1, item.stall_torque / 2.0, item.stall_torque):
            mujoco.mj_resetData(model, data)
            for _ in range(3000):
                data.qfrc_applied[1] = torque
                mujoco.mj_step(model, data)
            travel.append(math.degrees(float(data.qpos[1])) * 60.0)
        held = " ".join(f"{value:5.1f}" for value in travel)
        print(f"{item.name:15s} {math.degrees(limit) * 120:13.0f}' "
              f"{held:>28s}   (limit {math.degrees(limit) * 60:.1f}')")
    print("\ncolumns are travel at 0.1 N m, half stall and stall")
    return 0


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("check", nargs="?", default="all",
                        choices=("all", "datasheet", "damping", "backlash"))
    args = parser.parse_args(argv)
    status = 0
    for name, runner in (("datasheet", run_datasheet),
                         ("damping", run_damping),
                         ("backlash", run_backlash)):
        if args.check in ("all", name):
            print(f"\n=== {name} ===")
            status |= runner()
    return status


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
