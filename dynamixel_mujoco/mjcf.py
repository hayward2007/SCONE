"""Emit MuJoCo MJCF for DYNAMIXEL actuators, and add gear backlash.

The one rule worth remembering
-----------------------------
**Encode the torque-speed line exactly once.**

The e-Manual stall torque and no-load speed are measured at the output shaft of
the assembled actuator, so gearbox and motor loss already lives inside them. A
``dcmotor`` built from that pair reaches zero net torque exactly at the rated
no-load speed. Adding ``damping`` or ``frictionloss`` on top double-counts the
loss and makes the simulated actuator slower than the hardware.

Published models that use a ``position`` actuator must do the opposite, because
a position source has no back-EMF term. Their ``damping`` *is* the torque-speed
line. Two independent examples:

* MuJoCo Menagerie ``robotis_op3`` (XM430-W350):
  ``(5 - 0.03) / 1.084 = 4.59 rad/s``, the rated no-load speed.
* Open Duck Mini v2 (Feetech STS3215):
  ``(3.23 - 0.068) / 0.56 = 5.65 rad/s``, likewise.

What the e-Manual does *not* contain is inertia. A torque-speed pair is a
steady-state characteristic, so reflected rotor inertia has to be supplied
separately; that is what :attr:`DynamixelSpec.armature` is for, and leaving it
out is the largest single sim-to-real error in a geared servo model.
"""

from __future__ import annotations

import math
import re
import xml.etree.ElementTree as ET
from typing import Callable, Mapping

from .specs import DynamixelSpec, spec


BACKLASH_SUFFIX = "_backlash"
# MuJoCo's default limit is soft enough that a working torque pushes several
# times past the stop. These make the dead-band edge behave like a tooth flank.
BACKLASH_SOLREFLIMIT = "0.002 1"
BACKLASH_SOLIMPLIMIT = "0.95 0.99 0.0005 0.5 2"
BACKLASH_ARMATURE = 1.0e-5
BACKLASH_DAMPING = 0.01


def dcmotor(name: str, joint: str, actuator: DynamixelSpec | str) -> ET.Element:
    """A voltage-input ``dcmotor`` whose torque-speed line matches the manual.

    Deliberately emits no ``damping`` or ``frictionloss``: see the module
    docstring. ``saturation`` is the stall torque, which is an *instantaneous*
    limit; continuous duty is far lower and needs a thermal budget elsewhere.
    """

    item = spec(actuator) if isinstance(actuator, str) else actuator
    return ET.Element("dcmotor", {
        "name": name,
        "joint": joint,
        "nominal": f"{item.volts:g} {item.stall_torque:g} {item.no_load_speed!r}",
        "saturation": f"{item.stall_torque:g} 0 0",
        "ctrllimited": "true",
        "ctrlrange": f"-{item.volts:g} {item.volts:g}",
    })


def joint_attributes(actuator: DynamixelSpec | str) -> dict[str, str]:
    """Attributes to merge into the driven ``<joint>``: armature only."""

    item = spec(actuator) if isinstance(actuator, str) else actuator
    return {"armature": f"{item.armature:.5g}"}


def backlash_joint(
    driven: ET.Element,
    actuator: DynamixelSpec | str,
    *,
    angle: str = "radian",
) -> ET.Element:
    """A limited free joint carrying half the play either side of zero.

    MuJoCo composes the joints of one body in order, so a second hinge on the
    same axis makes the body angle the sum of the driven angle and the play.

    ``angle`` must match the host model's ``<compiler angle=...>``. MuJoCo
    defaults to degrees when the attribute is absent, so emitting radians into
    such a model silently shrinks the dead band by 57x. :func:`add_backlash`
    reads the setting for you.
    """

    item = spec(actuator) if isinstance(actuator, str) else actuator
    half = item.backlash / 2.0
    if angle == "degree":
        half = math.degrees(half)
    elif angle != "radian":
        raise ValueError(f"angle must be 'radian' or 'degree', got {angle!r}")
    return ET.Element("joint", {
        "name": f"{driven.get('name')}{BACKLASH_SUFFIX}",
        "type": "hinge",
        "axis": driven.get("axis", "0 0 1"),
        "pos": driven.get("pos", "0 0 0"),
        "limited": "true",
        "range": f"{-half:.9g} {half:.9g}",
        "armature": f"{BACKLASH_ARMATURE:g}",
        "damping": f"{BACKLASH_DAMPING:g}",
        "frictionloss": "0",
        "stiffness": "0",
        "solreflimit": BACKLASH_SOLREFLIMIT,
        "solimplimit": BACKLASH_SOLIMPLIMIT,
    })


def add_backlash(
    root: ET.Element,
    actuator_for_joint: Mapping[str, str] | Callable[[str], str | None],
) -> int:
    """Insert a play joint after every joint the mapping assigns an actuator to.

    ``actuator_for_joint`` maps a joint name to a catalog key, or to ``None`` to
    leave that joint alone. Returns how many play joints were added.

    A real X-series encoder sits on the output shaft, so whatever reads joint
    position for control or observation must sum the driven angle and the play.
    """

    lookup = (
        actuator_for_joint.get
        if isinstance(actuator_for_joint, Mapping)
        else actuator_for_joint
    )
    compiler = root.find("compiler")
    # MuJoCo's own default is degrees when the attribute is absent.
    angle = "degree" if compiler is None else compiler.get("angle", "degree")
    added = 0
    for body in root.iter("body"):
        for joint in list(body.findall("joint")):
            name = joint.get("name")
            if not name or name.endswith(BACKLASH_SUFFIX):
                continue
            actuator = lookup(name)
            if actuator is None:
                continue
            body.insert(
                list(body).index(joint) + 1,
                backlash_joint(joint, actuator, angle=angle),
            )
            added += 1
    return added


def by_pattern(*rules: tuple[str, str]) -> Callable[[str], str | None]:
    """Build a joint-name to actuator mapping from regex rules, first match wins.

    >>> mapping = by_pattern((r"M0[1-6]_", "MX-28AT"), (r"M1[3-8]_", "XM430-W210-T"))
    >>> mapping("M03_body_L3")
    'MX-28AT'
    """

    compiled = [(re.compile(pattern), actuator) for pattern, actuator in rules]

    def lookup(joint_name: str) -> str | None:
        for pattern, actuator in compiled:
            if pattern.match(joint_name):
                return actuator
        return None

    return lookup


__all__ = [
    "BACKLASH_SUFFIX", "add_backlash", "backlash_joint", "by_pattern",
    "dcmotor", "joint_attributes",
]
