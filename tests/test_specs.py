"""The catalog is the e-Manual; everything else must follow from it."""

from __future__ import annotations

import math

import pytest

from dynamixel_mujoco.bench import run_damping, run_datasheet
from dynamixel_mujoco.mjcf import add_backlash, by_pattern, dcmotor
from dynamixel_mujoco.specs import CATALOG, arcmin, rpm, spec

import xml.etree.ElementTree as ET


@pytest.mark.parametrize("name", sorted(CATALOG))
def test_derived_constants_invert_the_datasheet(name: str) -> None:
    item = spec(name)
    # K and R are defined so the torque-speed line passes through both endpoints.
    assert item.torque_constant * item.no_load_speed == pytest.approx(item.volts)
    stall = item.torque_constant * item.volts / item.resistance
    assert stall == pytest.approx(item.stall_torque)


@pytest.mark.parametrize("name", sorted(CATALOG))
def test_sheet_stall_current_exceeds_the_ideal_model(name: str) -> None:
    """The sheet torque is post-gearbox, so V/R under-predicts stall current."""
    item = spec(name)
    assert item.modelled_stall_current < item.stall_current
    assert 0.6 < item.gear_efficiency < 1.0


@pytest.mark.parametrize("name", sorted(CATALOG))
def test_mechanical_time_constant_is_physically_plausible(name: str) -> None:
    """Coreless motors this size sit in roughly 5-25 ms."""
    assert 0.005 < spec(name).mechanical_time_constant < 0.025


def test_dcmotor_carries_no_friction_terms() -> None:
    """The nominal triple already contains gearbox loss; adding more double-counts."""
    element = dcmotor("a", "j", "XM430-W350-T")
    assert "damping" not in element.attrib
    assert "frictionloss" not in element.attrib
    assert element.get("nominal", "").startswith("12 4.1 ")


def test_backlash_respects_the_host_angle_unit() -> None:
    """A radian range emitted into a degree model would shrink the play 57x."""
    def build(angle: str | None) -> ET.Element:
        root = ET.Element("mujoco")
        if angle is not None:
            ET.SubElement(root, "compiler", {"angle": angle})
        body = ET.SubElement(ET.SubElement(root, "worldbody"), "body")
        ET.SubElement(body, "joint", {"name": "M01_a", "type": "hinge",
                                      "axis": "0 0 1"})
        assert add_backlash(root, by_pattern((r"M01_", "MX-28AT"))) == 1
        return body.findall("joint")[1]

    half = spec("MX-28AT").backlash / 2.0
    radian = float(build("radian").get("range").split()[1])
    degree = float(build("degree").get("range").split()[1])
    missing = float(build(None).get("range").split()[1])
    assert radian == pytest.approx(half)
    assert degree == pytest.approx(math.degrees(half))
    assert missing == pytest.approx(math.degrees(half))  # MuJoCo defaults to degrees


def test_unit_helpers() -> None:
    assert rpm(60.0) == pytest.approx(2.0 * math.pi)
    assert arcmin(60.0) == pytest.approx(math.radians(1.0))


def test_simulated_model_matches_the_datasheet() -> None:
    assert run_datasheet() == 0


def test_critical_damping_survives_the_armature() -> None:
    assert run_damping() == 0
