"""Minimal public API example for a physical SCONE controller."""

from SCONE import SCONE
from src.hardware import Controller, discover_hardware


def main() -> None:
    probe = discover_hardware()
    if not probe.available or probe.device_name is None:
        raise SystemExit(f"SCONE hardware is unavailable: {probe.detail}")

    with SCONE(Controller(probe.device_name), profile="standard") as robot:
        robot.forward()


if __name__ == "__main__":
    main()
