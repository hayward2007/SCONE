"""Non-mutating discovery for the physical SCONE controller bus."""

from __future__ import annotations

import glob
import os
from dataclasses import dataclass

from .config import DEFAULT_BAUDRATE, DEFAULT_DEVICE_NAME


@dataclass(frozen=True)
class HardwareProbe:
    available: bool
    device_name: str | None = None
    detail: str = ""


def candidate_device_names() -> tuple[str, ...]:
    configured = os.getenv("SCONE_DEVICE")
    patterns = (
        "/dev/cu.usbserial-*",
        "/dev/ttyUSB*",
        "/dev/ttyACM*",
    )
    devices: list[str] = []
    if configured:
        devices.append(configured)
    devices.extend(path for pattern in patterns for path in sorted(glob.glob(pattern)))
    if DEFAULT_DEVICE_NAME not in devices:
        devices.append(DEFAULT_DEVICE_NAME)
    return tuple(dict.fromkeys(devices))


def _has_scone_actuator(port, packet_handlers: tuple[tuple[float, object], ...]) -> bool:
    # Ping representative IDs without changing torque, modes, or positions.
    for _protocol, handler in packet_handlers:
        for motor_id in (1, 7, 13):
            try:
                _model_number, comm_result, device_error = handler.ping(port, motor_id)
            except Exception:
                continue
            if comm_result == 0 and device_error == 0:
                return True
    return False


def discover_hardware(baudrate: int = DEFAULT_BAUDRATE) -> HardwareProbe:
    try:
        from dynamixel_sdk import PacketHandler, PortHandler
    except ImportError as error:
        return HardwareProbe(False, detail=f"dynamixel_sdk unavailable: {error}")

    errors: list[str] = []
    handlers = ((1.0, PacketHandler(1.0)), (2.0, PacketHandler(2.0)))
    for device_name in candidate_device_names():
        port = PortHandler(device_name)
        opened = False
        try:
            try:
                opened = bool(port.openPort())
            except Exception as error:
                errors.append(f"{device_name}: open failed ({error})")
                continue
            if not opened:
                errors.append(f"{device_name}: open failed")
                continue
            if not port.setBaudRate(baudrate):
                errors.append(f"{device_name}: baudrate failed")
                continue
            if _has_scone_actuator(port, handlers):
                return HardwareProbe(True, device_name, "DYNAMIXEL response detected")
            errors.append(f"{device_name}: no DYNAMIXEL response")
        finally:
            if opened:
                try:
                    port.closePort()
                except Exception as error:
                    errors.append(f"{device_name}: close failed ({error})")
    detail = "; ".join(errors) if errors else "no serial controller candidates"
    return HardwareProbe(False, detail=detail)


__all__ = ["HardwareProbe", "candidate_device_names", "discover_hardware"]
