"""Physical actuator metadata and lazily loaded hardware backend."""

from .actuator import Actuator, ActuatorIndex, OperatingMode, model_for_id
from .interface import ControllerProtocol

__all__ = [
    "Actuator",
    "ActuatorIndex",
    "Controller",
    "ControllerError",
    "ControllerProtocol",
    "HardwareProbe",
    "OperatingMode",
    "discover_hardware",
    "model_for_id",
]


def __getattr__(name: str):
    # ``import SCONE`` and MuJoCo-only use must not require dynamixel_sdk.
    if name in {"Controller", "ControllerError"}:
        from .controller import Controller, ControllerError

        return {"Controller": Controller, "ControllerError": ControllerError}[name]
    if name in {"HardwareProbe", "discover_hardware"}:
        from .discovery import HardwareProbe, discover_hardware

        return {
            "HardwareProbe": HardwareProbe,
            "discover_hardware": discover_hardware,
        }[name]
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
