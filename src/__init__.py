"""SCONE package root exports.

Keep the package import surface lightweight and lazy to avoid circular imports
between the package root, locomotion, and simulation layers.
"""

from __future__ import annotations

__all__ = [
    "SCONE",
    "Actuator",
    "Controller",
    "Mode",
    "Walk",
    "Drive",
    "Climb",
    "MuJoCoController",
    "MotionRunner",
]


def __getattr__(name: str):
    if name == "SCONE":
        from .SCONE import SCONE
        return SCONE
    if name in {"Actuator", "Controller"}:
        from .hardware import Actuator, Controller
        return {"Actuator": Actuator, "Controller": Controller}[name]
    if name in {"Mode", "Walk", "Drive", "Climb"}:
        from .locomotion import Climb, Drive, Mode, Walk
        return {"Mode": Mode, "Walk": Walk, "Drive": Drive, "Climb": Climb}[name]
    if name in {"MuJoCoController", "MotionRunner"}:
        from .simulation import MotionRunner, MuJoCoController
        return {"MuJoCoController": MuJoCoController, "MotionRunner": MotionRunner}[name]
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
