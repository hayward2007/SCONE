"""Model a DYNAMIXEL actuator in MuJoCo without double-counting gearbox loss."""

from .mjcf import add_backlash, by_pattern, dcmotor, joint_attributes
from .specs import CATALOG, DynamixelSpec, spec

__version__ = "0.1.0"
__all__ = [
    "CATALOG", "DynamixelSpec", "add_backlash", "by_pattern", "dcmotor",
    "joint_attributes", "spec",
]
