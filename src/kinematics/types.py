"""Value types shared by leg and whole-robot kinematics."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike, NDArray


Vector3 = NDArray[np.float64]
Matrix3 = NDArray[np.float64]


def vector3(value: ArrayLike, *, name: str = "vector") -> Vector3:
    array = np.asarray(value, dtype=np.float64)
    if array.shape != (3,):
        raise ValueError(f"{name} must have shape (3,), got {array.shape}")
    return array


@dataclass(frozen=True)
class JointAngles:
    """Three leg-joint angles in radians around raw position 2048.

    ``body``, ``stage1``, and ``stage2`` correspond to motor IDs
    ``leg``, ``leg + 6``, and ``leg + 12`` respectively. Zero radians is the
    hardware center value, 2048 (180 degrees in the existing controller API).
    """

    body: float
    stage1: float
    stage2: float

    @classmethod
    def from_array(cls, value: ArrayLike) -> "JointAngles":
        array = vector3(value, name="joint angles")
        return cls(*(float(item) for item in array))

    @classmethod
    def from_motor_degrees(cls, value: ArrayLike) -> "JointAngles":
        degrees = vector3(value, name="motor degrees")
        return cls.from_array(np.radians(degrees - 180.0))

    @classmethod
    def from_raw(cls, value: ArrayLike) -> "JointAngles":
        raw = vector3(value, name="raw motor positions")
        return cls.from_motor_degrees(raw / 4096.0 * 360.0)

    def as_array(self) -> Vector3:
        return np.array([self.body, self.stage1, self.stage2], dtype=np.float64)

    def as_motor_degrees(self) -> Vector3:
        return np.degrees(self.as_array()) + 180.0

    def as_raw(self) -> NDArray[np.int64]:
        return np.rint(self.as_motor_degrees() / 360.0 * 4096.0).astype(np.int64)


@dataclass(frozen=True)
class EndEffectorPose:
    """Position and orientation of a tire frame in body or world coordinates."""

    position: Vector3
    rotation: Matrix3
    frame: str


@dataclass(frozen=True)
class IKResult:
    angles: JointAngles
    residual: float
    iterations: int
    converged: bool


class IKConvergenceError(RuntimeError):
    def __init__(self, leg: int, result: IKResult) -> None:
        super().__init__(
            f"leg {leg} IK did not converge after {result.iterations} iterations; "
            f"residual={result.residual:.6g} m"
        )
        self.leg = leg
        self.result = result


__all__ = [
    "EndEffectorPose",
    "IKConvergenceError",
    "IKResult",
    "JointAngles",
    "Matrix3",
    "Vector3",
    "vector3",
]
