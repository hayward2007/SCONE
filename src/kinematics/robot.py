"""Six-leg FK/IK convenience API built on :class:`LegKinematics`."""

from __future__ import annotations

from collections.abc import Mapping
from pathlib import Path

import mujoco
import numpy as np
from numpy.typing import ArrayLike, NDArray

from .leg import DEFAULT_MODEL_PATH, Frame, LegKinematics
from .types import EndEffectorPose, IKResult, JointAngles


LegAngleInput = Mapping[int, JointAngles | ArrayLike] | ArrayLike
LegTargetInput = Mapping[int, ArrayLike] | ArrayLike


class RobotKinematics:
    """FK/IK for all six independent leg branches in ``model.xml``."""

    def __init__(
        self,
        model_path: str | Path = DEFAULT_MODEL_PATH,
        *,
        end_effector_points: Mapping[int, ArrayLike] | None = None,
    ) -> None:
        self.model_path = Path(model_path).expanduser().resolve()
        self.model = mujoco.MjModel.from_xml_path(str(self.model_path))
        self.data = mujoco.MjData(self.model)
        points = end_effector_points or {}
        self.legs = {
            leg: LegKinematics(
                leg,
                self.model_path,
                end_effector_point=points.get(leg, (0.0, 0.0, 0.0)),
                model=self.model,
                data=self.data,
            )
            for leg in range(1, 7)
        }

    @staticmethod
    def _angle_map(value: LegAngleInput) -> dict[int, JointAngles]:
        if isinstance(value, Mapping):
            missing = set(range(1, 7)) - set(value)
            if missing:
                raise ValueError(f"joint angle mapping is missing legs {sorted(missing)}")
            return {
                leg: item if isinstance(item, JointAngles) else JointAngles.from_array(item)
                for leg, item in value.items()
                if leg in range(1, 7)
            }

        array = np.asarray(value, dtype=np.float64)
        if array.shape == (6, 3):
            return {
                leg: JointAngles.from_array(array[leg - 1])
                for leg in range(1, 7)
            }
        if array.shape == (18,):
            return {
                leg: JointAngles(
                    float(array[leg - 1]),
                    float(array[leg + 5]),
                    float(array[leg + 11]),
                )
                for leg in range(1, 7)
            }
        raise ValueError(
            "joint angles must be a leg mapping, shape (6, 3), or actuator-order shape (18,)"
        )

    @staticmethod
    def _target_map(value: LegTargetInput) -> dict[int, NDArray[np.float64]]:
        if isinstance(value, Mapping):
            missing = set(range(1, 7)) - set(value)
            if missing:
                raise ValueError(f"target mapping is missing legs {sorted(missing)}")
            return {
                leg: np.asarray(item, dtype=np.float64)
                for leg, item in value.items()
                if leg in range(1, 7)
            }
        array = np.asarray(value, dtype=np.float64)
        if array.shape != (6, 3):
            raise ValueError("targets must be a leg mapping or have shape (6, 3)")
        return {leg: array[leg - 1] for leg in range(1, 7)}

    def forward(
        self, angles: LegAngleInput, *, frame: Frame = "body"
    ) -> dict[int, EndEffectorPose]:
        """Calculate all six tire poses.

        Arrays may be shaped ``(6, 3)`` by leg or ``(18,)`` in actuator-ID
        order. Mapping keys are leg numbers 1..6.
        """

        by_leg = self._angle_map(angles)
        return {
            leg: self.legs[leg].forward(by_leg[leg], frame=frame)
            for leg in range(1, 7)
        }

    fk = forward

    def forward_motor_degrees(
        self, motor_degrees: ArrayLike, *, frame: Frame = "body"
    ) -> dict[int, EndEffectorPose]:
        degrees = np.asarray(motor_degrees, dtype=np.float64)
        if degrees.shape not in ((6, 3), (18,)):
            raise ValueError("motor_degrees must have shape (6, 3) or (18,)")
        return self.forward(np.radians(degrees - 180.0), frame=frame)

    def forward_raw(
        self, raw_positions: ArrayLike, *, frame: Frame = "body"
    ) -> dict[int, EndEffectorPose]:
        raw = np.asarray(raw_positions, dtype=np.float64)
        if raw.shape not in ((6, 3), (18,)):
            raise ValueError("raw_positions must have shape (6, 3) or (18,)")
        return self.forward_motor_degrees(raw / 4096.0 * 360.0, frame=frame)

    def inverse(
        self,
        targets: LegTargetInput,
        *,
        initial_angles: LegAngleInput | None = None,
        frame: Frame = "body",
        **solver_options,
    ) -> dict[int, IKResult]:
        """Solve each independent leg IK and return results keyed by leg."""

        target_by_leg = self._target_map(targets)
        initial_by_leg = (
            {leg: JointAngles(0.0, 0.0, 0.0) for leg in range(1, 7)}
            if initial_angles is None
            else self._angle_map(initial_angles)
        )
        return {
            leg: self.legs[leg].inverse(
                target_by_leg[leg],
                initial_angles=initial_by_leg[leg],
                frame=frame,
                **solver_options,
            )
            for leg in range(1, 7)
        }

    ik = inverse

    @staticmethod
    def results_as_actuator_radians(
        results: Mapping[int, IKResult]
    ) -> NDArray[np.float64]:
        """Convert six IK results to motor-ID order 1..18."""

        output = np.empty(18, dtype=np.float64)
        for leg in range(1, 7):
            angles = results[leg].angles
            output[leg - 1] = angles.body
            output[leg + 5] = angles.stage1
            output[leg + 11] = angles.stage2
        return output

    @classmethod
    def results_as_motor_degrees(
        cls, results: Mapping[int, IKResult]
    ) -> NDArray[np.float64]:
        return np.degrees(cls.results_as_actuator_radians(results)) + 180.0

    @classmethod
    def results_as_raw(
        cls, results: Mapping[int, IKResult]
    ) -> NDArray[np.int64]:
        degrees = cls.results_as_motor_degrees(results)
        return np.rint(degrees / 360.0 * 4096.0).astype(np.int64)


SCONEKinematics = RobotKinematics


__all__ = ["RobotKinematics", "SCONEKinematics"]
