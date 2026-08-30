"""Model-derived forward and inverse kinematics for one SCONE leg."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Literal

import mujoco
import numpy as np
from numpy.typing import ArrayLike, NDArray

from .types import (
    EndEffectorPose,
    IKConvergenceError,
    IKResult,
    JointAngles,
    vector3,
)


Frame = Literal["body", "world"]
DEFAULT_MODEL_PATH = Path(__file__).resolve().parents[1] / "assets" / "model.xml"


class LegKinematics:
    """FK/IK for one 3-DOF leg, derived directly from MJCF.

    The default end-effector point is the origin of ``TIRE_<leg>``. Pass a
    different ``end_effector_point`` to solve for a calibrated contact point
    expressed in that tire body's local frame.
    """

    def __init__(
        self,
        leg: int,
        model_path: str | Path = DEFAULT_MODEL_PATH,
        *,
        end_effector_point: ArrayLike = (0.0, 0.0, 0.0),
        model: mujoco.MjModel | None = None,
        data: mujoco.MjData | None = None,
    ) -> None:
        if leg not in range(1, 7):
            raise ValueError(f"leg must be between 1 and 6, got {leg}")
        if (model is None) != (data is None):
            raise ValueError("model and data must be supplied together")

        self.leg = leg
        self.model_path = Path(model_path).expanduser().resolve()
        self.model = (
            mujoco.MjModel.from_xml_path(str(self.model_path))
            if model is None
            else model
        )
        self.data = mujoco.MjData(self.model) if data is None else data
        self.end_effector_point = vector3(
            end_effector_point, name="end_effector_point"
        )

        motor_ids = (leg, leg + 6, leg + 12)
        self.motor_ids = motor_ids
        joint_names = (
            f"M{motor_ids[0]:02d}_body_L{leg}",
            f"M{motor_ids[1]:02d}_stage1_L{leg}",
            f"M{motor_ids[2]:02d}_stage2_L{leg}",
        )
        self.joint_ids = np.array(
            [self._required_id(mujoco.mjtObj.mjOBJ_JOINT, name) for name in joint_names],
            dtype=np.int32,
        )
        self.qpos_addresses = np.array(
            [self.model.jnt_qposadr[joint_id] for joint_id in self.joint_ids],
            dtype=np.int32,
        )
        self.dof_addresses = np.array(
            [self.model.jnt_dofadr[joint_id] for joint_id in self.joint_ids],
            dtype=np.int32,
        )
        self.end_effector_body_id = self._required_id(
            mujoco.mjtObj.mjOBJ_BODY, f"TIRE_{leg}"
        )
        root_joint_id = self._required_id(
            mujoco.mjtObj.mjOBJ_JOINT, "root_freejoint"
        )
        self.root_body_id = int(self.model.jnt_bodyid[root_joint_id])
        self._jacobian_position = np.zeros(
            (3, self.model.nv), dtype=np.float64
        )
        self._jacobian_rotation = np.zeros(
            (3, self.model.nv), dtype=np.float64
        )

    def _required_id(self, object_type, name: str) -> int:
        object_id = mujoco.mj_name2id(self.model, object_type, name)
        if object_id < 0:
            raise ValueError(f"model is missing required object {name!r}")
        return int(object_id)

    @staticmethod
    def _angles(value: JointAngles | ArrayLike) -> JointAngles:
        return value if isinstance(value, JointAngles) else JointAngles.from_array(value)

    @staticmethod
    def _validate_frame(frame: str) -> Frame:
        if frame not in ("body", "world"):
            raise ValueError("frame must be 'body' or 'world'")
        return frame

    def _set_angles(self, angles: JointAngles | ArrayLike) -> JointAngles:
        parsed = self._angles(angles)
        self.data.qpos[self.qpos_addresses] = parsed.as_array()
        mujoco.mj_forward(self.model, self.data)
        return parsed

    def _world_point(self) -> NDArray[np.float64]:
        rotation = self.data.xmat[self.end_effector_body_id].reshape(3, 3)
        return (
            self.data.xpos[self.end_effector_body_id]
            + rotation @ self.end_effector_point
        )

    def _point_in_frame(self, frame: Frame) -> NDArray[np.float64]:
        point = self._world_point()
        if frame == "world":
            return point.copy()
        world_from_body = self.data.xmat[self.root_body_id].reshape(3, 3)
        return world_from_body.T @ (point - self.data.xpos[self.root_body_id])

    def forward(
        self,
        angles: JointAngles | ArrayLike,
        *,
        frame: Frame = "body",
    ) -> EndEffectorPose:
        """Return the tire-frame pose for three joint angles in radians."""

        frame = self._validate_frame(frame)
        self._set_angles(angles)
        end_rotation = self.data.xmat[self.end_effector_body_id].reshape(3, 3)
        if frame == "world":
            rotation = end_rotation.copy()
        else:
            world_from_body = self.data.xmat[self.root_body_id].reshape(3, 3)
            rotation = world_from_body.T @ end_rotation
        return EndEffectorPose(
            position=self._point_in_frame(frame),
            rotation=rotation,
            frame=frame,
        )

    fk = forward

    def forward_motor_degrees(
        self, motor_degrees: ArrayLike, *, frame: Frame = "body"
    ) -> EndEffectorPose:
        return self.forward(JointAngles.from_motor_degrees(motor_degrees), frame=frame)

    def forward_raw(
        self, raw_positions: ArrayLike, *, frame: Frame = "body"
    ) -> EndEffectorPose:
        return self.forward(JointAngles.from_raw(raw_positions), frame=frame)

    def jacobian(
        self,
        angles: JointAngles | ArrayLike,
        *,
        frame: Frame = "body",
    ) -> NDArray[np.float64]:
        """Return the 3x3 translational Jacobian for the end-effector point."""

        frame = self._validate_frame(frame)
        self._set_angles(angles)
        point = self._world_point()
        self._jacobian_position.fill(0.0)
        self._jacobian_rotation.fill(0.0)
        mujoco.mj_jac(
            self.model,
            self.data,
            self._jacobian_position,
            self._jacobian_rotation,
            point,
            self.end_effector_body_id,
        )
        result = self._jacobian_position[:, self.dof_addresses]
        if frame == "body":
            world_from_body = self.data.xmat[self.root_body_id].reshape(3, 3)
            result = world_from_body.T @ result
        return result.copy()

    def inverse(
        self,
        target_position: ArrayLike,
        *,
        initial_angles: JointAngles | ArrayLike = (0.0, 0.0, 0.0),
        frame: Frame = "body",
        tolerance: float = 1e-5,
        max_iterations: int = 100,
        damping: float = 1e-3,
        max_step: float = 0.25,
        joint_lower: ArrayLike = (-math.pi, -math.pi, -math.pi),
        joint_upper: ArrayLike = (math.pi, math.pi, math.pi),
        raise_on_failure: bool = False,
    ) -> IKResult:
        """Solve position IK with damped least squares.

        The three joints can solve a 3D point but cannot independently specify
        the tire orientation. ``initial_angles`` selects the preferred branch;
        passing current measured angles is recommended during real control.
        """

        frame = self._validate_frame(frame)
        target = vector3(target_position, name="target_position")
        lower = vector3(joint_lower, name="joint_lower")
        upper = vector3(joint_upper, name="joint_upper")
        if np.any(lower >= upper):
            raise ValueError("every joint_lower value must be less than joint_upper")
        if tolerance <= 0.0 or max_iterations < 1 or damping < 0.0 or max_step <= 0.0:
            raise ValueError("invalid IK solver configuration")

        angles = np.clip(self._angles(initial_angles).as_array(), lower, upper)
        best_angles = angles.copy()
        best_residual = math.inf
        iterations = 0

        for iterations in range(max_iterations + 1):
            pose = self.forward(angles, frame=frame)
            error = target - pose.position
            residual = float(np.linalg.norm(error))
            if residual < best_residual:
                best_residual = residual
                best_angles = angles.copy()
            if residual <= tolerance:
                result = IKResult(
                    JointAngles.from_array(angles), residual, iterations, True
                )
                return result
            if iterations == max_iterations:
                break

            jacobian = self.jacobian(angles, frame=frame)
            regularized = (
                jacobian @ jacobian.T + damping**2 * np.eye(3)
            )
            delta = jacobian.T @ np.linalg.solve(regularized, error)
            delta_norm = float(np.linalg.norm(delta))
            if delta_norm > max_step:
                delta *= max_step / delta_norm

            # Backtracking keeps large Newton steps from jumping to a worse
            # configuration near a folded or nearly singular posture.
            step_scale = 1.0
            accepted = False
            for _ in range(10):
                candidate = np.clip(angles + step_scale * delta, lower, upper)
                candidate_error = target - self.forward(candidate, frame=frame).position
                if np.linalg.norm(candidate_error) < residual:
                    angles = candidate
                    accepted = True
                    break
                step_scale *= 0.5
            if not accepted:
                break

        result = IKResult(
            JointAngles.from_array(best_angles),
            best_residual,
            iterations,
            False,
        )
        if raise_on_failure:
            raise IKConvergenceError(self.leg, result)
        return result

    ik = inverse


__all__ = ["DEFAULT_MODEL_PATH", "Frame", "LegKinematics"]
