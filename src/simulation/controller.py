"""DYNAMIXEL-like controller backed by MuJoCo dcmotor actuators."""

from __future__ import annotations

import math
import threading
from collections.abc import Sequence

import mujoco
import numpy as np

from ..core.actuator import Actuator
from .pid import DCMotorPID, default_gains_for_motor_id, spec_for_motor_id


class MuJoCoController:
    """Implement the existing ``Controller`` API without opening a serial port.

    The providers in :mod:`src.provider` can use this class unchanged. Every
    actuator follows the same raw-position direction: 0 is the CCW limit, 2048
    is center, and 4096 is the CW limit. Profile velocity and acceleration
    generate a setpoint exactly as before; that setpoint is then tracked by a
    :class:`~src.simulation.pid.DCMotorPID` per joint, whose output voltage
    drives the model's ``<dcmotor>`` actuators (see model.xml and pid.py).
    """

    _POSITION_MODE = Actuator.model.XM.operating_mode.position
    _EXTENDED_POSITION_MODE = Actuator.model.XM.operating_mode.extended_position
    _VELOCITY_MODE = Actuator.model.XM.operating_mode.velocity
    _CENTER_RAW = 2048.0
    _RAW_PER_REVOLUTION = 4096.0
    _MX_SPEED_UNIT_RPM = 0.114
    _XM_SPEED_UNIT_RPM = 0.229
    _XM_ACCELERATION_UNIT_RPM_PER_MINUTE = 214.577

    # A rough "standing" reference pose (degrees), used only to seed the
    # simulation's initial qpos instead of the raw CAD rest pose (every
    # joint at raw 2048). It does not need to match any particular SCONE
    # profile exactly -- see _seed_stable_pose.
    _STANDING_UPPER_DEGREES = (135.0, 135.0, 180.0, 180.0, 225.0, 225.0)
    _STANDING_MIDDLE_DEGREES = 240.0
    _STANDING_LOWER_DEGREES = 255.0
    _STANDING_POSE_DEGREES = (
        _STANDING_UPPER_DEGREES
        + (_STANDING_MIDDLE_DEGREES,) * 6
        + (_STANDING_LOWER_DEGREES,) * 6
    )

    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        *,
        verbose: bool = True,
        standing_pose_degrees: Sequence[float] | None = None,
    ) -> None:
        if model.nu < len(Actuator.index):
            raise ValueError(
                f"SCONE requires 18 actuators, but the model contains {model.nu}."
            )

        self.model = model
        self.data = data
        self.verbose = verbose
        self.lock = threading.RLock()

        self._actuator_ids = np.empty(19, dtype=int)
        self._joint_ids = np.empty(19, dtype=int)
        self._qpos_addresses = np.empty(19, dtype=int)
        self._dof_addresses = np.empty(19, dtype=int)
        for motor_id in Actuator.index:
            actuator_id = self._find_actuator(motor_id)
            joint_id = int(model.actuator_trnid[actuator_id, 0])
            self._actuator_ids[motor_id] = actuator_id
            self._joint_ids[motor_id] = joint_id
            self._qpos_addresses[motor_id] = int(model.jnt_qposadr[joint_id])
            self._dof_addresses[motor_id] = int(model.jnt_dofadr[joint_id])

        # Each dcmotor actuator is voltage-driven (see model.xml); this PID
        # supplies the outer position/velocity loop that turns a target
        # angle into that voltage. See src/simulation/pid.py.
        self._pid: list[DCMotorPID | None] = [None] * 19
        for motor_id in Actuator.index:
            kp, kd = default_gains_for_motor_id(motor_id)
            self._pid[motor_id] = DCMotorPID(spec_for_motor_id(motor_id), kp, kd)

        self._torque_enabled = np.ones(19, dtype=bool)
        self._mode = np.full(19, self._POSITION_MODE, dtype=int)
        self._target = np.zeros(19)
        self._setpoint = np.zeros(19)
        self._setpoint_velocity = np.zeros(19)
        self._velocity_command = np.zeros(19)
        self._profile_velocity = np.full(19, math.inf)
        self._profile_acceleration = np.full(19, math.inf)

        seed_pose = np.asarray(
            self._STANDING_POSE_DEGREES
            if standing_pose_degrees is None
            else standing_pose_degrees,
            dtype=np.float64,
        )
        if seed_pose.shape != (len(Actuator.index),):
            raise ValueError(
                "standing_pose_degrees must contain one value for each of "
                f"the {len(Actuator.index)} actuators"
            )
        self._seed_stable_pose(model, data, seed_pose)
        mujoco.mj_forward(model, data)
        for motor_id in Actuator.index:
            current = self._joint_position(motor_id)
            self._target[motor_id] = current
            self._setpoint[motor_id] = current
            self._pid[motor_id].reset(current)
            data.ctrl[self._actuator_ids[motor_id]] = 0.0

        self._log("MuJoCo controller ready (18 actuator mapping validated)")

    def _seed_stable_pose(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        pose_degrees: np.ndarray,
    ) -> None:
        """Start the simulation already standing, not at the raw CAD rest pose.

        Spawning at the CAD rest pose (every joint at raw 2048, an unsupported
        folded configuration) and then letting ``home()`` cut torque on all 18
        joints at once for its initial mode-switch causes the structure to
        collapse under gravity before the profile-velocity ramp can recover --
        confirmed by direct measurement to be a startup-transient problem, not
        a torque/kp shortfall. Seeding a roughly-standing pose here (and, if a
        root freejoint exists, lifting the whole body so the lowest contact
        point rests on the floor rather than dangling or penetrating) avoids
        that fall. ``home()`` still runs its own transition to the exact
        target profile pose afterward.
        """

        for motor_id in Actuator.index:
            raw = self.degrees_to_raw(motor_id, pose_degrees[motor_id - 1])
            data.qpos[self._qpos_addresses[motor_id]] = self.raw_to_radians(raw)

        freejoint_id = self._find_root_freejoint(model)
        if freejoint_id is None:
            return
        floor_z = self._find_floor_height(model)
        if floor_z is None:
            return

        mujoco.mj_forward(model, data)
        lowest = self._lowest_contact_point(model, data)
        if lowest is None:
            return
        qpos_adr = int(model.jnt_qposadr[freejoint_id])
        clearance = 0.002
        data.qpos[qpos_adr + 2] += (floor_z - lowest) + clearance

    @staticmethod
    def _find_root_freejoint(model: mujoco.MjModel) -> int | None:
        for joint_id in range(model.njnt):
            if model.jnt_type[joint_id] == mujoco.mjtJoint.mjJNT_FREE:
                return joint_id
        return None

    @staticmethod
    def _find_floor_height(model: mujoco.MjModel) -> float | None:
        geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "simulation_floor")
        if geom_id < 0:
            return None
        return float(model.geom_pos[geom_id][2])

    @staticmethod
    def _lowest_contact_point(model: mujoco.MjModel, data: mujoco.MjData) -> float | None:
        lowest = None
        for geom_id in range(model.ngeom):
            if model.geom_contype[geom_id] == 0:
                continue
            if model.geom_type[geom_id] != mujoco.mjtGeom.mjGEOM_MESH:
                continue
            mesh_id = int(model.geom_dataid[geom_id])
            if mesh_id < 0:
                continue
            vert_adr = int(model.mesh_vertadr[mesh_id])
            vert_num = int(model.mesh_vertnum[mesh_id])
            verts = model.mesh_vert[vert_adr : vert_adr + vert_num]
            rotation = data.geom_xmat[geom_id].reshape(3, 3)
            world = verts @ rotation.T + data.geom_xpos[geom_id]
            zmin = float(world[:, 2].min())
            if lowest is None or zmin < lowest:
                lowest = zmin
        return lowest

    def _find_actuator(self, motor_id: int) -> int:
        prefix = f"A{motor_id:02d}_"
        matches = []
        for actuator_id in range(self.model.nu):
            name = mujoco.mj_id2name(
                self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_id
            )
            if name and name.startswith(prefix):
                matches.append(actuator_id)
        if len(matches) != 1:
            raise ValueError(
                f"Expected exactly one actuator whose name starts with {prefix!r}; "
                f"found {len(matches)}."
            )
        return matches[0]

    def _log(self, message: str) -> None:
        if self.verbose:
            print(f"[SIM CONTROLLER] {message}")

    @staticmethod
    def _rpm_to_radians_per_second(rpm: float) -> float:
        return rpm * 2.0 * math.pi / 60.0

    def _speed_to_radians_per_second(self, motor_id: int, speed: float) -> float:
        unit = self._MX_SPEED_UNIT_RPM if motor_id <= 6 else self._XM_SPEED_UNIT_RPM
        return self._rpm_to_radians_per_second(speed * unit)

    def _acceleration_to_radians_per_second_squared(self, value: float) -> float:
        # ROBOTIS specifies the X-series profile-acceleration unit in rev/min^2.
        return value * self._XM_ACCELERATION_UNIT_RPM_PER_MINUTE * 2.0 * math.pi / 3600.0

    def _joint_position(self, motor_id: int) -> float:
        return float(self.data.qpos[self._qpos_addresses[motor_id]])

    def _joint_velocity(self, motor_id: int) -> float:
        return float(self.data.qvel[self._dof_addresses[motor_id]])

    @staticmethod
    def _id_label(motor_id: int) -> str:
        return f"ID {motor_id:02d}"

    @classmethod
    def degrees_to_raw(cls, motor_id: int, degrees: float) -> int:
        # motor_id remains part of the public signature for compatibility, but
        # direction no longer depends on odd/even IDs.
        del motor_id
        return int(degrees / 360.0 * cls._RAW_PER_REVOLUTION)

    @classmethod
    def raw_to_radians(cls, raw_position: float) -> float:
        return (raw_position - cls._CENTER_RAW) * 2.0 * math.pi / cls._RAW_PER_REVOLUTION

    @classmethod
    def radians_to_raw(cls, radians: float) -> int:
        raw = cls._CENTER_RAW + radians * cls._RAW_PER_REVOLUTION / (2.0 * math.pi)
        return int(round(raw))

    def set_mode(self, id: int, mode: int) -> None:
        if id not in Actuator.lower_index:
            return
        if mode not in (
            self._VELOCITY_MODE,
            self._POSITION_MODE,
            self._EXTENDED_POSITION_MODE,
        ):
            raise ValueError(f"Unsupported simulated operating mode: {mode}")

        with self.lock:
            current = self._joint_position(id)
            self._mode[id] = mode
            self._setpoint[id] = current
            self._target[id] = current
            self._setpoint_velocity[id] = 0.0
            self._velocity_command[id] = 0.0
            self._pid[id].reset(current)
            self.data.ctrl[self._actuator_ids[id]] = 0.0
        self._log(f"{self._id_label(id)}: operating mode -> {mode}")

    def set_all_mode(self, mode: int) -> None:
        for motor_id in Actuator.lower_index:
            self.set_mode(motor_id, mode)

    def get_mode(self, id: int) -> int:
        return int(self._mode[id])

    def set_speed(
        self,
        id: int,
        speed: int,
        address: int = Actuator.model.XM.address.profile_velocity,
    ) -> None:
        with self.lock:
            if address == Actuator.model.XM.address.goal_velocity:
                self._velocity_command[id] = self._speed_to_radians_per_second(
                    id, speed
                )
                self._log(
                    f"{self._id_label(id)}: goal velocity -> "
                    f"{self._velocity_command[id]:+.3f} rad/s"
                )
                return

            # A DYNAMIXEL profile velocity of zero means that the profile is not
            # velocity-limited. Preserve that behavior in the simulation.
            self._profile_velocity[id] = (
                math.inf
                if speed == 0
                else abs(self._speed_to_radians_per_second(id, speed))
            )
        self._log(f"{self._id_label(id)}: profile velocity -> {speed}")

    def set_all_speed(self, speed: int) -> None:
        for motor_id in Actuator.index:
            self.set_speed(motor_id, speed)

    def set_acceleration(self, id: int, acceleration: int) -> None:
        if id <= 6:
            return
        with self.lock:
            self._profile_acceleration[id] = (
                math.inf
                if acceleration == 0
                else abs(self._acceleration_to_radians_per_second_squared(acceleration))
            )
        self._log(f"{self._id_label(id)}: profile acceleration -> {acceleration}")

    def set_torque(self, id: int, torque: int) -> None:
        enabled = torque == Actuator.torque.on
        actuator_id = self._actuator_ids[id]
        with self.lock:
            self._torque_enabled[id] = enabled
            if enabled:
                current = self._joint_position(id)
                self._setpoint[id] = current
                self._target[id] = current
                self._setpoint_velocity[id] = 0.0
                self._pid[id].reset(current)
                self.data.ctrl[actuator_id] = 0.0
            else:
                # Zero terminal voltage: the dcmotor's own back-EMF brakes
                # the joint instead of it going instantly limp, and update()
                # keeps re-applying 0 V every step while disabled.
                self.data.ctrl[actuator_id] = 0.0
        self._log(f"{self._id_label(id)}: torque {'on' if enabled else 'off'}")

    def set_all_torque(self, torque: int) -> None:
        for motor_id in Actuator.index:
            self.set_torque(motor_id, torque)

    def enable_torque(self) -> None:
        self.set_all_torque(Actuator.torque.on)

    def disable_torque(self) -> None:
        self.set_all_torque(Actuator.torque.off)

    def set_position(self, id: int, position: float) -> None:
        raw_position = self.degrees_to_raw(id, position)
        self._set_raw_position(id, raw_position)

    def set_raw_position(self, id: int, position: int) -> None:
        self._set_raw_position(id, position)

    def _set_raw_position(self, id: int, raw_position: float) -> None:
        target = self.raw_to_radians(raw_position)
        with self.lock:
            self._target[id] = target
        self._log(f"{self._id_label(id)}: target -> {target:+.3f} rad (raw {raw_position:g})")

    def get_position(self, id: int) -> int:
        with self.lock:
            result = self.radians_to_raw(self._joint_position(id))
        self._log(f"{self._id_label(id)}: position -> raw {result}")
        return result

    def update(self, timestep: float) -> None:
        """Advance profile generators and write the next MuJoCo controls."""

        with self.lock:
            for motor_id in Actuator.index:
                actuator_id = self._actuator_ids[motor_id]
                if not self._torque_enabled[motor_id]:
                    self.data.ctrl[actuator_id] = 0.0
                    continue

                if self._mode[motor_id] == self._VELOCITY_MODE:
                    requested_velocity = self._velocity_command[motor_id]
                else:
                    error = self._target[motor_id] - self._setpoint[motor_id]
                    requested_velocity = error / timestep
                    max_velocity = self._profile_velocity[motor_id]
                    requested_velocity = float(
                        np.clip(requested_velocity, -max_velocity, max_velocity)
                    )

                acceleration = self._profile_acceleration[motor_id]
                velocity_delta = requested_velocity - self._setpoint_velocity[motor_id]
                max_velocity_delta = acceleration * timestep
                velocity_delta = float(
                    np.clip(velocity_delta, -max_velocity_delta, max_velocity_delta)
                )
                velocity = self._setpoint_velocity[motor_id] + velocity_delta

                step = velocity * timestep
                if self._mode[motor_id] != self._VELOCITY_MODE:
                    remaining = self._target[motor_id] - self._setpoint[motor_id]
                    reaches_target = (
                        remaining == 0.0
                        or (remaining * step >= 0.0 and abs(step) >= abs(remaining))
                    )
                    if reaches_target:
                        self._setpoint[motor_id] = self._target[motor_id]
                        velocity = 0.0
                    else:
                        self._setpoint[motor_id] += step
                else:
                    self._setpoint[motor_id] += step

                self._setpoint_velocity[motor_id] = velocity

                position = self._joint_position(motor_id)
                actual_velocity = self._joint_velocity(motor_id)
                self.data.ctrl[actuator_id] = self._pid[motor_id].step(
                    timestep,
                    position,
                    actual_velocity,
                    self._setpoint[motor_id],
                    self._setpoint_velocity[motor_id],
                )

    def close(self) -> None:
        """Match the hardware controller lifecycle API."""

        self._log("MuJoCo controller closed")
