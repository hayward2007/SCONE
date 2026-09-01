"""DYNAMIXEL-backed implementation of the SCONE controller API."""

from __future__ import annotations

import os
import time
from collections import defaultdict
from collections.abc import Iterable, Mapping

from dynamixel_sdk import GroupSyncWrite, PacketHandler, PortHandler

from .actuator import Actuator, model_for_id
from .actuator_control_table import Register
from .config import (
    DEFAULT_BAUDRATE as _DEFAULT_BAUDRATE,
    DEFAULT_DEVICE_NAME as _DEFAULT_DEVICE_NAME,
)


class ControllerError(RuntimeError):
    pass


class Controller:
    """Control the physical SCONE DYNAMIXEL bus.

    All register selection goes through :mod:`actuator_control_table`. Public
    motion code therefore works in terms of positions, speeds, and groups,
    without knowing protocol versions or register addresses.
    """

    DEFAULT_BAUDRATE = _DEFAULT_BAUDRATE
    DEFAULT_DEVICE_NAME = _DEFAULT_DEVICE_NAME

    def __init__(
        self,
        device_name: str | None = None,
        *,
        baudrate: int = DEFAULT_BAUDRATE,
        verbose: bool = True,
    ) -> None:
        self.device_name = device_name or os.getenv(
            "SCONE_DEVICE", self.DEFAULT_DEVICE_NAME
        )
        self.baudrate = baudrate
        self.verbose = verbose
        self._closed = False
        self._port_open = False
        self.port_handler = PortHandler(self.device_name)
        self._packet_handlers = {
            1.0: PacketHandler(1.0),
            2.0: PacketHandler(2.0),
        }

        self._log(f"opening {self.device_name}")
        try:
            opened = self.port_handler.openPort()
        except Exception as error:
            self._closed = True
            raise ControllerError(
                f"failed to open controller port: {self.device_name} ({error})"
            ) from error
        if not opened:
            self._closed = True
            raise ControllerError(f"failed to open controller port: {self.device_name}")
        self._port_open = True
        if not self.port_handler.setBaudRate(self.baudrate):
            self.close()
            raise ControllerError(f"failed to set controller baudrate: {self.baudrate}")
        self._log(f"ready at {self.baudrate:,} baud")

    def _log(self, message: str) -> None:
        if self.verbose:
            print(f"[HARDWARE] {message}")

    @staticmethod
    def _validate_id(motor_id: int) -> None:
        model_for_id(motor_id)

    def _handler(self, motor_id: int):
        return self._packet_handlers[model_for_id(motor_id).protocol_version]

    def _check(self, motor_id: int, comm_result: int, device_error: int) -> None:
        handler = self._handler(motor_id)
        if comm_result != 0:
            raise ControllerError(
                f"ID {motor_id}: {handler.getTxRxResult(comm_result)}"
            )
        if device_error:
            raise ControllerError(
                f"ID {motor_id}: {handler.getRxPacketError(device_error)}"
            )

    def _write(self, motor_id: int, register: Register, value: int) -> None:
        self._validate_id(motor_id)
        method = getattr(self._handler(motor_id), f"write{register.size}ByteTxRx")
        comm_result, device_error = method(
            self.port_handler, motor_id, register.address, int(value)
        )
        self._check(motor_id, comm_result, device_error)

    def _read(self, motor_id: int, register: Register) -> int:
        self._validate_id(motor_id)
        method = getattr(self._handler(motor_id), f"read{register.size}ByteTxRx")
        value, comm_result, device_error = method(
            self.port_handler, motor_id, register.address
        )
        self._check(motor_id, comm_result, device_error)
        return int(value)

    def _sync_write(self, values: Mapping[int, int], register_name: str) -> None:
        """Write one logical register to multiple motors, grouped by model."""

        grouped: dict[tuple[float, Register], dict[int, int]] = defaultdict(dict)
        for motor_id, value in values.items():
            model = model_for_id(motor_id)
            register = getattr(model.table, register_name)
            if register is None:
                raise ControllerError(
                    f"{model.name} does not support register {register_name!r}"
                )
            grouped[(model.protocol_version, register)][motor_id] = int(value)

        for (protocol, register), group in grouped.items():
            writer = GroupSyncWrite(
                self.port_handler,
                self._packet_handlers[protocol],
                register.address,
                register.size,
            )
            try:
                mask = (1 << (register.size * 8)) - 1
                for motor_id, value in group.items():
                    payload = (value & mask).to_bytes(register.size, "little")
                    if not writer.addParam(motor_id, payload):
                        raise ControllerError(
                            f"failed to queue sync write for actuator ID {motor_id}"
                        )
                comm_result = writer.txPacket()
                if comm_result != 0:
                    handler = self._packet_handlers[protocol]
                    raise ControllerError(handler.getTxRxResult(comm_result))
            finally:
                writer.clearParam()

    def set_mode(self, motor_id: int, mode: int) -> None:
        table = model_for_id(motor_id).table
        if table.operating_mode is None:
            return
        self.set_torque(motor_id, Actuator.Torque.OFF)
        self._write(motor_id, table.operating_mode, mode)
        self.set_torque(motor_id, Actuator.Torque.ON)
        self._log(f"ID {motor_id:02d}: operating mode -> {mode}")

    def set_all_mode(self, mode: int) -> None:
        # Only the distal wheel motors switch between position and velocity.
        for motor_id in Actuator.Index.LOWER:
            self.set_mode(motor_id, mode)

    def get_mode(self, motor_id: int) -> int | None:
        register = model_for_id(motor_id).table.operating_mode
        return None if register is None else self._read(motor_id, register)

    def set_speed(self, motor_id: int, speed: int) -> None:
        table = model_for_id(motor_id).table
        register = table.moving_speed or table.profile_velocity
        if register is None:
            raise ControllerError(f"ID {motor_id} has no profile speed register")
        self._write(motor_id, register, speed)

    def set_speeds(self, speeds: Mapping[int, int]) -> None:
        mx = {
            motor_id: value
            for motor_id, value in speeds.items()
            if motor_id in Actuator.Index.UPPER
        }
        xm = {
            motor_id: value
            for motor_id, value in speeds.items()
            if motor_id in Actuator.Index.XM
        }
        if mx:
            self._sync_write(mx, "moving_speed")
        if xm:
            self._sync_write(xm, "profile_velocity")

    def set_all_speed(self, speed: int) -> None:
        self.set_speeds({motor_id: speed for motor_id in Actuator.Index.ALL})

    def set_velocity(self, motor_id: int, velocity: int) -> None:
        register = model_for_id(motor_id).table.goal_velocity
        if register is None:
            raise ControllerError(f"ID {motor_id} does not support velocity mode")
        self._write(motor_id, register, velocity)

    def set_velocities(self, velocities: Mapping[int, int]) -> None:
        self._sync_write(velocities, "goal_velocity")

    def set_acceleration(self, motor_id: int, acceleration: int) -> None:
        register = model_for_id(motor_id).table.profile_acceleration
        if register is not None:
            self._write(motor_id, register, acceleration)

    def set_accelerations(self, accelerations: Mapping[int, int]) -> None:
        supported = {
            motor_id: value
            for motor_id, value in accelerations.items()
            if model_for_id(motor_id).table.profile_acceleration is not None
        }
        if supported:
            self._sync_write(supported, "profile_acceleration")

    def set_torque(self, motor_id: int, torque: int) -> None:
        self._write(motor_id, model_for_id(motor_id).table.torque_enable, torque)

    def set_torques(self, motor_ids: Iterable[int], torque: int) -> None:
        self._sync_write(
            {motor_id: torque for motor_id in motor_ids}, "torque_enable"
        )

    def enable_torque(self) -> None:
        self.set_torques(Actuator.Index.ALL, Actuator.Torque.ON)

    def disable_torque(self) -> None:
        self.set_torques(Actuator.Index.ALL, Actuator.Torque.OFF)

    @staticmethod
    def degrees_to_raw(position: float) -> int:
        return int(position / 360.0 * Actuator.Position.END)

    def set_position(self, motor_id: int, position: float) -> None:
        self.set_raw_position(motor_id, self.degrees_to_raw(position))

    def set_positions(self, positions: Mapping[int, float]) -> None:
        self.set_raw_positions(
            {
                motor_id: self.degrees_to_raw(value)
                for motor_id, value in positions.items()
            }
        )

    def set_raw_position(self, motor_id: int, position: int) -> None:
        self._write(motor_id, model_for_id(motor_id).table.goal_position, position)

    def set_raw_positions(self, positions: Mapping[int, int]) -> None:
        self._sync_write(positions, "goal_position")

    def get_position(self, motor_id: int) -> int:
        value = self._read(motor_id, model_for_id(motor_id).table.present_position)
        self._log(f"ID {motor_id:02d}: position -> {value}")
        return value

    def wait_until_raw_positions(
        self,
        positions: Mapping[int, int],
        *,
        tolerance: int = 64,
        timeout: float = 4.0,
    ) -> bool:
        """Read back physical positions until every requested joint settles."""

        if tolerance < 0 or timeout <= 0.0:
            raise ValueError("tolerance must be non-negative and timeout positive")
        deadline = time.monotonic() + timeout
        while True:
            if all(
                abs(
                    self._read(
                        motor_id,
                        model_for_id(motor_id).table.present_position,
                    )
                    - int(target)
                )
                <= tolerance
                for motor_id, target in positions.items()
            ):
                return True
            if time.monotonic() >= deadline:
                return False
            time.sleep(0.02)

    def verify_drive_stage1_settings(
        self,
        *,
        profile_velocity: int,
        profile_acceleration: int,
        goal_position: int = Actuator.Position.CENTER,
        position_tolerance: int = 64,
    ) -> dict[int, dict[str, int]]:
        """Read and validate the six load-bearing stage-1 XM registers.

        This method never writes the bus.  It runs only on the physical
        backend when Drive is entered, after the posture transition has
        already waited for the centre targets.
        """

        readings: dict[int, dict[str, int]] = {}
        failures: list[str] = []
        for motor_id in Actuator.Index.MIDDLE:
            table = model_for_id(motor_id).table
            values = {
                "operating_mode": self._read(motor_id, table.operating_mode),
                "torque_enable": self._read(motor_id, table.torque_enable),
                "profile_velocity": self._read(motor_id, table.profile_velocity),
                "profile_acceleration": self._read(
                    motor_id, table.profile_acceleration
                ),
                "goal_position": self._read(motor_id, table.goal_position),
                "present_position": self._read(motor_id, table.present_position),
            }
            readings[motor_id] = values
            expected = {
                "operating_mode": int(Actuator.OperatingMode.POSITION),
                "torque_enable": int(Actuator.Torque.ON),
                "profile_velocity": int(profile_velocity),
                "profile_acceleration": int(profile_acceleration),
                "goal_position": int(goal_position),
            }
            for name, target in expected.items():
                if values[name] != target:
                    failures.append(
                        f"ID {motor_id} {name}={values[name]} expected={target}"
                    )
            if abs(values["present_position"] - goal_position) > position_tolerance:
                failures.append(
                    f"ID {motor_id} present_position={values['present_position']} "
                    f"outside {goal_position}±{position_tolerance}"
                )
        if failures:
            raise ControllerError(
                "Drive stage-1 read-back failed: " + "; ".join(failures)
            )
        return readings

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        if self._port_open:
            self.port_handler.closePort()
            self._port_open = False
        self._log("controller port closed")

    def __enter__(self) -> "Controller":
        return self

    def __exit__(self, *_: object) -> None:
        self.close()


__all__ = ["Controller", "ControllerError"]
