"""Run the existing SCONE motion providers against a simulated controller."""

from __future__ import annotations

import queue
import threading
import time
from collections.abc import Callable

from ..SCONE import SCONE
from ..hardware.actuator import Actuator
from .controller import MuJoCoController


class MotionRunner:
    """Serialize existing provider calls on a background worker thread."""

    _STARTING_MIDDLE_POSITION = 135

    def __init__(self, controller: MuJoCoController, profile: str = "standard") -> None:
        self.controller = controller
        if profile.lower() == "standard":
            self.operate = SCONE.Standard(controller)
        elif profile.lower() == "sport":
            self.operate = SCONE.Sport(controller)
        else:
            raise ValueError(f"Unknown SCONE profile: {profile!r}")

        self.profile = profile.lower()
        self._inspection_centered = False
        self._commands: queue.Queue[tuple[str, Callable[[], None]] | None] = queue.Queue()
        self._stop = threading.Event()
        self._worker = threading.Thread(
            target=self._work,
            name="scone-motion-worker",
            daemon=True,
        )

    @property
    def mode_name(self) -> str:
        return type(self.operate.mode).__name__

    @property
    def busy(self) -> bool:
        return self._commands.unfinished_tasks > 0

    def start(self, initialize: bool = True) -> None:
        self._worker.start()
        if initialize:
            self.submit("home", self.home)

    def submit(self, name: str, command: Callable[[], None]) -> None:
        if self._stop.is_set():
            return
        print(f"[SIM] queued: {name}")
        self._commands.put((name, command))

    def command(self, name: str) -> None:
        normalized = name.lower()
        if normalized == "home":
            self.submit("home", self.home)
            return
        if normalized in {"initial", "toggle-initial", "inspection"}:
            self.submit("toggle inspection pose", self.toggle_inspection_pose)
            return
        if normalized in {"mode", "change-mode"}:
            if self._inspection_centered:
                print("[SIM] Press I again to restore the initial pose first.")
                return
            self.submit("change mode", self.change_mode)
            return

        if self._inspection_centered:
            print("[SIM] Press I again to restore the initial pose first.")
            return

        if not hasattr(self.operate.mode, normalized):
            print(f"[SIM] {self.mode_name} mode does not implement {normalized!r}.")
            return

        def run_current_mode_movement() -> None:
            movement = getattr(self.operate.mode, normalized, None)
            if movement is None:
                print(
                    f"[SIM] {self.mode_name} mode does not implement "
                    f"{normalized!r}."
                )
                return
            movement()

        self.submit(f"{self.mode_name}.{normalized}", run_current_mode_movement)

    def _work(self) -> None:
        while not self._stop.is_set():
            item = self._commands.get()
            if item is None:
                self._commands.task_done()
                break
            name, command = item
            print(f"[SIM] running: {name}")
            try:
                command()
            except Exception as error:  # Keep the viewer alive after a provider error.
                print(f"[SIM] command failed ({name}): {error}")
            finally:
                self._commands.task_done()

    def home(self) -> None:
        """Reproduce ``SCONE.Cli.__start_position`` for the simulation.

        Unlike the real hardware sequence, this never cuts torque on all 18
        joints at once: with a floating base, that briefly left the whole
        structure unsupported and gravity settled it into a lower, sagged
        resting height that the rest of the sequence could not recover from.
        Confirmed to be this startup transient, not insufficient torque --
        the same torque budget recovers fine via toggle_inspection_pose(),
        which never touches torque at all. Mode/acceleration are still
        applied to every joint up front, matching the real sequence.
        """

        restore_velocity_mode = self.mode_name == "Drive"
        for motor_id in Actuator.Index.ALL:
            self.controller.set_mode(
                motor_id, Actuator.Model.XM.OperatingMode.POSITION
            )
            self.controller.set_acceleration(motor_id, 20)
        time.sleep(0.1)

        self.controller.enable_torque()
        self.controller.set_all_speed(self.operate.safety_speed)

        for motor_id in Actuator.Index.MIDDLE:
            self.controller.set_position(motor_id, self._STARTING_MIDDLE_POSITION)
        time.sleep(0.5)

        for motor_id in Actuator.Index.UPPER:
            self.controller.set_position(
                motor_id, self.operate.upper_initial_position[motor_id - 1]
            )
        for motor_id in Actuator.Index.LOWER:
            self.controller.set_speed(motor_id, self.operate.boost_speed)
            self.controller.set_position(motor_id, self.operate.lower_initial_position)
        time.sleep(0.7)

        self.controller.set_all_speed(self.operate.safety_speed)
        for motor_id in Actuator.Index.MIDDLE:
            self.controller.set_position(motor_id, self.operate.middle_initial_position)
        time.sleep(1.0)
        self.controller.set_all_speed(self.operate.walking_speed)
        if restore_velocity_mode:
            self.controller.set_all_mode(Actuator.Model.XM.OperatingMode.VELOCITY)
        self._inspection_centered = False
        print(f"[SIM] home complete ({self.profile}/{self.mode_name})")

    def toggle_inspection_pose(self) -> None:
        """Toggle between raw 2048 on all motors and the profile home pose."""

        if self._inspection_centered:
            self.home()
            return

        self.controller.set_all_mode(Actuator.Model.XM.OperatingMode.POSITION)
        self.controller.enable_torque()
        self.controller.set_all_speed(self.operate.safety_speed)
        for motor_id in Actuator.Index.ALL:
            self.controller.set_raw_position(motor_id, Actuator.Position.CENTER)
        self._inspection_centered = True
        print("[SIM] inspection pose: all actuator raw positions -> 2048")

    def change_mode(self) -> None:
        previous = self.mode_name
        self.operate.mode = self.operate.mode.change_mode()
        print(f"[SIM] operating mode: {previous} -> {self.mode_name}")

    def stop(self) -> None:
        if self._stop.is_set():
            return
        self._stop.set()
        self._commands.put(None)
        if self._worker.is_alive():
            self._worker.join(timeout=8.0)
        self.controller.close()
