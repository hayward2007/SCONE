"""Run the common SCONE API/CLI against a MuJoCo controller backend."""

from __future__ import annotations

import sys
import threading
import time
from enum import Enum
from pathlib import Path

import mujoco
import mujoco.viewer

from ...cli import run_joystick_cli, run_legacy_joystick_cli
from ...main import SCONE
from .controller import MuJoCoController
from .model import DEFAULT_MODEL_PATH, load_model
from ..terrain import TerrainType


class SimulationControl(str, Enum):
    OLD = "old"
    NON_RL = "non_rl"
    RL = "rl"

    @classmethod
    def parse(cls, value: "SimulationControl | str") -> "SimulationControl":
        if isinstance(value, cls):
            return value
        try:
            return cls(value)
        except ValueError as error:
            raise ValueError(
                f"unknown simulation control {value!r}; "
                f"choose from {tuple(item.value for item in cls)}"
            ) from error


def run(
    model_path: str | Path = DEFAULT_MODEL_PATH,
    *,
    profile: str = "standard",
    floating_base: bool = True,
    terrain: TerrainType | str = TerrainType.FLAT,
    terrain_seed: int = 7,
    control: SimulationControl | str = SimulationControl.NON_RL,
    checkpoint: str | Path | None = None,
    rl_device: str = "auto",
    verbose: bool = False,
) -> None:
    """Open one viewer while terminal input drives the shared robot API.

    MuJoCo receives no robot keyboard callback. The selected locomotion path
    consumes the common terminal ``[vx, vy, yaw_rate]`` joystick command.
    """

    selected_terrain = TerrainType.parse(terrain)
    selected_control = SimulationControl.parse(control)
    if selected_control is SimulationControl.RL:
        if checkpoint is None:
            raise ValueError("RL simulation control requires a PPO checkpoint")
        from ...rl.joystick_control import run_rl_joystick

        run_rl_joystick(
            checkpoint,
            model_path=model_path,
            terrain=selected_terrain,
            terrain_seed=terrain_seed,
            device=rl_device,
        )
        return

    model = load_model(
        model_path,
        floating_base=floating_base,
        terrain=selected_terrain,
        terrain_seed=terrain_seed,
    )
    data = mujoco.MjData(model)
    controller = MuJoCoController(model, data, verbose=verbose)
    robot = SCONE(controller, profile=profile)
    stop_event = threading.Event()
    cli_errors: list[BaseException] = []

    def control_worker() -> None:
        try:
            robot.initialize()
            if selected_control is SimulationControl.OLD:
                run_legacy_joystick_cli(robot, stop_event=stop_event)
            else:
                run_joystick_cli(robot, stop_event=stop_event)
            robot.close()
        except BaseException as error:
            cli_errors.append(error)
        finally:
            stop_event.set()

    worker = threading.Thread(
        target=control_worker,
        name="scone-command-interpreter",
        daemon=True,
    )

    print(
        "\n[SIM] MuJoCo uses the velocity joystick in the terminal "
        f"(control={selected_control.value})."
    )
    print(f"[SIM] Terrain: {selected_terrain.value} (seed={terrain_seed})")
    print("[SIM] The viewer has no SCONE-specific keyboard mapping.\n")
    try:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            viewer.cam.lookat[:] = model.stat.center
            viewer.cam.distance = model.stat.extent * 2.2
            viewer.opt.label = mujoco.mjtLabel.mjLABEL_JOINT
            worker.start()

            timestep = model.opt.timestep
            while viewer.is_running() and not stop_event.is_set():
                frame_start = time.perf_counter()
                with controller.lock:
                    controller.update(timestep)
                    mujoco.mj_step(model, data)
                viewer.sync()
                remaining = timestep - (time.perf_counter() - frame_start)
                if remaining > 0:
                    time.sleep(remaining)
    except RuntimeError as error:
        if sys.platform == "darwin" and "mjpython" not in Path(sys.executable).name:
            raise RuntimeError(
                "On macOS launch the SCONE CLI with `mjpython SCONE.py`."
            ) from error
        raise
    finally:
        stop_event.set()
        if worker.is_alive():
            worker.join()
        controller.close()

    if cli_errors:
        raise cli_errors[0]


__all__ = ["SimulationControl", "run"]
