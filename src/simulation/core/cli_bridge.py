"""Run the common SCONE API/CLI against a MuJoCo controller backend."""

from __future__ import annotations

import sys
import threading
import time
from collections.abc import Sequence
from enum import Enum
from pathlib import Path

import mujoco
import mujoco.viewer

from ...cli import run_joystick_cli, run_legacy_joystick_cli
from ...locomotion import GaitConfig
from ...main import SCONE
from ...rl.stance import SPORT_STANDING_DEGREES
from .controller import MuJoCoController
from .model import DEFAULT_MODEL_PATH, load_model
from .viewer import configure_simulation_viewer
from ..terrain import TerrainType


# The high Standard stance reaches the edge of its horizontal IK workspace at
# the shared 70 mm stride. Keep this conservative override simulation-local.
NON_RL_SIMULATION_GAIT_CONFIG = GaitConfig(max_stride=0.050)


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
    rl_standing_pose_degrees: Sequence[float] = SPORT_STANDING_DEGREES,
    rl_reference_motion: str = "non_rl",
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
            standing_pose_degrees=rl_standing_pose_degrees,
            reference_motion=rl_reference_motion,
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
                run_joystick_cli(
                    robot,
                    stop_event=stop_event,
                    gait_config=NON_RL_SIMULATION_GAIT_CONFIG,
                    # The loaded Standard pose sags under gravity before the
                    # first frame.  Re-centering IK on that edge-of-workspace
                    # transient makes legs 2/5 fail immediately.  Simulation
                    # instead keeps the selected, known-solvable profile pose.
                    calibrate_from_controller=False,
                )
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
        "\n[SIM] MuJoCo uses the terminal controller "
        f"(control={selected_control.value})."
    )
    print(f"[SIM] Terrain: {selected_terrain.value} (seed={terrain_seed})")
    print("[SIM] The viewer has no SCONE-specific keyboard mapping.\n")
    try:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            root_body_id = mujoco.mj_name2id(
                model,
                mujoco.mjtObj.mjOBJ_BODY,
                "UPPER_BODY_1",
            )
            if root_body_id < 0:
                raise ValueError("simulation model is missing UPPER_BODY_1")
            configure_simulation_viewer(
                viewer,
                model,
                data,
                tracking_body_id=root_body_id,
            )
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


__all__ = ["NON_RL_SIMULATION_GAIT_CONFIG", "SimulationControl", "run"]
