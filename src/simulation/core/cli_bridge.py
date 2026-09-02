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

from ...cli_i18n import Language, localize
from ...cli import (
    run_legacy_joystick_cli,
    run_tripod_gait_joystick_cli,
)
from ...hardware import Actuator
from ...locomotion import GaitConfig, SconeGaitConfig
from ...main import SCONE
from ...rl.stance import SPORT_STANDING_DEGREES
from .controller import MuJoCoController
from .model import DEFAULT_MODEL_PATH, load_model
from .scone_rolling_gait import (
    RollGaitConfig,
    run_roll_gait_joystick_cli,
)
from .stair_climber import run_scone_stair_joystick_cli
from .viewer import configure_simulation_viewer
from ..terrain import TerrainType


# Model gait tuning is deliberately opt-in after ``SCONE.initialize``.  PPO
# replay retains the dynamics, gains, and unlimited profile used in training.
TRIPOD_GAIT_SIMULATION_CONFIG = GaitConfig(
    cycle_frequency=1.0,
    step_height=0.025,
    max_stride=0.090,
    max_lateral_stride=0.070,
    ik_tolerance=1e-3,
    ik_stride_backoff_attempts=4,
)
SCONE_GAIT_SIMULATION_CONFIG = SconeGaitConfig(
    ik_tolerance=1e-3,
    ik_stride_backoff_attempts=4,
)
ROLL_GAIT_SIMULATION_CONFIG = RollGaitConfig()
# Compatibility constant for code written before continuous rotation was
# renamed from scone-gait to roll-gait.
SCONE_ROLLING_GAIT_SIMULATION_CONFIG = ROLL_GAIT_SIMULATION_CONFIG
# Compatibility constant for external code written before the gait rename.
NON_RL_SIMULATION_GAIT_CONFIG = TRIPOD_GAIT_SIMULATION_CONFIG


def configure_model_gait_controller(controller: MuJoCoController) -> None:
    """Apply measured non-RL gait tuning without changing PPO or hardware."""

    # Zero means "not profile-limited" in the DYNAMIXEL API. The MuJoCo
    # dcmotor PID, voltage, and torque limits still apply. The previous 160/50
    # profile lagged the clipped Cartesian gait by up to 52 degrees, which
    # made the sector contacts alternately push forward and backward.
    controller.set_all_speed(0)
    controller.set_accelerations(
        {motor_id: 0 for motor_id in Actuator.Index.XM}
    )
    controller.set_gait_position_stiffness(2.0)


class SimulationControl(str, Enum):
    OLD = "old"
    TRIPOD_GAIT = "tripod-gait"
    SCONE_GAIT = "scone-gait"
    ROLL_GAIT = "roll-gait"
    SCONE_STAIR = "scone-stair"
    RL = "rl"
    NON_RL = "tripod-gait"

    @classmethod
    def parse(cls, value: "SimulationControl | str") -> "SimulationControl":
        if isinstance(value, cls):
            return value
        if value == "non_rl":
            return cls.TRIPOD_GAIT
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
    control: SimulationControl | str = SimulationControl.TRIPOD_GAIT,
    checkpoint: str | Path | None = None,
    rl_device: str = "auto",
    rl_standing_pose_degrees: Sequence[float] = SPORT_STANDING_DEGREES,
    rl_reference_motion: str = "hardcoded",
    verbose: bool = False,
    language: Language | str = Language.ENGLISH,
) -> None:
    """Open one viewer while terminal input drives the shared robot API.

    MuJoCo receives no robot keyboard callback. The selected locomotion path
    consumes the common terminal ``[vx, vy, yaw_rate]`` joystick command.
    """

    selected_terrain = TerrainType.parse(terrain)
    selected_control = SimulationControl.parse(control)
    if selected_control in (SimulationControl.RL, SimulationControl.SCONE_GAIT):
        if checkpoint is None:
            raise ValueError(
                f"{selected_control.value} simulation control requires a PPO "
                "checkpoint"
            )
        from ...rl.joystick_control import run_rl_joystick

        run_rl_joystick(
            checkpoint,
            model_path=model_path,
            terrain=selected_terrain,
            terrain_seed=terrain_seed,
            device=rl_device,
            standing_pose_degrees=rl_standing_pose_degrees,
            reference_motion=rl_reference_motion,
            hybrid_scone=selected_control is SimulationControl.SCONE_GAIT,
            language=language,
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
                run_legacy_joystick_cli(
                    robot,
                    stop_event=stop_event,
                    language=language,
                )
            elif selected_control is SimulationControl.TRIPOD_GAIT:
                configure_model_gait_controller(controller)
                run_tripod_gait_joystick_cli(
                    robot,
                    stop_event=stop_event,
                    gait_config=TRIPOD_GAIT_SIMULATION_CONFIG,
                    # The loaded Standard pose sags under gravity before the
                    # first frame.  Re-centering IK on that edge-of-workspace
                    # transient makes legs 2/5 fail immediately.  Simulation
                    # instead keeps the selected, known-solvable profile pose.
                    calibrate_from_controller=False,
                    language=language,
                )
            elif selected_control is SimulationControl.ROLL_GAIT:
                run_roll_gait_joystick_cli(
                    robot,
                    stop_event=stop_event,
                    config=ROLL_GAIT_SIMULATION_CONFIG,
                    language=language,
                )
            elif selected_control is SimulationControl.SCONE_STAIR:
                run_scone_stair_joystick_cli(
                    robot,
                    terrain=selected_terrain,
                    stop_event=stop_event,
                    language=language,
                )
            else:  # pragma: no cover - SimulationControl.parse rejects this.
                raise AssertionError(
                    f"unhandled simulation control: {selected_control}"
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

    print(localize(
        language,
        "\n[SIM] Terminal control is active "
        f"(controller={selected_control.value}).",
        "\n[SIM] 터미널 제어를 시작합니다 "
        f"(제어기={selected_control.value}).",
    ))
    print(localize(
        language,
        f"[SIM] Terrain={selected_terrain.value} · seed={terrain_seed}",
        f"[SIM] 지형={selected_terrain.value} · seed={terrain_seed}",
    ))
    print(localize(
        language,
        "[SIM] Robot keys stay in the terminal; the MuJoCo viewer keeps its native controls.\n",
        "[SIM] 로봇 키는 터미널에서만 처리하며 MuJoCo viewer 키와 충돌하지 않습니다.\n",
    ))
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
                localize(
                    language,
                    "On macOS launch the SCONE CLI with `mjpython SCONE.py`.",
                    "macOS에서는 `mjpython SCONE.py`로 SCONE CLI를 실행하세요.",
                )
            ) from error
        raise
    finally:
        stop_event.set()
        if worker.is_alive():
            worker.join()
        controller.close()

    if cli_errors:
        raise cli_errors[0]


__all__ = [
    "NON_RL_SIMULATION_GAIT_CONFIG",
    "ROLL_GAIT_SIMULATION_CONFIG",
    "SCONE_GAIT_SIMULATION_CONFIG",
    "SCONE_ROLLING_GAIT_SIMULATION_CONFIG",
    "SimulationControl",
    "TRIPOD_GAIT_SIMULATION_CONFIG",
    "configure_model_gait_controller",
    "run",
]
