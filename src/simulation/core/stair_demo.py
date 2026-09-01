"""Non-interactive MuJoCo demonstrations of two SCONE stair strategies."""

from __future__ import annotations

import math
import sys
import threading
import time
from dataclasses import dataclass
from enum import Enum
from pathlib import Path

import mujoco
import mujoco.viewer

from ...hardware import Actuator
from ...locomotion import VelocityCommand
from ...main import SCONE
from ..terrain import STAIR_PRESETS, TerrainType
from .controller import MuJoCoController
from .model import DEFAULT_MODEL_PATH, load_model
from .stair_climber import SconeStairClimber, prepare_scone_stair_pose
from .viewer import configure_simulation_viewer


class StairDemoStrategy(str, Enum):
    HARDCODED = "hardcoded"
    IMPROVED = "improved"
    COMPARE = "compare"

    @classmethod
    def parse(cls, value: "StairDemoStrategy | str") -> "StairDemoStrategy":
        if isinstance(value, cls):
            return value
        try:
            return cls(value)
        except ValueError as error:
            raise ValueError(
                f"unknown stair demo {value!r}; choose hardcoded, improved, or compare"
            ) from error


@dataclass(frozen=True)
class StairDemoResult:
    strategy: StairDemoStrategy
    terrain: TerrainType
    top_reached: bool
    time_to_top_seconds: float | None
    final_y: float
    final_z: float
    assist_entries: int


class HardcodedStairRoller:
    """Fixed six-frame rotation used as the no-feedback baseline."""

    def __init__(self, controller: MuJoCoController, *, velocity: int = 150) -> None:
        if velocity <= 0:
            raise ValueError("hardcoded stair velocity must be positive")
        self.controller = controller
        self.velocity = velocity
        self._active = False

    def update(self) -> None:
        if not self._active:
            self.controller.set_all_mode(Actuator.OperatingMode.VELOCITY)
            self._active = True
        # Stair pose advances along world +Y; this is negative raw velocity
        # before the odd/even mirrored-axis adapter.
        self.controller.set_velocities(
            self.controller.arc_wheel_velocities(-self.velocity)
        )

    def stop(self) -> None:
        self.controller.set_velocities(
            {motor_id: 0 for motor_id in Actuator.Index.LOWER}
        )


def _top_thresholds(terrain: TerrainType, start_z: float) -> tuple[float, float]:
    profile = STAIR_PRESETS[terrain]
    top_y = (
        0.35
        + sum(profile.tread_depths[:-1])
        + 0.4 * profile.tread_depths[-1]
    )
    top_z = start_z + 0.70 * profile.total_height
    return top_y, top_z


def _run_single_demo(
    strategy: StairDemoStrategy,
    terrain: TerrainType,
    *,
    model_path: str | Path,
    timeout_seconds: float,
) -> StairDemoResult:
    model = load_model(model_path, floating_base=True, terrain=terrain)
    data = mujoco.MjData(model)
    controller = MuJoCoController(model, data, verbose=False)
    robot = SCONE(controller, profile="standard")
    root_id = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_BODY,
        "UPPER_BODY_1",
    )
    if root_id < 0:
        raise ValueError("simulation model is missing UPPER_BODY_1")

    finished = threading.Event()
    worker_errors: list[BaseException] = []
    results: list[StairDemoResult] = []

    def worker() -> None:
        hardcoded: HardcodedStairRoller | None = None
        improved: SconeStairClimber | None = None
        try:
            robot.initialize()
            prepare_scone_stair_pose(robot)
            with controller.lock:
                start_z = float(data.xpos[root_id, 2])
            top_y, top_z = _top_thresholds(terrain, start_z)
            if strategy is StairDemoStrategy.HARDCODED:
                hardcoded = HardcodedStairRoller(controller)
            else:
                improved = SconeStairClimber(controller, terrain=terrain)

            with controller.lock:
                simulation_started_at = float(data.time)
            elapsed = 0.0
            control_period = 0.02
            next_control_time = simulation_started_at
            last_control_time = simulation_started_at
            reached_at: float | None = None
            while elapsed < timeout_seconds and not finished.is_set():
                with controller.lock:
                    simulation_time = float(data.time)
                    root_y = float(data.xpos[root_id, 1])
                    root_z = float(data.xpos[root_id, 2])
                elapsed = simulation_time - simulation_started_at
                if root_y >= top_y and root_z >= top_z:
                    reached_at = elapsed
                    break
                if elapsed >= timeout_seconds:
                    break
                if simulation_time + 1e-9 < next_control_time:
                    time.sleep(0.001)
                    continue

                if hardcoded is not None:
                    hardcoded.update()
                else:
                    assert improved is not None
                    control_dt = max(
                        float(model.opt.timestep),
                        simulation_time - last_control_time,
                    )
                    improved.update(
                        VelocityCommand(vy=improved.config.max_vy),
                        control_dt,
                    )
                last_control_time = simulation_time
                next_control_time = simulation_time + control_period

            if hardcoded is not None:
                hardcoded.stop()
            if improved is not None:
                improved.stop()
            # Keep the final pose visible briefly without accepting input.
            if not finished.is_set():
                time.sleep(1.5)
            with controller.lock:
                final_y = float(data.xpos[root_id, 1])
                final_z = float(data.xpos[root_id, 2])
            results.append(
                StairDemoResult(
                    strategy=strategy,
                    terrain=terrain,
                    top_reached=reached_at is not None,
                    time_to_top_seconds=reached_at,
                    final_y=final_y,
                    final_z=final_z,
                    assist_entries=0 if improved is None else improved.assist_entries,
                )
            )
        except BaseException as error:
            worker_errors.append(error)
        finally:
            finished.set()

    thread = threading.Thread(
        target=worker,
        name=f"scone-stair-demo-{strategy.value}",
        daemon=True,
    )
    print(
        f"\n[SIM DEMO] {strategy.value} / {terrain.value} — "
        "키 입력 없이 자동 실행합니다."
    )
    try:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            configure_simulation_viewer(
                viewer,
                model,
                data,
                tracking_body_id=root_id,
            )
            viewer.opt.label = mujoco.mjtLabel.mjLABEL_JOINT
            thread.start()
            timestep = float(model.opt.timestep)
            render_period = 1.0 / 60.0
            previous_wall_time = time.perf_counter()
            physics_time_debt = 0.0
            while viewer.is_running() and not finished.is_set():
                frame_start = time.perf_counter()
                physics_time_debt += min(
                    frame_start - previous_wall_time,
                    0.10,
                )
                previous_wall_time = frame_start
                while physics_time_debt >= timestep:
                    with controller.lock:
                        controller.update(timestep)
                        mujoco.mj_step(model, data)
                    physics_time_debt -= timestep
                viewer.sync()
                remaining = render_period - (time.perf_counter() - frame_start)
                if remaining > 0.0:
                    time.sleep(remaining)
    except RuntimeError as error:
        if sys.platform == "darwin" and "mjpython" not in Path(sys.executable).name:
            raise RuntimeError(
                "On macOS launch the SCONE CLI with `mjpython SCONE.py`."
            ) from error
        raise
    finally:
        finished.set()
        if thread.is_alive():
            thread.join()
        controller.close()

    if worker_errors:
        raise worker_errors[0]
    if not results:
        raise RuntimeError("stair demo viewer closed before the trial completed")
    result = results[0]
    outcome = (
        f"top in {result.time_to_top_seconds:.2f}s"
        if result.time_to_top_seconds is not None
        else "top not reached before timeout"
    )
    print(
        f"[SIM DEMO] {strategy.value}: {outcome}; "
        f"final y/z={result.final_y:.3f}/{result.final_z:.3f} m, "
        f"assist={result.assist_entries}"
    )
    return result


def run_automatic_stair_demo(
    strategy: StairDemoStrategy | str = StairDemoStrategy.COMPARE,
    *,
    terrain: TerrainType | str = TerrainType.STAIRS_2,
    model_path: str | Path = DEFAULT_MODEL_PATH,
    timeout_seconds: float = 16.0,
) -> tuple[StairDemoResult, ...]:
    """Show one or both automatic stair strategies in sequential viewers."""

    parsed_strategy = StairDemoStrategy.parse(strategy)
    parsed_terrain = TerrainType.parse(terrain)
    if parsed_terrain not in STAIR_PRESETS:
        raise ValueError("automatic stair demo requires stairs-1, stairs-2, or stairs-3")
    if timeout_seconds <= 0.0 or not math.isfinite(timeout_seconds):
        raise ValueError("stair demo timeout must be finite and positive")
    strategies = (
        (StairDemoStrategy.HARDCODED, StairDemoStrategy.IMPROVED)
        if parsed_strategy is StairDemoStrategy.COMPARE
        else (parsed_strategy,)
    )
    return tuple(
        _run_single_demo(
            selected,
            parsed_terrain,
            model_path=model_path,
            timeout_seconds=timeout_seconds,
        )
        for selected in strategies
    )


__all__ = [
    "HardcodedStairRoller",
    "StairDemoResult",
    "StairDemoStrategy",
    "run_automatic_stair_demo",
]
