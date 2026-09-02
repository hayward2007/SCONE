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
import numpy as np

from ...cli_i18n import Language, localize
from ...hardware import Actuator
from ...locomotion import VelocityCommand
from ...main import SCONE
from ..terrain import STAIR_PRESETS, TerrainType
from .controller import MuJoCoController
from .model import DEFAULT_MODEL_PATH, load_model
from .stair_climber import (
    SconeStairClimber,
    SconeStairConfig,
    prepare_scone_stair_pose,
    synchronized_lower_degrees,
    synchronized_phase_spread_degrees,
)
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
    front_stage1_degrees: float
    front_stage1_actual_degrees: float
    front_stage1_sync_entries: int
    phase_sync_entries: int
    maximum_phase_spread_degrees: float


class HardcodedStairRoller:
    """Legacy vertical front brace plus open-loop lower velocity baseline.

    This preserves the defining all-six-sector stair phase but does not correct
    contact-induced phase drift after switching to velocity mode.  The improved
    controller advances the same phase with extended-position feedback.
    """

    def __init__(
        self,
        controller: MuJoCoController,
        *,
        velocity: int = 200,
        config: SconeStairConfig | None = None,
    ) -> None:
        if velocity <= 0:
            raise ValueError("hardcoded stair velocity must be positive")
        self.controller = controller
        self.velocity = velocity
        self.config = config or SconeStairConfig()
        self.phase_degrees = self.config.synchronized_phase_degrees
        self.front_stage1_degrees = self.config.legacy_front_stage1_degrees
        self.front_stage1_sync_entries = 0
        self.phase_sync_entries = 0
        self.maximum_phase_spread_degrees = 0.0
        self._prepared = False
        self._active = False

    def prepare_front_stage1(self) -> dict[int, int]:
        """Reproduce Legacy Climb's fully vertical leading stage-1 pose."""

        self.controller.set_speeds(
            {
                motor_id: self.config.front_stage1_profile_velocity
                for motor_id in Actuator.Index.MIDDLE_RIGHT
            }
        )
        targets = {
            motor_id: self.front_stage1_degrees
            for motor_id in Actuator.Index.MIDDLE_RIGHT
        }
        self.controller.set_positions(targets)
        self.front_stage1_sync_entries += 1
        return {
            motor_id: self.controller.degrees_to_raw(motor_id, degrees)
            for motor_id, degrees in targets.items()
        }

    def prepare(self) -> dict[int, int]:
        self.controller.set_all_mode(Actuator.OperatingMode.POSITION)
        self.controller.set_speeds(
            {
                motor_id: self.config.profile_velocity
                for motor_id in Actuator.Index.LOWER
            }
        )
        self.controller.set_accelerations(
            {
                motor_id: self.config.profile_acceleration
                for motor_id in Actuator.Index.LOWER
            }
        )
        targets = synchronized_lower_degrees(self.phase_degrees)
        self.controller.set_positions(targets)
        self.phase_sync_entries += 1
        self._prepared = True
        return {
            motor_id: self.controller.degrees_to_raw(motor_id, degrees)
            for motor_id, degrees in targets.items()
        }

    def activate(self) -> None:
        if not self._prepared:
            raise RuntimeError("prepare HardcodedStairRoller before activating it")
        self.controller.set_all_mode(Actuator.OperatingMode.VELOCITY)
        self.controller.set_velocities(
            {motor_id: 0 for motor_id in Actuator.Index.LOWER}
        )
        self._active = True
        self._record_phase_spread()

    def _record_phase_spread(self) -> None:
        self.maximum_phase_spread_degrees = max(
            self.maximum_phase_spread_degrees,
            synchronized_phase_spread_degrees(self.controller),
        )

    def update(self) -> None:
        if not self._active:
            raise RuntimeError("prepare and activate HardcodedStairRoller first")
        # Negative odd geometric phase is the preset world +Y ascent direction.
        self.controller.set_velocities(
            self.controller.arc_wheel_velocities(-self.velocity)
        )
        self._record_phase_spread()

    def stop(self) -> None:
        if self._active:
            self.controller.set_velocities(
                {motor_id: 0 for motor_id in Actuator.Index.LOWER}
            )
            self._record_phase_spread()
        self._active = False


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
    language: Language | str = Language.ENGLISH,
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
            if strategy is StairDemoStrategy.HARDCODED:
                hardcoded = HardcodedStairRoller(controller)
                front_targets = hardcoded.prepare_front_stage1()
                front_tolerance = hardcoded.config.front_stage1_tolerance_raw
                front_timeout = hardcoded.config.front_stage1_sync_timeout
                phase_tolerance = hardcoded.config.phase_tolerance_raw
                phase_timeout = hardcoded.config.phase_sync_timeout
            else:
                improved = SconeStairClimber(controller, terrain=terrain)
                front_targets = improved.prepare_front_stage1()
                front_tolerance = improved.config.front_stage1_tolerance_raw
                front_timeout = improved.config.front_stage1_sync_timeout
                phase_tolerance = improved.config.phase_tolerance_raw
                phase_timeout = improved.config.phase_sync_timeout
            if not controller.wait_until_raw_positions(
                front_targets,
                tolerance=front_tolerance,
                timeout=front_timeout,
            ):
                raise RuntimeError(
                    "automatic stair demo could not acquire its front stage-1 brace"
                )
            if hardcoded is not None:
                phase_targets = hardcoded.prepare()
            else:
                assert improved is not None
                phase_targets = improved.prepare()
            if not controller.wait_until_raw_positions(
                phase_targets,
                tolerance=phase_tolerance,
                timeout=phase_timeout,
            ):
                raise RuntimeError(
                    "automatic stair demo could not synchronize all six C-frames"
                )
            # Record after lower phase acquisition so GUI and headless
            # benchmark report the same loaded front-brace state.
            front_stage1_actual_degrees = float(
                np.mean(
                    [
                        controller.get_position(motor_id) / 4096.0 * 360.0
                        for motor_id in Actuator.Index.MIDDLE_RIGHT
                    ]
                )
            )
            if hardcoded is not None:
                hardcoded.activate()
            else:
                assert improved is not None
                improved.activate()
            with controller.lock:
                start_z = float(data.xpos[root_id, 2])
            top_y, top_z = _top_thresholds(terrain, start_z)

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
                    front_stage1_degrees=(
                        hardcoded.front_stage1_degrees
                        if hardcoded is not None
                        else improved.front_stage1_degrees
                    ),
                    front_stage1_actual_degrees=front_stage1_actual_degrees,
                    front_stage1_sync_entries=(
                        hardcoded.front_stage1_sync_entries
                        if hardcoded is not None
                        else improved.front_stage1_sync_entries
                    ),
                    phase_sync_entries=(
                        hardcoded.phase_sync_entries
                        if hardcoded is not None
                        else improved.phase_sync_entries
                    ),
                    maximum_phase_spread_degrees=(
                        hardcoded.maximum_phase_spread_degrees
                        if hardcoded is not None
                        else improved.maximum_phase_spread_degrees
                    ),
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
        + localize(
            language,
            "running automatically; no keyboard input is required.",
            "키 입력 없이 자동 실행합니다.",
        )
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
        if (
            sys.platform == "darwin"
            and "launch_passive requires" in str(error)
        ):
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
    outcome = localize(
        language,
        f"top in {result.time_to_top_seconds:.2f}s",
        f"{result.time_to_top_seconds:.2f}초에 상단 도달",
    ) if result.time_to_top_seconds is not None else localize(
        language,
        "top not reached before timeout",
        "제한 시간 내 상단 미도달",
    )
    print(
        f"[SIM DEMO] {strategy.value}: {outcome}; "
        f"final y/z={result.final_y:.3f}/{result.final_z:.3f} m, "
        f"front-stage1 target/actual={result.front_stage1_degrees:.1f}/"
        f"{result.front_stage1_actual_degrees:.1f} deg, "
        f"phase-sync={result.phase_sync_entries}, "
        f"max phase spread={result.maximum_phase_spread_degrees:.2f} deg"
    )
    return result


def run_automatic_stair_demo(
    strategy: StairDemoStrategy | str = StairDemoStrategy.COMPARE,
    *,
    terrain: TerrainType | str = TerrainType.STAIRS_2,
    model_path: str | Path = DEFAULT_MODEL_PATH,
    timeout_seconds: float = 16.0,
    language: Language | str = Language.ENGLISH,
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
    results = []
    for index, selected in enumerate(strategies):
        if index and sys.platform == "darwin":
            # MuJoCo's Cocoa viewer closes asynchronously on its UI thread.
            # Launching the second compare window immediately can race that
            # teardown and raise "another MuJoCo viewer is already open".
            time.sleep(1.0)
        results.append(
            _run_single_demo(
                selected,
                parsed_terrain,
                model_path=model_path,
                timeout_seconds=timeout_seconds,
                language=language,
            )
        )
    return tuple(results)


__all__ = [
    "HardcodedStairRoller",
    "StairDemoResult",
    "StairDemoStrategy",
    "run_automatic_stair_demo",
]
