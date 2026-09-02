"""Same-model A/B/C flat-ground locomotion benchmark."""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Sequence

import numpy as np

from src.locomotion import VelocityCommand
from src.simulation.terrain import TERRAIN_CHOICES, TerrainType

from .common import (
    BenchmarkConfig,
    COMMANDS,
    MetricsRecorder,
    Perturbation,
    SimulationTrial,
    make_record,
    print_summary,
    write_records,
)
from .controllers import CONTROLLER_CHOICES, make_controller
from .model_variants import CONTACT_GEOMETRIES


DEFAULT_OUTPUT = Path("benchmark/results/flat.jsonl")


def run_flat_trial(
    controller_name: str,
    command: Sequence[float],
    *,
    command_name: str = "custom",
    trial_index: int = 0,
    seed: int = 0,
    terrain: TerrainType | str = TerrainType.FLAT,
    terrain_seed: int = 7,
    perturbation: Perturbation | None = None,
    config: BenchmarkConfig | None = None,
    contact_geometry: str = "open-arc",
    record_extra: dict[str, object] | None = None,
) -> dict[str, object]:
    """Run one fresh, headless trial and return one serializable record."""

    selected_terrain = TerrainType.parse(terrain)
    selected_config = config or BenchmarkConfig()
    selected_perturbation = perturbation or Perturbation()
    parsed_command = np.asarray(command, dtype=np.float64)
    if parsed_command.shape != (3,):
        raise ValueError("command must contain vx, vy, yaw_rate")

    with SimulationTrial(
        terrain=selected_terrain,
        terrain_seed=terrain_seed,
        perturbation=selected_perturbation,
        contact_geometry=contact_geometry,
    ) as trial:
        trial.model.opt.timestep = (
            selected_config.physics_dt
            if selected_config.physics_dt is not None
            else min(float(trial.model.opt.timestep), selected_config.control_dt)
        )
        runtime_settings = {
            "physics_timestep_s": float(trial.model.opt.timestep),
            "solver_iterations": int(trial.model.opt.iterations),
            "solver_tolerance": float(trial.model.opt.tolerance),
            "solver_code": int(trial.model.opt.solver),
            "integrator_code": int(trial.model.opt.integrator),
            "cone_code": int(trial.model.opt.cone),
        }
        trial.initialize()
        controller = make_controller(
            controller_name,
            trial,
            phase=selected_perturbation.gait_phase,
        )
        controller.prepare(trial)

        neutral = VelocityCommand()
        settle_steps = round(
            selected_config.settle_seconds / selected_config.control_dt
        )
        for _ in range(settle_steps):
            diagnostics = controller.update(neutral, selected_config.control_dt)
            trial.advance(selected_config.control_dt)
            if not diagnostics.converged:
                raise RuntimeError("controller IK failed during benchmark settling")

        recorder = MetricsRecorder(
            trial,
            parsed_command,
            max_tilt_degrees=selected_config.max_tilt_degrees,
            contact_force_threshold=selected_config.contact_force_threshold,
        )
        velocity_command = VelocityCommand.from_array(parsed_command)
        measure_steps = round(
            selected_config.measure_seconds / selected_config.control_dt
        )
        for _ in range(measure_steps):
            diagnostics = controller.update(
                velocity_command,
                selected_config.control_dt,
            )
            recorder.record_control(
                converged=diagnostics.converged,
                stride_clip_fraction=diagnostics.stride_clip_fraction,
                ik_backoff_scale=diagnostics.ik_backoff_scale,
            )
            trial.advance(selected_config.control_dt, recorder)
            if recorder.termination_reason is not None:
                break
        controller.stop()
        metrics = recorder.finalize()

    return make_record(
        benchmark="flat",
        controller=controller_name,
        command_name=command_name,
        command=parsed_command,
        trial_index=trial_index,
        seed=seed,
        terrain=selected_terrain.value,
        perturbation=selected_perturbation,
        metrics=metrics,
        extra={
            "control_dt_s": selected_config.control_dt,
            "settle_seconds": selected_config.settle_seconds,
            "requested_measure_seconds": selected_config.measure_seconds,
            "terrain_seed": terrain_seed,
            "contact_geometry": contact_geometry,
            **runtime_settings,
            **(record_extra or {}),
        },
    )


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run same-model SCONE flat-ground controller comparisons",
    )
    parser.add_argument(
        "--controller",
        action="append",
        choices=CONTROLLER_CHOICES,
        help="repeat to compare selected controllers",
    )
    parser.add_argument(
        "--all",
        action="store_true",
        help="run paper A/B/C: articulated, distal-only, and full-roll",
    )
    parser.add_argument(
        "--command",
        action="append",
        choices=tuple(COMMANDS),
        help="named command; repeat for a command grid",
    )
    parser.add_argument(
        "--custom-command",
        nargs=3,
        type=float,
        metavar=("VX", "VY", "YAW_RATE"),
    )
    parser.add_argument("--trials", type=int, default=1)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--terrain", choices=TERRAIN_CHOICES, default="flat")
    parser.add_argument("--terrain-seed", type=int, default=7)
    parser.add_argument(
        "--contact-geometry",
        choices=CONTACT_GEOMETRIES,
        default="open-arc",
    )
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--settle", type=float, default=0.5)
    parser.add_argument(
        "--random-phase",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    parser.add_argument("--mass-scale", type=float, default=1.0)
    parser.add_argument("--friction-scale", type=float, default=1.0)
    parser.add_argument("--actuator-strength-scale", type=float, default=1.0)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    if args.trials <= 0:
        raise SystemExit("--trials must be positive")
    controllers = (
        ("articulated-walk", "distal-only-roll", "full-roll")
        if args.all
        else tuple(args.controller or ("articulated-walk",))
    )
    commands: list[tuple[str, tuple[float, float, float]]] = [
        (name, COMMANDS[name]) for name in (args.command or ("forward",))
    ]
    if args.custom_command is not None:
        commands.append(("custom", tuple(args.custom_command)))

    rng = np.random.default_rng(args.seed)
    records: list[dict[str, object]] = []
    for controller_name in controllers:
        for command_name, command in commands:
            for trial_index in range(args.trials):
                perturbation = Perturbation(
                    mass_scale=args.mass_scale,
                    friction_scale=args.friction_scale,
                    actuator_strength_scale=args.actuator_strength_scale,
                    gait_phase=(float(rng.random()) if args.random_phase else 0.0),
                )
                record = run_flat_trial(
                    controller_name,
                    command,
                    command_name=command_name,
                    trial_index=trial_index,
                    seed=args.seed,
                    terrain=args.terrain,
                    terrain_seed=args.terrain_seed + trial_index,
                    perturbation=perturbation,
                    config=BenchmarkConfig(
                        settle_seconds=args.settle,
                        measure_seconds=args.duration,
                    ),
                    contact_geometry=args.contact_geometry,
                )
                records.append(record)
                print(
                    f"[flat] {controller_name}/{command_name}/trial-{trial_index}: "
                    f"vx={record['mean_vx_mps']:+.4f} m/s, "
                    f"upright={record['minimum_upright']:.4f}, "
                    f"completed={record['completed']}"
                )
    output = write_records(records, args.output)
    print_summary(records)
    print(f"[flat] wrote {len(records)} records to {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = ["run_flat_trial"]
