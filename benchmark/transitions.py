"""Measure position/velocity-mode transitions on one simulated robot."""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Sequence

import numpy as np

from src.hardware import Actuator
from src.locomotion import VelocityCommand

from .common import (
    BenchmarkConfig,
    MetricsRecorder,
    Perturbation,
    SimulationTrial,
    make_record,
    print_summary,
    write_records,
)
from .controllers import make_controller


TRANSITIONS = ("walk-to-roll", "roll-to-walk")
DEFAULT_OUTPUT = Path("benchmark/results/transitions.jsonl")


def _run_control_segment(
    trial: SimulationTrial,
    controller,
    command: VelocityCommand,
    seconds: float,
    dt: float,
    recorder: MetricsRecorder | None = None,
) -> None:
    for _ in range(round(seconds / dt)):
        diagnostics = controller.update(command, dt)
        if recorder is not None:
            recorder.record_control(
                converged=diagnostics.converged,
                stride_clip_fraction=diagnostics.stride_clip_fraction,
                ik_backoff_scale=diagnostics.ik_backoff_scale,
            )
        trial.advance(dt, recorder)


def run_transition_trial(
    transition: str,
    *,
    trial_index: int = 0,
    seed: int = 0,
    perturbation: Perturbation | None = None,
    pre_seconds: float = 1.0,
    recovery_seconds: float = 2.0,
    config: BenchmarkConfig | None = None,
) -> dict[str, object]:
    if transition not in TRANSITIONS:
        raise ValueError(f"unknown transition {transition!r}")
    selected_config = config or BenchmarkConfig(measure_seconds=recovery_seconds)
    selected_perturbation = perturbation or Perturbation()
    command = VelocityCommand(vx=0.18)

    with SimulationTrial(perturbation=selected_perturbation) as trial:
        trial.initialize()
        source_name, destination_name = (
            ("articulated-walk", "full-roll")
            if transition == "walk-to-roll"
            else ("full-roll", "articulated-walk")
        )
        source = make_controller(
            source_name,
            trial,
            phase=selected_perturbation.gait_phase,
        )
        source.prepare(trial)
        _run_control_segment(
            trial,
            source,
            command,
            pre_seconds,
            selected_config.control_dt,
        )

        recorder = MetricsRecorder(
            trial,
            command.as_array(),
            max_tilt_degrees=selected_config.max_tilt_degrees,
            contact_force_threshold=selected_config.contact_force_threshold,
        )
        _run_control_segment(
            trial,
            source,
            VelocityCommand(),
            0.20,
            selected_config.control_dt,
            recorder,
        )
        source.stop()
        transition_started = trial.elapsed
        if transition == "roll-to-walk":
            trial.controller.set_all_mode(Actuator.OperatingMode.POSITION)
        destination = make_controller(
            destination_name,
            trial,
            phase=selected_perturbation.gait_phase,
        )
        destination.prepare(trial, recorder=recorder)
        if transition == "roll-to-walk":
            diagnostics = destination.update(
                VelocityCommand(),
                selected_config.control_dt,
            )
            recorder.record_control(
                converged=diagnostics.converged,
                stride_clip_fraction=diagnostics.stride_clip_fraction,
                ik_backoff_scale=diagnostics.ik_backoff_scale,
            )
            trial.advance(selected_config.control_dt, recorder)
            profile = trial.robot.profile
            position_targets = {
                **{
                    motor_id: trial.controller.degrees_to_raw(
                        motor_id,
                        profile.upper_initial_position[motor_id - 1],
                    )
                    for motor_id in Actuator.Index.UPPER
                },
                **{
                    motor_id: trial.controller.degrees_to_raw(
                        motor_id,
                        profile.middle_initial_position,
                    )
                    for motor_id in Actuator.Index.MIDDLE
                },
                **{
                    motor_id: trial.controller.degrees_to_raw(
                        motor_id,
                        profile.lower_initial_position,
                    )
                    for motor_id in Actuator.Index.LOWER
                },
            }
            if not trial.wait_until_raw_positions(
                position_targets,
                tolerance=96,
                timeout=4.0,
                recorder=recorder,
            ):
                raise RuntimeError("walk posture did not settle after roll mode")
        mode_switch_duration = trial.elapsed - transition_started
        _run_control_segment(
            trial,
            destination,
            command,
            recovery_seconds,
            selected_config.control_dt,
            recorder,
        )
        destination.stop()
        metrics = recorder.finalize()

    return make_record(
        benchmark="transition",
        controller=transition,
        command_name="forward",
        command=command.as_array(),
        trial_index=trial_index,
        seed=seed,
        terrain="flat",
        perturbation=selected_perturbation,
        metrics=metrics,
        extra={
            "source_controller": source_name,
            "destination_controller": destination_name,
            "pre_seconds": pre_seconds,
            "recovery_seconds": recovery_seconds,
            "mode_switch_duration_s": mode_switch_duration,
        },
    )


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run SCONE locomotion transitions")
    parser.add_argument("--transition", action="append", choices=TRANSITIONS)
    parser.add_argument("--all", action="store_true")
    parser.add_argument("--trials", type=int, default=10)
    parser.add_argument("--seed", type=int, default=2027)
    parser.add_argument("--pre-seconds", type=float, default=1.0)
    parser.add_argument("--recovery-seconds", type=float, default=2.0)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    if args.trials <= 0:
        raise SystemExit("--trials must be positive")
    transitions = TRANSITIONS if args.all else tuple(args.transition or TRANSITIONS)
    rng = np.random.default_rng(args.seed)
    records: list[dict[str, object]] = []
    for transition in transitions:
        for trial_index in range(args.trials):
            perturbation = Perturbation(
                initial_x_m=float(rng.uniform(-0.01, 0.01)),
                initial_y_m=float(rng.uniform(-0.01, 0.01)),
                initial_yaw_degrees=float(rng.uniform(-2.0, 2.0)),
                gait_phase=float(rng.random()),
            )
            record = run_transition_trial(
                transition,
                trial_index=trial_index,
                seed=args.seed,
                perturbation=perturbation,
                pre_seconds=args.pre_seconds,
                recovery_seconds=args.recovery_seconds,
            )
            records.append(record)
            print(
                f"[transition] {transition}/trial-{trial_index}: "
                f"switch={record['mode_switch_duration_s']:.3f}s, "
                f"upright={record['minimum_upright']:.4f}, "
                f"completed={record['completed']}"
            )
    output = write_records(records, args.output)
    print_summary(records)
    print(f"[transition] wrote {len(records)} records to {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = ["run_transition_trial"]
