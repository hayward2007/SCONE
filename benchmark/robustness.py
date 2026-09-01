"""Monte-Carlo robustness wrapper around the flat A/B/C benchmark."""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Sequence

import numpy as np

from .common import BenchmarkConfig, COMMANDS, Perturbation, print_summary, write_records
from .controllers import CONTROLLER_CHOICES
from .flat import run_flat_trial


DEFAULT_OUTPUT = Path("benchmark/results/robustness.jsonl")


def _range(values: Sequence[float], name: str) -> tuple[float, float]:
    low, high = (float(value) for value in values)
    if low <= 0.0 or high < low:
        raise ValueError(f"invalid {name} range")
    return low, high


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run randomized SCONE flat-ground robustness trials",
    )
    parser.add_argument(
        "--controller",
        action="append",
        choices=CONTROLLER_CHOICES,
    )
    parser.add_argument("--all", action="store_true")
    parser.add_argument("--command", choices=tuple(COMMANDS), default="forward")
    parser.add_argument("--trials", type=int, default=100)
    parser.add_argument("--seed", type=int, default=2027)
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--settle", type=float, default=0.5)
    parser.add_argument("--terrain", default="uneven")
    parser.add_argument("--terrain-seed", type=int, default=1000)
    parser.add_argument("--mass-scale", nargs=2, type=float, default=(0.9, 1.1))
    parser.add_argument("--friction-scale", nargs=2, type=float, default=(0.4, 1.2))
    parser.add_argument(
        "--actuator-strength-scale",
        nargs=2,
        type=float,
        default=(0.85, 1.15),
    )
    parser.add_argument("--initial-position-mm", type=float, default=30.0)
    parser.add_argument("--initial-yaw-degrees", type=float, default=5.0)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    if args.trials <= 0:
        raise SystemExit("--trials must be positive")
    mass_range = _range(args.mass_scale, "mass")
    friction_range = _range(args.friction_scale, "friction")
    strength_range = _range(args.actuator_strength_scale, "actuator strength")
    controllers = (
        ("articulated-walk", "distal-only-roll", "full-roll")
        if args.all
        else tuple(args.controller or ("full-roll",))
    )
    rng = np.random.default_rng(args.seed)
    command = COMMANDS[args.command]
    records: list[dict[str, object]] = []
    for controller_name in controllers:
        for trial_index in range(args.trials):
            position_limit = args.initial_position_mm / 1000.0
            perturbation = Perturbation(
                mass_scale=float(rng.uniform(*mass_range)),
                friction_scale=float(rng.uniform(*friction_range)),
                actuator_strength_scale=float(rng.uniform(*strength_range)),
                initial_x_m=float(rng.uniform(-position_limit, position_limit)),
                initial_y_m=float(rng.uniform(-position_limit, position_limit)),
                initial_yaw_degrees=float(
                    rng.uniform(-args.initial_yaw_degrees, args.initial_yaw_degrees)
                ),
                gait_phase=float(rng.random()),
            )
            record = run_flat_trial(
                controller_name,
                command,
                command_name=args.command,
                trial_index=trial_index,
                seed=args.seed,
                terrain=args.terrain,
                terrain_seed=args.terrain_seed + trial_index,
                perturbation=perturbation,
                config=BenchmarkConfig(
                    settle_seconds=args.settle,
                    measure_seconds=args.duration,
                ),
            )
            records.append(record)
            print(
                f"[robustness] {controller_name}/trial-{trial_index}: "
                f"completed={record['completed']}, "
                f"vx={record['mean_vx_mps']:+.4f} m/s"
            )
    output = write_records(records, args.output)
    print_summary(records)
    print(f"[robustness] wrote {len(records)} records to {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
