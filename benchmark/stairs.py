"""Paper-facing stair A/B/C wrapper and custom riser/tread sweep."""

from __future__ import annotations

import argparse
import math
from dataclasses import asdict
from pathlib import Path
from typing import Sequence
from unittest.mock import patch

import numpy as np

from src.simulation import stair_benchmark
from src.simulation.core.model import load_model as load_base_model
from src.simulation.terrain import STAIR_PRESETS, StairProfile, TerrainType

from .common import (
    Perturbation,
    apply_model_perturbation,
    print_summary,
    source_revision,
    temporary_stair_profile,
    write_records,
)
from .model_variants import CONTACT_GEOMETRIES, transform_for_contact_geometry


PAPER_STRATEGIES = {
    "distal-only": "pure-rolling",
    "synchronized-open-loop": "synchronized-open-loop",
    "full-scone": "adaptive",
}
DEFAULT_OUTPUT = Path("benchmark/results/stairs.jsonl")


def run_stair_trial(
    strategy: str,
    terrain: TerrainType | str,
    *,
    trial_index: int = 0,
    seed: int = 0,
    perturbation: Perturbation | None = None,
    profile: StairProfile | None = None,
    terrain_seed: int = 7,
    contact_geometry: str = "open-arc",
    record_extra: dict[str, object] | None = None,
) -> dict[str, object]:
    """Run one existing stair hypothesis with optional model sensitivity."""

    if strategy not in PAPER_STRATEGIES:
        raise ValueError(f"unknown paper stair strategy {strategy!r}")
    selected_terrain = TerrainType.parse(terrain)
    if selected_terrain not in STAIR_PRESETS:
        raise ValueError("stair benchmark requires a stair terrain")
    selected_perturbation = perturbation or Perturbation()
    if contact_geometry not in CONTACT_GEOMETRIES:
        raise ValueError(
            f"unknown contact geometry {contact_geometry!r}; "
            f"choose from {CONTACT_GEOMETRIES}"
        )

    def perturbed_loader(*args, **kwargs):
        kwargs["xml_transform"] = transform_for_contact_geometry(contact_geometry)
        model = load_base_model(*args, **kwargs)
        apply_model_perturbation(model, selected_perturbation)
        return model

    context = (
        temporary_stair_profile(selected_terrain, profile)
        if profile is not None
        else _null_context()
    )
    with context, patch.object(
        stair_benchmark,
        "load_model",
        side_effect=perturbed_loader,
    ):
        result = stair_benchmark.run_hypothesis(
            selected_terrain,
            PAPER_STRATEGIES[strategy],
            terrain_seed=terrain_seed,
            initial_x_m=selected_perturbation.initial_x_m,
            initial_y_m=selected_perturbation.initial_y_m,
            initial_yaw_degrees=selected_perturbation.initial_yaw_degrees,
        )

    active_profile = profile or STAIR_PRESETS[selected_terrain]
    failure_reason = None
    if not result.top_reached:
        failure_reason = (
            "tilt-limit"
            if result.minimum_upright < math.cos(math.radians(60.0))
            else "measurement-timeout-no-top"
        )
    return {
        "schema_version": 1,
        "benchmark": "stairs",
        "controller": strategy,
        "source_strategy": PAPER_STRATEGIES[strategy],
        "command_name": (
            f"riser-{1000.0 * max(active_profile.rises):g}mm_"
            f"tread-{1000.0 * min(active_profile.tread_depths):g}mm"
        ),
        "trial_index": trial_index,
        "seed": seed,
        "terrain": selected_terrain.value,
        "terrain_seed": terrain_seed,
        "contact_geometry": contact_geometry,
        "rises_m": list(active_profile.rises),
        "tread_depths_m": list(active_profile.tread_depths),
        "maximum_riser_m": max(active_profile.rises),
        "minimum_tread_m": min(active_profile.tread_depths),
        "completed": result.top_reached,
        "failure_reason": failure_reason,
        "timing_scope": "ascent-after-pose-and-phase-preparation",
        **asdict(selected_perturbation),
        **asdict(result),
        **source_revision(),
        **(record_extra or {}),
    }


class _null_context:
    def __enter__(self):
        return None

    def __exit__(self, *_: object) -> None:
        return None


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run SCONE distal/synchronized/full stair comparisons",
    )
    parser.add_argument(
        "--strategy",
        action="append",
        choices=tuple(PAPER_STRATEGIES),
    )
    parser.add_argument("--all", action="store_true")
    parser.add_argument(
        "--preset",
        action="append",
        choices=("stairs-1", "stairs-2", "stairs-3"),
    )
    parser.add_argument("--riser-mm", action="append", type=float)
    parser.add_argument("--tread-mm", action="append", type=float)
    parser.add_argument("--trials", type=int, default=1)
    parser.add_argument("--seed", type=int, default=2027)
    parser.add_argument("--terrain-seed", type=int, default=7000)
    parser.add_argument(
        "--contact-geometry",
        choices=CONTACT_GEOMETRIES,
        default="open-arc",
    )
    parser.add_argument(
        "--randomize",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="sample mass, friction, and actuator strength for repeated trials",
    )
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    if args.trials <= 0:
        raise SystemExit("--trials must be positive")
    if (args.riser_mm is None) != (args.tread_mm is None):
        raise SystemExit("use --riser-mm and --tread-mm together")
    if args.trials > 1 and not args.randomize:
        print(
            "[stairs] warning: repeated nominal trials are deterministic; "
            "use --randomize for success-rate statistics"
        )
    strategies = (
        tuple(PAPER_STRATEGIES)
        if args.all
        else tuple(args.strategy or ("full-scone",))
    )
    jobs: list[tuple[TerrainType, StairProfile | None]] = [
        (TerrainType.parse(name), None)
        for name in (args.preset or ("stairs-1", "stairs-2", "stairs-3"))
    ]
    if args.riser_mm is not None and args.tread_mm is not None:
        jobs = []
        for riser_mm in args.riser_mm:
            for tread_mm in args.tread_mm:
                profile = StairProfile(
                    rises=(riser_mm / 1000.0,) * 3,
                    tread_depths=(tread_mm / 1000.0,) * 3,
                    widths=(1.0, 1.0, 1.0),
                    landing_length=0.70,
                )
                jobs.append((TerrainType.STAIRS_3, profile))

    rng = np.random.default_rng(args.seed)
    records: list[dict[str, object]] = []
    for terrain, profile in jobs:
        for strategy in strategies:
            for trial_index in range(args.trials):
                perturbation = (
                    Perturbation(
                        mass_scale=float(rng.uniform(0.9, 1.1)),
                        friction_scale=float(rng.uniform(0.4, 1.2)),
                        actuator_strength_scale=float(rng.uniform(0.85, 1.15)),
                    )
                    if args.randomize
                    else Perturbation()
                )
                record = run_stair_trial(
                    strategy,
                    terrain,
                    trial_index=trial_index,
                    seed=args.seed,
                    perturbation=perturbation,
                    profile=profile,
                    terrain_seed=args.terrain_seed + trial_index,
                    contact_geometry=args.contact_geometry,
                )
                records.append(record)
                print(
                    f"[stairs] {strategy}/{record['command_name']}/"
                    f"trial-{trial_index}: top={record['top_reached']}, "
                    f"time={record['time_to_top_s']}"
                )
    output = write_records(records, args.output)
    print_summary(records)
    print(f"[stairs] wrote {len(records)} records to {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = ["PAPER_STRATEGIES", "run_stair_trial"]
