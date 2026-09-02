"""Locked, paired simulation protocol for publication-facing SCONE evidence."""

from __future__ import annotations

import argparse
import json
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Sequence

import numpy as np

from src.simulation.terrain import TerrainType

from .common import (
    BenchmarkConfig,
    Perturbation,
    simulation_provenance,
    source_revision,
    write_records,
)
from .controllers import MATCHED_CONTROLLER_CHOICES, MATCHED_ROLL_CONFIG
from .flat import run_flat_trial
from .model_variants import CONTACT_GEOMETRIES
from .report import summarize_paired_differences, summarize_records, write_summary
from .stairs import run_stair_trial


PROTOCOL_VERSION = "scone-icra-sim-v1"
DEFAULT_ROOT = Path("benchmark/results/icra")
PHYSICS_TIMESTEPS_S = (0.001, 0.002, 0.004)


@dataclass(frozen=True)
class ProtocolProfile:
    trials: int
    settle_seconds: float
    measure_seconds: float
    flat_commands: tuple[tuple[str, tuple[float, float, float]], ...]
    stair_terrains: tuple[TerrainType, ...]


PROFILES = {
    "smoke": ProtocolProfile(
        trials=1,
        settle_seconds=0.02,
        measure_seconds=0.06,
        flat_commands=(("forward-0.10", (0.10, 0.0, 0.0)),),
        stair_terrains=(TerrainType.STAIRS_2,),
    ),
    "pilot": ProtocolProfile(
        trials=5,
        settle_seconds=0.5,
        measure_seconds=4.0,
        flat_commands=(
            ("forward-0.10", (0.10, 0.0, 0.0)),
            ("forward-0.18", (0.18, 0.0, 0.0)),
            ("lateral-0.08", (0.0, 0.08, 0.0)),
            ("yaw-0.45", (0.0, 0.0, 0.45)),
        ),
        stair_terrains=(TerrainType.STAIRS_2,),
    ),
    "evaluation": ProtocolProfile(
        trials=20,
        settle_seconds=1.0,
        measure_seconds=8.0,
        flat_commands=(
            ("forward-0.06", (0.06, 0.0, 0.0)),
            ("forward-0.12", (0.12, 0.0, 0.0)),
            ("forward-0.18", (0.18, 0.0, 0.0)),
            ("reverse-0.12", (-0.12, 0.0, 0.0)),
            ("left-0.08", (0.0, 0.08, 0.0)),
            ("right-0.08", (0.0, -0.08, 0.0)),
            ("yaw-left-0.45", (0.0, 0.0, 0.45)),
            ("yaw-right-0.45", (0.0, 0.0, -0.45)),
            ("forward-turn", (0.12, 0.0, 0.35)),
        ),
        stair_terrains=(
            TerrainType.STAIRS_1,
            TerrainType.STAIRS_2,
            TerrainType.STAIRS_3,
        ),
    ),
}


def sample_paired_perturbations(
    *,
    seed: int,
    count: int,
    include_phase: bool = True,
) -> tuple[Perturbation, ...]:
    """Draw one immutable sample per pair, reused by every condition."""

    rng = np.random.default_rng(seed)
    samples: list[Perturbation] = []
    for _ in range(count):
        samples.append(
            Perturbation(
                mass_scale=float(rng.uniform(0.90, 1.10)),
                friction_scale=float(rng.uniform(0.60, 1.20)),
                actuator_strength_scale=float(rng.uniform(0.85, 1.15)),
                initial_x_m=float(rng.uniform(-0.015, 0.015)),
                initial_y_m=float(rng.uniform(-0.015, 0.015)),
                initial_yaw_degrees=float(rng.uniform(-3.0, 3.0)),
                gait_phase=float(rng.random()) if include_phase else 0.0,
            )
        )
    return tuple(samples)


def _write_manifest(
    output_dir: Path,
    *,
    profile_name: str,
    profile: ProtocolProfile,
    seed: int,
    suites: tuple[str, ...],
) -> Path:
    output_dir.mkdir(parents=True, exist_ok=True)
    manifest_path = output_dir / "protocol.json"
    if manifest_path.exists():
        raise FileExistsError(
            f"refusing to overwrite existing protocol: {manifest_path}"
        )
    manifest = {
        "protocol_version": PROTOCOL_VERSION,
        "profile": profile_name,
        "seed": seed,
        "suites": suites,
        "profile_config": {
            **asdict(profile),
            "stair_terrains": [terrain.value for terrain in profile.stair_terrains],
        },
        "contact_geometries": CONTACT_GEOMETRIES,
        "matched_controllers": MATCHED_CONTROLLER_CHOICES,
        "matched_roll_config": asdict(MATCHED_ROLL_CONFIG),
        "physics_timestep_sensitivity": PHYSICS_TIMESTEPS_S,
        "perturbation_ranges": {
            "mass_scale": [0.90, 1.10],
            "friction_scale": [0.60, 1.20],
            "actuator_strength_scale": [0.85, 1.15],
            "initial_x_m": [-0.015, 0.015],
            "initial_y_m": [-0.015, 0.015],
            "initial_yaw_degrees": [-3.0, 3.0],
            "gait_phase": [0.0, 1.0],
        },
        **simulation_provenance(protocol_version=PROTOCOL_VERSION),
        **source_revision(),
    }
    manifest_path.write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return manifest_path


def _run_flat(
    output_dir: Path,
    *,
    profile: ProtocolProfile,
    seed: int,
) -> list[dict[str, object]]:
    perturbations = sample_paired_perturbations(seed=seed, count=profile.trials)
    records: list[dict[str, object]] = []
    config = BenchmarkConfig(
        settle_seconds=profile.settle_seconds,
        measure_seconds=profile.measure_seconds,
    )
    for command_index, (command_name, command) in enumerate(profile.flat_commands):
        for trial_index, perturbation in enumerate(perturbations):
            pair_id = f"flat-c{command_index:02d}-t{trial_index:03d}"
            terrain_seed = seed + 10_000 + command_index * 1_000 + trial_index
            for geometry in CONTACT_GEOMETRIES:
                for controller in MATCHED_CONTROLLER_CHOICES:
                    record = run_flat_trial(
                        controller,
                        command,
                        command_name=command_name,
                        trial_index=trial_index,
                        seed=seed,
                        terrain=TerrainType.FLAT,
                        terrain_seed=terrain_seed,
                        perturbation=perturbation,
                        config=config,
                        contact_geometry=geometry,
                        record_extra={
                            "pair_id": pair_id,
                            **simulation_provenance(
                                protocol_version=PROTOCOL_VERSION
                            ),
                        },
                    )
                    records.append(record)
                    print(
                        f"[icra/flat] {pair_id}/{geometry}/{controller}: "
                        f"completed={record['completed']}"
                    )
    write_records(records, output_dir / "flat.jsonl")
    write_summary(
        summarize_records(
            records,
            group_by=("contact_geometry", "controller", "command_name"),
            bootstrap_seed=seed,
        ),
        output_dir / "flat-summary.csv",
    )
    write_summary(
        summarize_paired_differences(
            records,
            condition_field="contact_geometry",
            reference="closed-wheel",
            candidate="open-arc",
            group_by=("controller", "command_name"),
            metrics=(
                "completed",
                "mean_vx_mps",
                "velocity_rmse_mps",
                "absolute_mechanical_work_j",
                "slip_distance_m",
                "minimum_upright",
            ),
            bootstrap_seed=seed,
        ),
        output_dir / "flat-geometry-paired.csv",
    )
    for reference in ("matched-articulated", "matched-distal-only"):
        write_summary(
            summarize_paired_differences(
                records,
                condition_field="controller",
                reference=reference,
                candidate="matched-coordinated",
                group_by=("contact_geometry", "command_name"),
                metrics=(
                    "completed",
                    "mean_vx_mps",
                    "mean_vy_mps",
                    "mean_yaw_rate_rps",
                    "velocity_rmse_mps",
                    "yaw_rate_rmse_rps",
                    "absolute_mechanical_work_j",
                    "slip_distance_m",
                    "minimum_upright",
                ),
                bootstrap_seed=seed,
            ),
            output_dir / f"flat-coordinated-vs-{reference}-paired.csv",
        )
    return records


def _run_stairs(
    output_dir: Path,
    *,
    profile: ProtocolProfile,
    seed: int,
) -> list[dict[str, object]]:
    perturbations = sample_paired_perturbations(
        seed=seed + 1,
        count=profile.trials,
        include_phase=False,
    )
    records: list[dict[str, object]] = []
    strategies = ("distal-only", "full-scone")
    for terrain_index, terrain in enumerate(profile.stair_terrains):
        for trial_index, perturbation in enumerate(perturbations):
            pair_id = f"stairs-p{terrain_index:02d}-t{trial_index:03d}"
            terrain_seed = seed + 20_000 + terrain_index * 1_000 + trial_index
            for geometry in CONTACT_GEOMETRIES:
                for strategy in strategies:
                    record = run_stair_trial(
                        strategy,
                        terrain,
                        trial_index=trial_index,
                        seed=seed,
                        perturbation=perturbation,
                        terrain_seed=terrain_seed,
                        contact_geometry=geometry,
                        record_extra={
                            "pair_id": pair_id,
                            **simulation_provenance(
                                protocol_version=PROTOCOL_VERSION
                            ),
                        },
                    )
                    records.append(record)
                    print(
                        f"[icra/stairs] {pair_id}/{geometry}/{strategy}: "
                        f"top={record['top_reached']}"
                    )
    if records:
        write_records(records, output_dir / "stairs.jsonl")
        write_summary(
            summarize_records(
                records,
                group_by=("contact_geometry", "controller", "command_name"),
                bootstrap_seed=seed,
            ),
            output_dir / "stairs-summary.csv",
        )
        write_summary(
            summarize_paired_differences(
                records,
                condition_field="contact_geometry",
                reference="closed-wheel",
                candidate="open-arc",
                group_by=("controller", "command_name"),
                metrics=(
                    "top_reached",
                    "time_to_top_s",
                    "work_to_top_j",
                    "minimum_upright_to_top",
                    "peak_contact_force_to_top_n",
                ),
                bootstrap_seed=seed,
            ),
            output_dir / "stairs-geometry-paired.csv",
        )
        write_summary(
            summarize_paired_differences(
                records,
                condition_field="controller",
                reference="distal-only",
                candidate="full-scone",
                group_by=("contact_geometry", "command_name"),
                metrics=(
                    "top_reached",
                    "time_to_top_s",
                    "work_to_top_j",
                    "minimum_upright_to_top",
                    "peak_contact_force_to_top_n",
                ),
                bootstrap_seed=seed,
            ),
            output_dir / "stairs-full-vs-distal-paired.csv",
        )
    return records


def _run_sensitivity(
    output_dir: Path,
    *,
    seed: int,
) -> list[dict[str, object]]:
    """Check numerical convergence without mixing it into controller statistics."""

    records: list[dict[str, object]] = []
    for geometry in CONTACT_GEOMETRIES:
        for timestep in PHYSICS_TIMESTEPS_S:
            record = run_flat_trial(
                "matched-coordinated",
                (0.12, 0.0, 0.0),
                command_name="numerical-forward-0.12",
                trial_index=0,
                seed=seed,
                terrain=TerrainType.FLAT,
                terrain_seed=seed + 30_000,
                perturbation=Perturbation(gait_phase=0.25),
                config=BenchmarkConfig(
                    settle_seconds=0.5,
                    measure_seconds=2.0,
                    physics_dt=timestep,
                ),
                contact_geometry=geometry,
                record_extra={
                    "pair_id": f"numerical-{geometry}",
                    "sensitivity_axis": "physics_timestep_s",
                    **simulation_provenance(protocol_version=PROTOCOL_VERSION),
                },
            )
            records.append(record)
            print(
                f"[icra/sensitivity] {geometry}/dt={timestep:g}: "
                f"vx={record['mean_vx_mps']:+.5f}"
            )
    write_records(records, output_dir / "numerical-sensitivity.jsonl")
    write_summary(
        summarize_records(
            records,
            group_by=("contact_geometry", "physics_timestep_s"),
            bootstrap_seed=seed,
        ),
        output_dir / "numerical-sensitivity-summary.csv",
    )
    return records


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run locked SCONE ICRA protocol")
    parser.add_argument("--profile", choices=tuple(PROFILES), default="smoke")
    parser.add_argument(
        "--suite",
        choices=("flat", "stairs", "sensitivity", "all"),
        default="all",
    )
    parser.add_argument("--seed", type=int, default=2027)
    parser.add_argument("--output-dir", type=Path)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    profile = PROFILES[args.profile]
    revision = source_revision()
    if args.profile == "evaluation" and revision.get("git_dirty") is not False:
        raise SystemExit(
            "evaluation profile requires a clean Git revision; commit or archive "
            "the intended source state before producing publication numbers"
        )
    suites = (
        ("flat", "stairs", "sensitivity")
        if args.suite == "all"
        else (args.suite,)
    )
    output_dir = (
        args.output_dir
        or DEFAULT_ROOT / f"{args.profile}-seed-{args.seed}"
    ).expanduser().resolve()
    manifest = _write_manifest(
        output_dir,
        profile_name=args.profile,
        profile=profile,
        seed=args.seed,
        suites=suites,
    )
    print(f"[icra] locked protocol: {manifest}")
    if "flat" in suites:
        _run_flat(output_dir, profile=profile, seed=args.seed)
    if "stairs" in suites:
        _run_stairs(output_dir, profile=profile, seed=args.seed)
    if "sensitivity" in suites:
        _run_sensitivity(output_dir, seed=args.seed)
    print(f"[icra] results: {output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = [
    "PROFILES",
    "PROTOCOL_VERSION",
    "ProtocolProfile",
    "sample_paired_perturbations",
]
