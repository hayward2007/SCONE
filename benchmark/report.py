"""Aggregate JSONL benchmark records into paper-ready summary statistics."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence

import numpy as np


DEFAULT_METRICS = (
    "mean_vx_mps",
    "mean_vy_mps",
    "velocity_rmse_mps",
    "minimum_upright",
    "absolute_mechanical_work_j",
    "estimated_absolute_electrical_energy_j",
    "mechanical_cost_of_transport",
    "slip_distance_m",
    "peak_contact_force_n",
    "time_to_top_s",
    "work_to_top_j",
    "minimum_upright_to_top",
    "peak_contact_force_to_top_n",
)


def load_records(path: str | Path) -> list[dict[str, Any]]:
    source = Path(path).expanduser().resolve()
    if source.suffix.lower() == ".csv":
        with source.open(newline="", encoding="utf-8") as stream:
            return [dict(row) for row in csv.DictReader(stream)]
    with source.open(encoding="utf-8") as stream:
        return [json.loads(line) for line in stream if line.strip()]


def _as_bool(value: Any) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ("1", "true", "yes")


def _wilson_interval(successes: int, count: int) -> tuple[float, float]:
    if count == 0:
        return math.nan, math.nan
    z = 1.959963984540054
    p = successes / count
    denominator = 1.0 + z * z / count
    center = (p + z * z / (2.0 * count)) / denominator
    radius = (
        z
        * math.sqrt(p * (1.0 - p) / count + z * z / (4.0 * count * count))
        / denominator
    )
    return center - radius, center + radius


def _stable_seed(base_seed: int, *parts: object) -> int:
    digest = hashlib.sha256(
        "|".join(str(part) for part in (base_seed, *parts)).encode("utf-8")
    ).digest()
    return int.from_bytes(digest[:8], "little", signed=False)


def _bootstrap_mean_interval(
    values: np.ndarray,
    *,
    seed: int,
    samples: int,
) -> tuple[float, float]:
    if len(values) < 2:
        return math.nan, math.nan
    rng = np.random.default_rng(seed)
    indices = rng.integers(0, len(values), size=(samples, len(values)))
    means = np.mean(values[indices], axis=1)
    low, high = np.quantile(means, (0.025, 0.975))
    return float(low), float(high)


def summarize_records(
    records: Iterable[Mapping[str, Any]],
    *,
    group_by: Sequence[str] = ("benchmark", "controller", "command_name"),
    metrics: Sequence[str] = DEFAULT_METRICS,
    bootstrap_seed: int = 2027,
    bootstrap_samples: int = 10_000,
) -> list[dict[str, Any]]:
    if bootstrap_samples <= 0:
        raise ValueError("bootstrap_samples must be positive")
    groups: dict[tuple[Any, ...], list[Mapping[str, Any]]] = {}
    for record in records:
        key = tuple(record.get(field) for field in group_by)
        groups.setdefault(key, []).append(record)

    summaries: list[dict[str, Any]] = []
    for key, rows in sorted(groups.items(), key=lambda item: str(item[0])):
        summary: dict[str, Any] = dict(zip(group_by, key, strict=True))
        completed = [
            _as_bool(row.get("completed", row.get("top_reached", False)))
            for row in rows
        ]
        successes = int(sum(completed))
        low, high = _wilson_interval(successes, len(rows))
        summary.update(
            {
                "N": len(rows),
                "successes": successes,
                "success_rate": successes / len(rows),
                "success_wilson95_low": low,
                "success_wilson95_high": high,
            }
        )
        for metric in metrics:
            values: list[float] = []
            for row in rows:
                value = row.get(metric)
                if value in (None, "", "None", "nan"):
                    continue
                try:
                    parsed = float(value)
                except (TypeError, ValueError):
                    continue
                if math.isfinite(parsed):
                    values.append(parsed)
            if not values:
                continue
            array = np.asarray(values, dtype=np.float64)
            mean = float(np.mean(array))
            std = float(np.std(array, ddof=1)) if len(array) > 1 else 0.0
            ci_low, ci_high = _bootstrap_mean_interval(
                array,
                seed=_stable_seed(bootstrap_seed, *key, metric),
                samples=bootstrap_samples,
            )
            summary[f"{metric}_N"] = len(array)
            summary[f"{metric}_mean"] = mean
            summary[f"{metric}_std"] = std
            summary[f"{metric}_ci95_low"] = ci_low
            summary[f"{metric}_ci95_high"] = ci_high
        summaries.append(summary)
    return summaries


def summarize_paired_differences(
    records: Iterable[Mapping[str, Any]],
    *,
    condition_field: str,
    reference: str,
    candidate: str,
    group_by: Sequence[str],
    metrics: Sequence[str],
    pair_field: str = "pair_id",
    bootstrap_seed: int = 2027,
    bootstrap_samples: int = 10_000,
) -> list[dict[str, Any]]:
    """Summarize candidate-minus-reference differences from matched trials."""

    indexed: dict[tuple[Any, ...], dict[str, Mapping[str, Any]]] = {}
    for record in records:
        condition = str(record.get(condition_field))
        if condition not in (reference, candidate):
            continue
        key = tuple(record.get(field) for field in (*group_by, pair_field))
        indexed.setdefault(key, {})[condition] = record

    grouped: dict[tuple[Any, ...], list[tuple[Mapping[str, Any], Mapping[str, Any]]]] = {}
    for key, conditions in indexed.items():
        if reference in conditions and candidate in conditions:
            grouped.setdefault(key[:-1], []).append(
                (conditions[reference], conditions[candidate])
            )

    rows: list[dict[str, Any]] = []
    for key, pairs in sorted(grouped.items(), key=lambda item: str(item[0])):
        row: dict[str, Any] = dict(zip(group_by, key, strict=True))
        row.update(
            {
                "condition_field": condition_field,
                "reference": reference,
                "candidate": candidate,
                "paired_N": len(pairs),
            }
        )
        for metric in metrics:
            differences: list[float] = []
            for reference_record, candidate_record in pairs:
                try:
                    reference_value = float(reference_record[metric])
                    candidate_value = float(candidate_record[metric])
                except (KeyError, TypeError, ValueError):
                    continue
                if math.isfinite(reference_value) and math.isfinite(candidate_value):
                    differences.append(candidate_value - reference_value)
            if not differences:
                continue
            array = np.asarray(differences, dtype=np.float64)
            low, high = _bootstrap_mean_interval(
                array,
                seed=_stable_seed(
                    bootstrap_seed,
                    *key,
                    condition_field,
                    reference,
                    candidate,
                    metric,
                ),
                samples=bootstrap_samples,
            )
            row[f"{metric}_paired_N"] = len(array)
            row[f"{metric}_mean_difference"] = float(np.mean(array))
            row[f"{metric}_ci95_low"] = low
            row[f"{metric}_ci95_high"] = high
        rows.append(row)
    return rows


def write_summary(rows: Sequence[Mapping[str, Any]], output: str | Path) -> Path:
    path = Path(output).expanduser().resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = sorted({field for row in rows for field in row})
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    return path


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Summarize SCONE benchmark JSONL")
    parser.add_argument("input", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument(
        "--group-by",
        nargs="+",
        default=("benchmark", "controller", "command_name"),
    )
    parser.add_argument("--bootstrap-seed", type=int, default=2027)
    parser.add_argument("--bootstrap-samples", type=int, default=10_000)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    records = load_records(args.input)
    summaries = summarize_records(
        records,
        group_by=args.group_by,
        bootstrap_seed=args.bootstrap_seed,
        bootstrap_samples=args.bootstrap_samples,
    )
    output = args.output or args.input.with_name(f"{args.input.stem}-summary.csv")
    path = write_summary(summaries, output)
    print(f"[report] wrote {len(summaries)} groups to {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = [
    "load_records",
    "summarize_paired_differences",
    "summarize_records",
    "write_summary",
]
