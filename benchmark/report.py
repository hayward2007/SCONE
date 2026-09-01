"""Aggregate JSONL benchmark records into paper-ready summary statistics."""

from __future__ import annotations

import argparse
import csv
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


def summarize_records(
    records: Iterable[Mapping[str, Any]],
    *,
    group_by: Sequence[str] = ("benchmark", "controller", "command_name"),
    metrics: Sequence[str] = DEFAULT_METRICS,
) -> list[dict[str, Any]]:
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
            half_width = 1.96 * std / math.sqrt(len(array)) if len(array) > 1 else math.nan
            summary[f"{metric}_N"] = len(array)
            summary[f"{metric}_mean"] = mean
            summary[f"{metric}_std"] = std
            summary[f"{metric}_ci95_low"] = mean - half_width
            summary[f"{metric}_ci95_high"] = mean + half_width
        summaries.append(summary)
    return summaries


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
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    records = load_records(args.input)
    summaries = summarize_records(records, group_by=args.group_by)
    output = args.output or args.input.with_name(f"{args.input.stem}-summary.csv")
    path = write_summary(summaries, output)
    print(f"[report] wrote {len(summaries)} groups to {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = ["load_records", "summarize_records", "write_summary"]
