"""Unified ``python -m benchmark`` command."""

from __future__ import annotations

import argparse
import sys
from typing import Sequence


def main(argv: Sequence[str] | None = None) -> int:
    arguments = list(sys.argv[1:] if argv is None else argv)
    parser = argparse.ArgumentParser(description="SCONE paper benchmarks")
    parser.add_argument(
        "suite",
        choices=(
            "flat",
            "stairs",
            "robustness",
            "transitions",
            "report",
            "capture",
        ),
    )
    if not arguments or arguments[0] in ("-h", "--help"):
        parser.parse_args(arguments)
        return 0
    args = parser.parse_args(arguments[:1])
    remaining = arguments[1:]
    if args.suite == "flat":
        from .flat import main as run
    elif args.suite == "stairs":
        from .stairs import main as run
    elif args.suite == "robustness":
        from .robustness import main as run
    elif args.suite == "transitions":
        from .transitions import main as run
    elif args.suite == "capture":
        from .capture import main as run
    else:
        from .report import main as run
    return run(remaining)


if __name__ == "__main__":
    raise SystemExit(main())
