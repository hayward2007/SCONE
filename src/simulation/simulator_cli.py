"""Direct simulation CLI entry point; the root launcher uses the same bridge."""

from __future__ import annotations

import argparse
from pathlib import Path

from .cli_bridge import run
from .model import DEFAULT_MODEL_PATH


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Control SCONE in MuJoCo")
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL_PATH)
    parser.add_argument("--profile", choices=("standard", "sport"), default="standard")
    parser.add_argument("--fixed-base", action="store_true")
    parser.add_argument("--verbose", action="store_true")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if not args.model.expanduser().exists():
        raise SystemExit(f"model not found: {args.model}")
    run(
        args.model,
        profile=args.profile,
        floating_base=not args.fixed_base,
        verbose=args.verbose,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
