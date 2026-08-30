"""Command-line entry point for running the real SCONE CLI against MuJoCo.

Unlike simulator.py's simplified W/A/S/D preview, this launches the actual
production SCONE.Cli (menus, Remote Control, Actuator/System Settings) with
a MuJoCoController standing in for the real hardware controller.
"""

import argparse
from pathlib import Path

from src.simulation.cli_bridge import run


def main() -> int:
    project_root = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(
        description="Run the real, interactive SCONE.Cli against a MuJoCo simulation."
    )
    parser.add_argument(
        "--model",
        type=Path,
        default=project_root / "model.xml",
        help="MJCF path (default: project model.xml)",
    )
    parser.add_argument(
        "--floating-base",
        action="store_true",
        default=True,
        help=(
            "Kept for parity with simulator.py; model.xml already ships a "
            "floating root and floor, so this only matters if those were "
            "removed from the file again."
        ),
    )
    parser.add_argument(
        "--quiet",
        action="store_true",
        help="Hide individual simulated DYNAMIXEL commands",
    )
    args = parser.parse_args()

    model_path = args.model.expanduser().resolve()
    if not model_path.exists():
        raise SystemExit(f"Model not found: {model_path}")

    run(model_path, floating_base=args.floating_base, verbose=not args.quiet)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
