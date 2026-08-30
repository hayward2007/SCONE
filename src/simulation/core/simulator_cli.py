"""Direct simulation CLI entry point owned by :mod:`simulation.core`."""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any

from .cli_bridge import SimulationControl, run
from .model import DEFAULT_MODEL_PATH
from ..terrain import TERRAIN_CHOICES, TERRAIN_LABELS, TerrainType


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Control SCONE in MuJoCo")
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL_PATH)
    parser.add_argument("--profile", choices=("standard", "sport"), default="standard")
    parser.add_argument(
        "--terrain",
        choices=TERRAIN_CHOICES,
        default=TerrainType.FLAT.value,
        help="procedural terrain preset",
    )
    parser.add_argument(
        "--terrain-seed",
        type=int,
        default=7,
        help="seed used by the uneven and mixed terrain generators",
    )
    parser.add_argument("--fixed-base", action="store_true")
    parser.add_argument(
        "--control",
        choices=tuple(control.value for control in SimulationControl),
        default=SimulationControl.NON_RL.value,
        help="simulation locomotion controller",
    )
    parser.add_argument(
        "--checkpoint",
        type=Path,
        help="PPO checkpoint required by --control rl",
    )
    parser.add_argument("--rl-device", default="auto")
    parser.add_argument("--verbose", action="store_true")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if not args.model.expanduser().exists():
        raise SystemExit(f"model not found: {args.model}")
    if args.control == SimulationControl.RL.value and args.checkpoint is None:
        raise SystemExit("--checkpoint is required with --control rl")
    if args.control == SimulationControl.RL.value and args.fixed_base:
        raise SystemExit("--fixed-base is not supported with --control rl")
    run(
        args.model,
        profile=args.profile,
        floating_base=not args.fixed_base,
        terrain=args.terrain,
        terrain_seed=args.terrain_seed,
        control=args.control,
        checkpoint=args.checkpoint,
        rl_device=args.rl_device,
        verbose=args.verbose,
    )
    return 0


def select_terrain() -> TerrainType:
    """Interactive terrain picker used by the root SCONE launcher."""

    try:
        from InquirerPy import inquirer
        from InquirerPy.base.control import Choice
    except ImportError as error:
        raise RuntimeError(
            "InquirerPy가 필요합니다. `python -m pip install -r requirements.txt` "
            "후 다시 실행하세요."
        ) from error

    prompt: Any = inquirer.select(
        message="시뮬레이션 지형을 선택하세요.",
        choices=[
            Choice(
                value=terrain,
                name=f"{TERRAIN_LABELS[terrain]} · {terrain.value}",
            )
            for terrain in TerrainType
        ],
        default=TerrainType.FLAT,
    )
    return prompt.execute()


def select_simulation_control() -> SimulationControl:
    """Choose which locomotion implementation consumes x/y/yaw input."""

    try:
        from InquirerPy import inquirer
        from InquirerPy.base.control import Choice
    except ImportError as error:
        raise RuntimeError(
            "InquirerPy가 필요합니다. `python -m pip install -r requirements.txt` "
            "후 다시 실행하세요."
        ) from error

    prompt: Any = inquirer.select(
        message="시뮬레이션 로코모션 제어기를 선택하세요.",
        choices=[
            Choice(
                value=SimulationControl.OLD,
                name="Old control · 기존 blocking Walk (strafe 미지원)",
            ),
            Choice(
                value=SimulationControl.NON_RL,
                name="Non-RL control · 모델 기반 연속 보행",
            ),
            Choice(
                value=SimulationControl.RL,
                name="RL control · PPO 체크포인트 정책",
            ),
        ],
        default=SimulationControl.NON_RL,
    )
    return prompt.execute()


def select_rl_checkpoint() -> Path:
    """Choose a downloaded local PPO checkpoint for interactive RL control."""

    try:
        from InquirerPy import inquirer
        from InquirerPy.base.control import Choice
    except ImportError as error:
        raise RuntimeError(
            "InquirerPy가 필요합니다. `python -m pip install -r requirements.txt` "
            "후 다시 실행하세요."
        ) from error
    from ...rl.inquiry import PROJECT_ROOT, local_model_files

    checkpoints = local_model_files()
    if not checkpoints:
        raise FileNotFoundError(
            "runs/ 아래에 RL 체크포인트가 없습니다. 먼저 모델을 내려받거나 학습하세요."
        )
    prompt: Any = inquirer.select(
        message="조이스틱으로 실행할 RL 체크포인트를 선택하세요.",
        choices=[
            Choice(
                value=checkpoint,
                name=str(checkpoint.relative_to(PROJECT_ROOT)),
            )
            for checkpoint in checkpoints
        ],
        default=checkpoints[0],
    )
    return prompt.execute()


if __name__ == "__main__":
    raise SystemExit(main())
