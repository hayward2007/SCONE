"""Direct simulation CLI entry point owned by :mod:`simulation.core`."""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any

from .cli_bridge import SimulationControl, run
from .model import DEFAULT_MODEL_PATH
from .stair_demo import (
    StairDemoStrategy,
    run_automatic_stair_demo,
)
from ...rl.stance import SPORT_STANDING_DEGREES
from ..terrain import TERRAIN_CHOICES, TERRAIN_LABELS, TerrainType


RL_REFERENCE_MOTION_CHOICES = (
    "tripod-gait",
    "scone-gait",
    "hardcoded",
    "non_rl",
)


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
        choices=(
            *(control.value for control in SimulationControl),
            "non_rl",
        ),
        default=SimulationControl.TRIPOD_GAIT.value,
        help="simulation locomotion controller",
    )
    parser.add_argument(
        "--demo",
        choices=tuple(strategy.value for strategy in StairDemoStrategy),
        help=(
            "run a non-interactive stair demo; flat default becomes stairs-2, "
            "and compare shows hardcoded then improved"
        ),
    )
    parser.add_argument(
        "--checkpoint",
        type=Path,
        help="PPO checkpoint required by --control rl",
    )
    parser.add_argument("--rl-device", default="auto")
    parser.add_argument(
        "--rl-reference-motion",
        choices=RL_REFERENCE_MOTION_CHOICES,
        default="hardcoded",
        help=(
            "residual reference used when replaying PPO; legacy checkpoints "
            "use hardcoded; tripod-gait/scone-gait require matching models; "
            "non_rl is a legacy alias for tripod-gait"
        ),
    )
    parser.add_argument(
        "--rl-standing-pose-degrees",
        type=float,
        nargs=18,
        metavar="DEG",
        default=SPORT_STANDING_DEGREES,
    )
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
    if args.demo is not None:
        if args.fixed_base:
            raise SystemExit("--fixed-base is not supported with --demo")
        demo_terrain = (
            TerrainType.STAIRS_2
            if args.terrain == TerrainType.FLAT.value
            else TerrainType.parse(args.terrain)
        )
        run_automatic_stair_demo(
            args.demo,
            terrain=demo_terrain,
            model_path=args.model,
        )
        return 0
    run(
        args.model,
        profile=args.profile,
        floating_base=not args.fixed_base,
        terrain=args.terrain,
        terrain_seed=args.terrain_seed,
        control=args.control,
        checkpoint=args.checkpoint,
        rl_device=args.rl_device,
        rl_reference_motion=args.rl_reference_motion,
        rl_standing_pose_degrees=args.rl_standing_pose_degrees,
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
                name="Legacy mode control · Walk/Drive/Climb (R로 전환)",
            ),
            Choice(
                value=SimulationControl.TRIPOD_GAIT,
                name="tripod-gait · 고전 교대 삼각보 + IK",
            ),
            Choice(
                value=SimulationControl.SCONE_GAIT,
                name="scone-gait · SCONE 부채꼴 rolling/creep 보행 (실험)",
            ),
            Choice(
                value=SimulationControl.SCONE_STAIR,
                name=(
                    "scone-stair · 여섯 부채꼴 공통 위상 폐루프 "
                    "계단 모션 (시뮬레이션)"
                ),
            ),
            Choice(
                value=SimulationControl.RL,
                name="RL control · PPO Walk + R로 Drive/Climb 전환",
            ),
        ],
        default=SimulationControl.TRIPOD_GAIT,
    )
    return prompt.execute()


def select_stair_demo_strategy() -> StairDemoStrategy:
    """Choose a no-input stair demonstration."""

    try:
        from InquirerPy import inquirer
        from InquirerPy.base.control import Choice
    except ImportError as error:
        raise RuntimeError(
            "InquirerPy가 필요합니다. `python -m pip install -r requirements.txt` "
            "후 다시 실행하세요."
        ) from error
    prompt: Any = inquirer.select(
        message="자동 계단 시뮬레이션 방식을 선택하세요.",
        choices=[
            Choice(
                value=StairDemoStrategy.COMPARE,
                name="비교 · 공통 위상 개방루프 후 폐루프 개선형",
            ),
            Choice(
                value=StairDemoStrategy.HARDCODED,
                name="하드코딩 · 앞 1단 270° 수직 + 고정 속도 개방루프",
            ),
            Choice(
                value=StairDemoStrategy.IMPROVED,
                name="개선형 · 높이별 앞 1단 지지 + 공통 위상 폐루프",
            ),
        ],
        default=StairDemoStrategy.COMPARE,
    )
    return prompt.execute()


def select_stair_terrain() -> TerrainType:
    """Choose one of the three deterministic stair courses."""

    try:
        from InquirerPy import inquirer
        from InquirerPy.base.control import Choice
    except ImportError as error:
        raise RuntimeError(
            "InquirerPy가 필요합니다. `python -m pip install -r requirements.txt` "
            "후 다시 실행하세요."
        ) from error
    stairs = (
        TerrainType.STAIRS_1,
        TerrainType.STAIRS_2,
        TerrainType.STAIRS_3,
    )
    prompt: Any = inquirer.select(
        message="자동 계단 지형을 선택하세요.",
        choices=[
            Choice(
                value=terrain,
                name=f"{TERRAIN_LABELS[terrain]} · {terrain.value}",
            )
            for terrain in stairs
        ],
        default=TerrainType.STAIRS_2,
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
