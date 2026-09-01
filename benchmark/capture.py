"""Create compact videos and stills for the SCONE benchmark scenes."""

from __future__ import annotations

import argparse
import json
import subprocess
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Sequence
from unittest.mock import patch

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from src.locomotion import VelocityCommand
from src.simulation import stair_benchmark
from src.simulation.terrain import TerrainType

from .common import Perturbation, SimulationTrial
from .controllers import make_controller
from . import transitions as transition_benchmark


DEFAULT_OUTPUT = Path("archive/simulation_media")
FLAT_CONTROLLERS = (
    "articulated-walk",
    "distal-only-roll",
    "full-roll",
)
STAIR_STRATEGIES = (
    "pure-rolling",
    "synchronized-open-loop",
    "adaptive",
)
TRANSITION_SCENES = ("walk-to-roll", "roll-to-walk")


@dataclass(frozen=True)
class CaptureConfig:
    width: int = 640
    height: int = 360
    fps: int = 15
    crf: int = 34

    def __post_init__(self) -> None:
        if min(self.width, self.height, self.fps) <= 0:
            raise ValueError("capture dimensions and fps must be positive")
        if self.width % 2 or self.height % 2:
            raise ValueError("H.264 yuv420p dimensions must be even")
        if not 0 <= self.crf <= 51:
            raise ValueError("H.264 CRF must be in [0, 51]")


class FrameSink:
    """Render RGB frames directly into FFmpeg and retain one JPEG poster."""

    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        *,
        root_body_id: int,
        video_path: Path,
        image_path: Path,
        label: str,
        config: CaptureConfig,
        camera_distance: float,
        camera_azimuth: float,
        camera_elevation: float,
    ) -> None:
        self.model = model
        self.data = data
        self.root_body_id = root_body_id
        self.video_path = video_path
        self.image_path = image_path
        self.label = label
        self.config = config
        self.video_path.parent.mkdir(parents=True, exist_ok=True)
        self.image_path.parent.mkdir(parents=True, exist_ok=True)
        self.renderer = mujoco.Renderer(
            model,
            height=config.height,
            width=config.width,
        )
        self.camera = mujoco.MjvCamera()
        self.camera.type = mujoco.mjtCamera.mjCAMERA_FREE
        self.camera.distance = camera_distance
        self.camera.azimuth = camera_azimuth
        self.camera.elevation = camera_elevation
        self.scene_option = mujoco.MjvOption()
        self.scene_option.geomgroup[0] = 1
        command = [
            "ffmpeg",
            "-y",
            "-loglevel",
            "error",
            "-f",
            "rawvideo",
            "-pixel_format",
            "rgb24",
            "-video_size",
            f"{config.width}x{config.height}",
            "-framerate",
            str(config.fps),
            "-i",
            "-",
            "-an",
            "-c:v",
            "libx264",
            "-preset",
            "medium",
            "-crf",
            str(config.crf),
            "-pix_fmt",
            "yuv420p",
            "-movflags",
            "+faststart",
            str(video_path),
        ]
        self.process = subprocess.Popen(
            command,
            stdin=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        self.frame_count = 0
        self.next_frame_time = float(data.time)
        self.last_frame: Image.Image | None = None
        try:
            self.font = ImageFont.load_default(size=max(12, config.height // 25))
        except TypeError:
            self.font = ImageFont.load_default()
        self.closed = False

    def _render_frame(self) -> Image.Image:
        self.camera.lookat[:] = self.data.xpos[self.root_body_id]
        self.renderer.update_scene(
            self.data,
            camera=self.camera,
            scene_option=self.scene_option,
        )
        image = Image.fromarray(self.renderer.render())
        draw = ImageDraw.Draw(image, "RGBA")
        text_box = draw.textbbox((0, 0), self.label, font=self.font)
        bar_height = max(30, text_box[3] - text_box[1] + 14)
        draw.rectangle(
            (0, 0, self.config.width, bar_height),
            fill=(0, 0, 0, 168),
        )
        draw.text((10, 7), self.label, fill=(255, 255, 255, 255), font=self.font)
        return image

    def capture(self, *, force: bool = False) -> None:
        simulation_time = float(self.data.time)
        if not force and simulation_time + 1e-9 < self.next_frame_time:
            return
        image = self._render_frame()
        if self.process.stdin is None:
            raise RuntimeError("FFmpeg stdin is unavailable")
        try:
            self.process.stdin.write(np.asarray(image, dtype=np.uint8).tobytes())
        except BrokenPipeError as error:
            details = (
                self.process.stderr.read().decode("utf-8", errors="replace")
                if self.process.stderr is not None
                else ""
            )
            raise RuntimeError(f"FFmpeg stopped while encoding: {details}") from error
        self.last_frame = image.copy()
        self.frame_count += 1
        frame_period = 1.0 / self.config.fps
        while self.next_frame_time <= simulation_time + 1e-9:
            self.next_frame_time += frame_period

    def close(self) -> None:
        if self.closed:
            return
        self.closed = True
        if self.frame_count == 0:
            self.capture(force=True)
        if self.last_frame is not None:
            self.last_frame.save(
                self.image_path,
                format="JPEG",
                quality=72,
                optimize=True,
                progressive=True,
            )
        if self.process.stdin is not None:
            self.process.stdin.close()
        errors = (
            self.process.stderr.read().decode("utf-8", errors="replace")
            if self.process.stderr is not None
            else ""
        )
        return_code = self.process.wait()
        self.renderer.close()
        if return_code != 0:
            raise RuntimeError(f"FFmpeg exited with {return_code}: {errors}")

    def __enter__(self) -> "FrameSink":
        return self

    def __exit__(self, *_: object) -> None:
        self.close()


def _paths(root: Path, stem: str) -> tuple[Path, Path]:
    return root / "videos" / f"{stem}.mp4", root / "images" / f"{stem}.jpg"


def _file_record(root: Path, sink: FrameSink) -> dict[str, Any]:
    return {
        "video": str(sink.video_path.relative_to(root)),
        "video_bytes": sink.video_path.stat().st_size,
        "image": str(sink.image_path.relative_to(root)),
        "image_bytes": sink.image_path.stat().st_size,
        "frames": sink.frame_count,
        "encoded_duration_s": sink.frame_count / sink.config.fps,
        "width": sink.config.width,
        "height": sink.config.height,
        "fps": sink.config.fps,
        "crf": sink.config.crf,
    }


def _advance_controlled(
    trial: SimulationTrial,
    controller: Any,
    command: VelocityCommand,
    *,
    seconds: float,
    sink: FrameSink,
    dt: float = 0.02,
) -> None:
    sink.capture(force=True)
    for _ in range(round(seconds / dt)):
        controller.update(command, dt)
        trial.advance(dt)
        sink.capture()
    sink.capture(force=True)


def capture_locomotion_scene(
    root: Path,
    controller_name: str,
    *,
    terrain: TerrainType,
    config: CaptureConfig,
    perturbation: Perturbation | None = None,
    terrain_seed: int = 7,
) -> dict[str, Any]:
    suite_name = "flat" if terrain is TerrainType.FLAT else "robustness"
    stem = f"{suite_name}_{controller_name}"
    video_path, image_path = _paths(root, stem)
    selected_perturbation = perturbation or Perturbation()
    with SimulationTrial(
        terrain=terrain,
        terrain_seed=terrain_seed,
        perturbation=selected_perturbation,
    ) as trial:
        trial.initialize()
        controller = make_controller(
            controller_name,
            trial,
            phase=selected_perturbation.gait_phase,
        )
        controller.prepare(trial)
        for _ in range(25):
            controller.update(VelocityCommand(), 0.02)
            trial.advance(0.02)
        start = trial.data.xpos[trial.root_body_id].copy()
        start_rotation = trial.data.xmat[trial.root_body_id].reshape(3, 3).copy()
        label = (
            f"{suite_name.upper()} | {controller_name.upper()} | vx=0.18 m/s"
        )
        with FrameSink(
            trial.model,
            trial.data,
            root_body_id=trial.root_body_id,
            video_path=video_path,
            image_path=image_path,
            label=label,
            config=config,
            camera_distance=1.35,
            camera_azimuth=135.0,
            camera_elevation=-24.0,
        ) as sink:
            _advance_controlled(
                trial,
                controller,
                VelocityCommand(vx=0.18),
                seconds=4.0,
                sink=sink,
            )
            controller.stop()
        final = trial.data.xpos[trial.root_body_id].copy()
        displacement_world = final - start
        displacement_body = start_rotation.T @ displacement_world
        record = {
            "suite": suite_name,
            "scene": stem,
            "controller": controller_name,
            "terrain": terrain.value,
            "terrain_seed": terrain_seed,
            "command": [0.18, 0.0, 0.0],
            "simulation_duration_s": 4.0,
            "displacement_world_m": [
                float(value) for value in displacement_world
            ],
            "displacement_body_m": [float(value) for value in displacement_body],
            "mean_body_vx_mps": float(displacement_body[0]) / 4.0,
            "perturbation": asdict(selected_perturbation),
            **_file_record(root, sink),
        }
    return record


def capture_stair_scene(
    root: Path,
    strategy: str,
    *,
    config: CaptureConfig,
) -> dict[str, Any]:
    stem_name = {
        "pure-rolling": "distal-only",
        "synchronized-open-loop": "synchronized-open-loop",
        "adaptive": "full-scone",
    }[strategy]
    stem = f"stairs_200mm_{stem_name}"
    video_path, image_path = _paths(root, stem)
    original_trial = stair_benchmark._Trial
    created: list[Any] = []

    class CapturingStairTrial(original_trial):
        def __init__(self, terrain: TerrainType) -> None:
            super().__init__(terrain)
            self.capture_enabled = False
            self.sink = FrameSink(
                self.model,
                self.data,
                root_body_id=self.root_id,
                video_path=video_path,
                image_path=image_path,
                label=f"STAIRS 200 mm | {stem_name.upper()}",
                config=config,
                camera_distance=2.25,
                camera_azimuth=0.0,
                camera_elevation=-20.0,
            )
            created.append(self)

        def begin_measurement(self, start: np.ndarray) -> None:
            super().begin_measurement(start)
            self.capture_enabled = True
            self.sink.capture(force=True)

        def advance(self, seconds: float) -> None:
            if not self.capture_enabled:
                super().advance(seconds)
                return
            remaining = float(seconds)
            while remaining > 1e-9:
                chunk = min(0.02, remaining)
                super().advance(chunk)
                self.sink.capture()
                remaining -= chunk

        def close(self) -> None:
            try:
                self.sink.capture(force=True)
                self.sink.close()
            finally:
                super().close()

    with patch.object(stair_benchmark, "_Trial", CapturingStairTrial):
        result = stair_benchmark.run_hypothesis(TerrainType.STAIRS_3, strategy)
    trial = created[0]
    return {
        "suite": "stairs",
        "scene": stem,
        "controller": stem_name,
        "source_strategy": strategy,
        "terrain": TerrainType.STAIRS_3.value,
        "maximum_riser_m": 0.20,
        "minimum_tread_m": 0.35,
        "top_reached": result.top_reached,
        "time_to_top_s": result.time_to_top_s,
        "simulation_duration_s": result.elapsed_s,
        **_file_record(root, trial.sink),
    }


def capture_transition_scene(
    root: Path,
    transition: str,
    *,
    config: CaptureConfig,
) -> dict[str, Any]:
    stem = f"transition_{transition}"
    video_path, image_path = _paths(root, stem)
    original_trial = transition_benchmark.SimulationTrial
    created: list[Any] = []

    class CapturingTransitionTrial(original_trial):
        def __init__(self, *args: Any, **kwargs: Any) -> None:
            super().__init__(*args, **kwargs)
            self.capture_enabled = False
            self.sink = FrameSink(
                self.model,
                self.data,
                root_body_id=self.root_body_id,
                video_path=video_path,
                image_path=image_path,
                label=f"TRANSITION | {transition.upper()}",
                config=config,
                camera_distance=1.35,
                camera_azimuth=135.0,
                camera_elevation=-24.0,
            )
            created.append(self)

        def initialize(self) -> None:
            super().initialize()
            self.capture_enabled = True
            self.sink.capture(force=True)

        def advance(self, seconds: float, recorder: Any = None) -> None:
            if not self.capture_enabled:
                super().advance(seconds, recorder)
                return
            remaining = float(seconds)
            while remaining > 1e-9:
                chunk = min(0.02, remaining)
                super().advance(chunk, recorder)
                self.sink.capture()
                remaining -= chunk

        def close(self) -> None:
            try:
                self.sink.capture(force=True)
                self.sink.close()
            finally:
                super().close()

    with patch.object(
        transition_benchmark,
        "SimulationTrial",
        CapturingTransitionTrial,
    ):
        result = transition_benchmark.run_transition_trial(
            transition,
            perturbation=Perturbation(gait_phase=0.25),
            pre_seconds=1.0,
            recovery_seconds=2.0,
        )
    trial = created[0]
    return {
        "suite": "transition",
        "scene": stem,
        "controller": transition,
        "completed": result["completed"],
        "mode_switch_duration_s": result["mode_switch_duration_s"],
        "minimum_upright": result["minimum_upright"],
        "simulation_duration_s": result["duration_s"],
        **_file_record(root, trial.sink),
    }


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Capture compact SCONE benchmark videos and JPEG stills",
    )
    parser.add_argument(
        "--suite",
        action="append",
        choices=("flat", "stairs", "transitions", "robustness", "all"),
    )
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=360)
    parser.add_argument("--fps", type=int, default=15)
    parser.add_argument("--crf", type=int, default=34)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    config = CaptureConfig(
        width=args.width,
        height=args.height,
        fps=args.fps,
        crf=args.crf,
    )
    root = args.output.expanduser().resolve()
    selected = set(args.suite or ("all",))
    if "all" in selected:
        selected = {"flat", "stairs", "transitions", "robustness"}
    records: list[dict[str, Any]] = []

    if "flat" in selected:
        for controller_name in FLAT_CONTROLLERS:
            print(f"[capture] flat/{controller_name}")
            records.append(
                capture_locomotion_scene(
                    root,
                    controller_name,
                    terrain=TerrainType.FLAT,
                    config=config,
                )
            )

    if "stairs" in selected:
        for strategy in STAIR_STRATEGIES:
            print(f"[capture] stairs-3/{strategy}")
            records.append(capture_stair_scene(root, strategy, config=config))

    if "transitions" in selected:
        for transition in TRANSITION_SCENES:
            print(f"[capture] transition/{transition}")
            records.append(capture_transition_scene(root, transition, config=config))

    if "robustness" in selected:
        perturbation = Perturbation(
            mass_scale=1.08,
            friction_scale=0.65,
            actuator_strength_scale=0.90,
            initial_x_m=0.02,
            initial_y_m=-0.015,
            initial_yaw_degrees=4.0,
            gait_phase=0.37,
        )
        for controller_name in FLAT_CONTROLLERS:
            print(f"[capture] robustness/{controller_name}")
            records.append(
                capture_locomotion_scene(
                    root,
                    controller_name,
                    terrain=TerrainType.UNEVEN,
                    config=config,
                    perturbation=perturbation,
                    terrain_seed=2027,
                )
            )

    manifest = root / "manifest.json"
    manifest.parent.mkdir(parents=True, exist_ok=True)
    merged_records: dict[str, dict[str, Any]] = {}
    if manifest.exists():
        try:
            existing = json.loads(manifest.read_text(encoding="utf-8"))
            merged_records.update(
                {
                    str(record["scene"]): record
                    for record in existing.get("scenes", ())
                }
            )
        except (OSError, ValueError, TypeError, KeyError):
            merged_records.clear()
    merged_records.update({str(record["scene"]): record for record in records})
    all_records = [merged_records[key] for key in sorted(merged_records)]
    manifest.write_text(
        json.dumps(
            {
                "capture_config": asdict(config),
                "scene_count": len(all_records),
                "scenes": all_records,
            },
            ensure_ascii=False,
            indent=2,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )
    total_bytes = sum(
        int(record["video_bytes"]) + int(record["image_bytes"])
        for record in records
    )
    print(
        f"[capture] wrote {len(records)} videos and stills to {root} "
        f"({total_bytes / 1024.0 / 1024.0:.2f} MiB); "
        f"manifest contains {len(all_records)} scenes"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = ["CaptureConfig", "FrameSink", "main"]
