"""Interactive MuJoCo viewer for the existing SCONE control sequences."""

from __future__ import annotations

import argparse
import os
import sys
import threading
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

from .controller import MuJoCoController
from .runner import MotionRunner


KEY_COMMANDS = {
    "w": "forward",
    "s": "backward",
    "a": "left",
    "d": "right",
    "r": "mode",
    "h": "home",
    "i": "toggle-initial",
}


def _mesh_assets(model_path: Path, root: ET.Element) -> dict[str, bytes]:
    assets: dict[str, bytes] = {}
    for mesh in root.findall("./asset/mesh"):
        filename = mesh.get("file")
        if not filename:
            continue
        asset_path = (model_path.parent / filename).resolve()
        assets[filename] = asset_path.read_bytes()
    return assets


def _contact_mesh_floor_height(model: mujoco.MjModel) -> float:
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    lowest = np.inf
    for geom_id in range(model.ngeom):
        if model.geom_contype[geom_id] == 0:
            continue
        mesh_id = int(model.geom_dataid[geom_id])
        if mesh_id < 0 or model.geom_type[geom_id] != mujoco.mjtGeom.mjGEOM_MESH:
            continue
        vertex_address = int(model.mesh_vertadr[mesh_id])
        vertex_count = int(model.mesh_vertnum[mesh_id])
        vertices = model.mesh_vert[vertex_address : vertex_address + vertex_count]
        rotation = data.geom_xmat[geom_id].reshape(3, 3)
        world_vertices = vertices @ rotation.T + data.geom_xpos[geom_id]
        lowest = min(lowest, float(world_vertices[:, 2].min()))
    if not np.isfinite(lowest):
        raise ValueError("Cannot create a floor: the model has no contact mesh geoms.")
    return lowest - 0.001


def load_model(model_path: Path, floating_base: bool) -> mujoco.MjModel:
    """Load the checked-in model, optionally adding a runtime floor/free joint.

    model.xml now ships with its own floor and root freejoint, so this is a
    no-op in the common case; --floating-base is kept only for a model.xml
    that has since had those removed again.
    """

    fixed_model = mujoco.MjModel.from_xml_path(str(model_path))
    if not floating_base:
        return fixed_model

    root = ET.parse(model_path).getroot()
    worldbody = root.find("worldbody")
    if worldbody is None:
        raise ValueError("MJCF is missing <worldbody>.")
    root_body = worldbody.find("body")
    if root_body is None:
        raise ValueError("MJCF is missing its root robot body.")
    if root_body.find("freejoint") is None:
        root_body.insert(0, ET.Element("freejoint", {"name": "root_freejoint"}))

    has_floor = any(
        geom.get("name") == "simulation_floor" for geom in worldbody.findall("geom")
    )
    if not has_floor:
        floor_height = _contact_mesh_floor_height(fixed_model)
        floor = ET.Element(
            "geom",
            {
                "name": "simulation_floor",
                "type": "plane",
                "pos": f"0 0 {floor_height:.9g}",
                "size": "3 3 0.1",
                "rgba": "0.22 0.24 0.27 1",
                "friction": "1.0 0.005 0.0005",
                "condim": "6",
            },
        )
        worldbody.insert(1, floor)
    xml = ET.tostring(root, encoding="unicode")
    return mujoco.MjModel.from_xml_string(xml, _mesh_assets(model_path, root))


class SimulationApp:
    def __init__(
        self,
        model_path: Path,
        *,
        profile: str,
        floating_base: bool,
        verbose: bool,
    ) -> None:
        self.model = load_model(model_path, floating_base)
        self.data = mujoco.MjData(self.model)
        self.controller = MuJoCoController(self.model, self.data, verbose=verbose)
        self.runner = MotionRunner(self.controller, profile)
        self.stop_event = threading.Event()

    def _key_callback(self, keycode: int) -> None:
        if 0 <= keycode < 256:
            key = chr(keycode).lower()
        else:
            return
        if key == "q":
            self.stop_event.set()
        elif key in KEY_COMMANDS:
            self.runner.command(KEY_COMMANDS[key])

    def _step(self) -> None:
        with self.controller.lock:
            self.controller.update(self.model.opt.timestep)
            mujoco.mj_step(self.model, self.data)

    def run_headless(self, duration: float, demo: str | None) -> None:
        self.runner.start(initialize=True)
        if demo and demo != "home":
            self.runner.command(demo)
        steps = max(1, int(duration / self.model.opt.timestep))
        for _ in range(steps):
            frame_start = time.perf_counter()
            self._step()
            remaining = self.model.opt.timestep - (time.perf_counter() - frame_start)
            if remaining > 0:
                time.sleep(remaining)
        self.stop()
        if not np.isfinite(self.data.qpos).all() or not np.isfinite(self.data.qvel).all():
            raise RuntimeError("Simulation produced non-finite state.")
        print(
            f"[SIM] headless check complete: {steps} steps, "
            f"simulation time={self.data.time:.3f} s"
        )

    def run_viewer(self, demo: str | None) -> None:
        print("\nSCONE MuJoCo controls")
        print("  W/S: forward/backward   A/D: left/right")
        print("  R: Walk -> Drive -> Climb   H: home")
        print("  I: all raw 2048 <-> initial pose   Q: quit")
        print("  Close the viewer window to finish.\n")

        try:
            with mujoco.viewer.launch_passive(
                self.model,
                self.data,
                key_callback=self._key_callback,
            ) as viewer:
                viewer.cam.lookat[:] = self.model.stat.center
                viewer.cam.distance = self.model.stat.extent * 2.2
                # Show each joint's motor number (M01_body_L1, etc.) by
                # default so hardware IDs are visible without opening
                # Rendering > Label > Joint by hand. Toggle it off there
                # if it gets in the way.
                viewer.opt.label = mujoco.mjtLabel.mjLABEL_JOINT
                self.runner.start(initialize=True)
                if demo and demo != "home":
                    self.runner.command(demo)

                while viewer.is_running() and not self.stop_event.is_set():
                    frame_start = time.perf_counter()
                    with viewer.lock():
                        self._step()
                    viewer.sync()
                    frame_duration = self.model.opt.timestep
                    remaining = frame_duration - (time.perf_counter() - frame_start)
                    if remaining > 0:
                        time.sleep(remaining)
        except RuntimeError as error:
            if sys.platform == "darwin" and "mjpython" not in Path(sys.executable).name:
                raise RuntimeError(
                    "On macOS the passive MuJoCo viewer must be launched with "
                    "`mjpython simulator.py`, not regular `python`."
                ) from error
            raise
        finally:
            self.stop()

    def stop(self) -> None:
        self.stop_event.set()
        self.runner.stop()


def build_parser() -> argparse.ArgumentParser:
    project_root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(
        description="Preview the existing SCONE Walk/Drive/Climb providers in MuJoCo."
    )
    parser.add_argument(
        "--model",
        type=Path,
        default=project_root / "model.xml",
        help="MJCF path (default: project model.xml)",
    )
    parser.add_argument(
        "--profile",
        choices=("standard", "sport"),
        default="standard",
        help="Existing SCONE motion profile",
    )
    parser.add_argument(
        "--floating-base",
        action="store_true",
        help="Add a runtime free joint and floor; model.xml is not modified",
    )
    parser.add_argument(
        "--demo",
        choices=("home", "forward", "backward", "left", "right"),
        help="Queue one movement after homing",
    )
    parser.add_argument(
        "--quiet",
        action="store_true",
        help="Hide individual simulated DYNAMIXEL commands",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Run a finite smoke test without opening the viewer",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=2.0,
        help="Headless simulation duration in seconds (default: 2.0)",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.duration <= 0:
        raise SystemExit("--duration must be greater than zero")

    model_path = args.model.expanduser().resolve()
    if not model_path.exists():
        raise SystemExit(f"Model not found: {model_path}")

    app = SimulationApp(
        model_path,
        profile=args.profile,
        floating_base=args.floating_base,
        verbose=not args.quiet,
    )
    if args.headless:
        app.run_headless(args.duration, args.demo)
    else:
        app.run_viewer(args.demo)
    return os.EX_OK
