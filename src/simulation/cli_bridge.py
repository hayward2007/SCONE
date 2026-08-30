"""Run the common SCONE API/CLI against a MuJoCo controller backend."""

from __future__ import annotations

import sys
import threading
import time
from pathlib import Path

import mujoco
import mujoco.viewer

from ..cli import run_control_cli
from ..main import SCONE
from .controller import MuJoCoController
from .model import DEFAULT_MODEL_PATH, load_model


def run(
    model_path: str | Path = DEFAULT_MODEL_PATH,
    *,
    profile: str = "standard",
    floating_base: bool = True,
    verbose: bool = False,
) -> None:
    """Open one viewer while terminal input drives the shared robot API.

    MuJoCo receives no keyboard callback. W/A/S/D are read exclusively from
    the terminal by :func:`src.cli.run_control_cli`, exactly like hardware.
    """

    model = load_model(model_path, floating_base=floating_base)
    data = mujoco.MjData(model)
    controller = MuJoCoController(model, data, verbose=verbose)
    robot = SCONE(controller, profile=profile)
    stop_event = threading.Event()
    cli_errors: list[BaseException] = []

    def control_worker() -> None:
        try:
            robot.initialize()
            run_control_cli(robot)
            robot.close()
        except BaseException as error:
            cli_errors.append(error)
        finally:
            stop_event.set()

    worker = threading.Thread(
        target=control_worker,
        name="scone-command-interpreter",
        daemon=True,
    )

    print("\n[SIM] MuJoCo uses the same terminal commands as physical SCONE.")
    print("[SIM] The viewer has no SCONE-specific keyboard mapping.\n")
    try:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            viewer.cam.lookat[:] = model.stat.center
            viewer.cam.distance = model.stat.extent * 2.2
            viewer.opt.label = mujoco.mjtLabel.mjLABEL_JOINT
            worker.start()

            timestep = model.opt.timestep
            while viewer.is_running() and not stop_event.is_set():
                frame_start = time.perf_counter()
                with controller.lock:
                    controller.update(timestep)
                    mujoco.mj_step(model, data)
                viewer.sync()
                remaining = timestep - (time.perf_counter() - frame_start)
                if remaining > 0:
                    time.sleep(remaining)
    except RuntimeError as error:
        if sys.platform == "darwin" and "mjpython" not in Path(sys.executable).name:
            raise RuntimeError(
                "On macOS launch the SCONE CLI with `mjpython SCONE.py`."
            ) from error
        raise
    finally:
        stop_event.set()
        controller.close()

    if cli_errors:
        raise cli_errors[0]


__all__ = ["run"]
