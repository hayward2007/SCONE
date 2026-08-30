"""Run the real, unmodified SCONE.Cli against a MuJoCo simulation.

This is not a second, simplified control surface like ``app.py``'s
``SimulationApp`` (W/A/S/D/R/H/I/Q only) -- it drives the actual production
CLI (menus, Remote Control, Actuator/System Settings, the same
``getch()``/InquirerPy prompts) by injecting a :class:`MuJoCoController` in
place of the real hardware ``Controller``. ``src/SCONE.py`` only needed one
small change to make that injection possible (``Cli(controller=None)``);
none of its logic was duplicated.
"""

from __future__ import annotations

import sys
import threading
import time
from pathlib import Path

import mujoco
import mujoco.viewer

from ..SCONE import SCONE
from .app import load_model
from .controller import MuJoCoController


def run(model_path: Path, *, floating_base: bool = True, verbose: bool = True) -> None:
    """Launch the real SCONE.Cli, rendering it live in the MuJoCo viewer.

    The CLI runs on a background thread and reads this terminal's stdin
    exactly as it would on the real robot (a separate input surface from the
    MuJoCo window, so there is no conflict). The viewer/physics loop runs on
    the main thread, as ``mujoco.viewer.launch_passive`` requires on macOS,
    and keeps stepping in real time so the CLI's internal ``time.sleep()``
    based motion timing behaves the same as it does against real hardware.
    """

    model = load_model(model_path, floating_base)
    data = mujoco.MjData(model)
    controller = MuJoCoController(model, data, verbose=verbose)

    stop_event = threading.Event()
    cli_error: list[BaseException] = []

    def run_cli() -> None:
        try:
            SCONE(controller)
        except BaseException as error:  # surfaced after the viewer loop exits
            cli_error.append(error)
        finally:
            stop_event.set()

    cli_thread = threading.Thread(target=run_cli, name="scone-cli", daemon=True)

    print("\nSCONE CLI is running against the MuJoCo simulation.")
    print("Use this terminal for the real SCONE menu / Remote Control, exactly as with real hardware.")
    print("Choose Shutdown in the CLI to stop cleanly, or close the MuJoCo window to stop immediately.\n")

    try:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            viewer.cam.lookat[:] = model.stat.center
            viewer.cam.distance = model.stat.extent * 2.2
            viewer.opt.label = mujoco.mjtLabel.mjLABEL_JOINT

            cli_thread.start()

            dt = model.opt.timestep
            while viewer.is_running() and not stop_event.is_set():
                frame_start = time.perf_counter()
                with controller.lock:
                    controller.update(dt)
                    mujoco.mj_step(model, data)
                viewer.sync()
                remaining = dt - (time.perf_counter() - frame_start)
                if remaining > 0:
                    time.sleep(remaining)
    except RuntimeError as error:
        if sys.platform == "darwin" and "mjpython" not in Path(sys.executable).name:
            raise RuntimeError(
                "On macOS the passive MuJoCo viewer must be launched with "
                "`mjpython simulator_cli.py`, not regular `python`."
            ) from error
        raise
    finally:
        stop_event.set()

    if cli_error:
        raise cli_error[0]
