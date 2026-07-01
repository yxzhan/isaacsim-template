"""
sim_app.py -- SimulationApp bootstrap helper for the cup_to_sink benchmark.

Usage:
    from cup_to_sink.sim_app import start
    simulation_app = start(headless=True)
    # ... do Isaac Sim work ...
    simulation_app.close()

IMPORTANT: Import pxr / isaacsim / omni modules ONLY after calling start().
"""

import os
import shutil
import sys


def _force_utf8_stdio() -> None:
    """Make stdout/stderr tolerate non-ASCII so a stray Unicode char in a
    print() can never raise UnicodeEncodeError.

    When stdout is redirected to a pipe/file the default encoding is often
    ASCII; printing characters like '...' or '<->' then raises inside the Isaac
    Sim event loop and crashes the kit process. backslashreplace guarantees a
    print never raises regardless of the target encoding.
    """
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8", errors="backslashreplace")
        except (AttributeError, ValueError):
            pass  # not a reconfigurable text stream -- best effort only


def start(headless: bool = True, width: int = 1280, height: int = 720,
          hide_ui: bool = True):
    """Start Isaac Sim and return the SimulationApp instance.

    Mirrors the startup block from examples/apartment.py:
      - Copies the precompiled kit cache if needed.
      - Constructs SimulationApp with the given headless flag.
      - Restores stdout/stderr that Isaac hijacks.

    Returns:
        SimulationApp instance (caller must call .close() when done).
    """
    # Harden stdout/stderr against non-ASCII before anything prints.
    _force_utf8_stdio()

    # Copy the precompiled kit cache if it is not present yet (same as apartment.py:36-40)
    target_dir = "/isaac-sim/kit/cache"
    source_dir = "/mnt/isaacsim-cache/cache"
    if os.path.isdir(source_dir) and not os.path.isdir(target_dir):
        shutil.copytree(source_dir, target_dir)

    # Save stdout/stderr before SimulationApp hijacks them (apartment.py:62-63)
    original_stdout = sys.stdout
    original_stderr = sys.stderr

    # isaacsim must be imported here -- NOT at module top level -- because the
    # pxr/omni/isaacsim extension system is not initialised until SimulationApp runs.
    from isaacsim import SimulationApp  # noqa: PLC0415

    simulation_app = SimulationApp({
        "headless": headless,
        "hide_ui": hide_ui,
        "width": width,
        "height": height,
        "renderer": "RaytracedLighting",
        "display_options": 3286,
    })

    # Restore stdout/stderr that Isaac Sim hijacks (apartment.py:75-76)
    sys.stdout = original_stdout
    sys.stderr = original_stderr

    print("SimulationApp Ready!")
    return simulation_app
