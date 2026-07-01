"""
edit_scene.py -- Open the FULL interactive Isaac Sim GUI with this scene loaded.

Loads the apartment scene + Franka + cup + sink marker + cameras (clutter
hidden), opens the complete Isaac Sim editor (Stage tree + Property panel +
transform gizmos), and keeps it live so you can DRAG prims in the viewport and
READ their values. Physics is left paused so nothing falls while you edit.

While it runs it also PRINTS the world pose of the key prims every few seconds,
in config-ready format ([w,x,y,z] quats), so you can move something in the GUI
and copy the printed numbers straight into configs/cup_to_sink.yaml.

Run it, then open the virtual desktop (VNC) to see the Isaac Sim window:

    /isaac-sim/python.sh -m cup_to_sink.edit_scene \
        --config cup_to_sink/configs/cup_to_sink.yaml

How to use in the GUI:
  - Stage panel (top-right): click a prim to select it
      /World/Franka            -> the robot base (read Translate = base_position)
      /World/Objects/Cup       -> cup spawn
      /World/EditMarkers/sink  -> the sink target (drag it into the basin)
      /World/Cameras/front|left|right -> the cameras (drag to reframe)
  - Property panel (bottom-right): read/edit Transform > Translate / Orient.
  - Viewport 'Camera' menu: look THROUGH front/left/right/wrist to check framing.
  - Move a prim, then read the value printed in this terminal and paste it into
    the YAML (base_position / cup_spawn_position / sink_target_pose_world /
    cameras.*.position ; for a camera, keep look_at on the sink).

Close the Isaac Sim window to exit.
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np

_REPO_ROOT = Path(__file__).resolve().parents[1]


def _fmt(v, n=3):
    return "[" + ", ".join(f"{float(x):.{n}f}" for x in v) + "]"


def main() -> int:
    p = argparse.ArgumentParser(description="Interactive Isaac Sim scene editor.")
    p.add_argument("--config", default=str(_REPO_ROOT / "cup_to_sink/configs/cup_to_sink.yaml"))
    p.add_argument("--print-every", type=float, default=4.0,
                   help="Seconds between pose printouts (0 to disable).")
    args = p.parse_args()

    from cup_to_sink import sim_app as sim_app_mod
    sim_app_mod._force_utf8_stdio()
    # Full GUI: not headless, and DO NOT hide the UI (we want the editor panels).
    app = sim_app_mod.start(headless=False, hide_ui=False)

    from cup_to_sink import config as cfg_mod, env_builder
    from cup_to_sink.task import Task
    from isaacsim.core.prims import SingleXFormPrim
    from isaacsim.core.utils.prims import create_prim

    cfg = cfg_mod.load(args.config)
    env = env_builder.build_env(cfg, app)
    task = Task(env, cfg)
    task.reset(0)

    # Visible marker at the sink target so it can be seen and dragged.
    sink_xyz = [float(v) for v in cfg["scene"]["sink_target_pose_world"][:3]]
    create_prim(
        prim_path="/World/EditMarkers/sink", prim_type="Sphere",
        attributes={"radius": 0.03, "primvars:displayColor": [(1.0, 0.1, 0.1)]},
        position=tuple(sink_xyz),
    )

    # Stop the timeline so prims stay put and are freely movable in edit mode.
    try:
        import omni.timeline
        omni.timeline.get_timeline_interface().stop()
    except Exception:
        pass

    # Prims to report.
    watch = {
        "robot.base": "/World/Franka",
        "cup": cfg["scene"]["cup_prim_path"],
        "sink_marker": "/World/EditMarkers/sink",
        "cam.front": "/World/Cameras/front",
        "cam.left": "/World/Cameras/left",
        "cam.right": "/World/Cameras/right",
    }
    xforms = {k: SingleXFormPrim(v) for k, v in watch.items()}

    print("\n" + "=" * 70)
    print("Interactive editor ready. Open the virtual desktop to see Isaac Sim.")
    print("Drag prims in the viewport; values print here. Close the window to exit.")
    print("=" * 70 + "\n", flush=True)

    last = 0.0
    while app.is_running():
        app.update()
        if args.print_every > 0 and (time.time() - last) >= args.print_every:
            last = time.time()
            lines = ["--- current prim world poses (paste into YAML) ---"]
            for k, xf in xforms.items():
                try:
                    pos, quat = xf.get_world_pose()  # quat = [w,x,y,z]
                    if k.startswith("cam."):
                        lines.append(f"  {k:14s} position={_fmt(pos)}   (keep look_at on the sink)")
                    elif k == "robot.base":
                        lines.append(f"  base_position={_fmt(pos)}  base_quat={_fmt(quat,4)}")
                    else:
                        lines.append(f"  {k:14s} pos={_fmt(pos)} quat={_fmt(quat,4)}")
                except Exception:
                    pass
            print("\n".join(lines), flush=True)

    app.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
