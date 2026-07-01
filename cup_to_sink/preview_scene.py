"""
preview_scene.py -- Fast scene/camera calibration preview.

Builds the FULL env exactly as collection does (scene + Franka + cup + sink +
cameras, with clutter hidden), settles, and saves a montage of what every
enabled camera sees -- WITHOUT running an episode. Use this to iterate on the
config (Franka base, cup spawn, sink target, camera position/look_at/fov).

    /isaac-sim/python.sh -m cup_to_sink.preview_scene \
        --config cup_to_sink/configs/cup_to_sink.yaml \
        --out datasets/cup_to_sink/preview/scene.png

    # Live window on the virtual desktop instead of a PNG:
    /isaac-sim/python.sh -m cup_to_sink.preview_scene --viewer

Edit configs/cup_to_sink.yaml, re-run, look at the PNG, repeat.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

_REPO_ROOT = Path(__file__).resolve().parents[1]


def main() -> int:
    p = argparse.ArgumentParser(description="Preview the scene + camera framing.")
    p.add_argument("--config", default=str(_REPO_ROOT / "cup_to_sink/configs/cup_to_sink.yaml"))
    p.add_argument("--out", default=str(_REPO_ROOT / "datasets/cup_to_sink/preview/scene.png"))
    p.add_argument("--viewer", action="store_true",
                   help="Run non-headless (window on the virtual desktop).")
    p.add_argument("--settle", type=int, default=60, help="Physics steps before capture.")
    args = p.parse_args()

    from cup_to_sink import sim_app as sim_app_mod
    sim_app_mod._force_utf8_stdio()
    app = sim_app_mod.start(headless=not args.viewer)

    from cup_to_sink import config as cfg_mod, env_builder
    from cup_to_sink.cameras import capture
    from cup_to_sink.task import Task

    cfg = cfg_mod.load(args.config)
    env = env_builder.build_env(cfg, app)
    task = Task(env, cfg)
    task.reset(0)  # place the cup + sink so the preview matches an episode start

    for _ in range(max(1, args.settle)):
        env.world.step(render=True)

    enabled = list(cfg["dataset"]["enabled_cameras"])
    caps = capture(env.cams, enabled)

    # Print the calibrated layout for reference.
    print("[preview] calibrated layout:")
    print(f"  robot.base_position   = {cfg['robot']['base_position']}")
    print(f"  cup_spawn_position    = {cfg['scene']['cup_spawn_position']}")
    print(f"  sink_target_pose_world= {cfg['scene']['sink_target_pose_world']}")
    for name in enabled:
        c = cfg["cameras"].get(name, {})
        print(f"  camera[{name}]: position={c.get('position')} "
              f"look_at={c.get('look_at', c.get('quat'))} fov={c.get('fov')}")

    # Build an RGB montage (one column per camera).
    from PIL import Image
    tiles = []
    for name in enabled:
        if name in caps:
            tiles.append(caps[name]["rgb"])
        else:
            h = cfg["cameras"][name]["height"]
            w = cfg["cameras"][name]["width"]
            tiles.append(np.zeros((h, w, 3), np.uint8))
    if tiles:
        out_path = Path(args.out)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        Image.fromarray(np.concatenate(tiles, axis=1)).save(str(out_path))
        print(f"[preview] saved camera montage ({enabled}) -> {out_path}")

    app.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
