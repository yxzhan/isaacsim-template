"""
Smoke test for cup_to_sink env_builder.

Boots Isaac Sim headless, builds the full kitchen environment, steps 60 frames,
then captures RGB-D from all 4 cameras and asserts the expected shapes/dtypes.

Run:
    cd /mnt/dev-tools/isaacsim-template
    /isaac-sim/python.sh -m cup_to_sink.tests.smoke_env
"""
from __future__ import annotations

import sys
from pathlib import Path

# Repository root: this file is at cup_to_sink/tests/smoke_env.py
_REPO_ROOT = Path(__file__).resolve().parents[2]


def main() -> None:
    # -- Delayed imports (must be after SimulationApp starts) ------------------
    from cup_to_sink.sim_app import start
    from cup_to_sink.config import load
    from cup_to_sink.env_builder import build_env
    from cup_to_sink.cameras import capture

    import numpy as np

    # -- Boot ------------------------------------------------------------------
    print("=== cup_to_sink smoke_env ===")
    print("Starting Isaac Sim (headless)...")
    app = start(headless=True)

    cfg_path = _REPO_ROOT / "cup_to_sink/configs/cup_to_sink.yaml"
    print(f"Loading config: {cfg_path}")
    cfg = load(str(cfg_path))

    # -- Build environment -----------------------------------------------------
    print("Building environment (kitchen + Franka + cup + cameras)...")
    env = build_env(cfg, app)

    # -- Step simulation -------------------------------------------------------
    # Physics-only steps to let cup settle on counter
    print("Stepping 57 physics frames...")
    for _ in range(57):
        env.world.step(render=False)

    # Render steps to populate camera frame buffers
    print("Stepping 3 render frames (populating camera buffers)...")
    for _ in range(3):
        env.world.step(render=True)

    # -- Diagnostics -----------------------------------------------------------
    print("\n=== Smoke Test Results ===")

    # Franka DOF
    dof = env.franka.num_dof
    print(f"franka num_dof: {dof}  (expected 9)")

    # Cup world pose
    cup_pos, cup_ori = env.cup.get_world_pose()
    print(f"cup world pose: pos={cup_pos}  ori={cup_ori}")
    cup_z = float(cup_pos[2])
    counter_top_z = float(cfg["scene"]["counter_top_z"])
    print(f"  cup z={cup_z:.4f}  counter_top_z={counter_top_z:.4f}  "
          f"above_counter={cup_z - counter_top_z:.4f} m")

    # Sink target
    from pxr import UsdGeom, Usd  # safe: app is running
    sink_prim = env.stage.GetPrimAtPath(env.sink_path)
    sink_ok = sink_prim and sink_prim.IsValid()
    if sink_ok:
        xfc = UsdGeom.XformCache()
        t = xfc.GetLocalToWorldTransform(sink_prim).ExtractTranslation()
        vis_attr = UsdGeom.Imageable(sink_prim).GetVisibilityAttr()
        vis_val = vis_attr.Get() if vis_attr else "?"
        print(f"sink path: {env.sink_path}  translate=({t[0]:.3f}, {t[1]:.3f}, {t[2]:.3f})"
              f"  visibility={vis_val}")
    else:
        print(f"sink path: {env.sink_path}  WARNING: prim not found or invalid")

    # -- Camera capture --------------------------------------------------------
    enabled = cfg["dataset"]["enabled_cameras"]
    caps = capture(env.cams, enabled)

    print(f"\n=== Camera Captures ({len(caps)}/{len(enabled)} cameras returned frames) ===")
    all_ok = True
    for name in enabled:
        if name not in caps:
            print(f"  {name}: SKIPPED (frame not yet populated)")
            continue
        data = caps[name]
        rgb = data["rgb"]
        depth = data["depth"]
        depth_nonzero = int(np.count_nonzero(depth))
        print(
            f"  {name}: rgb={rgb.shape} {rgb.dtype}  "
            f"depth={depth.shape} {depth.dtype}  "
            f"depth min={int(depth.min())} max={int(depth.max())}  "
            f"nonzero={depth_nonzero}"
        )
        try:
            assert rgb.shape == (256, 256, 3), f"rgb shape {rgb.shape} != (256,256,3)"
            assert rgb.dtype == np.uint8, f"rgb dtype {rgb.dtype} != uint8"
            assert depth.shape == (256, 256), f"depth shape {depth.shape} != (256,256)"
            assert depth.dtype == np.uint16, f"depth dtype {depth.dtype} != uint16"
        except AssertionError as exc:
            print(f"    ASSERTION FAILED: {exc}")
            all_ok = False

    # -- Summary ---------------------------------------------------------------
    print("\n=== Summary ===")
    print(f"franka dof: {dof}")
    print(f"cup z: {cup_z:.4f} m  (counter top: {counter_top_z:.4f} m)")
    print(f"cameras captured: {len(caps)}/{len(enabled)}")
    passes = all_ok and len(caps) == len(enabled)
    print(f"all assertions: {'PASS' if passes else 'FAIL'}")

    app.close()

    if not passes:
        print("\nSmoke test FAILED.", file=sys.stderr)
        sys.exit(1)
    print("\nSmoke test PASSED.")


if __name__ == "__main__":
    main()
