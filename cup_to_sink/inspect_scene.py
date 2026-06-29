"""
inspect_scene.py — Boot Isaac Sim, load the kitchen USD, and print bounding boxes
for all prims whose name/path matches kitchen features (sink, counter, etc.).

Run as:
    /isaac-sim/python.sh -m cup_to_sink.inspect_scene

Outputs:
  - Per-prim: path, type, world AABB (min xyz / max xyz / center xyz / size xyz)
  - Overall kitchen AABB
  - Ground plane z
  - SUGGESTED config values: sink_target_pose_world, robot base_position, cup spawn region
"""

import os
import re
import sys
from pathlib import Path

# Resolve repo root so USD paths are absolute regardless of cwd
REPO_ROOT = Path(__file__).resolve().parent.parent

KITCHEN_USD = str(REPO_ROOT / "usd" / "kitchen" / "kitchen.usd")
GROUND_USD  = str(REPO_ROOT / "usd" / "Grid" / "default_environment.usd")

# Prim name/path patterns to report (case-insensitive)
PATTERN = re.compile(
    r"sink|counter|basin|faucet|table|worktop|cabinet|stove|top|island",
    re.IGNORECASE,
)


def main() -> None:
    # Force line-buffered stdout so summary sections are not lost when Isaac Sim
    # terminates the process before the OS pipe buffer is flushed.
    import io
    try:
        sys.stdout.reconfigure(line_buffering=True)
    except (AttributeError, io.UnsupportedOperation):
        pass

    # --- 1. Start Isaac Sim ---
    from cup_to_sink.sim_app import start
    simulation_app = start(headless=True)

    # All isaacsim/pxr/omni imports AFTER SimulationApp is constructed
    import omni
    import numpy as np
    from isaacsim.core.api import World
    from isaacsim.core.utils.stage import add_reference_to_stage
    from pxr import Usd, UsdGeom, Gf

    # --- 2. Build world & load stage ---
    world = World(stage_units_in_meters=1.0, physics_dt=1 / 60, rendering_dt=1 / 30)
    world.reset()

    stage = omni.usd.get_context().get_stage()

    # Load ground plane
    ground_prim = add_reference_to_stage(usd_path=GROUND_USD, prim_path="/World/Ground")
    print(f"[inspect] Loaded ground: {GROUND_USD}")

    # Load kitchen
    kitchen_prim = add_reference_to_stage(usd_path=KITCHEN_USD, prim_path="/World/Kitchen")
    print(f"[inspect] Loaded kitchen: {KITCHEN_USD}")

    # Step a few times so physics/transforms settle
    for _ in range(10):
        world.step(render=False)

    stage = omni.usd.get_context().get_stage()

    # --- 3. BBox traversal ---
    time_code = Usd.TimeCode.Default()
    bbox_cache = UsdGeom.BBoxCache(
        time_code,
        [UsdGeom.Tokens.default_, UsdGeom.Tokens.render],
        useExtentsHint=True,
    )

    print("\n" + "=" * 72, flush=True)
    print("PRIM BOUNDING BOXES  (world space, meters)", flush=True)
    print("=" * 72, flush=True)

    matched_prims = []

    try:
        for prim in stage.Traverse():
            path_str = str(prim.GetPath())
            name_str = prim.GetName()

            if not PATTERN.search(path_str) and not PATTERN.search(name_str):
                continue

            try:
                bbox = bbox_cache.ComputeWorldBound(prim)
                aligned = bbox.ComputeAlignedRange()
                mn = aligned.GetMin()
                mx = aligned.GetMax()

                # Skip degenerate / uncomputed boxes
                if mn == mx or (mx[0] - mn[0] < 1e-6 and mx[1] - mn[1] < 1e-6 and mx[2] - mn[2] < 1e-6):
                    continue

                center = (
                    (mn[0] + mx[0]) / 2,
                    (mn[1] + mx[1]) / 2,
                    (mn[2] + mx[2]) / 2,
                )
                size = (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])

                print(f"\nPRIM  : {path_str}")
                print(f"  type  : {prim.GetTypeName()}")
                print(f"  min   : ({mn[0]:.4f}, {mn[1]:.4f}, {mn[2]:.4f})")
                print(f"  max   : ({mx[0]:.4f}, {mx[1]:.4f}, {mx[2]:.4f})")
                print(f"  center: ({center[0]:.4f}, {center[1]:.4f}, {center[2]:.4f})")
                print(f"  size  : ({size[0]:.4f}, {size[1]:.4f}, {size[2]:.4f})")

                matched_prims.append({
                    "path": path_str,
                    "type": prim.GetTypeName(),
                    "min": mn,
                    "max": mx,
                    "center": center,
                    "size": size,
                })
            except BaseException as exc:
                print(f"  [warn] {path_str}: {exc}")
    except BaseException as exc:
        print(f"\n[warn] stage.Traverse() aborted early: {exc}  — proceeding to summary sections")

    # --- 4. Overall kitchen AABB ---
    print("\n" + "=" * 72, flush=True)
    print("OVERALL KITCHEN AABB", flush=True)
    print("=" * 72, flush=True)
    kitchen_prim_obj = stage.GetPrimAtPath("/World/Kitchen")
    kmn = kmx = kc = None
    try:
        kb = bbox_cache.ComputeWorldBound(kitchen_prim_obj)
        kar = kb.ComputeAlignedRange()
        kmn, kmx = kar.GetMin(), kar.GetMax()
        kc = ((kmn[0]+kmx[0])/2, (kmn[1]+kmx[1])/2, (kmn[2]+kmx[2])/2)
        print(f"  min   : ({kmn[0]:.4f}, {kmn[1]:.4f}, {kmn[2]:.4f})")
        print(f"  max   : ({kmx[0]:.4f}, {kmx[1]:.4f}, {kmx[2]:.4f})")
        print(f"  center: ({kc[0]:.4f}, {kc[1]:.4f}, {kc[2]:.4f})")
    except BaseException as exc:
        print(f"  [warn] {exc}")

    # --- 5. Ground z ---
    print("\n" + "=" * 72, flush=True)
    print("GROUND PLANE", flush=True)
    print("=" * 72, flush=True)
    ground_prim_obj = stage.GetPrimAtPath("/World/Ground")
    ground_z = 0.0
    try:
        gb = bbox_cache.ComputeWorldBound(ground_prim_obj)
        gar = gb.ComputeAlignedRange()
        gmn, gmx = gar.GetMin(), gar.GetMax()
        ground_z = gmx[2]  # top surface of the ground mesh
        print(f"  min   : ({gmn[0]:.4f}, {gmn[1]:.4f}, {gmn[2]:.4f})")
        print(f"  max   : ({gmx[0]:.4f}, {gmx[1]:.4f}, {gmx[2]:.4f})")
        print(f"  ground top z = {ground_z:.4f}")
    except BaseException as exc:
        print(f"  [warn] {exc}")

    # --- 6. Compute suggested config values ---
    print("\n" + "=" * 72, flush=True)
    print("SUGGESTED CONFIG VALUES", flush=True)
    print("=" * 72, flush=True)

    try:
        # Find best sink/basin prim (prefer path containing 'sink' or 'basin')
        sink_prims = [p for p in matched_prims
                      if re.search(r"sink|basin", p["path"], re.IGNORECASE)]
        counter_prims = [p for p in matched_prims
                         if re.search(r"counter|worktop|island|table|stove|top", p["path"], re.IGNORECASE)]

        chosen_sink = None
        chosen_counter = None

        if sink_prims:
            # Pick the prim with the largest volume (the enclosing sink_area prim)
            sink_prims_sorted = sorted(sink_prims,
                                       key=lambda p: p["size"][0] * p["size"][1] * p["size"][2],
                                       reverse=True)
            chosen_sink = sink_prims_sorted[0]
            print(f"\n  [sink] best match: {chosen_sink['path']}")
            # Target: center of basin top surface
            sx = chosen_sink["center"][0]
            sy = chosen_sink["center"][1]
            sz = chosen_sink["max"][2] - 0.05  # slightly below basin rim
            print(f"  sink_target_pose_world  = [{sx:.4f}, {sy:.4f}, {sz:.4f}]")
        elif counter_prims:
            chosen_counter = sorted(counter_prims,
                                    key=lambda p: p["size"][0] * p["size"][1],
                                    reverse=True)[0]
            sx = chosen_counter["center"][0]
            sy = chosen_counter["center"][1]
            sz = chosen_counter["max"][2] + 0.02  # on top of counter
            print(f"\n  [NO SINK FOUND] using counter: {chosen_counter['path']}")
            print(f"  sink_target_pose_world  = [{sx:.4f}, {sy:.4f}, {sz:.4f}]")
        else:
            # Fall back to kitchen center
            if kc:
                sx, sy = kc[0], kc[1]
                sz = (kmx[2] + kc[2]) / 2 if kmx else 0.9
            else:
                sx, sy, sz = 0.0, 0.0, 0.9
            print(f"\n  [NO SINK/COUNTER FOUND] falling back to kitchen center")
            print(f"  sink_target_pose_world  = [{sx:.4f}, {sy:.4f}, {sz:.4f}]")

        # Franka base: ~0.55 m in front of (toward -Y) sink at counter-base height
        # "In front" is assumed to be -Y direction (robot faces +Y)
        if chosen_sink:
            ref_z = chosen_sink["min"][2]          # base of sink == approximate counter top
        elif chosen_counter:
            ref_z = chosen_counter["max"][2]       # top of counter surface
        else:
            ref_z = ground_z

        franka_base_y = sy - 0.55  # 0.55 m closer to robot on Y axis
        franka_base = [round(sx, 4), round(franka_base_y, 4), round(ref_z, 4)]
        print(f"\n  robot.base_position = {franka_base}")
        print(f"  (Franka placed {0.55:.2f} m from sink/target along -Y, base z={ref_z:.4f})")

        # Cup spawn: small patch on counter between Franka and sink (~0.3-0.5 m from Franka)
        # cup_xlim / cup_ylim are offsets added to a cup base xy.
        # Base xy for cup spawn = Franka base xy offset by +0.35..+0.45 in +Y
        cup_base_x = franka_base[0]
        cup_base_y = franka_base[1] + 0.40  # 0.4 m in front of Franka toward sink
        cup_z = ref_z  # cup sits on the counter

        print(f"\n  cup spawn base xy (world) = [{cup_base_x:.4f}, {cup_base_y:.4f}]")
        print(f"  cup_xlim (offsets from base_x) = [-0.10, 0.10]  # ±0.1 m along X")
        print(f"  cup_ylim (offsets from base_y) = [-0.05, 0.05]  # ±0.05 m along Y")
        print(f"  counter_top_z = {cup_z:.4f}  # cup spawn z")

        # Reachability check: Franka reach ~0.85 m
        dist_franka_to_sink = ((sx - franka_base[0])**2 + (sy - franka_base[1])**2)**0.5
        dist_franka_to_cup  = 0.40  # by construction
        print(f"\n  Reachability check (Franka reach ~0.85 m):")
        print(f"    Franka-to-sink distance = {dist_franka_to_sink:.3f} m  {'OK' if dist_franka_to_sink < 0.85 else 'TOO FAR'}")
        print(f"    Franka-to-cup distance  = {dist_franka_to_cup:.3f} m  {'OK' if dist_franka_to_cup < 0.85 else 'TOO FAR'}")
    except BaseException as exc:
        print(f"  [warn] SUGGESTED CONFIG section failed: {exc}")

    print("\n" + "=" * 72, flush=True)
    print("INSPECT DONE", flush=True)
    print("=" * 72, flush=True)

    simulation_app.close()


if __name__ == "__main__":
    main()
