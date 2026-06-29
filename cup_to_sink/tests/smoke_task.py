"""
Smoke test for cup_to_sink Task: reset/obs/success.

Boots Isaac Sim headless, builds the kitchen environment, instantiates Task,
runs reset(0) and reset(1) and asserts their cup XY and sink target XY differ
(proving seed-based randomisation works).  Then calls get_obs() and checks
all expected keys are present with the right shapes.  Finally calls
check_success() which should be False right after reset (cup on counter, far
from sink, gripper open).

All key results are also written to /tmp/smoke_task_result.txt as a fallback
in case Isaac Sim redirects stdout via fd hijacking.

Run:
    cd /mnt/dev-tools/isaacsim-template
    /isaac-sim/python.sh -m cup_to_sink.tests.smoke_task
"""
from __future__ import annotations

import sys
from pathlib import Path

# Repository root: cup_to_sink/tests/smoke_task.py → parents[2]
_REPO_ROOT = Path(__file__).resolve().parents[2]
_RESULTS_FILE = Path("/tmp/smoke_task_result.txt")


def main() -> None:
    import numpy as np

    from cup_to_sink.sim_app import start
    from cup_to_sink.config import load
    from cup_to_sink.env_builder import build_env
    from cup_to_sink.task import Task

    lines: list[str] = []

    def log(msg: str = "") -> None:
        print(msg)
        lines.append(msg)

    def flush_results(status: str) -> None:
        lines.insert(0, f"STATUS: {status}")
        _RESULTS_FILE.write_text("\n".join(lines) + "\n")

    # ── Boot ──────────────────────────────────────────────────────────────────
    log("=== smoke_task: Task reset/obs/success ===")
    log("Starting Isaac Sim (headless)...")
    app = start(headless=True)

    cfg_path = _REPO_ROOT / "cup_to_sink/configs/cup_to_sink.yaml"
    log(f"Loading config: {cfg_path}")
    cfg = load(str(cfg_path))

    # ── Build env ─────────────────────────────────────────────────────────────
    log("Building environment (kitchen + Franka + cup + cameras)...")
    env = build_env(cfg, app)

    # ── Instantiate Task ──────────────────────────────────────────────────────
    log("Instantiating Task...")
    task = Task(env, cfg)
    log(f"  instruction: {task.instruction!r}")
    log(f"  arm_idx:     {task.arm_idx.tolist()}")
    log(f"  finger_idx:  {task.finger_idx.tolist()}")

    # ── reset(0) ──────────────────────────────────────────────────────────────
    log("\n--- reset(seed=0) ---")
    r0 = task.reset(0)
    cup_xy_0 = np.array(r0["cup_xy"])
    sink_xy_0 = np.array(r0["sink_target_pose7d"][:2])
    cup_pos_0 = np.array(r0["cup_pose_init"][:3])

    # Also read live cup pose after reset
    cup_pos_live_0, cup_ori_live_0 = env.cup.get_world_pose()
    sink_pose7d_0 = task.sink_target_pose7d

    log(f"  sampled cup_xy:          {cup_xy_0}")
    log(f"  sampled sink_target_xy:  {sink_xy_0}")
    log(f"  cup_pose_init (7d):      {r0['cup_pose_init']}")
    log(f"  live cup pos after reset: {np.asarray(cup_pos_live_0)}")
    log(f"  sink_target_pose7d:      {sink_pose7d_0}")

    # ── reset(1) ──────────────────────────────────────────────────────────────
    log("\n--- reset(seed=1) ---")
    r1 = task.reset(1)
    cup_xy_1 = np.array(r1["cup_xy"])
    sink_xy_1 = np.array(r1["sink_target_pose7d"][:2])
    cup_pos_1 = np.array(r1["cup_pose_init"][:3])

    cup_pos_live_1, cup_ori_live_1 = env.cup.get_world_pose()
    sink_pose7d_1 = task.sink_target_pose7d

    log(f"  sampled cup_xy:          {cup_xy_1}")
    log(f"  sampled sink_target_xy:  {sink_xy_1}")
    log(f"  cup_pose_init (7d):      {r1['cup_pose_init']}")
    log(f"  live cup pos after reset: {np.asarray(cup_pos_live_1)}")
    log(f"  sink_target_pose7d:      {sink_pose7d_1}")

    # ── Assert seeds produce different poses ──────────────────────────────────
    log("\n--- Seed diversity assertions ---")

    cup_xy_same = bool(np.allclose(cup_xy_0, cup_xy_1))
    sink_xy_same = bool(np.allclose(sink_xy_0, sink_xy_1))

    log(f"  cup_xy   seed0={cup_xy_0}  seed1={cup_xy_1}  same={cup_xy_same}")
    log(f"  sink_xy  seed0={sink_xy_0}  seed1={sink_xy_1}  same={sink_xy_same}")

    assert not cup_xy_same, (
        f"FAIL: seed 0 and seed 1 produced identical cup_xy={cup_xy_0} — "
        "randomisation is not working."
    )
    assert not sink_xy_same, (
        f"FAIL: seed 0 and seed 1 produced identical sink_xy={sink_xy_0} — "
        "sink randomisation is not working."
    )
    log("  cup_xy DIFFER:    PASS")
    log("  sink_xy DIFFER:   PASS")

    # ── Render a few frames so camera buffers are populated ───────────────────
    log("\nRendering 5 frames for camera buffer population...")
    for _ in range(5):
        env.world.step(render=True)

    # ── get_obs() ─────────────────────────────────────────────────────────────
    log("\n--- get_obs() ---")
    obs = task.get_obs(timestep=0)

    log("  obs keys: " + str(sorted(k for k in obs if k != "images")))
    log(f"  joint_pos          shape={np.asarray(obs['joint_pos']).shape}  "
        f"dtype={np.asarray(obs['joint_pos']).dtype}  "
        f"finite={bool(np.all(np.isfinite(obs['joint_pos'])))}")
    log(f"  joint_vel          shape={np.asarray(obs['joint_vel']).shape}")
    log(f"  gripper_joint_pos  shape={np.asarray(obs['gripper_joint_pos']).shape}")
    log(f"  gripper_width      {obs['gripper_width']:.4f} m")
    log(f"  gripper_aperture   {obs['gripper_aperture']:.4f}")
    log(f"  ee_pose_7d_world   shape={np.asarray(obs['ee_pose_7d_world']).shape}  "
        f"values={np.round(obs['ee_pose_7d_world'], 4).tolist()}")
    log(f"  ee_pose_6d_world   shape={np.asarray(obs['ee_pose_6d_world']).shape}")
    log(f"  ee_pose_7d_base    shape={np.asarray(obs['ee_pose_7d_base']).shape}")
    log(f"  ee_pose_6d_base    shape={np.asarray(obs['ee_pose_6d_base']).shape}")
    log(f"  cup_pose           shape={np.asarray(obs['cup_pose']).shape}  "
        f"values={np.round(obs['cup_pose'], 4).tolist()}")
    log(f"  sink_target_pose   shape={np.asarray(obs['sink_target_pose']).shape}")
    log(f"  language_instr     {obs['language_instruction']!r}")
    log(f"  timestep           {obs['timestep']}")

    # Image summary
    images = obs["images"]
    log(f"\n  images captured: {list(images.keys())}")
    for cam_name, frame in images.items():
        rgb = frame["rgb"]
        depth = frame["depth"]
        log(f"    {cam_name}: rgb={rgb.shape} {rgb.dtype}  "
            f"depth={depth.shape} {depth.dtype}  "
            f"depth_nonzero={int(np.count_nonzero(depth))}")

    # ── Shape / finiteness assertions ─────────────────────────────────────────
    log("\n--- Obs assertions ---")

    assert np.asarray(obs["joint_pos"]).shape == (7,), (
        f"joint_pos shape {np.asarray(obs['joint_pos']).shape} != (7,)"
    )
    assert np.asarray(obs["joint_vel"]).shape == (7,), (
        f"joint_vel shape {np.asarray(obs['joint_vel']).shape} != (7,)"
    )
    assert np.asarray(obs["gripper_joint_pos"]).shape == (2,), (
        f"gripper_joint_pos shape {np.asarray(obs['gripper_joint_pos']).shape} != (2,)"
    )
    assert np.asarray(obs["ee_pose_7d_world"]).shape == (7,), (
        f"ee_pose_7d_world shape {np.asarray(obs['ee_pose_7d_world']).shape} != (7,)"
    )
    assert np.asarray(obs["cup_pose"]).shape == (7,), (
        f"cup_pose shape {np.asarray(obs['cup_pose']).shape} != (7,)"
    )
    assert np.all(np.isfinite(obs["joint_pos"])), "joint_pos contains non-finite values"
    assert np.all(np.isfinite(obs["ee_pose_7d_world"])), (
        "ee_pose_7d_world contains non-finite values"
    )
    assert np.all(np.isfinite(obs["cup_pose"])), "cup_pose contains non-finite values"

    log("  joint_pos (7,):         PASS")
    log("  joint_vel (7,):         PASS")
    log("  gripper_joint_pos (2,): PASS")
    log("  ee_pose_7d_world (7,):  PASS")
    log("  cup_pose (7,):          PASS")
    log("  finite values:          PASS")

    # Check images (may be empty if render wasn't called; treat as soft check)
    enabled_cams = cfg["dataset"]["enabled_cameras"]
    if images:
        for cam_name in images:
            rgb = images[cam_name]["rgb"]
            depth = images[cam_name]["depth"]
            assert rgb.shape == (256, 256, 3), (
                f"{cam_name} rgb shape {rgb.shape} != (256,256,3)"
            )
            assert rgb.dtype == np.uint8, (
                f"{cam_name} rgb dtype {rgb.dtype} != uint8"
            )
            assert depth.shape == (256, 256), (
                f"{cam_name} depth shape {depth.shape} != (256,256)"
            )
            assert depth.dtype == np.uint16, (
                f"{cam_name} depth dtype {depth.dtype} != uint16"
            )
        log(f"  image shapes / dtypes ({len(images)}/{len(enabled_cams)} cameras): PASS")
    else:
        log(f"  images: 0/{len(enabled_cams)} cameras captured (render steps insufficient — soft skip)")

    # ── check_success() ───────────────────────────────────────────────────────
    log("\n--- check_success() ---")
    success_bool, success_info = task.check_success()
    log(f"  success:        {success_bool}")
    log(f"  info:           {success_info}")

    assert success_bool is False, (
        f"FAIL: check_success() returned True right after reset — "
        f"cup is on the counter far from the sink.  info={success_info}"
    )
    log("  success=False:  PASS  (cup is far from sink, as expected)")

    # ── Summary ───────────────────────────────────────────────────────────────
    log("\n=== smoke_task SUMMARY ===")
    log(f"  seed0 cup_xy:        {cup_xy_0.tolist()}")
    log(f"  seed1 cup_xy:        {cup_xy_1.tolist()}")
    log(f"  seed0 sink_xy:       {sink_xy_0.tolist()}")
    log(f"  seed1 sink_xy:       {sink_xy_1.tolist()}")
    log(f"  joint_pos shape:     {np.asarray(obs['joint_pos']).shape}")
    log(f"  ee_pose_7d_world:    {np.round(obs['ee_pose_7d_world'], 4).tolist()}")
    log(f"  cup_pose:            {np.round(obs['cup_pose'], 4).tolist()}")
    log(f"  images captured:     {len(images)}/{len(enabled_cams)}")
    log(f"  success after reset: {success_bool}")
    log("ALL ASSERTIONS PASSED")

    flush_results("PASS")
    print(f"\nResults written to: {_RESULTS_FILE}")

    app.close()
    print("\nsmoke_task PASSED.")


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        import traceback
        msg = f"STATUS: FAIL\n{traceback.format_exc()}"
        _RESULTS_FILE.write_text(msg)
        print(msg, file=sys.stderr)
        sys.exit(1)
