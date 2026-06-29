"""
Smoke test for cup_to_sink executor.execute().

Boots Isaac Sim headless, builds the kitchen + Franka environment, resets the
task, then executes a 2-action sequence:
  1. gripper open (phase="open")
  2. move +0.10 m in Z (phase="lift")

Asserts:
  - recorder.num_steps > 0
  - HDF5 round-trip: /observations/joint_pos has (num_steps, 7) shape
  - HDF5 round-trip: /actions/policy_action_command has (num_steps, 8) shape
  - EE z-position increased (motion happened)

Key results written to /tmp/smoke_execute_result.txt to survive Isaac's
stdout fd-hijacking.

Run:
    cd /mnt/dev-tools/isaacsim-template
    /isaac-sim/python.sh -m cup_to_sink.tests.smoke_execute
"""
from __future__ import annotations

import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[2]
_RESULTS_FILE = Path("/tmp/smoke_execute_result.txt")
_HDF5_PATH = "/tmp/smoke_exec_ep.hdf5"


def main() -> None:
    import numpy as np

    from cup_to_sink.sim_app import start
    from cup_to_sink.config import load
    from cup_to_sink.env_builder import build_env
    from cup_to_sink.task import Task
    from cup_to_sink.planner import RMPFlowPlanner
    from cup_to_sink.dataset_writer import Recorder
    from cup_to_sink.executor import execute
    import cup_to_sink.skills as skills

    lines: list[str] = []

    def log(msg: str = "") -> None:
        print(msg)
        lines.append(msg)

    def flush_results(status: str) -> None:
        lines.insert(0, f"STATUS: {status}")
        _RESULTS_FILE.write_text("\n".join(lines) + "\n")

    # ── Boot ──────────────────────────────────────────────────────────────────
    log("=== smoke_execute: executor records dense steps ===")
    log("Starting Isaac Sim (headless)...")
    app = start(headless=True)

    cfg_path = _REPO_ROOT / "cup_to_sink/configs/cup_to_sink.yaml"
    log(f"Loading config: {cfg_path}")
    cfg = load(str(cfg_path))

    # ── Build env ─────────────────────────────────────────────────────────────
    log("Building environment (kitchen + Franka + cup + cameras)...")
    env = build_env(cfg, app)

    # ── Instantiate task and planner ──────────────────────────────────────────
    log("Instantiating Task and RMPFlowPlanner...")
    task = Task(env, cfg)
    planner = RMPFlowPlanner(env)

    log(f"  arm_idx:    {task.arm_idx.tolist()}")
    log(f"  finger_idx: {task.finger_idx.tolist()}")

    # ── Reset ─────────────────────────────────────────────────────────────────
    log("\nResetting task (seed=0)...")
    task.reset(0)
    planner.reset()

    # ── Render a few frames so camera buffers are populated ───────────────────
    log("Rendering 5 frames for camera buffer population...")
    for _ in range(5):
        env.world.step(render=True)

    # ── Read initial EE pose (planner right_gripper frame) ────────────────────
    e0 = planner.get_ee_pose().copy()
    log(f"\nInitial EE pose (right_gripper frame): {np.round(e0, 4).tolist()}")
    log(f"  z = {e0[2]:.4f} m")

    # ── Build action sequence ─────────────────────────────────────────────────
    # 1. Open gripper (it is already open after reset, but this records steps)
    actions: list[dict] = [
        {"type": "gripper", "width": 0.08, "phase": "open"},
    ]
    # 2. Move +0.10 m in Z using panda_hand pose as base
    #    (right_gripper vs panda_hand frame difference is small; motion happens)
    ee_world_now = task.ee_pose_world()  # panda_hand frame
    lift_actions = skills.move_by_displacement(
        ee_world_now, 0.0, 0.0, 0.10, phase="lift"
    )
    actions.extend(lift_actions)

    log(f"\nAction sequence ({len(actions)} actions):")
    for i, a in enumerate(actions):
        if a["type"] == "gripper":
            log(f"  [{i}] gripper width={a['width']}  phase={a['phase']}")
        else:
            log(f"  [{i}] move  target_z={a['ee_pose7d'][2]:.4f}  phase={a['phase']}")

    # ── Execute ───────────────────────────────────────────────────────────────
    log("\nExecuting actions...")
    rec = Recorder()
    result = execute(env, task, planner, actions, rec, cfg)

    log(f"\nExecute result: {result}")
    log(f"  completed:  {result['completed']}")
    log(f"  num_steps:  {result['steps']}")
    log(f"  rec.num_steps: {rec.num_steps}")

    # ── Read final EE pose ────────────────────────────────────────────────────
    e1 = planner.get_ee_pose().copy()
    dz = e1[2] - e0[2]
    log(f"\nFinal EE pose (right_gripper frame): {np.round(e1, 4).tolist()}")
    log(f"  z change: {dz:+.4f} m  (target was +0.10 m)")

    # ── Assertions ────────────────────────────────────────────────────────────
    log("\n--- Assertions ---")

    assert rec.num_steps > 0, (
        f"FAIL: recorder has 0 steps — expected > 0 (gripper_steps={cfg['motion']['gripper_steps']})"
    )
    log(f"  rec.num_steps > 0:  PASS  ({rec.num_steps} steps)")

    assert dz > 0.0, (
        f"FAIL: EE z did not increase — dz={dz:.4f} m, "
        f"expected > 0 (lift +0.10 m target was commanded)"
    )
    log(f"  EE z increased:     PASS  (dz={dz:+.4f} m)")

    # ── Write HDF5 ────────────────────────────────────────────────────────────
    log(f"\nWriting HDF5 to {_HDF5_PATH}...")
    meta = {
        "task_name": "cup_to_sink",
        "language_instruction": cfg["task"]["instruction"],
        "seed": 0,
        "success": False,
    }
    rec.write(_HDF5_PATH, meta=meta)
    log(f"  HDF5 written: {_HDF5_PATH}")

    # ── HDF5 round-trip assertions ────────────────────────────────────────────
    log("\n--- HDF5 round-trip ---")
    import h5py

    with h5py.File(_HDF5_PATH, "r") as f:
        jp_shape = f["/observations/joint_pos"].shape
        pac_shape = f["/actions/policy_action_command"].shape
        log(f"  /observations/joint_pos shape:          {jp_shape}")
        log(f"  /actions/policy_action_command shape:   {pac_shape}")

    expected_rows = rec.num_steps
    assert jp_shape == (expected_rows, 7), (
        f"FAIL: /observations/joint_pos shape {jp_shape} != ({expected_rows}, 7)"
    )
    log(f"  /observations/joint_pos ({expected_rows}, 7):       PASS")

    assert pac_shape == (expected_rows, 8), (
        f"FAIL: /actions/policy_action_command shape {pac_shape} != ({expected_rows}, 8)"
    )
    log(f"  /actions/policy_action_command ({expected_rows}, 8): PASS")

    # ── Summary ───────────────────────────────────────────────────────────────
    log("\n=== smoke_execute SUMMARY ===")
    log(f"  num_steps:                    {rec.num_steps}")
    log(f"  EE z change:                  {dz:+.4f} m")
    log(f"  execute completed:            {result['completed']}")
    log(f"  HDF5 /observations/joint_pos: {jp_shape}")
    log(f"  HDF5 /actions/policy_cmd:     {pac_shape}")
    log("ALL ASSERTIONS PASSED")

    flush_results("PASS")
    print(f"\nResults written to: {_RESULTS_FILE}")

    app.close()
    print("\nsmoke_execute PASSED.")


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        import traceback
        msg = f"STATUS: FAIL\n{traceback.format_exc()}"
        _RESULTS_FILE.write_text(msg)
        print(msg, file=sys.stderr)
        sys.exit(1)
