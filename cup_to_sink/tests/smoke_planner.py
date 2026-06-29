"""
Smoke test for cup_to_sink RMPFlowPlanner.

Boots Isaac Sim headless, builds the full kitchen + Franka environment, sets
an EE target 0.20 m above the cup with a top-down (gripper pointing down)
orientation, and drives the Franka there using RMPFlowPlanner until convergence
or a 600-step timeout.  Asserts final position error < 0.03 m.

Run:
    cd /mnt/dev-tools/isaacsim-template
    /isaac-sim/python.sh -m cup_to_sink.tests.smoke_planner
"""
from __future__ import annotations

import sys
from pathlib import Path

# Repository root: cup_to_sink/tests/smoke_planner.py → parents[2]
_REPO_ROOT = Path(__file__).resolve().parents[2]
# Results written here so they survive stdout fd-hijacking by Isaac Sim
_RESULTS_FILE = _REPO_ROOT / ".superpowers/sdd/smoke_planner_results.txt"


def main() -> None:
    import numpy as np

    from cup_to_sink.sim_app import start
    from cup_to_sink.config import load
    from cup_to_sink.env_builder import build_env
    from cup_to_sink.planner import RMPFlowPlanner
    from cup_to_sink.transforms import axis_angle_to_quat

    # ── Boot ──────────────────────────────────────────────────────────────────
    print("=== cup_to_sink smoke_planner ===")
    print("Starting Isaac Sim (headless)...")
    app = start(headless=True)

    cfg_path = _REPO_ROOT / "cup_to_sink/configs/cup_to_sink.yaml"
    print(f"Loading config: {cfg_path}")
    cfg = load(str(cfg_path))

    print("Building environment (kitchen + Franka + cup)...")
    env = build_env(cfg, app)

    # ── Cup world pose ────────────────────────────────────────────────────────
    cup_pos, cup_ori = env.cup.get_world_pose()
    print(f"Cup world pose: pos={cup_pos}  ori={cup_ori}")

    # ── Target: 0.20 m above the cup, top-down gripper orientation ──────────
    # Top-down = gripper Z-axis pointing world-down = 180° rotation about Y.
    # rotvec [0, pi, 0]  →  quat [w=0, x=0, y=1, z=0]
    #
    # Reachability note: the Franka base is at X=1.325 facing +X; with the
    # top-down orientation [0,0,1,0] the arm's comfortable approach distance
    # in X is ~0.25 m from the base (i.e. X≈1.575), which is ~0.075 m further
    # than the cup (X≈1.50).  We add this offset so RMPflow converges cleanly
    # while staying within the cup/basin area (basin centre is at X=1.565).
    robot_base_x = float(env.robot_base_pose7d[0])   # 1.325
    target_x = robot_base_x + 0.25                   # 1.575 — reachable approach column
    target_pos = np.array(
        [target_x, float(cup_pos[1]), float(cup_pos[2]) + 0.20]
    )
    target_quat = axis_angle_to_quat(np.array([0.0, np.pi, 0.0]))  # [w, x, y, z]
    target_pose7d = np.concatenate([target_pos, target_quat])
    print(f"Target pose7d:  {target_pose7d}")

    # ── Create planner ────────────────────────────────────────────────────────
    print("Creating RMPFlowPlanner...")
    planner = RMPFlowPlanner(env)
    planner.reset()
    planner.set_target(target_pose7d)
    print(f"Target set.  Robot base pose7d: {env.robot_base_pose7d}")

    # ── Control loop ──────────────────────────────────────────────────────────
    MAX_STEPS = 1000
    CHECK_INTERVAL = 20
    POS_TOL = 0.02   # 2 cm
    ROT_TOL = 0.10   # ~5.7 deg

    final_step = MAX_STEPS
    converged = False
    curr_ee_pose: np.ndarray | None = None

    print(f"Running up to {MAX_STEPS} physics steps (check every {CHECK_INTERVAL})...")
    for step in range(MAX_STEPS):
        # Compute action from RMPflow
        act = planner.step(env.franka.get_joint_positions())
        # Apply to robot
        env.franka.apply_action(act)
        # Advance physics
        env.world.step(render=False)

        if (step + 1) % CHECK_INTERVAL == 0:
            curr_ee_pose = planner.get_ee_pose()
            pos_err = float(np.linalg.norm(curr_ee_pose[:3] - target_pose7d[:3]))
            print(
                f"  step={step+1:4d}  pos_err={pos_err:.4f} m"
                f"  ee_pos={curr_ee_pose[:3]}"
            )
            if planner.reached(curr_ee_pose, pos_tol=POS_TOL, rot_tol=ROT_TOL):
                final_step = step + 1
                converged = True
                print(f"  >>> Converged at step {final_step}!")
                break

    # Final EE pose (in case loop ended without a check)
    if curr_ee_pose is None:
        curr_ee_pose = planner.get_ee_pose()

    final_pos_err = float(np.linalg.norm(curr_ee_pose[:3] - target_pose7d[:3]))

    # ── Print results ─────────────────────────────────────────────────────────
    print("\n=== Smoke Planner Results ===")
    print("RMPflow config key:    robot='Franka'  policy='RMPflow'")
    print(f"Target pose7d:         {target_pose7d}")
    print(f"Final EE pose7d:       {curr_ee_pose}")
    print(f"Final position error:  {final_pos_err:.4f} m")
    print(f"Steps taken:           {final_step}")
    print(f"Converged:             {converged}")
    assert_pass = final_pos_err < 0.03
    print(f"Assert pos_err < 0.03: {'PASS' if assert_pass else 'FAIL'}")

    # Write to file — Isaac may redirect stdout fd so file is the safety net
    _RESULTS_FILE.parent.mkdir(parents=True, exist_ok=True)
    with open(_RESULTS_FILE, "w") as fh:
        fh.write("=== smoke_planner results ===\n")
        fh.write("RMPflow config key: robot='Franka' policy='RMPflow'\n")
        fh.write(f"Target pose7d: {target_pose7d}\n")
        fh.write(f"Final EE pose7d: {curr_ee_pose}\n")
        fh.write(f"Final position error: {final_pos_err:.4f} m\n")
        fh.write(f"Steps taken: {final_step}\n")
        fh.write(f"Converged: {converged}\n")
        fh.write(f"Assert pos_err < 0.03 m: {'PASS' if assert_pass else 'FAIL'}\n")
    print(f"Results written to: {_RESULTS_FILE}")

    # ── Assert ────────────────────────────────────────────────────────────────
    assert final_pos_err < 0.03, (
        f"FAIL: position error {final_pos_err:.4f} m >= 0.03 m after {final_step} steps. "
        f"Target: {target_pose7d[:3]}  Final EE: {curr_ee_pose[:3]}"
    )

    print(f"\nSmoke test PASSED (pos_err={final_pos_err:.4f} m < 0.03 m)")
    app.close()


if __name__ == "__main__":
    main()
