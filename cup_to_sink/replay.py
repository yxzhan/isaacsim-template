"""
replay.py -- Action replay tool for cup_to_sink demonstrations.

Reloads the scene, resets to an episode's recorded initial state, and replays
the recorded per-step commands, then reports whether task.check_success() still
holds.  This validates that the saved actions reproduce the task in simulation.

Two action modes:
    joint_pos : send /actions/joint_pos_target + /actions/gripper_width_target
                each step (the expert executor's commanded arm joint targets).
    ee_pose   : drive the planner to /actions/ee_pose_target_7d each step
                (slower; reactive).  joint_pos is the recommended default.

Physics replay is not perfectly deterministic, so use this as a
validation/debug tool -- the collection-time success verdict remains
authoritative.

Usage::

    /isaac-sim/python.sh -m cup_to_sink.replay \\
        --episode datasets/cup_to_sink/episode_0000.hdf5 \\
        --mode action --action-mode joint_pos
"""
from __future__ import annotations

import argparse
import sys
import tempfile
from pathlib import Path

import numpy as np

_REPO_ROOT = Path(__file__).resolve().parents[1]


def _load_config_for_episode(f, override_config: str | None) -> dict:
    """Return the config dict for replay.

    Uses --config if given, otherwise reconstructs it from the episode's
    embedded /meta/config_yaml.
    """
    from cup_to_sink import config as cfg_mod

    if override_config:
        return cfg_mod.load(override_config)

    raw = f["meta/config_yaml"][()]
    if isinstance(raw, bytes):
        raw = raw.decode("utf-8")
    tmp = Path(tempfile.mkdtemp()) / "episode_config.yaml"
    tmp.write_text(raw, encoding="utf-8")
    return cfg_mod.load(str(tmp))


def _set_initial_state(env, task, cup_pose7d, sink_pose7d, robot_qpos) -> None:
    """Reset the scene to the episode's recorded initial state (no randomization)."""
    from isaacsim.core.utils.types import ArticulationAction

    # Cup pose.
    env.cup.set_world_pose(np.asarray(cup_pose7d[:3]), np.asarray(cup_pose7d[3:]))

    # Sink target Xform translate (visual only; success uses task.sink_target_pose7d).
    from pxr import UsdGeom
    sink_prim = env.stage.GetPrimAtPath(env.sink_path)
    if sink_prim and sink_prim.IsValid():
        xform = UsdGeom.Xformable(sink_prim)
        xform.ClearXformOpOrder()
        xform.AddTranslateOp().Set(
            tuple(float(v) for v in np.asarray(sink_pose7d[:3]))
        )

    # Store the recorded sink pose so check_success() compares against it.
    task.sink_target_pose7d = np.asarray(sink_pose7d, dtype=np.float64)

    # Robot joint state: write the full recorded qpos vector.
    robot_qpos = np.asarray(robot_qpos, dtype=np.float64)
    env.franka.set_joint_positions(robot_qpos)

    for _ in range(60):
        env.world.step(render=False)


def replay_episode(episode_path: str, mode: str, action_mode: str,
                   override_config: str | None) -> bool:
    """Replay one episode; return whether check_success() holds afterwards."""
    import h5py

    from cup_to_sink import sim_app as sim_app_mod
    sim_app_mod._force_utf8_stdio()

    app = sim_app_mod.start(headless=True)

    from cup_to_sink import env_builder, task as task_mod
    from cup_to_sink.gripper import width_to_finger_joints
    from isaacsim.core.utils.types import ArticulationAction

    with h5py.File(episode_path, "r") as f:
        cfg = _load_config_for_episode(f, override_config)

        # Recorded actions / initial state.
        joint_pos_target = f["actions/joint_pos_target"][:]          # (T,7)
        gripper_width_target = f["actions/gripper_width_target"][:]  # (T,)
        ee_pose_target_7d = f["actions/ee_pose_target_7d"][:]        # (T,7)
        cup0 = f["meta/initial_cup_pose"][:]
        sink_pose = f["meta/sink_target_pose"][:]
        robot_qpos0 = f["meta/initial_robot_qpos"][:]
        n_steps = joint_pos_target.shape[0]

    print(f"[replay] episode={episode_path}")
    print(f"[replay] mode={mode}, action_mode={action_mode}, steps={n_steps}")

    env = env_builder.build_env(cfg, app)
    task = task_mod.Task(env, cfg)

    _set_initial_state(env, task, cup0, sink_pose, robot_qpos0)
    print(f"[replay] reset to recorded initial state; cup={cup0[:3]}, sink={sink_pose[:3]}")

    if mode != "action":
        print(f"[replay] mode '{mode}' not supported in v1; only 'action'. Exiting.")
        app.close()
        return False

    if action_mode == "joint_pos":
        for t in range(n_steps):
            arm_t = np.asarray(joint_pos_target[t], dtype=np.float64)
            fingers = width_to_finger_joints(float(gripper_width_target[t]))
            joint_positions = np.concatenate([arm_t, fingers])
            joint_indices = np.concatenate([task.arm_idx, task.finger_idx])
            env.franka.apply_action(ArticulationAction(
                joint_positions=joint_positions, joint_indices=joint_indices,
            ))
            env.world.step(render=False)
    elif action_mode == "ee_pose":
        from cup_to_sink import planner as planner_mod
        planner = planner_mod.RMPFlowPlanner(env)
        planner.reset()
        for t in range(n_steps):
            planner.set_target(np.asarray(ee_pose_target_7d[t], dtype=np.float64))
            act = planner.step(env.franka.get_joint_positions())
            env.franka.apply_action(act)
            fingers = width_to_finger_joints(float(gripper_width_target[t]))
            env.franka.apply_action(ArticulationAction(
                joint_positions=fingers, joint_indices=task.finger_idx,
            ))
            env.world.step(render=False)
    else:
        print(f"[replay] unknown action_mode '{action_mode}'. Exiting.")
        app.close()
        return False

    # Settle, then evaluate.
    for _ in range(50):
        env.world.step(render=False)

    success, info = task.check_success()
    cup_final, _ = env.cup.get_world_pose()
    print(f"[replay] replay finished: check_success={success}, info={info}")
    print(f"[replay] cup_final={np.asarray(cup_final)}, sink={np.asarray(sink_pose[:3])}")

    app.close()
    return bool(success)


def main() -> int:
    parser = argparse.ArgumentParser(description="Replay a cup_to_sink demonstration.")
    parser.add_argument("--episode", required=True, help="Path to episode_XXXX.hdf5.")
    parser.add_argument("--mode", default="action", choices=["action"],
                        help="Replay mode (v1 supports 'action').")
    parser.add_argument("--action-mode", default="joint_pos",
                        choices=["joint_pos", "ee_pose"],
                        help="Which recorded action stream to replay.")
    parser.add_argument("--config", default=None,
                        help="Override config path (default: episode's embedded config_yaml).")
    args = parser.parse_args()

    from cup_to_sink.sim_app import _force_utf8_stdio
    _force_utf8_stdio()

    ok = replay_episode(args.episode, args.mode, args.action_mode, args.config)
    print(f"[replay] RESULT: {'SUCCESS' if ok else 'FAIL'}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
