"""
executor.py — Consume a skill action list, drive RMPFlow, and record every step.

Public API:
    execute(env, task, planner, actions, recorder, cfg, grasp_debug=None) -> dict

Step recording contract (matches dataset_writer.Recorder.append):
    Observation keys come from task.get_obs().
    Action sub-dict: joint_pos_target, ee_pose_target_7d/6d,
        delta_ee_pose_target_6d, gripper_joint_pos_target,
        gripper_width_target, gripper_aperture_target,
        policy_action_command (8,), policy_action_ee_command (7,).
    Debug sub-dict: sparse_ee_target_pose, grasp_pose_7d, pregrasp_pose_7d.
    phase: int (see _PHASE_CODES).

Frame contract:
    planner.get_ee_pose() returns Lula right_gripper world pose — used for
    reached() checks.  task.get_obs() ee_pose_* uses panda_hand frame.
    Both are recorded; the distinction only matters for convergence checks.
"""
from __future__ import annotations

from typing import Any

import numpy as np

# ── Phase string → integer code ───────────────────────────────────────────────
# dataset_writer.Recorder stacks phase as int(s["phase"]) so we must encode.
_PHASE_CODES: dict[str, int] = {
    "pregrasp":    0,
    "grasp":       1,
    "grasp_close": 2,
    "lift":        3,
    "preplace":    4,
    "place":       5,
    "release":     6,
    "retreat":     7,
    "open":        8,
    "move":        9,
}


def _phase_to_int(phase: str) -> int:
    """Return integer code for a phase string; -1 for unknown phases."""
    return _PHASE_CODES.get(phase, -1)


def execute(
    env: Any,
    task: Any,
    planner: Any,
    actions: list[dict],
    recorder: Any,
    cfg: dict,
    grasp_debug: dict | None = None,
) -> dict:
    """Execute a skill action list, driving the RMPFlow planner and recording steps.

    Args:
        env: Env dataclass from env_builder.build_env().
        task: Task instance (provides get_obs(), arm_idx, finger_idx, gripper_width()).
        planner: RMPFlowPlanner instance (reset() and set_target() already called
            by the caller before the first action, if needed).
        actions: Ordered list of action dicts from skills.*:
            {"type":"move",    "ee_pose7d": np.ndarray(7), "phase": str}
            {"type":"gripper", "width": float,              "phase": str}
        recorder: Recorder instance — .append(step) is called once per step.
        cfg: Config dict (uses cfg["planner"] and cfg["motion"] sub-dicts).
        grasp_debug: Optional dict with "grasp_pose_7d" and "pregrasp_pose_7d"
            (np.ndarray, shape (7,)) for the /debug HDF5 group.  Pass None to
            fill with zeros.

    Returns:
        dict:
            "completed" bool   — False if any move action hit max_move_steps
                                 without reaching the goal.
            "steps"     int    — total steps recorded (recorder.num_steps).
            "failed_phase" str — only present when completed=False; the phase
                                 label of the first timed-out move.
    """
    # Deferred Isaac Sim import — only available after SimulationApp is running.
    from isaacsim.core.utils.types import ArticulationAction

    from cup_to_sink.gripper import width_to_finger_joints, width_to_aperture
    from cup_to_sink.transforms import pose7d_to_6d, world_to_base

    # ── Config ────────────────────────────────────────────────────────────────
    planner_cfg: dict = cfg.get("planner", {})
    max_move_steps: int = int(planner_cfg.get("max_move_steps", 600))
    pos_tol: float = float(planner_cfg.get("pos_tol", 0.02))
    rot_tol: float = float(planner_cfg.get("rot_tol", 0.1))
    gripper_steps: int = int(cfg["motion"]["gripper_steps"])

    robot_cfg = cfg["robot"]
    open_w: float = float(robot_cfg["gripper_open_width"])
    closed_w: float = float(robot_cfg["gripper_closed_width"])

    # ── Debug payloads ────────────────────────────────────────────────────────
    _zero7 = np.zeros(7, dtype=np.float64)
    grasp_pose_7d: np.ndarray = np.asarray(
        grasp_debug["grasp_pose_7d"] if grasp_debug else _zero7, dtype=np.float64
    )
    pregrasp_pose_7d: np.ndarray = np.asarray(
        grasp_debug["pregrasp_pose_7d"] if grasp_debug else _zero7, dtype=np.float64
    )

    # ── Mutable state across actions ──────────────────────────────────────────
    # Gripper width target — initialised from current actual state.
    all_pos_init = env.franka.get_joint_positions()
    fj_init = all_pos_init[task.finger_idx]
    current_gripper_width_target: float = float(fj_init[0] + fj_init[1])

    # Sparse EE goal (world frame, 7d) — reused during gripper-only actions.
    # Initialise to current EE pose so the first gripper action has a valid target.
    current_sparse_goal7d: np.ndarray = planner.get_ee_pose().copy()

    # Goal 6d in robot-base frame — for delta computation.
    # None = "no previous goal yet" → delta defaults to zeros.
    prev_goal_6d_base: np.ndarray | None = None

    # Goal-level delta (constant within one action, recomputed at action start).
    current_delta_6d_base: np.ndarray = np.zeros(6, dtype=np.float64)

    # Current goal 6d in base frame (constant within one action).
    _goal7d_base = world_to_base(current_sparse_goal7d, env.robot_base_pose7d)
    current_goal_6d_base: np.ndarray = pose7d_to_6d(_goal7d_base)

    # Last arm joint targets — filled each step; default to current positions.
    last_arm_joint_targets: np.ndarray = all_pos_init[task.arm_idx].copy()

    # Outcome tracking.
    failed_phase: str | None = None

    # ── Per-step helper ───────────────────────────────────────────────────────
    def _record_step(
        phase_str: str,
        arm_joint_targets: np.ndarray,
        override_gripper_width_target: float | None = None,
    ) -> None:
        """Build one step dict and append to recorder.

        When override_gripper_width_target is provided, the step's gripper
        fields (gripper_width_target, gripper_joint_pos_target,
        gripper_aperture_target, and the gripper slot of
        policy_action_command / policy_action_ee_command) use the override
        width instead of the running current_gripper_width_target.  Used by
        the gripper interpolation loop so each sub-step reflects the
        interpolated width rather than the pre-interpolation target.
        """
        # Snapshot observations (increments task._timestep internally).
        obs = task.get_obs()

        # EE target: world-frame 7d/6d from current sparse goal.
        ee_target_7d = current_sparse_goal7d.copy()
        ee_target_6d = pose7d_to_6d(ee_target_7d)

        # Effective gripper width for this step.
        eff_gw = (
            override_gripper_width_target
            if override_gripper_width_target is not None
            else current_gripper_width_target
        )

        # Gripper targets.
        gjp_target = width_to_finger_joints(eff_gw)
        ga_target = width_to_aperture(eff_gw, open_w, closed_w)

        # Policy action vectors.
        policy_action_command = np.concatenate(
            [arm_joint_targets.astype(np.float64), [float(eff_gw)]]
        )  # (8,)
        policy_action_ee_command = np.concatenate(
            [current_goal_6d_base.astype(np.float64), [float(eff_gw)]]
        )  # (7,)

        step: dict = {
            **obs,
            "action": {
                "joint_pos_target":         arm_joint_targets.copy().astype(np.float64),
                "ee_pose_target_7d":        ee_target_7d,
                "ee_pose_target_6d":        ee_target_6d,
                "delta_ee_pose_target_6d":  current_delta_6d_base.copy(),
                "gripper_joint_pos_target": gjp_target,
                "gripper_width_target":     float(eff_gw),
                "gripper_aperture_target":  float(ga_target),
                "policy_action_command":    policy_action_command,
                "policy_action_ee_command": policy_action_ee_command,
            },
            "debug": {
                "sparse_ee_target_pose": current_sparse_goal7d.copy(),
                "grasp_pose_7d":         grasp_pose_7d.copy(),
                "pregrasp_pose_7d":      pregrasp_pose_7d.copy(),
            },
            "phase": _phase_to_int(phase_str),
        }
        recorder.append(step)

    # ── Goal tracking helper ──────────────────────────────────────────────────
    def _update_goal_tracking(new_goal7d: np.ndarray) -> None:
        """Recompute goal_6d_base and delta when sparse goal changes."""
        nonlocal current_goal_6d_base, current_delta_6d_base, prev_goal_6d_base
        goal7d_base = world_to_base(new_goal7d, env.robot_base_pose7d)
        new_goal_6d_base = pose7d_to_6d(goal7d_base)
        if prev_goal_6d_base is None:
            current_delta_6d_base = np.zeros(6, dtype=np.float64)
        else:
            current_delta_6d_base = new_goal_6d_base - prev_goal_6d_base
        current_goal_6d_base = new_goal_6d_base.copy()
        prev_goal_6d_base = new_goal_6d_base.copy()

    # ── Main action loop ──────────────────────────────────────────────────────
    for action in actions:
        a_type: str = action["type"]
        a_phase: str = action["phase"]

        if a_type == "move":
            goal7d = np.asarray(action["ee_pose7d"], dtype=np.float64)
            current_sparse_goal7d = goal7d.copy()
            _update_goal_tracking(goal7d)

            planner.set_target(goal7d)

            reached = False
            for _step_i in range(max_move_steps):
                qpos = env.franka.get_joint_positions()
                act = planner.step(qpos)
                env.franka.apply_action(act)
                env.world.step(render=True)

                # Extract arm joint targets from the articulation action.
                # ArticulationMotionPolicy returns targets for active (arm) joints only.
                arm_jp = getattr(act, "joint_positions", None)
                if arm_jp is not None:
                    arm_jp_arr = np.asarray(arm_jp, dtype=np.float64).ravel()
                    if arm_jp_arr.shape == (7,):
                        last_arm_joint_targets = arm_jp_arr
                    else:
                        last_arm_joint_targets = env.franka.get_joint_positions()[task.arm_idx].copy()
                else:
                    last_arm_joint_targets = env.franka.get_joint_positions()[task.arm_idx].copy()

                _record_step(a_phase, last_arm_joint_targets)

                if planner.reached(
                    planner.get_ee_pose(),
                    pos_tol=pos_tol,
                    rot_tol=rot_tol,
                ):
                    reached = True
                    break

            if not reached and failed_phase is None:
                # Mark first failure; continue recording remaining actions.
                failed_phase = a_phase
                print(
                    f"[executor] WARNING: move action phase='{a_phase}' hit "
                    f"max_move_steps={max_move_steps} without reaching goal."
                )

        elif a_type == "gripper":
            target_width = float(action["width"])
            start_width = current_gripper_width_target

            # Recompute goal tracking with unchanged sparse goal (delta stays 0
            # if goal hasn't moved since last update).
            _update_goal_tracking(current_sparse_goal7d)

            for i in range(gripper_steps):
                # Linear interpolation from start_width to target_width.
                t = (i + 1) / max(gripper_steps, 1)
                interp_width = start_width + t * (target_width - start_width)

                # Apply only finger joint targets — arm holds its last commanded pos.
                finger_pos = width_to_finger_joints(interp_width)
                gripper_act = ArticulationAction(
                    joint_positions=finger_pos,
                    joint_indices=task.finger_idx,
                )
                env.franka.apply_action(gripper_act)
                env.world.step(render=True)

                # During gripper steps use current arm sensor positions as targets.
                arm_targets = env.franka.get_joint_positions()[task.arm_idx].copy()

                _record_step(a_phase, arm_targets, override_gripper_width_target=interp_width)

            current_gripper_width_target = target_width

        else:
            print(f"[executor] WARNING: unknown action type '{a_type}' — skipped.")

    completed = failed_phase is None
    result: dict = {"completed": completed, "steps": recorder.num_steps}
    if not completed:
        result["failed_phase"] = failed_phase
    return result
