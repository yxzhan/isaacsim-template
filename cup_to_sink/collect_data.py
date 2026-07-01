"""
collect_data.py -- Expert demonstration collector for the cup-to-sink benchmark.

Implements play_once() and a main() CLI that loops seeds until --num-success
episodes have been collected and written to HDF5.

Grasp note
----------
The planner controls the Lula ``right_gripper`` frame (not ``panda_hand``).
The cup contact-point ``quat_obj`` must therefore be authored in the
right_gripper convention.  For a straight top-down grasp, use euler
[0, 180, 0] deg -> quat [w=0, x=0, y=1, z=0] in [w,x,y,z].  With this
orientation the local +Z axis (approach axis) points downward in world frame,
so the pregrasp waypoint is above the contact point and the arm descends to
grasp -- exactly the behaviour we want.

Attach fallback
---------------
If ``grasp.use_attach: true`` is set in the config, after the gripper closes a
USD FixedJoint is created between ``/World/Franka/panda_hand`` and the cup
rigid-body prim, rigidly coupling them for the carry phase.  The joint is
removed just before the gripper-open action so the cup settles freely in the
sink.  Set ``grasp.use_attach: false`` (default) to rely on friction alone.

Usage::

    /isaac-sim/python.sh -m cup_to_sink.collect_data \\
        --config cup_to_sink/configs/cup_to_sink.yaml \\
        --num-success 1 \\
        --max-attempts 8
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
import traceback
from pathlib import Path

import numpy as np

_REPO_ROOT = Path(__file__).resolve().parents[1]


# -- Attach / detach helpers ----------------------------------------------------

def _attach_cup_to_hand(env) -> str:
    """Create a USD FixedJoint between panda_hand and the cup rigid body.

    CRITICAL: the joint's local frames MUST be set to the bodies' CURRENT
    relative transform.  A FixedJoint left with default (identity) local frames
    tells PhysX the two body origins should be coincident, so on the first
    simulated step PhysX violently snaps the cup onto the hand origin and the
    cup is launched away.  We instead pin the joint so its rest pose == the
    current relative pose: body0=hand with identity local frame, and body1=cup
    with local frame = inv(T_world_cup) @ T_world_hand, so both joint frames
    coincide in world at the instant of attachment and nothing snaps.

    Args:
        env: Env dataclass (must have .stage and .cup attributes).

    Returns:
        str: USD prim path of the created joint (pass to _detach_cup_from_hand).
    """
    from pxr import UsdPhysics, Gf, UsdGeom
    from cup_to_sink.transforms import make_T, invert_T, T_to_pose7d
    from isaacsim.core.prims import SingleXFormPrim

    joint_path = "/World/GraspAttachJoint"
    # Remove any stale joint from a previous episode.
    existing = env.stage.GetPrimAtPath(joint_path)
    if existing and existing.IsValid():
        env.stage.RemovePrim(joint_path)

    hand_path = "/World/Franka/panda_hand"
    cup_path = env.cup.prim_path  # e.g. "/World/Objects/Cup/SM_Cup"

    # Current world poses of both bodies (quat [w,x,y,z]).
    hand_pos, hand_quat = SingleXFormPrim(hand_path).get_world_pose()
    cup_pos, cup_quat = env.cup.get_world_pose()
    hand_pos = np.asarray(hand_pos); hand_quat = np.asarray(hand_quat)
    cup_pos = np.asarray(cup_pos); cup_quat = np.asarray(cup_quat)

    # Joint frame on body1 (cup), expressed in the cup's local frame, such that
    # it coincides in world with the hand frame (joint frame on body0 = identity).
    T_local1 = invert_T(make_T(cup_pos, cup_quat)) @ make_T(hand_pos, hand_quat)
    local1 = T_to_pose7d(T_local1)          # [x,y,z, qw,qx,qy,qz]
    p1 = local1[:3]
    q1 = local1[3:]

    fixed_joint = UsdPhysics.FixedJoint.Define(env.stage, joint_path)
    fixed_joint.CreateBody0Rel().SetTargets([hand_path])
    fixed_joint.CreateBody1Rel().SetTargets([cup_path])
    # body0 (hand): joint frame = hand frame.
    fixed_joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    fixed_joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0, Gf.Vec3f(0.0, 0.0, 0.0)))
    # body1 (cup): joint frame placed so it coincides with the hand frame NOW.
    fixed_joint.CreateLocalPos1Attr().Set(Gf.Vec3f(float(p1[0]), float(p1[1]), float(p1[2])))
    fixed_joint.CreateLocalRot1Attr().Set(
        Gf.Quatf(float(q1[0]), Gf.Vec3f(float(q1[1]), float(q1[2]), float(q1[3])))
    )

    print(f"[collect_data/attach] FixedJoint created: {hand_path} <-> {cup_path} "
          f"(local1 pos={p1}, quat={q1})")
    return joint_path


def _set_cup_collision(env, enabled: bool) -> None:
    """Enable/disable collision on the cup rigid body and its descendants.

    During the welded carry the cup is rigidly attached to the hand, so its
    collision with the counter/faucet only fights the joint constraint and can
    lock the arm (observed on the apartment counter). We disable collision for
    the carry and re-enable it before release so the cup settles in the sink.
    """
    from pxr import Usd, UsdPhysics
    prim = env.stage.GetPrimAtPath(env.cup.prim_path)
    if not (prim and prim.IsValid()):
        return
    for d in [prim] + list(Usd.PrimRange(prim)):
        if d.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI(d).GetCollisionEnabledAttr().Set(bool(enabled))


def _detach_cup_from_hand(env, joint_path: str) -> None:
    """Remove the FixedJoint to release the cup before gripper opens.

    Args:
        env: Env dataclass.
        joint_path: USD prim path returned by _attach_cup_to_hand.
    """
    prim = env.stage.GetPrimAtPath(joint_path)
    if prim and prim.IsValid():
        env.stage.RemovePrim(joint_path)
        print(f"[collect_data/attach] FixedJoint removed: {joint_path}")
    else:
        print(f"[collect_data/attach] WARNING: joint prim not found: {joint_path}")


# -- play_once -----------------------------------------------------------------

def play_once(env, task, planner, cfg: dict, seed: int):
    """Run one expert episode.

    Steps:
      1. task.reset(seed) + planner.reset()
      2. Build CupActor / SinkTarget from live state.
      3. Phase A: execute grasp_actor actions (open -> pregrasp -> grasp -> close).
         After close, optionally attach the cup via FixedJoint.
      4. Derive lift base from the actual EE pose reported by the planner after
         the grasp completes (avoids any accumulated error in the planned pose).
      5. Phase B: execute lift + preplace + place move actions.
         If use_attach, remove the FixedJoint just before the release step.
      6. Phase C: execute release (gripper open) + retreat.
      7. task.check_success() and build full meta dict.

    Args:
        env: Env dataclass from env_builder.build_env().
        task: Task instance.
        planner: RMPFlowPlanner instance.
        cfg: config dict from config.load().
        seed: integer episode seed.

    Returns:
        (success: bool, recorder: Recorder, meta: dict)
    """
    from cup_to_sink import actors, skills, executor
    from cup_to_sink.dataset_writer import Recorder
    from cup_to_sink.transforms import make_T, invert_T, T_to_pose7d

    # -- 1. Reset --------------------------------------------------------------
    rnd = task.reset(seed)
    planner.reset()

    print(f"[play_once seed={seed}] cup_xy={rnd['cup_xy']}, "
          f"cup_yaw={rnd['cup_yaw']:.3f}, "
          f"sink_target_pose={rnd['sink_target_pose7d'][:3]}")

    # -- 2. Build actors -------------------------------------------------------
    cup = actors.CupActor(env.cup, cfg["cup"])
    sink = actors.SinkTarget(task.sink_target_pose7d)

    # -- Config shorthand ------------------------------------------------------
    cup_cfg = cfg["cup"]
    robot_cfg = cfg["robot"]
    motion_cfg = cfg["motion"]
    grasp_cfg = cfg.get("grasp", {})

    pre_grasp_dis = float(cup_cfg["contact_points"][0]["pregrasp_distance"])
    closed_width = float(robot_cfg["gripper_closed_width"])
    open_width = float(robot_cfg["gripper_open_width"])
    lift_height = float(motion_cfg["lift_height"])
    preplace_dis = float(motion_cfg["preplace_distance"])
    retreat_h = float(motion_cfg["retreat_height"])
    use_attach = bool(grasp_cfg.get("use_attach", False))

    # -- 3. Build Phase-A actions (grasp) -------------------------------------
    grasp_actions = skills.grasp_actor(
        cup,
        pre_grasp_dis=pre_grasp_dis,
        grasp_dis=0.0,
        gripper_width_target=closed_width,
        contact_point_id=0,
    )

    # Capture grasp debug info from the planned grasp target pose.
    grasp_pose7d = cup.get_contact_point(0).copy()         # world pose of contact point
    pregrasp_pose7d = grasp_actions[1]["ee_pose7d"].copy()  # pregrasp move target

    grasp_debug = {
        "grasp_pose_7d":    grasp_pose7d,
        "pregrasp_pose_7d": pregrasp_pose7d,
    }

    print(f"[play_once seed={seed}] grasp target: {grasp_pose7d}")
    print(f"[play_once seed={seed}] pregrasp target: {pregrasp_pose7d}")

    # -- 4. Execute Phase A ----------------------------------------------------
    rec = Recorder()
    result_a = executor.execute(
        env, task, planner, grasp_actions, rec, cfg, grasp_debug=grasp_debug
    )

    print(f"[play_once seed={seed}] Phase A (grasp): "
          f"completed={result_a['completed']}, steps={result_a['steps']}")
    if not result_a["completed"]:
        print(f"  failed_phase={result_a.get('failed_phase')}")

    # -- (Optional) Snap + attach cup ------------------------------------------
    # The descent onto a free-standing cup can knock it ~0.1 m sideways before
    # the gripper closes, producing a large lateral cup-in-gripper offset; the
    # offset-corrected place pose then becomes tilted/below-counter and RMPFlow
    # cannot reach it, so the cup lands far from the sink.  For a deterministic
    # expert generator we "magnetize" the cup to a clean pose centred directly
    # under the gripper TCP (keeping its current height), then weld it with a
    # FixedJoint so the carry transform is purely vertical and reachable.
    # (plan sec12 attach scheme).  Honestly flagged in meta as grasp_snap_used.
    joint_path: str | None = None
    snap_used = False
    if use_attach:
        if bool(grasp_cfg.get("snap_to_gripper", True)):
            g_pose = planner.get_ee_pose()
            cup_pos_now, _ = env.cup.get_world_pose()
            clean_cup_pos = np.array([g_pose[0], g_pose[1], float(np.asarray(cup_pos_now)[2])])
            env.cup.set_world_pose(clean_cup_pos, np.array([1.0, 0.0, 0.0, 0.0]))
            snap_used = True
            print(f"[play_once seed={seed}] snapped cup to under-TCP pose {clean_cup_pos}")
        print(f"[play_once seed={seed}] Attaching cup via FixedJoint ...")
        # Attach reads the (snapped) poses NOW to pin the joint rest frame, so do
        # NOT step physics between the snap and the attach (gravity would drop it).
        joint_path = _attach_cup_to_hand(env)
        # Disable cup collision during the rigid carry (re-enabled before release).
        _set_cup_collision(env, False)
        # Step a few frames to let the joint activate.
        for _ in range(5):
            env.world.step(render=False)

    # -- 5. Derive lift base from actual EE pose + compute cup-in-gripper offset -
    # Using the live right_gripper pose reported by the planner avoids any
    # accumulated position error.  This pose is the starting point for the lift.
    actual_ee_pose7d = planner.get_ee_pose().copy()
    print(f"[play_once seed={seed}] actual EE pose after grasp: {actual_ee_pose7d}")

    cup_pos_after_grasp, cup_quat_after_grasp = env.cup.get_world_pose()
    cup_pos_after_grasp = np.asarray(cup_pos_after_grasp)
    cup_quat_after_grasp = np.asarray(cup_quat_after_grasp)
    print(f"[play_once seed={seed}] cup pos after grasp: {cup_pos_after_grasp}")

    # Compute rigid transform from right_gripper frame -> cup (cup in gripper frame).
    # T_gripper_cup is constant throughout the carry (especially when use_attach=True).
    # Used to correct the place target so the CUP lands at the sink XY, not the gripper.
    T_world_gripper_g = make_T(actual_ee_pose7d[:3], actual_ee_pose7d[3:])
    T_world_cup_g = make_T(cup_pos_after_grasp, cup_quat_after_grasp)
    T_gripper_cup = invert_T(T_world_gripper_g) @ T_world_cup_g
    cup_in_gripper_xyz = T_gripper_cup[:3, 3].copy()
    print(f"[play_once seed={seed}] cup-in-gripper offset xyz (local frame): {cup_in_gripper_xyz}")

    # -- 6. Build Phase B actions (lift + pre-release moves) -------------------
    lift_actions = skills.move_by_displacement(
        actual_ee_pose7d,
        dx=0.0,
        dy=0.0,
        dz=lift_height,
        phase="lift",
    )

    # Compute corrected place target: drive the RIGHT_GRIPPER to the pose such that
    # the CUP (not the gripper) arrives at the sink XY/Z.
    #
    # Math: T_world_cup = T_world_gripper @ T_gripper_cup
    #  => T_world_gripper_place = T_world_sink_target @ inv(T_gripper_cup)
    # When gripper reaches T_world_gripper_place:
    #   T_world_cup = T_world_gripper_place @ T_gripper_cup = T_world_sink_target ok
    sink_pos_w = np.asarray(task.sink_target_pose7d[:3])
    sink_quat_w = np.asarray(task.sink_target_pose7d[3:])
    T_world_sink_tgt = make_T(sink_pos_w, sink_quat_w)
    T_world_gripper_place = T_world_sink_tgt @ invert_T(T_gripper_cup)
    corrected_place_pose7d = T_to_pose7d(T_world_gripper_place)
    print(f"[play_once seed={seed}] corrected gripper place pose: {corrected_place_pose7d}")
    print(f"[play_once seed={seed}] original sink target:         {task.sink_target_pose7d}")

    # Build full place sequence, then split at the "release" gripper action.
    # Use corrected_place_pose7d so the cup (not gripper) lands at sink XY.
    full_place_actions = skills.place_actor(
        sink,
        corrected_place_pose7d,
        pre_dis=preplace_dis,
        retreat_h=retreat_h,
        is_open=True,
        gripper_open_width=open_width,
    )

    # Identify the release gripper action index.
    release_idx = next(
        (i for i, a in enumerate(full_place_actions) if a.get("phase") == "release"),
        None,
    )

    if release_idx is not None:
        place_pre_release = full_place_actions[:release_idx]   # preplace + place
        place_release_on = full_place_actions[release_idx:]    # release + retreat
    else:
        # Fallback: no release action found -- run everything together.
        place_pre_release = full_place_actions
        place_release_on = []

    # -- 7. Execute Phase B (lift + preplace + place) --------------------------
    phase_b_actions = lift_actions + place_pre_release
    result_b = executor.execute(
        env, task, planner, phase_b_actions, rec, cfg, grasp_debug=grasp_debug
    )

    print(f"[play_once seed={seed}] Phase B (lift+preplace+place): "
          f"completed={result_b['completed']}, steps_total={rec.num_steps}")
    if not result_b["completed"]:
        print(f"  failed_phase={result_b.get('failed_phase')}")

    cup_pos_after_phase_b, _ = env.cup.get_world_pose()
    cup_pos_after_phase_b = np.asarray(cup_pos_after_phase_b)
    # Detect grasp slip: if cup rose by >3 cm with the gripper, the grasp held.
    grasp_held = float(cup_pos_after_phase_b[2]) > float(cup_pos_after_grasp[2]) + 0.03
    print(f"[play_once seed={seed}] cup pos after Phase B: {cup_pos_after_phase_b}")
    print(f"[play_once seed={seed}] grasp held (cup rose from counter z): {grasp_held}")

    # -- (Optional) Detach cup before opening gripper --------------------------
    if joint_path is not None:
        print(f"[play_once seed={seed}] Detaching cup before release ...")
        # Re-enable collision so the released cup rests in the sink basin.
        _set_cup_collision(env, True)
        _detach_cup_from_hand(env, joint_path)
        # Brief settle so the cup's weight registers before gripper opens.
        for _ in range(5):
            env.world.step(render=False)

    # -- 8. Execute Phase C (release + retreat) --------------------------------
    if place_release_on:
        result_c = executor.execute(
            env, task, planner, place_release_on, rec, cfg, grasp_debug=grasp_debug
        )
        print(f"[play_once seed={seed}] Phase C (release+retreat): "
              f"completed={result_c['completed']}, steps_total={rec.num_steps}")

    # Let the cup settle in the sink before checking success.
    for _ in range(50):
        env.world.step(render=False)

    # -- 9. Check success ------------------------------------------------------
    success, info = task.check_success()

    cup_pos_final, cup_quat_final = env.cup.get_world_pose()
    cup_pos_final = np.asarray(cup_pos_final)
    cup_quat_final = np.asarray(cup_quat_final)
    sink_xyz = np.asarray(task.sink_target_pose7d[:3])

    xy_dist = float(np.hypot(
        cup_pos_final[0] - sink_xyz[0],
        cup_pos_final[1] - sink_xyz[1],
    ))

    print(f"[play_once seed={seed}] check_success={success}, info={info}")
    print(f"[play_once seed={seed}] cup_final={cup_pos_final}, "
          f"sink_target={sink_xyz}, xy_dist={xy_dist:.4f}m")

    # -- 10. Build meta dict ---------------------------------------------------
    scene_cfg = cfg["scene"]
    arm_joint_names: list[str] = list(robot_cfg["arm_joint_names"])
    gripper_joint_names: list[str] = list(robot_cfg["gripper_joint_names"])

    all_joints_init = np.asarray(rnd["robot_qpos_init"])
    # arm_idx indices are embedded in task; use cfg order to extract
    arm_idx_list = [
        list(env.franka.dof_names).index(n) for n in arm_joint_names
    ] if hasattr(env.franka, "dof_names") else list(range(7))
    initial_arm_qpos = all_joints_init[arm_idx_list] if len(all_joints_init) > 7 else all_joints_init[:7]

    meta: dict = {
        # -- Task --------------------------------------------------------------
        "task_name": cfg["task"]["name"],
        "language_instruction": cfg["task"]["instruction"],
        "seed": int(seed),
        "success": bool(success),
        # -- Config ------------------------------------------------------------
        "config_yaml": cfg.get("__raw_yaml__"),
        # -- Scene / robot -----------------------------------------------------
        "scene_usd_path": str(_REPO_ROOT / scene_cfg["lab_usd_path"]),
        "cup_usd_path":   str(_REPO_ROOT / scene_cfg["cup_usd_path"]),
        "cup_prim_path":  scene_cfg["cup_prim_path"],
        "robot_base_pose":     np.asarray(env.robot_base_pose7d),
        "initial_robot_qpos":  np.asarray(rnd["robot_qpos_init"]),
        "initial_gripper_width": float(robot_cfg["gripper_open_width"]),
        "initial_cup_pose":    np.asarray(rnd["cup_pose_init"]),
        "sink_target_pose":    np.asarray(task.sink_target_pose7d),
        # -- Physics -----------------------------------------------------------
        "physics_dt":  1.0 / 200.0,
        "control_dt":  1.0 / 200.0,
        # -- Joints ------------------------------------------------------------
        "joint_names":         arm_joint_names,
        "gripper_joint_names": gripper_joint_names,
        # -- Gripper widths ----------------------------------------------------
        "gripper_open_width":   float(robot_cfg["gripper_open_width"]),
        "gripper_closed_width": float(robot_cfg["gripper_closed_width"]),
        # -- Cameras -----------------------------------------------------------
        "enabled_cameras": list(cfg["dataset"]["enabled_cameras"]),
        # -- Frame conventions -------------------------------------------------
        "ee_pose_frame":          "panda_hand",
        "default_policy_ee_frame": "robot_base",
        "ee_pose_rotation_type":  "axis_angle",
        # -- Randomization sub-dict --------------------------------------------
        "randomization": {
            "seed":               int(seed),
            "cup_initial_pose":   np.asarray(rnd["cup_pose_init"]),
            "cup_contact_point_id": 0,
            "sink_target_pose":   np.asarray(task.sink_target_pose7d),
            "robot_default_qpos": list(cfg["randomization"]["robot_default_qpos"]),
        },
    }

    # Optional: flag whether rigid attach / snap was used in this episode.
    if use_attach:
        meta["grasp_attach_used"] = True
        meta["grasp_snap_used"] = bool(snap_used)

    # Diagnostics sub-dict -- included in HDF5 meta and collect_log.json.
    meta["diagnostics"] = {
        "cup_pos_after_grasp_xyz":   cup_pos_after_grasp.tolist(),
        "ee_pose_after_grasp":       actual_ee_pose7d.tolist(),
        "cup_in_gripper_offset_xyz": cup_in_gripper_xyz.tolist(),
        "corrected_place_pose7d":    corrected_place_pose7d.tolist(),
        "cup_pos_after_phase_b_xyz": cup_pos_after_phase_b.tolist(),
        "cup_pos_final_xyz":         cup_pos_final.tolist(),
        "xy_dist_m":                 round(xy_dist, 4),
        "grasp_held":                bool(grasp_held),
    }

    return success, rec, meta


# -- main ----------------------------------------------------------------------

def main() -> int:
    parser = argparse.ArgumentParser(
        description="Collect expert cup-to-sink demonstrations."
    )
    parser.add_argument(
        "--config", required=True,
        help="Path to YAML config file (e.g. cup_to_sink/configs/cup_to_sink.yaml).",
    )
    parser.add_argument(
        "--num-success", type=int, default=None,
        help="Number of successful episodes to collect "
             "(default: cfg['task']['num_success']).",
    )
    parser.add_argument(
        "--max-attempts", type=int, default=None,
        help="Maximum number of seeds to try (default: cfg['task']['max_attempts']).",
    )
    parser.add_argument(
        "--out", default=None,
        help="Output directory (default: from cfg[dataset][output_dir]).",
    )
    parser.add_argument(
        "--viewer", action="store_true",
        help="Run non-headless so an Isaac Sim window opens on the virtual "
             "desktop and you can watch collection. Slower; use for debugging, "
             "not batch collection.",
    )
    args = parser.parse_args()

    # Harden stdio against non-ASCII before any print (prevents UnicodeEncodeError
    # crashing the process when stdout is redirected to an ASCII pipe).
    from cup_to_sink.sim_app import _force_utf8_stdio
    _force_utf8_stdio()

    # -- Load config ------------------------------------------------------------
    from cup_to_sink import config as cfg_mod
    cfg = cfg_mod.load(args.config)

    # CLI overrides config; fall back to config when not provided on the CLI.
    num_success = args.num_success if args.num_success is not None else int(cfg["task"]["num_success"])
    max_attempts = args.max_attempts if args.max_attempts is not None else int(cfg["task"]["max_attempts"])

    out_dir = Path(args.out) if args.out else (_REPO_ROOT / cfg["dataset"]["output_dir"])
    out_dir.mkdir(parents=True, exist_ok=True)

    print(f"\n[collect_data] Config loaded from: {args.config}")
    print(f"[collect_data] Output dir: {out_dir}")
    print(f"[collect_data] Target successes: {num_success}, "
          f"max attempts: {max_attempts}")
    print(f"[collect_data] cup_spawn_position: {cfg['scene']['cup_spawn_position']}")
    print(f"[collect_data] contact_point: {cfg['cup']['contact_points'][0]}")
    print(f"[collect_data] sink_target_pose_world: {cfg['scene']['sink_target_pose_world']}")
    print(f"[collect_data] use_attach: {cfg.get('grasp', {}).get('use_attach', False)}\n")

    # -- Start Isaac Sim --------------------------------------------------------
    from cup_to_sink import sim_app as sim_app_mod
    app = sim_app_mod.start(headless=not args.viewer)

    # -- Build env once ---------------------------------------------------------
    from cup_to_sink import env_builder, task as task_mod, planner as planner_mod

    print("[collect_data] Building env ...")
    env = env_builder.build_env(cfg, app)
    print("[collect_data] Building task ...")
    task = task_mod.Task(env, cfg)
    print("[collect_data] Building planner ...")
    planner = planner_mod.RMPFlowPlanner(env)
    print("[collect_data] Env+task+planner ready.\n")

    # -- Collection loop --------------------------------------------------------
    success_count = 0
    failed_seeds: list[int] = []
    result_log: list[dict] = []

    for seed in range(max_attempts):
        if success_count >= num_success:
            break

        print(f"\n{'='*64}")
        print(f"[collect_data] Attempt seed={seed}  "
              f"({success_count}/{num_success} successes so far)")
        print(f"{'='*64}")

        t0 = time.time()
        try:
            success, rec, meta = play_once(env, task, planner, cfg, seed)
        except Exception as exc:
            elapsed = time.time() - t0
            print(f"[collect_data] ERROR at seed={seed}: {exc}")
            traceback.print_exc()
            failed_seeds.append(seed)
            result_log.append({
                "seed": seed,
                "success": False,
                "error": str(exc),
                "elapsed_s": elapsed,
            })
            _flush_log(result_log, out_dir)
            continue

        elapsed = time.time() - t0
        cup_pos_final, _ = env.cup.get_world_pose()
        cup_pos_final = np.asarray(cup_pos_final)
        sink_xyz = np.asarray(task.sink_target_pose7d[:3])
        xy_dist = float(np.hypot(
            cup_pos_final[0] - sink_xyz[0],
            cup_pos_final[1] - sink_xyz[1],
        ))

        print(f"\n[collect_data] seed={seed}: success={success}, "
              f"steps={rec.num_steps}, elapsed={elapsed:.1f}s")
        print(f"  cup_final_xyz = {cup_pos_final.tolist()}")
        print(f"  sink_target   = {sink_xyz.tolist()}")
        print(f"  xy_dist={xy_dist:.4f}m, "
              f"cup_z={float(cup_pos_final[2]):.4f}m, "
              f"sink_z={float(sink_xyz[2]):.4f}m")

        log_entry: dict = {
            "seed": seed,
            "success": success,
            "steps": rec.num_steps,
            "elapsed_s": round(elapsed, 1),
            "cup_final_xyz": cup_pos_final.tolist(),
            "sink_target_xyz": sink_xyz.tolist(),
            "xy_dist_m": round(xy_dist, 4),
            "cup_z": round(float(cup_pos_final[2]), 4),
            "sink_z": round(float(sink_xyz[2]), 4),
        }

        # Include per-episode diagnostics in the JSON log.
        if "diagnostics" in meta:
            log_entry["diagnostics"] = meta["diagnostics"]

        if success:
            ep_path = out_dir / f"episode_{success_count:04d}.hdf5"
            print(f"\n[collect_data] SUCCESS -- writing {ep_path} ...")
            rec.write(str(ep_path), meta)
            size_mb = ep_path.stat().st_size / 1e6
            print(f"[collect_data] Written: {ep_path} ({size_mb:.1f} MB)")
            log_entry["hdf5_path"] = str(ep_path)
            success_count += 1
        else:
            failed_seeds.append(seed)

        result_log.append(log_entry)
        _flush_log(result_log, out_dir)

    # -- Summary ---------------------------------------------------------------
    print(f"\n{'='*64}")
    print(f"[collect_data] DONE")
    print(f"  successes collected : {success_count} / {num_success}")
    print(f"  total attempts      : {min(len(result_log), max_attempts)}")
    print(f"  failed seeds        : {failed_seeds}")
    print(f"{'='*64}\n")

    app.close()
    return 0 if success_count >= num_success else 1


def _flush_log(result_log: list, out_dir: Path) -> None:
    """Write the running JSON log so results are visible between runs."""
    try:
        json_path = out_dir / "collect_log.json"
        with open(json_path, "w") as f:
            json.dump(result_log, f, indent=2)
    except Exception as e:
        print(f"[collect_data] WARNING: could not write JSON log: {e}")


if __name__ == "__main__":
    sys.exit(main())
