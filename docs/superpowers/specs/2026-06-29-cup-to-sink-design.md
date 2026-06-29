# Design: `cup_to_sink` Kitchen Benchmark — Phase 1 (collect + replay)

**Date:** 2026-06-29
**Source:** `isaacsim_template_handoff.md`
**Scope this effort:** Phase 1 only — runnable expert data collection producing 50 successful
`cup_to_sink` demos, plus an action-replay tool. Policy interface and evaluation
(`policy_interface.py`, `eval_policy.py`, `policy_eval.yaml`) are explicitly deferred to a
later effort but the dataset schema is designed to support them.

## 1. Goal

In the kitchen digital twin, automatically generate **50 successful `cup_to_sink` expert
demonstrations** ("put the cup into the sink") saved as HDF5, with 4-camera RGB-D, joint-space
and EE-space actions, continuous gripper width, and full randomization metadata. Provide an
action-replay tool to validate the saved actions.

Final acceptance (plan §13): `num_success = 50`, `max_attempts <= 500`, every saved episode
satisfies `task.check_success()`.

## 2. Environment (verified)

- Isaac Sim **5.1.0**, Python **3.11**, torch **2.7.0+cu128**, CUDA available.
- **cuRobo is NOT installed**; installing it on this env is a non-trivial torch-matched build.
- The repo already has a **working Franka Panda pick-and-place** in `examples/apartment.py`
  using Isaac's `PickPlaceController` (RMPFlow + grasp state machine), a `ParallelGripper`, and
  a high-friction grasp material — i.e. a proven motion layer exists.
- `h5py` is **not** preinstalled but installs cleanly: `/isaac-sim/python.sh -m pip install h5py`
  (3.16.0). Documented as a setup step.
- Assets present: `usd/kitchen/kitchen.usd`, `usd/other/Cup.usd`. Franka loads from the Isaac
  assets server (as `apartment.py` does).
- `pxr`/Isaac USD libs require the kit runtime (SimulationApp) to be booted; standalone USD
  inspection is not available, so scene geometry must be calibrated from a live boot.

## 3. Key Decisions

### 3.1 Planner: RMPFlow now, cuRobo-ready interface
Define a `Planner` abstract interface:

```python
class Planner:
    def plan_to_pose(self, current_q, target_ee_pose_world) -> JointTargets | Trajectory: ...
```

`RMPFlowPlanner` wraps Isaac Lula `RmpFlow` / `ArticulationMotionPolicy` (proven in this repo).
It is reactive: the **executor** steps it toward each sparse EE goal until pose error < tolerance
(or a max-step cap), recording every control step. The recorded sequence of per-step joint
targets *is* the dense trajectory. A future `CuRoboPlanner` implements the same `plan_to_pose`
returning a precomputed dense trajectory; the task/skill/dataset layers do not change.

Rationale: fastest path to 50 demos with zero install risk, while honoring the plan's 4-layer
separation so cuRobo can drop in later.

### 3.2 Sink target = runtime invisible Xform (plan §4.2 fallback)
The kitchen crate exposes no named sink prim discoverable without booting the sim. The sink
target is a runtime invisible `/World/Cameras`-style `Xform` placed at a config world pose
(`scene.sink_target_pose_world`). `task.reset(seed)` samples its xy within
`randomization.sink_target_xlim/ylim` around that pose, and writes the sampled pose to HDF5 meta.
No dependency on a named USD prim. `inspect_scene.py` helps calibrate the base pose.

### 3.3 Ship the full plan §7 HDF5 schema in v1
Even though Phase 1 only exercises joint-space collection, the writer records the full schema so
later DP/ACT/VLA training and EE-control policies need no re-collection.

## 4. Architecture & Modules

Top-level package `cup_to_sink/` (runnable via `/isaac-sim/python.sh -m cup_to_sink.collect_data`).

Four layers (plan §3):

```
Task / Env        env_builder.py, task.py
Actor / Skill     actors.py, skills.py
Planner           planner.py
Executor/Dataset  executor.py, dataset_writer.py, cameras.py
```

| Module | Responsibility | Key interface |
|---|---|---|
| `env_builder.py` | Build World, load kitchen USD, spawn Franka (fixed base), spawn cup with collision + high-friction material, create sink Xform, create 4 cameras. Returns an `Env` holding handles. | `build_env(cfg) -> Env` |
| `task.py` | Episode lifecycle: `reset(seed)` (randomize cup pose, sample sink target, settle), `get_obs()` (assemble unified obs dict), `check_success()` (plan §11), instruction string, exposes sampled randomization values. | `reset(seed)`, `get_obs()`, `check_success()`, `sample_variation(seed)` |
| `actors.py` | `CupActor.get_contact_point(id)` and `get_functional_point(id)` transform object-frame points to world (`T_world_cup @ T_cup_x`); `SinkTarget.get_functional_point()` returns sampled sink target pose. | as named |
| `skills.py` | Produce **sparse EE target poses** + semantic gripper actions: `grasp_actor(env, actor, pre_grasp_dis, grasp_dis, gripper_width_target, contact_point_id)`, `move_by_displacement(env, x, y, z)`, `place_actor(env, actor, target_pose, functional_point_id, pre_dis, is_open)`. Returns an ordered action list of `{type: move|gripper, ...}`. | as named |
| `planner.py` | `Planner` ABC + `RMPFlowPlanner` (reactive RMPFlow toward EE pose). | `plan_to_pose(...)` |
| `executor.py` | Consume the skill action list. For `move`: drive planner to the EE goal, stepping sim, recording each control step. For `gripper`: interpolate finger joints to width target over `gripper_steps`. Records obs/action/debug each control step into the dataset buffer. | `execute(env, actions, recorder)` |
| `cameras.py` | Create + read 4 cameras (front/left/right world-fixed, wrist parented to `panda_hand`); return RGB (uint8 HxWx3) and depth (uint16 mm). Config-selectable enabled set. | `setup_cameras(cfg)`, `capture(enabled) -> dict` |
| `dataset_writer.py` | Per-step append buffers; on success, write the full plan §7 HDF5 layout (obs, actions, debug, meta, `/meta/randomization/*`). Depth as `uint16_mm`, gzip. | `Recorder` with `append(step)`, `write(path, meta)` |
| `collect_data.py` | Entry point. Build env once; loop seeds; per seed run `play_once()` (reset → expert skill sequence via executor → check_success); save HDF5 on success, discard on failure; stop at `num_success`; log success count + failed seeds. | `--config` |
| `replay.py` | Entry point. Action replay: reload scene, reset to recorded initial state, replay `joint_pos_target`+`gripper_width_target` (or EE), report whether `check_success()` still holds. | `--episode --mode action --action-mode joint_pos` |
| `inspect_scene.py` | Boot the kitchen, dump prim tree + world AABBs of candidate counters/sinks, to calibrate config coordinates. | `--scene` |
| `configs/cup_to_sink.yaml` | All config (plan §5 skeleton), with calibrated/placeholder scene coords. | — |
| `README.md` | How to install h5py, run collect/replay, recalibrate scene. | — |

## 5. Expert Action Flow (plan §6)

```
open gripper -> move to pregrasp -> move to grasp -> close gripper -> lift
-> move to preplace above sink -> move to place in sink -> open gripper -> retreat -> check_success
```

Grasp pose from cup config contact point: `T_world_grasp = T_world_cup @ T_cup_grasp`. v1 uses a
single fixed `side_grasp` (`contact_point_selection: fixed`); structure supports
`best_feasible` / `seed_random_feasible` later. Selected contact point id + grasp/pregrasp poses
written to `/meta/randomization/cup_contact_point_id`, `/debug/grasp_pose_7d`,
`/debug/pregrasp_pose_7d`.

## 6. Data Representations

- **EE pose**: `ee_pose_7d = [x,y,z,qw,qx,qy,qz]` and `ee_pose_6d = [x,y,z,rx,ry,rz]`
  (rotation vector / axis-angle). Saved in **world** and **robot_base** frames; default policy
  frame = `robot_base`. `ee_frame = panda_hand`.
- **Gripper** (plan §3.1): `gripper_width` (m, finger_joint1+finger_joint2, range [0,0.08]),
  `gripper_aperture` (normalized [0,1]), `gripper_joint_pos` (both fingers). Continuous width
  target → each finger = width/2. Skills use semantic open/close; dataset stores continuous values.
- **Actions**: joint-space `policy_action_command` (8d = 7 arm targets + gripper_width_target) and
  `policy_action_next_state` (next measured state); EE-space `policy_action_ee_command` (7d) and
  `policy_action_ee_next_state` (7d). Plus `joint_pos_target`, `ee_pose_target_7d/6d`,
  `delta_ee_pose_target_6d`, `gripper_joint_pos_target`, `gripper_width_target`,
  `gripper_aperture_target`.

## 7. HDF5 Layout

Implements plan §7 in full (`/observations/...`, `/observations/images/<cam>/{rgb,depth}`,
`/actions/...`, `/debug/...`, `/meta/...`, `/meta/randomization/...`). Depth = `uint16_mm`,
gzip compression. `/meta/config_yaml` stores the full resolved config; `/meta` includes scene/cup
USD paths, robot base pose, joint names, dt, gripper open/closed widths, ee frame, rotation type,
enabled cameras, and `policy_action_schema`/`sources`.

## 8. Randomization (plan §4.4)

Seeded `numpy` RNG per episode:
- cup initial pose: x/y within `cup_xlim/ylim`, yaw within `cup_yaw_range`, optional z offset.
- sink target xy within `sink_target_xlim/ylim` around `sink_target_pose_world`.
- Franka initial qpos = fixed default home; gripper starts open.
- settle: step `settle_steps` after reset.
Camera/lighting jitter default off. All sampled values written to `/meta/randomization/*`.

## 9. Success Condition (plan §11)

```
xy_ok = xy_distance(cup_pose, sink_target_pose) < xy_threshold
z_ok  = sink_z + z_min_offset <= cup_z <= sink_z + z_max_offset
gripper_ok = gripper_is_open()
success = xy_ok and z_ok and gripper_ok
```

## 10. Run Modes

- Collect: `/isaac-sim/python.sh -m cup_to_sink.collect_data --config cup_to_sink/configs/cup_to_sink.yaml`
  (runs `headless=True`; cameras still render).
- Replay: `/isaac-sim/python.sh -m cup_to_sink.replay --episode datasets/cup_to_sink/episode_0001.hdf5 --mode action --action-mode joint_pos`
- Inspect: `/isaac-sim/python.sh -m cup_to_sink.inspect_scene` (calibration helper).

## 11. Implementation Order (follows plan §10)

1. `inspect_scene.py` + boot kitchen + Franka + cup + sink Xform; print qpos, gripper, EE pose,
   cup/sink poses; calibrate config coordinates.
2. Joint target + continuous gripper width control verified.
3. `Planner`/`RMPFlowPlanner` single-segment `current qpos -> fixed EE pose`.
4. `actors.py` contact/functional points.
5. `skills.py` grasp/move/place; `executor.py` runs one `play_once()`.
6. `cameras.py` 4-cam RGB-D.
7. `dataset_writer.py` save one successful HDF5 (full schema).
8. `replay.py` action replay.
9. Scale to 50 successful demos; verify cup/sink pose distributions vary; log success/failed seeds.
10. `README.md`.

## 12. Risks / Open Items

- **Scene calibration** (Franka base reachability over both cup region and sink, counter height)
  is the main live-sim dependency; resolved in step 1. Config ships placeholders + recalibration doc.
- **Grasp stability**: reuse high-friction material; optional attach-object behind a config flag
  (default off, flagged in meta) if physics grasp is unreliable.
- **Throughput**: 4×RGB-D capture per control step is slow; record at `control_dt` decimation.
- RMPFlow may fail to reach some sampled poses; those seeds count as failed attempts (within the
  500 budget), consistent with the seed-search workflow.

## 13. Out of Scope (deferred)

`policy_interface.py`, `eval_policy.py`, `configs/policy_eval.yaml`, ACT/DP/VLA adapters, cuRobo
installation, mobile-manipulation, and additional contact points / task variations. The dataset
schema is designed so these need no re-collection.
