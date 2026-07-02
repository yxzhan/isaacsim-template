# cup_to_sink — Kitchen Manipulation Benchmark (Phase 1)

First task of the kitchen benchmark: **"put the cup into the sink."** This package
generates expert demonstrations in the Isaac Sim kitchen digital twin, records them
as HDF5 (multi-camera RGB-D + joint/EE actions + state + metadata), and can replay
saved actions to validate them.

Built on Isaac Sim 5.1 (Python 3.11). The motion planner is **Lula RMPFlow** behind a
swappable `Planner` interface, so cuRobo can be dropped in later without touching the
task/skill/dataset layers.

## Layout

```
cup_to_sink/
  sim_app.py          # SimulationApp bootstrap (UTF-8 stdio guard)
  env_builder.py      # build kitchen + Franka + cup + sink Xform + 4 cameras
  task.py             # reset(seed), get_obs(), check_success(), instruction
  actors.py           # CupActor / SinkTarget affordance points
  skills.py           # grasp_actor / move_by_displacement / place_actor (sparse EE goals)
  planner.py          # Planner ABC + RMPFlowPlanner (controls Lula right_gripper frame)
  executor.py         # drive planner to each goal, interpolate gripper, record dense steps
  dataset_writer.py   # buffer per-step data, write HDF5 (depth uint16-mm, gzip)
  cameras.py          # 4-cam RGB-D capture (front/left/right world, wrist on hand)
  collect_data.py     # entry: play_once() loop -> N successful demos
  replay.py           # entry: action replay of a saved episode
  inspect_scene.py    # entry: boot kitchen, dump prim tree + bounds (scene calibration)
  edit_scene.py       # entry: full Isaac Sim GUI with the scene loaded (drag & calibrate)
  convert_to_lerobot.py  # entry: HDF5 episodes -> LeRobot dataset (videos + parquet)
  configs/cup_to_sink.yaml
  tests/              # pytest (pure logic) + sim smoke scripts
```

## Setup

Everything runs under the Isaac Sim Python (`/isaac-sim/python.sh`). One extra
dependency is needed for HDF5:

```bash
/isaac-sim/python.sh -m pip install h5py
```

> Note: if the container is recreated, re-run the `pip install h5py`. cuRobo is **not**
> required for Phase 1.

## Running

All commands run from the repo root.

### Collect demonstrations

```bash
/isaac-sim/python.sh -m cup_to_sink.collect_data \
    --config cup_to_sink/configs/cup_to_sink.yaml
```

`--num-success` / `--max-attempts` default to `task.num_success` (50) /
`task.max_attempts` (500) in the config; override on the CLI for a quick test:

```bash
/isaac-sim/python.sh -m cup_to_sink.collect_data \
    --config cup_to_sink/configs/cup_to_sink.yaml --num-success 3 --max-attempts 5 --viewer
```

Outputs (`datasets/cup_to_sink/`, git-ignored):
- `episode_XXXX.hdf5` — one per successful demo
- `collect_log.json` — per-attempt log: seed, success, steps, cup-vs-sink distance,
  and grasp/placement diagnostics; failed seeds recorded too.

### Replay a demonstration

```bash
/isaac-sim/python.sh -m cup_to_sink.replay \
    --episode datasets/cup_to_sink/episode_0000.hdf5 \
    --mode action --action-mode joint_pos
```

Reloads the scene, resets to the episode's recorded initial state, replays the
recorded per-step commands, and reports whether `check_success()` still holds.
`--action-mode ee_pose` replays the EE-pose targets via the planner instead.
Physics replay is not perfectly deterministic — use it as a validation/debug tool.

### Inspect / calibrate the scene

```bash
/isaac-sim/python.sh -m cup_to_sink.inspect_scene
```

Boots the kitchen and prints the prim tree + world AABBs of counter/sink/etc., plus
suggested `sink_target_pose_world` / `robot.base_position` / cup spawn coordinates.
Use this to recalibrate `configs/cup_to_sink.yaml` if the USD scene changes.

### Edit the scene in the Isaac Sim GUI

```bash
/isaac-sim/python.sh -m cup_to_sink.edit_scene \
    --config cup_to_sink/configs/cup_to_sink.yaml
```

Opens the full interactive Isaac Sim editor (view via the VNC virtual desktop) with
the scene loaded and physics paused. Drag prims in the viewport (`/World/Franka`,
`/World/Objects/Cup`, `/World/EditMarkers/sink`, `/World/Cameras/*`); their world
poses are printed every few seconds in config-ready format (`[w,x,y,z]` quats) so
you can paste them straight into `configs/cup_to_sink.yaml`. Close the Isaac Sim
window to exit.

### Convert to LeRobot format

Runs under the system Python (`python3`), **not** `/isaac-sim/python.sh`; needs
`pip install lerobot h5py` once.

```bash
python3 cup_to_sink/convert_to_lerobot.py \
    --input datasets/cup_to_sink \
    --output datasets/cup_to_sink_lerobot
```

Mapping: `observation.state` = 7 joint pos + gripper width (8d),
`action` = `actions/policy_action_command` (8d), the four RGB cameras become
AV1-encoded `video` features, `task` = the language instruction. `--fps` defaults
to 25 (200 Hz physics / `record_every=8`). Depth images are not converted
(LeRobot video features are uint8 RGB only). Load the result with:

```python
from lerobot.datasets.lerobot_dataset import LeRobotDataset
ds = LeRobotDataset("yxzhan/cup_to_sink", root="datasets/cup_to_sink_lerobot")
```

### Tests

```bash
# Pure-logic unit tests (no GPU):
python3 -m pytest cup_to_sink/tests/test_transforms.py cup_to_sink/tests/test_gripper.py \
  cup_to_sink/tests/test_success.py cup_to_sink/tests/test_randomize.py \
  cup_to_sink/tests/test_actors.py cup_to_sink/tests/test_skills.py -q

# HDF5 writer round-trip (needs h5py):
/isaac-sim/python.sh -m pytest cup_to_sink/tests/test_dataset_writer.py -q

# Sim smoke tests (need GPU): cup_to_sink.tests.smoke_{env,planner,task,execute}
```

## Dataset format (per episode HDF5)

```
/observations/joint_pos, joint_vel, gripper_joint_pos, gripper_width, gripper_aperture
/observations/ee_pose_7d, ee_pose_6d            # robot_base frame (default policy frame)
/observations/ee_pose_7d_world, ee_pose_6d_world
/observations/cup_pose, sink_target_pose
/observations/images/{front,left,right,wrist}/{rgb,depth}   # rgb uint8, depth uint16 mm
/actions/joint_pos_target, ee_pose_target_7d/6d, delta_ee_pose_target_6d
/actions/gripper_{joint_pos,width,aperture}_target
/actions/policy_action_command (8d), policy_action_next_state (8d)
/actions/policy_action_ee_command (7d), policy_action_ee_next_state (7d)
/debug/sparse_ee_target_pose, grasp_pose_7d, pregrasp_pose_7d, phase
/meta/...   # task, seed, success, config_yaml, usd paths, robot base, dt, joint names,
            # gripper widths, frame conventions, enabled cameras,
            # randomization/{seed, cup_initial_pose, sink_target_pose, ...}
```

- EE pose: 7d = `[x,y,z,qw,qx,qy,qz]`, 6d = `[x,y,z,rx,ry,rz]` (axis-angle).
- Gripper: continuous `gripper_width` (m, `[0,0.08]`) + normalized `gripper_aperture`,
  plus the real two-finger joint positions — not a binary open/close label.
- Both joint-space and EE-space actions are saved, with `command` (what the expert
  sent) and `next_state` (next measured state) variants, so downstream training can
  pick `action_mode` (`joint_pos`/`ee_pose`) and `action_source` (`command`/`next_state`).

## Implementation notes & known limitations (Phase 1)

- **Planner frame.** RMPFlow controls the Lula `right_gripper` frame; `reached()` must
  be fed `planner.get_ee_pose()`, not `franka.end_effector.get_world_pose()` (different
  frame). Observations use the `panda_hand` frame.
- **Grasp.** A top-down `right_gripper` grasp is used. Because a free cup can be nudged
  during the descent, the expert generator welds the cup to the hand with a USD
  `FixedJoint` (`grasp.use_attach`) and snaps it to a clean under-TCP pose first
  (`grasp.snap_to_gripper`); both are flagged in HDF5 meta (`grasp_attach_used`,
  `grasp_snap_used`). Disable them to rely on friction only.
- **Sink target.** This kitchen asset has **no physical basin recess** at the sink
  location, so the cup rests on the counter surface; `sink_target_pose_world` z is set
  to counter level and "into the sink" means delivering the cup to the sink XY region.
  If a scene with a real basin is used, re-calibrate via `inspect_scene`.
- **Recording cadence.** Steps are recorded every `dataset.record_every` physics ticks
  (default 8 ≈ 25 Hz). Recording every 200 Hz tick buffers 4×RGB-D per step in RAM and
  can exhaust memory on long episodes.
- **Scope.** Phase 1 covers collection + replay. The policy interface and
  `eval_policy` are deferred; the dataset schema already supports them.

## Acceptance (Phase 1)

`num_success = 50`, `max_attempts <= 500`, every saved episode satisfies
`task.check_success()` (cup within `success.xy_threshold` of the sink XY, in the z band,
gripper open). `collect_log.json` records the per-episode cup-initial / sink-target
randomization so the distribution can be verified to vary across demos.
