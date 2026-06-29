# cup_to_sink Benchmark Phase 1 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a runnable `cup_to_sink` expert-data pipeline that auto-generates 50 successful "put the cup into the sink" demos as HDF5 in the Isaac Sim kitchen twin, plus an action-replay tool.

**Architecture:** Four layers (Task/Env, Actor/Skill, Planner, Executor/Dataset) in a `cup_to_sink/` package. Pure-logic modules (pose math, gripper conversions, randomization, success, HDF5 schema) are unit-tested with pytest. Sim-dependent modules (env build, cameras, RMPFlow planner, play_once) are verified by runnable smoke scripts inside Isaac Sim. The planner sits behind a `Planner` ABC; v1 uses Isaac Lula RMPFlow, cuRobo can drop in later.

**Tech Stack:** Isaac Sim 5.1.0 (Python 3.11), Isaac `isaacsim.core` / `isaacsim.robot.manipulators` / `isaacsim.robot.motion_generation` (Lula RMPFlow) / `isaacsim.sensors.camera`, USD (`pxr`), numpy, h5py, pyyaml.

## Global Constraints

- Python **3.11** via `/isaac-sim/python.sh`; Isaac Sim **5.1.0**. Sim code only imports `pxr`/`isaacsim`/`omni` AFTER `SimulationApp(...)` is constructed.
- **No cuRobo** in v1. Planner is behind an ABC so cuRobo can be added later without touching other layers.
- `h5py` must be installed into the Isaac env: `/isaac-sim/python.sh -m pip install h5py` (3.16.0).
- Never edit `examples/apartment.py` directly (it is generated from `apartment.ipynb`). Do not touch existing examples.
- Package lives at repo-root `cup_to_sink/`, runnable as `/isaac-sim/python.sh -m cup_to_sink.<entry>`.
- EE pose 7d = `[x,y,z,qw,qx,qy,qz]`; 6d = `[x,y,z,rx,ry,rz]` (axis-angle). Save world AND robot_base frames; default policy frame = robot_base. `ee_frame = panda_hand`.
- Gripper width in meters, range [0, 0.08] = finger_joint1+finger_joint2; each finger = width/2. aperture = (width - closed)/(open - closed).
- Depth saved as `uint16` millimetres; HDF5 gzip compressed. Image size 256×256.
- Quaternion order is `[w,x,y,z]` everywhere (matches Isaac `get_world_pose`).
- Pure-logic tests run with default python (`python3 -m pytest`, has numpy+yaml). Sim smoke scripts run with `/isaac-sim/python.sh`.
- Frequent commits: one per task. Author is already configured on branch `cup-to-sink-benchmark`.

---

### Task 1: Package scaffold + transforms (pure math)

**Files:**
- Create: `cup_to_sink/__init__.py` (empty)
- Create: `cup_to_sink/transforms.py`
- Test: `cup_to_sink/tests/__init__.py` (empty), `cup_to_sink/tests/test_transforms.py`

**Interfaces:**
- Produces (all numpy, quats `[w,x,y,z]`):
  - `quat_to_axis_angle(quat) -> np.ndarray(3)` (rotation vector)
  - `axis_angle_to_quat(rotvec) -> np.ndarray(4)`
  - `pose7d_to_6d(pose7d) -> np.ndarray(6)` ; `pose6d_to_7d(pose6d) -> np.ndarray(7)`
  - `make_T(position, quat) -> np.ndarray(4,4)` ; `T_to_pose7d(T) -> np.ndarray(7)`
  - `compose(T_a, T_b) -> np.ndarray(4,4)` (= T_a @ T_b)
  - `invert_T(T) -> np.ndarray(4,4)`
  - `world_to_base(pose7d_world, base_pose7d) -> np.ndarray(7)` (pose expressed in base frame)
  - `quat_mul(q1, q2)`, `quat_conj(q)`, `quat_rotate(q, v)`

- [ ] **Step 1: Write failing tests**

```python
# cup_to_sink/tests/test_transforms.py
import numpy as np
from cup_to_sink import transforms as T

def test_axis_angle_roundtrip():
    q = np.array([np.cos(0.3), 0.0, np.sin(0.3), 0.0])  # 0.6 rad about +Y
    rv = T.quat_to_axis_angle(q)
    np.testing.assert_allclose(np.linalg.norm(rv), 0.6, atol=1e-6)
    q2 = T.axis_angle_to_quat(rv)
    # same rotation (allow sign flip of quaternion)
    assert np.allclose(q2, q, atol=1e-6) or np.allclose(q2, -q, atol=1e-6)

def test_identity_axis_angle():
    rv = T.quat_to_axis_angle(np.array([1.0, 0, 0, 0]))
    np.testing.assert_allclose(rv, np.zeros(3), atol=1e-9)

def test_pose_6d_7d_roundtrip():
    p7 = np.array([1.0, 2.0, 3.0, np.cos(0.2), 0.0, 0.0, np.sin(0.2)])
    p6 = T.pose7d_to_6d(p7)
    p7b = T.pose6d_to_7d(p6)
    np.testing.assert_allclose(p7b[:3], p7[:3], atol=1e-9)
    assert np.allclose(p7b[3:], p7[3:], atol=1e-6) or np.allclose(p7b[3:], -p7[3:], atol=1e-6)

def test_make_T_and_compose_identity():
    Tid = T.make_T(np.zeros(3), np.array([1.0,0,0,0]))
    np.testing.assert_allclose(Tid, np.eye(4), atol=1e-9)
    A = T.make_T(np.array([1.,0,0]), np.array([1.,0,0,0]))
    np.testing.assert_allclose(T.compose(A, A)[:3,3], np.array([2.,0,0]), atol=1e-9)

def test_world_to_base_at_origin_is_identity():
    base = np.array([0.,0,0,1,0,0,0])
    p = np.array([1.,2,3,1,0,0,0])
    np.testing.assert_allclose(T.world_to_base(p, base), p, atol=1e-9)

def test_world_to_base_translation():
    base = np.array([1.,0,0,1,0,0,0])
    p = np.array([3.,0,0,1,0,0,0])
    out = T.world_to_base(p, base)
    np.testing.assert_allclose(out[:3], np.array([2.,0,0]), atol=1e-9)

def test_invert_T():
    A = T.make_T(np.array([1.,2,3]), T.axis_angle_to_quat(np.array([0.1,0.2,0.3])))
    np.testing.assert_allclose(T.compose(A, T.invert_T(A)), np.eye(4), atol=1e-6)
```

- [ ] **Step 2: Run, verify fail** — `cd /mnt/dev-tools/isaacsim-template && python3 -m pytest cup_to_sink/tests/test_transforms.py -q` → FAIL (module missing).

- [ ] **Step 3: Implement** `cup_to_sink/transforms.py` using numpy only. Quaternions `[w,x,y,z]`. `quat_to_axis_angle`: normalize, `angle=2*atan2(|v|,w)`, axis=`v/|v|`, return `axis*angle` (zero vector when angle≈0). `axis_angle_to_quat`: `angle=|rv|`, `w=cos(angle/2)`, `v=sin(angle/2)*rv/angle` (identity when angle≈0). `make_T`: rotation matrix from quat into top-left 3×3, position in last column. `T_to_pose7d`: extract position + quat from matrix (use a standard matrix→quat). `world_to_base(p, base) = T_to_pose7d(invert_T(make_T(base)) @ make_T(p))`. Provide `quat_mul/quat_conj/quat_rotate` helpers.

- [ ] **Step 4: Run, verify pass** — same pytest command → PASS (7 tests).

- [ ] **Step 5: Commit** — `git add cup_to_sink/__init__.py cup_to_sink/transforms.py cup_to_sink/tests && git commit -m "feat(cup_to_sink): pose/quaternion transform utilities"`

---

### Task 2: Gripper conversions + success condition + randomization (pure)

**Files:**
- Create: `cup_to_sink/gripper.py`, `cup_to_sink/success.py`, `cup_to_sink/randomize.py`
- Test: `cup_to_sink/tests/test_gripper.py`, `test_success.py`, `test_randomize.py`

**Interfaces:**
- `gripper.width_to_finger_joints(width) -> np.ndarray(2)` (= `[width/2, width/2]`)
- `gripper.finger_joints_to_width(joints) -> float` (= sum)
- `gripper.width_to_aperture(width, open_w, closed_w) -> float` (clipped [0,1])
- `gripper.aperture_to_width(aperture, open_w, closed_w) -> float`
- `success.check(cup_pos, sink_pos, gripper_width, cfg_success, open_w) -> (bool, dict)` where dict has `xy_ok,z_ok,gripper_ok,xy_dist,cup_z`
- `randomize.sample_episode(seed, rnd_cfg, sink_base_pose7d, cup_base_xy) -> dict` returns `{cup_xy, cup_yaw, cup_z_offset, sink_target_pose7d}` deterministically from seed.

- [ ] **Step 1: Write failing tests**

```python
# test_gripper.py
import numpy as np; from cup_to_sink import gripper as g
def test_width_to_fingers(): np.testing.assert_allclose(g.width_to_finger_joints(0.08), [0.04,0.04])
def test_fingers_to_width(): assert abs(g.finger_joints_to_width(np.array([0.04,0.04])) - 0.08) < 1e-9
def test_aperture(): assert abs(g.width_to_aperture(0.04,0.08,0.0)-0.5)<1e-9
def test_aperture_clip(): assert g.width_to_aperture(0.2,0.08,0.0)==1.0 and g.width_to_aperture(-1,0.08,0.0)==0.0

# test_success.py
from cup_to_sink import success
CFG={"xy_threshold":0.06,"z_min_offset":-0.03,"z_max_offset":0.15}
def test_success_true():
    ok,info=success.check([1.0,1.0,0.85],[1.01,1.0,0.80],0.08,CFG,0.08)
    assert ok and info["xy_ok"] and info["z_ok"] and info["gripper_ok"]
def test_fail_xy():
    ok,_=success.check([1.5,1.0,0.85],[1.0,1.0,0.80],0.08,CFG,0.08); assert not ok
def test_fail_gripper_closed():
    ok,info=success.check([1.0,1.0,0.85],[1.0,1.0,0.80],0.0,CFG,0.08); assert not ok and not info["gripper_ok"]

# test_randomize.py
import numpy as np; from cup_to_sink import randomize
RND={"cup_xlim":[0.0,0.1],"cup_ylim":[0.0,0.1],"cup_yaw_range":[-3.14,3.14],"cup_z_offset_range":[0.0,0.0],
     "sink_target_sample_xy":True,"sink_target_xlim":[-0.02,0.02],"sink_target_ylim":[-0.02,0.02]}
def test_determinism():
    a=randomize.sample_episode(7,RND,np.array([1.,1,0.8,1,0,0,0]),np.array([0.5,0.0]))
    b=randomize.sample_episode(7,RND,np.array([1.,1,0.8,1,0,0,0]),np.array([0.5,0.0]))
    np.testing.assert_allclose(a["cup_xy"],b["cup_xy"]); np.testing.assert_allclose(a["sink_target_pose7d"],b["sink_target_pose7d"])
def test_different_seeds_differ():
    a=randomize.sample_episode(1,RND,np.array([1.,1,0.8,1,0,0,0]),np.array([0.5,0.0]))
    b=randomize.sample_episode(2,RND,np.array([1.,1,0.8,1,0,0,0]),np.array([0.5,0.0]))
    assert not np.allclose(a["cup_xy"],b["cup_xy"])
def test_within_limits():
    a=randomize.sample_episode(3,RND,np.array([1.,1,0.8,1,0,0,0]),np.array([0.5,0.0]))
    assert RND["cup_xlim"][0] <= a["cup_xy"][0]-0.5 <= RND["cup_xlim"][1]
```

- [ ] **Step 2: Run, verify fail** — `python3 -m pytest cup_to_sink/tests/test_gripper.py cup_to_sink/tests/test_success.py cup_to_sink/tests/test_randomize.py -q` → FAIL.

- [ ] **Step 3: Implement**
  - `gripper.py`: the four functions above (numpy, clip aperture to [0,1]).
  - `success.py`: `check` computes `xy_dist=hypot(dx,dy)` between cup and sink xy; `xy_ok = xy_dist < xy_threshold`; `z_ok = sink_z + z_min_offset <= cup_z <= sink_z + z_max_offset`; `gripper_ok = gripper_width > 0.5*open_w` (open enough to have released). Return `(success, info)`.
  - `randomize.py`: `sample_episode` builds `rng = np.random.default_rng(seed)`; `cup_xy = cup_base_xy + [uniform(xlim), uniform(ylim)]`; `cup_yaw = uniform(cup_yaw_range)`; `cup_z_offset = uniform(cup_z_offset_range)`; if `sink_target_sample_xy`, offset sink xy by uniform within `sink_target_xlim/ylim` else use base; quat copied from `sink_base_pose7d`. Use the SAME rng draw order every call for determinism.

- [ ] **Step 4: Run, verify pass** → PASS.

- [ ] **Step 5: Commit** — `git commit -am "feat(cup_to_sink): gripper, success, randomization (pure logic)"`

---

### Task 3: Config + config loader + HDF5 writer (round-trip tested)

**Files:**
- Create: `cup_to_sink/configs/cup_to_sink.yaml` (from plan §5 skeleton; placeholder scene coords)
- Create: `cup_to_sink/config.py` (load + resolve to a dataclass-ish dict; expand env not needed)
- Create: `cup_to_sink/dataset_writer.py`
- Test: `cup_to_sink/tests/test_dataset_writer.py` (run with isaac python — needs h5py)

**Interfaces:**
- `config.load(path) -> dict` (yaml.safe_load; returns nested dict; also stores raw text under key `__raw_yaml__`).
- `dataset_writer.Recorder()` with:
  - `.append(step: dict)` — buffers one control step (keys: `joint_pos, joint_vel, gripper_joint_pos, gripper_width, gripper_aperture, ee_pose_7d_world, ee_pose_6d_world, ee_pose_7d_base, ee_pose_6d_base, cup_pose, sink_target_pose, images{cam:{rgb,depth}}, action{...}, debug{...}, phase`)
  - `.write(path, meta: dict)` — writes the full plan §7 HDF5 layout, depth as uint16 mm, gzip.
  - `.num_steps -> int`

- [ ] **Step 1: Write failing test** (synthetic 3-step episode, write+reopen, assert shapes/dtypes/meta)

```python
# cup_to_sink/tests/test_dataset_writer.py
import numpy as np, h5py, tempfile, os
from cup_to_sink.dataset_writer import Recorder
def _step(i):
    img={"front":{"rgb":np.zeros((4,4,3),np.uint8),"depth":np.full((4,4),1000,np.uint16)}}
    return dict(joint_pos=np.zeros(7),joint_vel=np.zeros(7),gripper_joint_pos=np.array([0.04,0.04]),
        gripper_width=0.08,gripper_aperture=1.0,ee_pose_7d_world=np.array([0.,0,0,1,0,0,0]),
        ee_pose_6d_world=np.zeros(6),ee_pose_7d_base=np.array([0.,0,0,1,0,0,0]),ee_pose_6d_base=np.zeros(6),
        cup_pose=np.array([1.,0,0,1,0,0,0]),sink_target_pose=np.array([1.,1,0,1,0,0,0]),images=img,
        action=dict(joint_pos_target=np.zeros(7),ee_pose_target_7d=np.array([0.,0,0,1,0,0,0]),
            ee_pose_target_6d=np.zeros(6),delta_ee_pose_target_6d=np.zeros(6),
            gripper_joint_pos_target=np.array([0.04,0.04]),gripper_width_target=0.08,gripper_aperture_target=1.0,
            policy_action_command=np.zeros(8),policy_action_ee_command=np.zeros(7)),
        debug=dict(sparse_ee_target_pose=np.array([0.,0,0,1,0,0,0]),grasp_pose_7d=np.array([0.,0,0,1,0,0,0]),
            pregrasp_pose_7d=np.array([0.,0,0,1,0,0,0])),phase=i)
def test_write_roundtrip():
    r=Recorder()
    for i in range(3): r.append(_step(i))
    p=os.path.join(tempfile.mkdtemp(),"ep.hdf5")
    r.write(p, meta=dict(task_name="cup_to_sink",language_instruction="put the cup into the sink",seed=5,
        success=True,enabled_cameras=["front"],joint_names=["j%d"%k for k in range(7)],
        randomization=dict(seed=5,cup_initial_pose=np.zeros(7),sink_target_pose=np.zeros(7))))
    with h5py.File(p,"r") as f:
        assert f["/observations/joint_pos"].shape==(3,7)
        assert f["/observations/images/front/rgb"].shape==(3,4,4,3)
        assert f["/observations/images/front/depth"].dtype==np.uint16
        assert f["/actions/policy_action_command"].shape==(3,8)
        assert f["/actions/policy_action_next_state"].shape==(3,8)   # derived from next joint+gripper
        assert f["/meta/seed"][()]==5
        assert f["/meta/randomization/seed"][()]==5
        assert f.attrs.get("schema_version") is not None or "/meta/task_name" in f
```

- [ ] **Step 2: Run, verify fail** — `/isaac-sim/python.sh -m pytest cup_to_sink/tests/test_dataset_writer.py -q` (cd repo root) → FAIL. *(If pytest missing in isaac env: `/isaac-sim/python.sh -m pip install pytest`.)*

- [ ] **Step 3: Implement** `dataset_writer.py`:
  - Buffer steps in lists. On `write`, stack arrays along axis 0.
  - Derive `policy_action_next_state[t] = concat(joint_pos[t+1], gripper_width[t+1])` (last step repeats itself), and `policy_action_ee_next_state[t] = concat(ee_pose_6d_base[t+1], gripper_width[t+1])`.
  - Write datasets under exact plan §7 paths: `/observations/{joint_pos,joint_vel,gripper_joint_pos,gripper_width,gripper_aperture,ee_pose_7d,ee_pose_6d,cup_pose,sink_target_pose}` (use base-frame ee for `/observations/ee_pose_*`, and also store world under `/observations/ee_pose_7d_world` etc.), `/observations/images/<cam>/{rgb,depth}`, `/actions/*`, `/debug/*`, `/meta/*`, `/meta/randomization/*`.
  - All image/array datasets `compression="gzip"`. Strings via `np.bytes_`. Store full config text at `/meta/config_yaml`.
  - Skip cameras not present in a step gracefully.

- [ ] **Step 4: Run, verify pass** → PASS.

- [ ] **Step 5: Commit** — `git add cup_to_sink/configs cup_to_sink/config.py cup_to_sink/dataset_writer.py cup_to_sink/tests/test_dataset_writer.py && git commit -m "feat(cup_to_sink): config loader + HDF5 dataset writer"`

---

### Task 4: `inspect_scene.py` — boot kitchen and calibrate coordinates

**Files:**
- Create: `cup_to_sink/sim_app.py` (SimulationApp bootstrap helper)
- Create: `cup_to_sink/inspect_scene.py`

**Interfaces:**
- `sim_app.start(headless=True, width=1280, height=720) -> SimulationApp` (mirrors `apartment.py` startup: stdout restore, kit cache copy). Must be called before importing `pxr`/`isaacsim`.
- `inspect_scene` is an entry (`python -m cup_to_sink.inspect_scene`): loads kitchen USD, traverses prims, prints prim path/type and world AABB for prims whose name matches `sink|counter|basin|table|worktop|cabinet|stove`, prints the kitchen overall bounds and the default ground height. Outputs a suggested `sink_target_pose_world`, `franka.base_position`, and `cup` spawn xy.

- [ ] **Step 1: Implement `sim_app.start`** — copy the startup block from `examples/apartment.py:60-86` (SimulationApp dict with `headless` param, stdout/stderr restore). Add kit-cache copy from `apartment.py:36-41`.

- [ ] **Step 2: Implement `inspect_scene.py`** — after `start(headless=True)`: `from isaacsim.core.utils.stage import add_reference_to_stage`; add kitchen at `/World/Kitchen`; `from pxr import Usd, UsdGeom`; `bb=UsdGeom.BBoxCache(...)`; traverse `stage.Traverse()`, print matches + AABB; finally `simulation_app.close()`.

- [ ] **Step 3: Run** — `cd /mnt/dev-tools/isaacsim-template && /isaac-sim/python.sh -m cup_to_sink.inspect_scene 2>&1 | tee /tmp/inspect.log` → observe printed prim tree + bounds. Expected: a usable counter/sink AABB and ground height.

- [ ] **Step 4: Calibrate config** — edit `cup_to_sink/configs/cup_to_sink.yaml`: set `scene.sink_target_pose_world` to a point inside the sink/counter region, `robot.base_position` so the Franka reaches both the cup spawn and sink (≈0.6 m from both), `randomization.cup_xlim/ylim` to a small reachable patch on the counter, counter-top z for cup spawn. Record reasoning as YAML comments.

- [ ] **Step 5: Commit** — `git add cup_to_sink/sim_app.py cup_to_sink/inspect_scene.py cup_to_sink/configs/cup_to_sink.yaml && git commit -m "feat(cup_to_sink): scene inspector + calibrated scene coords"`

---

### Task 5: `env_builder.py` — build kitchen + Franka + cup + sink + cameras handles

**Files:**
- Create: `cup_to_sink/cameras.py`
- Create: `cup_to_sink/env_builder.py`

**Interfaces:**
- `cameras.setup_cameras(cfg_cameras, enabled) -> dict[name -> Camera]` — create `isaacsim.sensors.camera.Camera` per enabled camera at config pose/resolution, `initialize()` + `add_distance_to_image_plane_to_frame()`.
- `cameras.capture(cams, enabled) -> dict[name -> {rgb:uint8(H,W,3), depth:uint16(H,W) mm}]` — read `get_rgba()[:,:,:3]` and `get_depth()` (m → `*1000` clipped to uint16).
- `env_builder.Env` dataclass holding: `world, franka (SingleManipulator), gripper, cup (SingleRigidPrim), sink_xform_path, cams, stage, cfg, robot_base_pose7d`.
- `env_builder.build_env(cfg, simulation_app) -> Env` — loads ground + kitchen, spawns Franka (fixed base, `AlternateFinger`/`Quality` variants, high-friction fingers as in `apartment.py:438-497`), spawns cup with SDF/convex collision + high-friction material, creates invisible sink `Xform` at `sink_target_pose_world`, sets up cameras, `world.reset()`, opens gripper.

- [ ] **Step 1: Implement `cameras.py`** per interface. World cams: create `Camera(prim_path, position, orientation, resolution=(w,h))`; set focal length from `fov` (or use `set_horizontal_aperture`/`set_focal_length`). Wrist cam: create under `parent_prim_path` (panda_hand) with local offset so it rides the hand.

- [ ] **Step 2: Implement `env_builder.build_env`** — reuse the proven Franka setup from `apartment.py` (ParallelGripper opened `[0.04,0.04]` closed `[0.0,0.0]`, `action_deltas=None`; high-friction material `/World/PhysicsMaterials/high_friction` bound to both fingers; cup gets the same material + `physx_utils.setRigidBody(prim, "convexHull"/"sdf", False)`). Sink Xform via `define_prim(sink_path,"Xform")` + set translate; make it invisible (`UsdGeom.Imageable(prim).MakeInvisible()`).

- [ ] **Step 3: Smoke test** — create `cup_to_sink/tests/smoke_env.py` that `start()`s, `build_env(load(cfg))`, steps 60 times, prints franka dof, cup world pose, sink pose, then `capture` shapes, then closes. Run: `/isaac-sim/python.sh -m cup_to_sink.tests.smoke_env`. Expected: prints `dof: 9`, finite cup pose on the counter, camera rgb shape `(256,256,3)` uint8, depth `(256,256)` uint16.

- [ ] **Step 4: Verify** — confirm cup settles on the counter (z stable across last 10 steps), cameras non-empty (depth has finite values). Adjust cup spawn z / camera poses in config if needed; re-run.

- [ ] **Step 5: Commit** — `git add cup_to_sink/cameras.py cup_to_sink/env_builder.py cup_to_sink/tests/smoke_env.py && git commit -m "feat(cup_to_sink): env builder + 4-camera RGB-D capture"`

---

### Task 6: `planner.py` — Planner ABC + RMPFlowPlanner

**Files:**
- Create: `cup_to_sink/planner.py`

**Interfaces:**
- `class Planner(ABC)`: `reset()`, `set_target(ee_pose7d_world)`, `step(current_joint_positions) -> ArticulationAction` (per-control-step joint targets), `reached(current_ee_pose7d, pos_tol=0.01, rot_tol=0.05) -> bool`.
- `class RMPFlowPlanner(Planner)`: wraps Isaac Lula RMPFlow for the Franka. Constructed with the `franka` articulation + robot description / urdf from the Isaac motion-generation Franka config (`isaacsim.robot.motion_generation` `interface_config_loader.load_supported_motion_policy_config("Franka","RMPflow")`). Uses `ArticulationMotionPolicy` to convert to `ArticulationAction` at `control_dt`.

- [ ] **Step 1: Implement** `RMPFlowPlanner` following Isaac's RMPFlow standalone pattern: build `RmpFlow(**rmp_cfg)`, set robot base pose to the Franka world pose, `articulation_rmpflow = ArticulationMotionPolicy(franka, rmpflow, default_physics_dt=control_dt)`. `set_target` → `rmpflow.set_end_effector_target(target_position, target_orientation)` (orientation `[w,x,y,z]`). `step` → `articulation_rmpflow.get_next_articulation_action()`. `reached` compares current EE pose (from `rmpflow.get_end_effector_pose` or franka hand prim) to target within tol.

- [ ] **Step 2: Smoke test** — `cup_to_sink/tests/smoke_planner.py`: build env, create planner, set a reachable target 0.2 m above the cup with a top-down orientation, loop `step→apply_action→world.step` up to 300 steps, break when `reached`. Print final EE pos error. Run `/isaac-sim/python.sh -m cup_to_sink.tests.smoke_planner`.

- [ ] **Step 3: Verify** — Expected: position error < 0.02 m within 300 steps. If RMPFlow config name differs in 5.1, list `interface_config_loader.get_supported_robot_policy_pairs()` and use the correct key; record it in a code comment.

- [ ] **Step 4: Commit** — `git add cup_to_sink/planner.py cup_to_sink/tests/smoke_planner.py && git commit -m "feat(cup_to_sink): Planner ABC + RMPFlow planner"`

---

### Task 7: `actors.py` + `skills.py` — affordances and sparse EE goals

**Files:**
- Create: `cup_to_sink/actors.py`, `cup_to_sink/skills.py`
- Test: `cup_to_sink/tests/test_actors.py` (pure: fake env with fixed cup pose)

**Interfaces:**
- `actors.CupActor(cup_prim, cfg_cup)`: `get_world_pose() -> pose7d`; `get_contact_point(id=0) -> pose7d_world` (= `T_world_cup @ T_cup_grasp_i`); `get_functional_point(id=0) -> pose7d_world`. `pregrasp_distance(id)` from cfg.
- `actors.SinkTarget(sampled_pose7d)`: `get_functional_point() -> pose7d`.
- `skills.grasp_actor(actor, pre_grasp_dis, grasp_dis, gripper_width_target, contact_point_id) -> list[action]`
- `skills.move_by_displacement(current_ee_pose7d, dx,dy,dz) -> list[action]`
- `skills.place_actor(sink_target, target_pose7d, pre_dis, retreat_h, is_open) -> list[action]`
- An `action` is `{"type":"move","ee_pose7d":...,"phase":str}` or `{"type":"gripper","width":float,"phase":str}`.

- [ ] **Step 1: Write failing test** for `CupActor.get_contact_point` using a fake cup object exposing `get_world_pose() -> (pos,quat)`:

```python
# test_actors.py
import numpy as np
from cup_to_sink.actors import CupActor
class FakeCup:
    def get_world_pose(self): return (np.array([1.,2,0.8]), np.array([1.,0,0,0]))
CFG={"contact_points":[{"name":"side","position_obj":[0.045,0,0.08],"quat_obj":[1.,0,0,0],"pregrasp_distance":0.10}],
     "functional_points":[{"name":"c","position_obj":[0.,0,0],"quat_obj":[1.,0,0,0]}]}
def test_contact_point_world():
    a=CupActor(FakeCup(),CFG)
    p=a.get_contact_point(0)
    np.testing.assert_allclose(p[:3], np.array([1.045,2.0,0.88]), atol=1e-6)
def test_pregrasp_distance(): assert CupActor(FakeCup(),CFG).pregrasp_distance(0)==0.10
```

- [ ] **Step 2: Run, verify fail** — `python3 -m pytest cup_to_sink/tests/test_actors.py -q` → FAIL.

- [ ] **Step 3: Implement** `actors.py` using `transforms.make_T`/`compose`/`T_to_pose7d`. `skills.py`: `grasp_actor` emits `[gripper open, move pregrasp, move grasp, gripper close]` where pregrasp = grasp pose translated `-pre_grasp_dis` along the grasp approach axis (gripper local +Z/-Z; use grasp orientation to offset along approach). `place_actor` emits `[move preplace (target + lift), move place (target), gripper open, move retreat (up by retreat_h)]`. `move_by_displacement` emits one move to `current + (dx,dy,dz)` keeping orientation.

- [ ] **Step 4: Run, verify pass** → PASS.

- [ ] **Step 5: Commit** — `git add cup_to_sink/actors.py cup_to_sink/skills.py cup_to_sink/tests/test_actors.py && git commit -m "feat(cup_to_sink): actors (affordances) + skills (sparse EE goals)"`

---

### Task 8: `task.py` — reset/randomization/observation/success

**Files:**
- Create: `cup_to_sink/task.py`

**Interfaces:**
- `class Task(env, cfg)`:
  - `reset(seed) -> dict` (sampled randomization): set cup pose from `randomize.sample_episode`, set sink Xform translate to sampled target, set Franka to default qpos + open gripper, step `settle_steps`. Returns randomization dict + records initial poses.
  - `get_obs() -> dict` (unified schema): joint_pos/vel, gripper_joint_pos/width/aperture, ee_pose 7d/6d in world+base, cup_pose, sink_target_pose, images (from `cameras.capture`), language_instruction, timestep.
  - `check_success() -> (bool, info)` via `success.check` using live cup + sampled sink pose + measured gripper width.
  - `instruction -> str` from cfg.
  - helpers: `ee_pose_world()`, `current_qpos()`, `gripper_width()`.

- [ ] **Step 1: Implement `task.py`.** EE pose from the `panda_hand` prim world pose (`XFormPrim`/`get_world_pose`); base pose from `env.robot_base_pose7d`; compute 6d via `transforms.pose7d_to_6d`; base-frame via `transforms.world_to_base`. Default qpos from `cfg.randomization.robot_default_qpos` applied via `franka.set_joint_positions` on the 7 arm joints.

- [ ] **Step 2: Smoke test** — `cup_to_sink/tests/smoke_task.py`: build env, `task.reset(0)` then `task.reset(1)`, print cup pose and sink target for both seeds (must differ), print one `get_obs()` summary (key shapes), print `check_success()` (should be False right after reset). Run with isaac python.

- [ ] **Step 3: Verify** — seeds 0 and 1 give different cup xy and sink xy; obs has all keys with finite values; success False initially.

- [ ] **Step 4: Commit** — `git add cup_to_sink/task.py cup_to_sink/tests/smoke_task.py && git commit -m "feat(cup_to_sink): Task reset/obs/success"`

---

### Task 9: `executor.py` — run skill actions, record steps

**Files:**
- Create: `cup_to_sink/executor.py`

**Interfaces:**
- `executor.execute(env, task, planner, actions, recorder, cfg) -> dict` — iterate the skill `actions`:
  - `move`: `planner.set_target(ee_pose7d)`; loop: `act=planner.step(qpos); franka.apply_action(act); world.step(render=True)`; each step build a full step dict via `task.get_obs()` + action fields (joint_pos_target from `act`, ee target from current sparse goal, gripper target held) and `recorder.append(step)`; stop when `planner.reached` or `max_move_steps`.
  - `gripper`: interpolate finger joints from current width to target over `gripper_steps`; each step apply gripper action + record.
  - tags each step with `phase`.
  - returns `{"completed":bool, "steps":recorder.num_steps}`.

- [ ] **Step 1: Implement `executor.py`.** Build the per-step `action` dict: `joint_pos_target` = commanded arm targets; `gripper_width_target` = current semantic target; `ee_pose_target_7d/6d` = current sparse goal (move) or last goal (gripper); `delta_ee_pose_target_6d` = goal_6d − prev_goal_6d; `policy_action_command` = `concat(joint_pos_target, [gripper_width_target])`; `policy_action_ee_command` = `concat(ee_pose_target_6d_base, [gripper_width_target])`. Debug: `sparse_ee_target_pose`, `grasp_pose_7d`, `pregrasp_pose_7d` (passed in via cfg/skill context).

- [ ] **Step 2: Smoke test** — `cup_to_sink/tests/smoke_execute.py`: build env+task+planner, reset(0), build a tiny 2-action sequence (open gripper, move 0.1 m up), execute with a Recorder, assert recorder.num_steps > 0 and last obs finite. Run isaac python.

- [ ] **Step 3: Verify** — steps recorded, EE moved ~0.1 m, gripper opened.

- [ ] **Step 4: Commit** — `git add cup_to_sink/executor.py cup_to_sink/tests/smoke_execute.py && git commit -m "feat(cup_to_sink): executor records dense steps"`

---

### Task 10: `collect_data.py` — play_once + single successful demo

**Files:**
- Create: `cup_to_sink/collect_data.py`

**Interfaces:**
- `collect_data.play_once(env, task, planner, cfg, seed) -> (success, recorder, meta)` — reset(seed); assemble the full expert skill sequence (plan §6) from `actors`+`skills`; `executor.execute`; `check_success`; build meta (config_yaml, scene/cup usd paths, robot base, initial poses, joint names, dt, gripper widths, ee frame/rotation type, enabled cameras, full randomization).
- `main(--config, [--num-success], [--max-attempts], [--out])` — build env once; loop seeds 0..max_attempts; on success write `datasets/cup_to_sink/episode_XXXX.hdf5` and increment; stop at num_success; print running log + final summary (success count, failed seeds).

- [ ] **Step 1: Implement `collect_data.py`.** Expert sequence: `grasp_actor(cup, pregrasp_dis, 0, closed_width, contact_id)` → `move_by_displacement(0,0,lift_height)` → `place_actor(sink, sink_target_pose, preplace_distance, retreat_height, is_open=True)`. Compose into the action list, run executor. Build meta dict matching dataset_writer's expected keys.

- [ ] **Step 2: Run single demo** — temporarily `--num-success 1 --max-attempts 5`: `cd /mnt/dev-tools/isaacsim-template && /isaac-sim/python.sh -m cup_to_sink.collect_data --config cup_to_sink/configs/cup_to_sink.yaml --num-success 1 --max-attempts 5 2>&1 | tee /tmp/collect1.log`.

- [ ] **Step 3: Verify** — Expected: at least one `episode_0000.hdf5` written; log shows `success`. Inspect: `/isaac-sim/python.sh -c "import h5py;f=h5py.File('datasets/cup_to_sink/episode_0000.hdf5');print(list(f['observations'].keys()));print(f['observations/images/front/rgb'].shape, f['observations/images/front/depth'].dtype);print('cup',f['meta/initial_cup_pose'][:]);print('sink',f['meta/sink_target_pose'][:])"`. If grasp/place fails, tune `motion.lift_height`, `grasp` contact point, `success` thresholds, friction; re-run.

- [ ] **Step 4: Commit** — `git add cup_to_sink/collect_data.py && git commit -m "feat(cup_to_sink): collect_data play_once + single demo"`

---

### Task 11: `replay.py` — action replay

**Files:**
- Create: `cup_to_sink/replay.py`

**Interfaces:**
- `main(--episode, --mode action, --action-mode joint_pos|ee_pose)` — read HDF5; build env from `/meta/config_yaml` (or `--config`); reset to recorded initial cup/sink/robot state; for each timestep apply `joint_pos_target`+`gripper_width_target` (or EE via planner); after the rollout call `task.check_success()` and print whether replay still succeeds.

- [ ] **Step 1: Implement `replay.py`.** Load episode meta, rebuild env, set cup to `/meta/initial_cup_pose`, sink Xform to `/meta/sink_target_pose`, franka to `/meta/initial_robot_qpos`. Loop `/actions/joint_pos_target` rows, apply ArticulationAction(joint_positions=row7 + finger targets from `gripper_width_target`), `world.step`. Report final success.

- [ ] **Step 2: Run** — `/isaac-sim/python.sh -m cup_to_sink.replay --episode datasets/cup_to_sink/episode_0000.hdf5 --mode action --action-mode joint_pos 2>&1 | tee /tmp/replay.log`.

- [ ] **Step 3: Verify** — runs to completion and prints a success/fail verdict (physics non-determinism may differ from collection; record the outcome).

- [ ] **Step 4: Commit** — `git add cup_to_sink/replay.py && git commit -m "feat(cup_to_sink): action replay tool"`

---

### Task 12: Scale to 50 demos + distribution check + README

**Files:**
- Modify: `cup_to_sink/collect_data.py` (final summary already present; add per-seed randomization log line + write `datasets/cup_to_sink/collect_log.json`)
- Create: `cup_to_sink/README.md`
- Create: `cup_to_sink/tests/check_distribution.py`

- [ ] **Step 1: Implement `collect_log.json`** — record per saved episode: seed, cup_initial_pose, sink_target_pose, contact_point_id, num_steps; record failed seeds list; final success_rate.

- [ ] **Step 2: Implement `check_distribution.py`** — open all `episode_*.hdf5`, print std-dev of cup xy and sink xy across episodes; assert each std > 0 (poses actually vary) and robot initial qpos identical across episodes.

- [ ] **Step 3: Run full collection** — `/isaac-sim/python.sh -m cup_to_sink.collect_data --config cup_to_sink/configs/cup_to_sink.yaml 2>&1 | tee datasets/cup_to_sink/collect_run.log` (num_success=50, max_attempts=500). This is long-running; run in background and monitor.

- [ ] **Step 4: Verify** — `success count == 50`, `attempts <= 500`; run `check_distribution.py` → cup/sink poses vary, qpos constant. Spot-replay one episode.

- [ ] **Step 5: Write README** — install (`h5py`), run collect/replay/inspect commands, scene recalibration note, output layout, deferred-eval note.

- [ ] **Step 6: Commit** — `git add cup_to_sink/README.md cup_to_sink/collect_data.py cup_to_sink/tests/check_distribution.py && git commit -m "feat(cup_to_sink): 50-demo collection, distribution check, README"`

---

## Self-Review Notes (coverage vs spec)

- Planner ABC + RMPFlow → Task 6 (cuRobo-ready). ✓
- Sink runtime Xform + seeded sampling → Tasks 4,5,8. ✓
- Full §7 HDF5 schema (joint+EE, command+next_state, world+base, debug, randomization meta) → Task 3. ✓
- 4-cam RGB-D 256² uint16-mm → Tasks 3,5. ✓
- Gripper continuous width/aperture + finger joints → Task 2, recorded Tasks 3,9. ✓
- Expert sequence (plan §6) → Tasks 7,10. ✓
- Success condition (plan §11) → Task 2. ✓
- Randomization meta + distribution check → Tasks 2,8,12. ✓
- collect_data / replay entries → Tasks 10,11. ✓
- README + setup → Task 12. ✓
- Deferred: policy_interface, eval_policy (per spec §13). Not in plan by design.
- Open risk carried into execution: exact Isaac 5.1 RMPFlow config key, kitchen geometry, grasp stability — each has a verify/tune step (Tasks 4,5,6,10).
