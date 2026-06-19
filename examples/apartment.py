#!/usr/bin/env python
# coding: utf-8

# # Isaac Sim Jupyter Notebook Tutorial
# 
# This notebook loads the IAI apartment and a Stretch robot, attaches a camera,
# and drives the robot over ROS 2 (`cmd_vel` / `joint_states` / camera image).
# 
# > Select the jupyter kernel `Isaac Sim Python 3.11` if you are running in VScode.
# >
# > "Control + Enter" to execute the selected code cell.

# ## Start the GPU monitor and virtual desktop
# 
# Make sure the GPU has at least 3000M free memory.
# 
# If the virtual desktop does not pop up, manually click the "Open Desktop in new Tab".

# In[ ]:


from IPython import get_ipython
in_notebook = get_ipython().__class__.__name__ == "ZMQInteractiveShell"

import os
import shutil
from utils import *
from pathlib import Path

# Current script directory
try:
    BASE_DIR = Path(__file__).resolve().parent
except NameError:
    BASE_DIR = Path(os.getcwd())

# Copy the precompiled kit cache if it is not present yet
target_dir = "/isaac-sim/kit/cache"
source_dir = "/mnt/isaacsim-cache/cache"
if os.path.isdir(source_dir) and not os.path.isdir(target_dir):
    shutil.copytree(source_dir, target_dir)

# only runs in Jupyter Notebook
if in_notebook:
    from gpu_monitor import GPUMonitor
    gpu_monitor = GPUMonitor()
    display_desktop()


# ## Start SimulationApp
# 
# The application window is frozen and non-interactive, which is normal.
# 
# <div style="color:red">This will take some time, so give it a minute and wait for it to say <b>"SimulationApp Ready!"</b> before you go to next step.</div>

# In[ ]:


from isaacsim import SimulationApp
from IPython.display import clear_output
import sys

original_stdout = sys.stdout
original_stderr = sys.stderr

simulation_app = SimulationApp({
    "headless": False,
    "hide_ui": True,
    "width": 1280,
    "height": 960,
    "renderer": "RaytracedLighting",
    "display_options": 3286,  # show the default grid
})

# Fix the issue where notebook output is being hijacked by Isaac Sim.
sys.stdout = original_stdout
sys.stderr = original_stderr
clear_output(wait=True)
print('SimulationApp Ready!')


# ## Create the simulation environment
# 
# Physics updates at 200 Hz and rendering at 25 Hz. We load the ground, the
# apartment, a few lights, and point the viewport camera at the scene.

# In[ ]:


import numpy as np
from isaacsim.core.api import World
from isaacsim.core.utils.prims import define_prim, create_prim
from isaacsim.core.utils import viewports
from isaacsim.core.utils.extensions import enable_extension

enable_extension("isaacsim.ros2.bridge")

my_world = World(stage_units_in_meters=1.0, physics_dt=1 / 200, rendering_dt=8 / 200)
my_world.reset()

# Ground
define_prim("/World/Ground", "Xform").GetReferences().AddReference(
    f"{BASE_DIR}/../usd/Grid/default_environment.usd"
)

# Apartment
create_prim(
    usd_path=f"{BASE_DIR}/../usd/apartment/apartmentICRA.usda",
    prim_path="/World/Apartment",
    position=np.array([-6, 5, 0.0701]),
)

# Lights so the raytraced scene is not black
for i in range(1, 4):
    create_prim(
        prim_path=f"/World/Ground/Light_{i}",
        prim_type="SphereLight",
        attributes={"inputs:intensity": 10000},
        position=(-4 * i, 0, 2),
    )

viewports.set_camera_view(eye=np.array([-6.5, -2, 2]), target=np.array([-1, 1, 1]))

for _ in range(30):
    my_world.step(render=True)


# ## Spawn the robots
# 
# We control the **Stretch** (it gets an `Articulation`). The other IAI robots
# (`pr2`, `hsrb`, `tiago_dual`) are spawned as static props only — no
# `Articulation`, so they are not simulated or controllable.

# In[ ]:


from isaacsim.core.prims import Articulation

# Stretch is the robot we control (it gets an Articulation below).
create_prim(
    usd_path=f"{BASE_DIR}/../usd/stretch/stretch.usd",
    prim_path="/World/stretch",
    position=np.array([-1.5, 0, 0.05]),
    orientation=np.array([0, 0, 0, 1]),
)

# The other IAI robots are spawned as static props only (no Articulation,
# so they are not simulated/controllable -- just there for the scene).
other_robots = ["pr2", 
                "hsrb", 
                # "tiago_dual"
               ]
for i, robot in enumerate(other_robots):
    create_prim(
        usd_path=f"{BASE_DIR}/../usd/{robot}/{robot}.usd",
        prim_path=f"/World/{robot}",
        position=np.array([-(i + 1) * 2 - 1.5, 0, 0.05]),
        orientation=np.array([0, 0, 0, 1]),
    )

stretch = Articulation(prim_paths_expr="/World/stretch", name="stretch")
my_world.reset()

for _ in range(10):
    my_world.step(render=True)


# ## Fix the telescoping arm joint gains
# 
# The Stretch arm extends through four serially-chained prismatic joints
# (`joint_arm_l0` … `joint_arm_l3`, each travelling 0.13 m for 0.52 m total). When
# the robot is imported from URDF, Isaac auto-generates their drive gains from the
# (tiny) link masses, giving a stiffness of only ~20 N/m — so commanding the arm
# to extend produces a restoring force of barely a couple of newtons and the
# segments sag straight back ("spring back") instead of holding.
# 
# We raise the PD gains on those four joints to values comparable to the
# (hand-tuned) `joint_lift` so the arm holds its commanded extension.

# In[ ]:


# The four telescoping segments share the load, so give each a stiff drive.
# (joint_lift, for comparison, uses stiffness 50000 / damping 300.)
arm_joints = ["joint_arm_l0", "joint_arm_l1", "joint_arm_l2", "joint_arm_l3"]
arm_dof = np.array([stretch.get_dof_index(n) for n in arm_joints])

kps = np.full((1, len(arm_dof)), 1.0e4)   # stiffness [N/m]
kds = np.full((1, len(arm_dof)), 2.0e2)   # damping  [N/(m/s)]
stretch.set_gains(kps=kps, kds=kds, joint_indices=arm_dof)

# A larger force budget so the drive can actually reach the target.
stretch.set_max_efforts(np.full((1, len(arm_dof)), 200.0), joint_indices=arm_dof)

# Cap the speed so the arm extends/retracts gently (URDF default is 1.0 m/s).
ARM_MAX_VEL = 0.1   # m/s per segment
stretch.set_max_joint_velocities(np.full((1, len(arm_dof)), ARM_MAX_VEL), joint_indices=arm_dof)

for _ in range(10):
    my_world.step(render=True)
print("Arm joint gains updated:", arm_joints)


# ## Make the base kinematic
# 
# The two differential-drive wheels have a hard **in-place-rotation deadzone** in
# simulation: the caster is merged into `base_link` and the centre of mass sits
# ahead of the wheel axle, so an in-place pivot needs the wheels to scrub sideways
# and wheel-ground static friction blocks that until `|angular.z|` is large
# (~0.9). That is exactly the "one wheel turns, the other doesn't, the base won't
# rotate" symptom seen in RViz.
# 
# Rather than fight the contact physics, we drive the base **kinematically**: the
# ROS node integrates the commanded twist into the base pose and teleports
# `base_link` there every step (`StretchROS.integrate_base`). Because the pose is
# forced directly, no contact/friction tweaks are needed -- we only leave the
# wheels undriven (`kp=kd=0`) so their cosmetic spin adds no chassis reaction.
# Odometry stays exact because it finite-differences the now exactly-controlled
# `base_link` pose.

# In[ ]:


# Undrive the wheels (kp=kd=0, zero joint friction). integrate_base spins them by
# setting their velocity STATE for visuals only, so they add no reaction torque;
# the base pose itself is forced kinematically (set_world_poses), so no contact
# or material changes are required. These are tensor-API calls only (no USD stage
# edit), so the physics view stays valid.
wheel_dof = np.array([stretch.get_dof_index("joint_left_wheel"),
                      stretch.get_dof_index("joint_right_wheel")])
stretch.set_gains(kps=np.zeros((1, 2)), kds=np.zeros((1, 2)), joint_indices=wheel_dof)
stretch.set_friction_coefficients(np.zeros((1, 2)), joint_indices=wheel_dof)

for _ in range(3):
    my_world.step(render=True)
print("Base set to kinematic: undriven wheels, pose-integrated base.")


# ## Add head and gripper cameras
# 
# The Stretch carries an RGB camera on its pan/tilt head and a second one on the
# wrist looking down past the gripper. We attach an Isaac `Camera` sensor to each.

# In[ ]:


import omni
from pxr import UsdGeom
from isaacsim.sensors.camera import Camera
import isaacsim.core.utils.numpy.rotations as rot_utils

stage = omni.usd.get_context().get_stage()


def create_cam(prim_path, focal_length=1.0, orientation=(0, 0, 0)):
    UsdGeom.Camera.Define(stage, prim_path)
    cam = Camera(
        prim_path=prim_path,
        frequency=30,
        resolution=(640, 360),
        orientation=rot_utils.euler_angles_to_quats(np.array(orientation), degrees=True),
    )
    cam.initialize()
    cam.set_focal_length(focal_length)
    cam.set_clipping_range(near_distance=0.01, far_distance=20)
    return cam


# Head camera (on the pan/tilt head) and gripper camera (on the wrist).
head_cam_prim = "/World/stretch/link_head_tilt/camera_bottom_screw_frame/camera_link/camera_color_frame/camera_color_optical_frame"
gripper_cam_prim = "/World/stretch/link_wrist_roll/link_gripper_s3_body/gripper_camera_bottom_screw_frame/gripper_camera_link/gripper_camera_color_frame/gripper_camera_color_optical_frame"

head_cam = create_cam(head_cam_prim, focal_length=1.5, orientation=[-90, 0, 0])
gripper_cam = create_cam(gripper_cam_prim, orientation=[0, 0, 0])

for _ in range(20):
    my_world.step(render=True)


# ## Add a 2D lidar (RTX)
# 
# The Stretch carries an RPLidar on its base (the URDF `laser` frame). We attach an
# Isaac **RTX lidar** there using the SLAMTEC `RPLIDAR_S2E` profile and the
# `IsaacComputeRTXLidarFlatScan` annotator, which produces a 2D `LaserScan`. The
# ROS node below publishes it on `/scan` — the input for SLAM and Nav2 costmaps.

# In[ ]:


# A 2D lidar (RTX) mounted on the Stretch base, at the URDF joint_laser pose
# (xyz 0.004 0 0.1664, yaw 180 deg). We use the SLAMTEC RPLIDAR S2E profile -- a
# 2D 360-degree rotating lidar -- and attach the flat-scan annotator, which
# yields LaserScan-compatible fields (ranges, intensities, azimuth range,
# horizontal resolution) that the ROS node below publishes on /scan.
from isaacsim.core.utils.extensions import enable_extension
enable_extension("isaacsim.sensors.rtx")

from isaacsim.sensors.rtx import LidarRtx

lidar = LidarRtx(
    prim_path="/World/stretch/base_link/lidar",
    name="stretch_lidar",
    translation=np.array([0.004, 0.0, 0.1664]),
    orientation=rot_utils.euler_angles_to_quats(np.array([0, 0, 180]), degrees=True),  # [w, x, y, z]
    config_file_name="RPLIDAR_S2E",
)
lidar.attach_annotator("IsaacComputeRTXLidarFlatScan")
lidar.initialize()

# Warm up the RTX pipeline so the first flat scan is populated.
for _ in range(10):
    my_world.step(render=True)

_frame = lidar.get_current_frame()
print("RTX lidar ready. beams:", len(_frame.get("linear_depth_data", [])),
      "| azimuth range (deg):", _frame.get("azimuth_range"))


# ## IK and gripper control
# 
# Helper methods for **inverse-kinematics** end-effector control and opening/closing
# the gripper. We use Lula's `LulaKinematicsSolver`, which reads the manipulator
# chain from `usd/stretch/stretch_descriptor.yaml` (lift + the four telescoping arm
# segments + wrist yaw) and solves for the joint angles that place the
# `link_grasp_center` frame at a Cartesian target.
# 
# - `move_ee(world_pos)` — solve IK for a world-frame target and drive the arm there.
# - `set_gripper(value)` — open/close the gripper (`GRIPPER_OPEN` / `GRIPPER_CLOSE`).
# 
# The arm reaches sideways (along the robot's −Y), so reachable targets sit roughly
# at `x ≈ base_x`, `y ∈ [base_y − 0.93, base_y − 0.42]`, `z = lift + 0.11`. These
# same `ik_solver` / `ik_dof` / `finger_dof` / `EE_QUAT` values also back the ROS
# `/stretch/ee_command` and `/stretch/gripper_command` control in the bridge below.
# 
# > Run this section **before** the ROS bridge below. The arm gain fix above must
# > already have been applied.

# In[ ]:


from isaacsim.core.utils.extensions import enable_extension
enable_extension("isaacsim.robot_motion.motion_generation")

from isaacsim.robot_motion.motion_generation import LulaKinematicsSolver

# --- IK solver over the Stretch manipulator chain ---
stretch_dir = f"{BASE_DIR}/../usd/stretch"
ik_solver = LulaKinematicsSolver(
    robot_description_path=f"{stretch_dir}/stretch_descriptor.yaml",
    urdf_path=f"{stretch_dir}/stretch.urdf",
)
EE_FRAME = "link_grasp_center"

ik_joints = ik_solver.get_joint_names()                       # cspace order
ik_dof = [stretch.get_dof_index(n) for n in ik_joints]
finger_dof = [stretch.get_dof_index(n)
              for n in ("joint_gripper_finger_left", "joint_gripper_finger_right")]

# Keep wrist yaw at 0 -> gripper points along the arm (a sideways grasp).
EE_QUAT = rot_utils.euler_angles_to_quats(np.array([0, 0, -90]), degrees=True)
GRIPPER_OPEN, GRIPPER_CLOSE = 0.55, 0.0       # finger joint at 0 = fully closed


def step_sim(n=1):
    """Step the world n times."""
    for _ in range(n):
        my_world.step(render=True)


def set_gripper(value, n=40):
    """Open/close the gripper: drive both finger joints to `value`
    (GRIPPER_OPEN / GRIPPER_CLOSE)."""
    tgt = stretch.get_joint_positions()[0].copy()
    for i in finger_dof:
        tgt[i] = value
    stretch.set_joint_position_targets([tgt])
    step_sim(n)


def move_ee(world_pos, n=160):
    """Solve IK for grasp_center at world_pos and drive the joints there."""
    base_p, base_q = stretch.get_world_poses()
    ik_solver.set_robot_base_pose(base_p[0], base_q[0])
    warm = stretch.get_joint_positions()[0][ik_dof]
    q, ok = ik_solver.compute_inverse_kinematics(
        EE_FRAME, np.asarray(world_pos, dtype=float), EE_QUAT, warm_start=warm,
    )
    if not ok:
        print(f"  IK did not converge for {np.round(world_pos, 3)} (best effort)")
    tgt = stretch.get_joint_positions()[0].copy()
    for j, v in zip(ik_dof, q):
        tgt[j] = v
    stretch.set_joint_position_targets([tgt])
    step_sim(n)
    return ok


print("IK ready. cspace joints:", ik_joints)


# ## Add a Franka arm doing looping pick-and-place
# 
# We drop a **Franka Panda** on the apartment table, yawed 90° counter-clockwise so
# it faces world **+Y**, and have it shuttle a **banana** back and forth across the
# table forever. The banana ships as a visual-only YCB mesh, so we add an **SDF**
# (signed-distance-field) mesh collision + rigid body at load — it follows the
# banana's real curved shape rather than a fat convex hull.
# 
# The built-in `PickPlaceController` (RMPflow + a 10-phase grasp state machine) does
# the motion; a tiny `FrankaPickPlace` manager runs the loop (side A → side B, then
# back). RMPflow uses the robot's **world** pose as its base, so all targets are
# world-frame; the grasp height is read from the banana's **settled** bounding box.
# A high-friction material on the fingers + banana keeps it from slipping out.
# 
# Because the banana is long and thin, the manager rotates the top-down grasp so the
# gripper closes across its **short** axis (picked from the bounding box each
# attempt). If the grasp ever closes along the banana's *length* instead, flip the
# convention with `GRASP_YAW_OFFSET = 90`.
# 
# > The arm is autonomous (no ROS topic). Run these three cells before the
# > simulation loop; the loop calls `franka_mgr.step()` each tick.

# In[ ]:


import omni
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.robot.manipulators import SingleManipulator
from isaacsim.robot.manipulators.grippers import ParallelGripper
from isaacsim.storage.native import get_assets_root_path
import isaacsim.core.utils.numpy.rotations as rot_utils
from pxr import UsdShade, UsdPhysics

stage = omni.usd.get_context().get_stage()

# Franka base sits on the apartment table; the arm has a fixed base so it stays
# put here regardless of what is underneath. The objects below derive their grasp
# height from where they actually settle, so this need not match the table exactly.
FRANKA_BASE = np.array([-2.0, -1.86, 0.75])
TABLE_TOP_Z = 1.0
# Yaw the arm 90 deg counter-clockwise about +Z, so its "forward" (+X local)
# now points along world +Y. The pick-and-place row is laid out to match below.
FRANKA_YAW = 90.0
FRANKA_QUAT = rot_utils.euler_angles_to_quats(np.array([0, 0, FRANKA_YAW]), degrees=True)  # [w, x, y, z]

assets_root = get_assets_root_path()
franka_usd = assets_root + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
franka_prim = add_reference_to_stage(usd_path=franka_usd, prim_path="/World/Franka")
# Grippy rubber fingers + quality meshes (no-op if the variant sets are absent).
franka_prim.GetVariantSet("Gripper").SetVariantSelection("AlternateFinger")
franka_prim.GetVariantSet("Mesh").SetVariantSelection("Quality")

# action_deltas=None so "close" commands the FULL closed target ([0, 0]) in one
# shot and holds it -- the position error against the grasped object becomes the
# grip force. (With deltas set, "close" only nudges current - delta each step and
# never fully clamps.)
franka_gripper = ParallelGripper(
    end_effector_prim_path="/World/Franka/panda_rightfinger",
    joint_prim_names=["panda_finger_joint1", "panda_finger_joint2"],
    joint_opened_positions=np.array([0.04, 0.04]),
    joint_closed_positions=np.array([0.0, 0.0]),
    action_deltas=None,
)
franka = my_world.scene.add(
    SingleManipulator(
        prim_path="/World/Franka",
        name="franka",
        end_effector_prim_path="/World/Franka/panda_rightfinger",
        position=FRANKA_BASE,
        orientation=FRANKA_QUAT,
        gripper=franka_gripper,
    )
)

# High-friction physics material so grasped objects don't slide out of the
# gripper during the lift. Bind it to both fingers here (before the reset so it
# is applied); the object's collision gets the same material in the next cell.
grip_mat = UsdShade.Material.Define(stage, "/World/PhysicsMaterials/high_friction")
_gm = UsdPhysics.MaterialAPI.Apply(grip_mat.GetPrim())
_gm.CreateStaticFrictionAttr().Set(2.0)
_gm.CreateDynamicFrictionAttr().Set(2.0)
_gm.CreateRestitutionAttr().Set(0.0)
for _finger in ["/World/Franka/panda_leftfinger", "/World/Franka/panda_rightfinger"]:
    UsdShade.MaterialBindingAPI.Apply(stage.GetPrimAtPath(_finger)).Bind(
        grip_mat, bindingStrength=UsdShade.Tokens.strongerThanDescendants, materialPurpose="physics")

# Re-init the physics view so the new articulation is registered. Stretch's drive
# gain / friction fixes are USD/PhysX attributes and survive this reset.
my_world.reset()
# Actually drive the gripper OPEN now. set_default_state only records the open
# default; with no reset after it the fingers keep the Franka articulation default
# (closed), so the first pick would descend with a shut gripper -- the controller
# only opens the gripper at the release phase, never before the first grasp.
franka.gripper.set_default_state(franka.gripper.joint_opened_positions)
franka.get_articulation_controller().apply_action(franka.gripper.forward("open"))
for _ in range(10):
    my_world.step(render=True)
print("Franka spawned at", FRANKA_BASE, "yaw", FRANKA_YAW, "deg | dof:", franka.num_dof)


# In[ ]:


import omni
from isaacsim.core.prims import SingleRigidPrim
from isaacsim.robot.manipulators.examples.franka.controllers.pick_place_controller import PickPlaceController
from omni.physx.scripts import utils as physx_utils
from pxr import Usd, UsdGeom, UsdShade

stage = omni.usd.get_context().get_stage()
grip_mat = UsdShade.Material.Get(stage, "/World/PhysicsMaterials/high_friction")  # high-friction grasp material (from the Franka cell)

# Just the banana now. It is a visual-only YCB mesh, so we add an *SDF* (signed-
# distance-field) mesh collision -- it follows the banana's actual curved shape
# (not a fat convex hull) and still works for a dynamic rigid body. grasp_frac
# picks the top-down grasp height along the settled object (0 = bottom, 1 = top).
YCB = "/Isaac/Props/YCB"
OBJECT_SPECS = [
    {"name": "banana", "usd": f"{YCB}/Axis_Aligned/011_banana.usd", "collision": "sdf", "yaw": 0.0, "grasp_frac": 0.5},
]

# Object(s) in front of the arm (+Y), shuttled across the table along X.
ROW_Y = [0.45]                                               # forward distance from base (+Y)
X_A, X_B = FRANKA_BASE[0] + 0.20, FRANKA_BASE[0] - 0.20      # side A / side B (along X)
# The gripper's fingers open across the object's SHORT horizontal axis (chosen
# from the settled bounding box). If it instead closes along the banana's length,
# the absolute gripper-axis convention is 90 deg off -- set this to 90.
GRASP_YAW_OFFSET = 0.0
# Rest each object just above the table (its surface is ~ the arm base height) so
# it barely settles -- a big drop would topple/scatter it.
SPAWN_TOP = FRANKA_BASE[2]
SPAWN_GAP = 0.02
_spawn_bb = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                              [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy])

franka_objs = []
for i, spec in enumerate(OBJECT_SPECS):
    prim_path = f"/World/franka_obj_{i}"
    prim = add_reference_to_stage(usd_path=assets_root + spec["usd"], prim_path=prim_path)
    # Lowest point of the asset relative to its origin, so we can sit it on the table.
    _min_z = float(_spawn_bb.ComputeWorldBound(prim).ComputeAlignedRange().GetMin()[2])
    if not np.isfinite(_min_z):
        _min_z = 0.0
    physx_utils.setRigidBody(prim, spec["collision"], False)   # collision + dynamic body
    if grip_mat:                                               # grippy contact (matches the fingers)
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(
            grip_mat, bindingStrength=UsdShade.Tokens.strongerThanDescendants, materialPurpose="physics")
    slot_a = np.array([X_A, FRANKA_BASE[1] + ROW_Y[i]])
    q = rot_utils.euler_angles_to_quats(np.array([0, 0, spec["yaw"]]), degrees=True)
    spawn_z = SPAWN_TOP + SPAWN_GAP - _min_z                  # object base sits SPAWN_GAP over the table
    obj = my_world.scene.add(SingleRigidPrim(
        prim_path=prim_path, name=f"franka_obj_{i}",
        position=np.array([slot_a[0], slot_a[1], spawn_z]),
        orientation=q,
    ))
    franka_objs.append({"name": spec["name"], "path": prim_path, "prim": obj,
                        "grasp_frac": spec["grasp_frac"], "slot_a": slot_a,
                        "slot_b": np.array([X_B, FRANKA_BASE[1] + ROW_Y[i]])})

# my_world.reset()
for _ in range(90):                                           # let the objects settle on the table
    my_world.step(render=True)

# Per-object top-down grasp height from the settled world AABB. Reading where the
# objects actually came to rest makes this robust to the real table height.
_bbox = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                          [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy])
for o in franka_objs:
    rng = _bbox.ComputeWorldBound(stage.GetPrimAtPath(o["path"])).ComputeAlignedRange()
    zmin, zmax = float(rng.GetMin()[2]), float(rng.GetMax()[2])
    o["grasp_z"] = zmin + o["grasp_frac"] * (zmax - zmin)
    p = o["prim"].get_world_pose()[0]
    print(f"{o['name']:9s} settled @ ({p[0]:.2f}, {p[1]:.2f}, {p[2]:.2f})  "
          f"z=[{zmin:.3f}, {zmax:.3f}]  grasp_z={o['grasp_z']:.3f}")

# Transit/approach height: clear the tallest settled object with margin.
LIFT_Z = max(o["grasp_z"] for o in franka_objs) + 0.20

# Pick-place controller -- built AFTER the Franka is positioned so RMPFlow
# captures the correct world base pose. All target heights are world-frame.
franka_controller = PickPlaceController(
    name="franka_pick_place",
    gripper=franka.gripper,
    robot_articulation=franka,
    end_effector_initial_height=LIFT_Z,
)


class FrankaPickPlace:
    """Shuttle the objects side A -> side B, then B -> A, forever, one at a time."""

    def __init__(self, robot, controller, objs, yaw_offset=0.0):
        self.robot, self.controller, self.objs = robot, controller, objs
        self.yaw_offset = yaw_offset
        self.idx, self.reverse = 0, False
        self.art = robot.get_articulation_controller()
        self._pick_xy = None            # latched per pick attempt (recomputed at event 0)
        self._grasp_q = None

    def _acquire(self, o):
        """Read the object's live settled bounding box: return its horizontal
        CENTRE (for the pre-grasp xy) and a straight-down grasp orientation whose
        fingers open across the object's SHORT axis."""
        bb = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                               [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy])
        r = bb.ComputeWorldBound(stage.GetPrimAtPath(o["path"])).ComputeAlignedRange()
        lo, hi = r.GetMin(), r.GetMax()
        cx, cy = 0.5 * (lo[0] + hi[0]), 0.5 * (lo[1] + hi[1])
        dx, dy = float(hi[0] - lo[0]), float(hi[1] - lo[1])
        # default fingers open along world Y at yaw 0: long axis X -> yaw 0; long axis Y -> yaw 90.
        yaw = (0.0 if dx >= dy else 90.0) + self.yaw_offset
        q = rot_utils.euler_angles_to_quats(np.array([0.0, 180.0, yaw]), degrees=True)
        return np.array([cx, cy]), q

    def step(self):
        o = self.objs[self.idx]
        if self.controller.get_current_event() == 0 and self._pick_xy is None:
            self._pick_xy, self._grasp_q = self._acquire(o)   # dynamically locate the object
        pick = np.array([self._pick_xy[0], self._pick_xy[1], o["grasp_z"]])
        slot = o["slot_a"] if self.reverse else o["slot_b"]
        place = np.array([slot[0], slot[1], o["grasp_z"]])
        actions = self.controller.forward(
            picking_position=pick,
            placing_position=place,
            current_joint_positions=self.robot.get_joint_positions(),
            end_effector_offset=np.array([0, 0.005, 0]),
            end_effector_orientation=self._grasp_q,
        )
        self.art.apply_action(actions)
        if self.controller.is_done():
            self.controller.reset()                              # ready for the next object
            self._pick_xy = None                                 # re-acquire the next object's pose
            self.idx += 1
            if self.idx >= len(self.objs):                       # whole side done -> swap
                self.idx = 0
                self.reverse = not self.reverse


franka_mgr = FrankaPickPlace(franka, franka_controller, franka_objs, yaw_offset=GRASP_YAW_OFFSET)
print("Franka pick-and-place manager ready (banana). LIFT_Z =", round(LIFT_Z, 3))


# ## ROS 2 bridge node
# 
# A single node for the Stretch robot:
# - subscribes to `/stretch/cmd_vel` (manual steering) and `/cmd_vel` (Nav2 goals) and drives the wheels,
# - subscribes to `/stretch/joint_command` and sets joint position targets,
# - subscribes to `/stretch/ee_command` (IK) and `/stretch/gripper_command`,
# - publishes `/stretch/joint_states`, `/head_camera/image_raw`, `/gripper_camera/image_raw`,
# - publishes `/scan` (RTX lidar) and `/odom` (`nav_msgs/Odometry`),
# - broadcasts TF: `odom` → `base_link` → every link, plus a static `base_link` → `laser`.
# 
# The base TF is now `odom → base_link` (ground-truth pose as perfect odometry);
# `slam_toolbox` supplies `map → odom` live from `/scan`.

# In[ ]:


import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, Point
from sensor_msgs.msg import JointState, Image, LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

if not rclpy.ok():
    rclpy.init(args=None)


# --- tiny quaternion helpers (scalar-last x, y, z, w), no external deps ---
def _qconj(q):
    return np.array([-q[0], -q[1], -q[2], q[3]])


def _qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ])


def _qrot(q, v):
    u = q[:3]
    s = q[3]
    return 2 * np.dot(u, v) * u + (s * s - np.dot(u, u)) * v + 2 * s * np.cross(u, v)


def _as_np(x):
    if hasattr(x, "numpy"):  # warp array or torch CPU tensor
        try:
            return x.numpy()
        except Exception:
            return x.detach().cpu().numpy()
    return np.asarray(x)


class StretchROS(Node):
    def __init__(self, robot, head_cam, gripper_cam, lidar=None, prefix="stretch",
                 ik_solver=None, ik_dof=None, finger_dof=None,
                 ee_quat=None, ee_frame="link_grasp_center"):
        super().__init__(f"{prefix}_ros")
        self.robot = robot
        self.head_cam = head_cam
        self.gripper_cam = gripper_cam
        self.lidar = lidar              # LidarRtx with the flat-scan annotator (or None)

        # IK config (optional): lets /ee_command drive move_ee over ROS.
        self.ik_solver = ik_solver
        self.ik_dof = ik_dof
        self.finger_dof = finger_dof
        self.ee_quat = ee_quat
        self.ee_frame = ee_frame

        # Differential base geometry (for the cosmetic wheel spin only -- the
        # base is driven kinematically, see integrate_base). wheel_radius is the
        # real URDF value 0.051 (the old 0.0125 + factor=0.2 were a broken hack).
        self.wheel_base = 0.3407
        self.wheel_radius = 0.051
        self.wheel_dof = np.array([robot.get_dof_index("joint_left_wheel"),
                                   robot.get_dof_index("joint_right_wheel")])

        # Internal commanded base pose for kinematic dead-reckoning, seeded from
        # the robot's current world pose; integrate_base advances it from cmd_vel.
        _p, _q = robot.get_world_poses()
        self._bx, self._by, self._bz = (float(_p[0][0]), float(_p[0][1]),
                                        float(_p[0][2]))
        _w, _x, _y, _z = _q[0]
        self._byaw = math.atan2(2.0 * (_w * _z + _x * _y),
                                1.0 - 2.0 * (_y * _y + _z * _z))
        self._cmd_v = 0.0
        self._cmd_w = 0.0

        # TF: link names and the base link index
        self.body_names = list(robot.body_names)
        self.base_idx = next(
            (i for i, n in enumerate(self.body_names) if "base_link" in n), 0
        )
        self._odom_prev = None          # (t, x, y, yaw) for finite-difference twist

        # cmd_vel: /stretch/cmd_vel for manual steering, /cmd_vel for Nav2 goals.
        self.create_subscription(Twist, f"/{prefix}/cmd_vel", self.cmd_vel_cb, 10)
        self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_cb, 10)
        self.create_subscription(JointState, f"/{prefix}/joint_command", self.joint_cmd_cb, 10)
        self.pub_js = self.create_publisher(JointState, f"/{prefix}/joint_states", 10)
        self.pub_head_img = self.create_publisher(Image, "/head_camera/image_raw", 10)
        self.pub_gripper_img = self.create_publisher(Image, "/gripper_camera/image_raw", 10)
        self.pub_scan = self.create_publisher(LaserScan, "/scan", 10)
        self.pub_odom = self.create_publisher(Odometry, "/odom", 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        self._publish_static_tf()

        # IK-driven end-effector control (only if a solver was provided).
        if self.ik_solver is not None:
            self.create_subscription(Point, f"/{prefix}/ee_command", self.ee_cmd_cb, 10)
            self.create_subscription(Float64, f"/{prefix}/gripper_command", self.gripper_cmd_cb, 10)

    def cmd_vel_cb(self, msg):
        # Just latch the twist; integrate_base (called every sim step) applies it.
        self._cmd_v = float(msg.linear.x)
        self._cmd_w = float(msg.angular.z)

    def integrate_base(self, dt):
        """Kinematic base: dead-reckon the latched twist into the base pose and
        teleport base_link there. Avoids the differential-wheel in-place-rotation
        deadzone (wheels scrub/stick below ~0.9 rad/s) and is exact at any speed.
        The wheels are spun cosmetically (undriven) for TF/visual consistency."""
        v, w = self._cmd_v, self._cmd_w
        yaw = self._byaw
        self._bx += v * math.cos(yaw) * dt
        self._by += v * math.sin(yaw) * dt
        self._byaw = yaw + w * dt
        ny = self._byaw
        quat = np.array([[math.cos(ny / 2.0), 0.0, 0.0, math.sin(ny / 2.0)]])
        self.robot.set_world_poses(
            np.array([[self._bx, self._by, self._bz]], dtype=float), quat)
        # zero root velocity so physics does not add a second displacement
        self.robot.set_velocities(np.zeros((1, 6)))
        # cosmetic wheel spin (velocity STATE, undriven -> no chassis reaction)
        vl = (v - w * self.wheel_base / 2.0) / self.wheel_radius
        vr = (v + w * self.wheel_base / 2.0) / self.wheel_radius
        self.robot.set_joint_velocities(
            np.array([[vl, vr]]), joint_indices=self.wheel_dof)

    def joint_cmd_cb(self, msg):
        target = self.robot.get_joint_positions()[0]
        for name, pos in zip(msg.name, msg.position):
            target[self.robot.get_dof_index(name)] = pos
        self.robot.set_joint_position_targets([target])

    def ee_cmd_cb(self, msg):
        """Solve IK for a base-relative grasp_center target and set joint targets.

        Non-blocking: we only update the position targets, the main loop steps
        the world. The target is relative to base_link, so it stays valid as the
        robot drives around (we solve with the base at the origin)."""
        target = np.array([msg.x, msg.y, msg.z], dtype=float)
        self.ik_solver.set_robot_base_pose(np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0]))
        warm = self.robot.get_joint_positions()[0][self.ik_dof]
        q, ok = self.ik_solver.compute_inverse_kinematics(
            self.ee_frame, target, self.ee_quat, warm_start=warm,
            position_tolerance=0.01, orientation_tolerance=0.6,
        )
        if not ok:
            # Near the workspace edge CCD can flag failure even when the
            # position was reached; accept it if FK is close, else it is truly
            # out of reach.
            reached, _ = self.ik_solver.compute_forward_kinematics(self.ee_frame, q)
            if np.linalg.norm(reached - target) > 0.03:
                self.get_logger().warn(f"IK out of reach: {np.round(target, 3)}")
                return
        tgt = self.robot.get_joint_positions()[0].copy()
        for j, v in zip(self.ik_dof, q):
            tgt[j] = v
        self.robot.set_joint_position_targets([tgt])

    def gripper_cmd_cb(self, msg):
        tgt = self.robot.get_joint_positions()[0].copy()
        for i in self.finger_dof:
            tgt[i] = float(msg.data)
        self.robot.set_joint_position_targets([tgt])

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.robot.dof_names)
        msg.position = self.robot.get_joint_positions()[0].tolist()
        self.pub_js.publish(msg)

    def _image_msg(self, rgb):
        msg = Image()
        msg.height = rgb.shape[0]
        msg.width = rgb.shape[1]
        msg.encoding = "rgb8"
        msg.step = rgb.shape[1] * 3
        msg.data = np.ascontiguousarray(rgb, dtype=np.uint8).tobytes()
        return msg

    def publish_camera(self):
        self.pub_head_img.publish(self._image_msg(self.head_cam.get_rgba()[:, :, :3]))
        self.pub_gripper_img.publish(self._image_msg(self.gripper_cam.get_rgba()[:, :, :3]))

    def publish_scan(self):
        """Publish the RTX lidar flat scan as sensor_msgs/LaserScan on /scan.

        The flat-scan annotator reports azimuth range and resolution in degrees;
        ranges/intensities are arrays of length numBeams in the "laser" frame."""
        if self.lidar is None:
            return
        frame = self.lidar.get_current_frame()
        ranges = _as_np(frame.get("linear_depth_data", []))
        if ranges is None or len(ranges) == 0:
            return
        az = _as_np(frame.get("azimuth_range", [-180.0, 180.0]))
        hres = float(frame.get("horizontal_resolution", 0.0))
        raw = frame.get("IsaacComputeRTXLidarFlatScan", {}) or {}
        depth = _as_np(raw.get("depthRange", [0.05, 30.0]))
        intens = _as_np(frame.get("intensities_data", []))

        n = len(ranges)
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "laser"
        msg.angle_min = math.radians(float(az[0]))
        msg.angle_max = math.radians(float(az[1]))
        msg.angle_increment = (math.radians(hres) if hres > 0
                               else (msg.angle_max - msg.angle_min) / max(n - 1, 1))
        msg.range_min = float(depth[0])
        msg.range_max = float(depth[1])
        msg.ranges = [float(r) for r in ranges]
        if len(intens) == n:
            msg.intensities = [float(i) for i in intens]
        self.pub_scan.publish(msg)

    def publish_odom(self):
        """Publish odom->base_link as nav_msgs/Odometry (ground-truth = perfect
        odometry). Twist is estimated by finite differences in the base frame."""
        lt = _as_np(self.robot._physics_view.get_link_transforms()).reshape(-1, 7)
        p, q = lt[self.base_idx, :3], lt[self.base_idx, 3:7]
        now = self.get_clock().now()

        msg = Odometry()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"
        msg.pose.pose.position.x = float(p[0])
        msg.pose.pose.position.y = float(p[1])
        msg.pose.pose.position.z = float(p[2])
        msg.pose.pose.orientation.x = float(q[0])
        msg.pose.pose.orientation.y = float(q[1])
        msg.pose.pose.orientation.z = float(q[2])
        msg.pose.pose.orientation.w = float(q[3])

        t = now.nanoseconds * 1e-9
        yaw = math.atan2(2.0 * (q[3] * q[2] + q[0] * q[1]),
                         1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2]))
        if self._odom_prev is not None:
            pt, px, py, pyaw = self._odom_prev
            dt = t - pt
            if dt > 1e-6:
                dx, dy = float(p[0]) - px, float(p[1]) - py
                cos_y, sin_y = math.cos(yaw), math.sin(yaw)
                msg.twist.twist.linear.x = (dx * cos_y + dy * sin_y) / dt
                msg.twist.twist.linear.y = (-dx * sin_y + dy * cos_y) / dt
                dyaw = math.atan2(math.sin(yaw - pyaw), math.cos(yaw - pyaw))
                msg.twist.twist.angular.z = dyaw / dt
        self._odom_prev = (t, float(p[0]), float(p[1]), yaw)
        self.pub_odom.publish(msg)

    def _publish_static_tf(self):
        # base_link -> laser, from the URDF joint_laser (xyz 0.004 0 0.1664,
        # yaw pi). The fixed "laser" link is merged out of the articulation on
        # URDF import, so /scan's frame is not in body_names -- publish it here.
        now = self.get_clock().now().to_msg()
        t = self._make_tf(now, "base_link", "laser",
                          np.array([0.004, 0.0, 0.1664]),
                          np.array([0.0, 0.0, 1.0, 0.0]))  # x,y,z,w -> yaw pi
        self.static_tf.sendTransform([t])

    def publish_tf(self):
        # World pose of every link: (num_links, 7) = x, y, z, qx, qy, qz, qw.
        # _physics_view is private but the only non-OmniGraph way to read all link poses.
        lt = _as_np(self.robot._physics_view.get_link_transforms()).reshape(-1, 7)
        pb, qb = lt[self.base_idx, :3], lt[self.base_idx, 3:7]
        qb_inv = _qconj(qb)
        now = self.get_clock().now().to_msg()

        # odom -> base_link (ground-truth pose as perfect odometry). slam_toolbox
        # supplies map -> odom from /scan.
        tfs = [self._make_tf(now, "odom", "base_link", pb, qb)]
        for i, name in enumerate(self.body_names):  # base_link -> every other link
            if i == self.base_idx:
                continue
            p_rel = _qrot(qb_inv, lt[i, :3] - pb)
            q_rel = _qmul(qb_inv, lt[i, 3:7])
            tfs.append(self._make_tf(now, "base_link", name, p_rel, q_rel))

        self.tf_broadcaster.sendTransform(tfs)

    @staticmethod
    def _make_tf(stamp, parent, child, p, q):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.translation.x = float(p[0])
        t.transform.translation.y = float(p[1])
        t.transform.translation.z = float(p[2])
        t.transform.rotation.x = float(q[0])
        t.transform.rotation.y = float(q[1])
        t.transform.rotation.z = float(q[2])
        t.transform.rotation.w = float(q[3])
        return t


# Pass the lidar (if created) so /scan is published, and the IK config (if the
# IK section above has been run) so /ee_command works.
stretch_node = StretchROS(
    stretch, head_cam, gripper_cam, lidar=globals().get("lidar"), prefix="stretch",
    ik_solver=globals().get("ik_solver"),
    ik_dof=globals().get("ik_dof"),
    finger_dof=globals().get("finger_dof"),
    ee_quat=globals().get("EE_QUAT"),
)


# ## Launch SLAM + Nav2
# 
# Start the navigation stack from the prebuilt `nav2_ws` workspace: `slam_toolbox`
# builds a map live from `/scan` and publishes `map → odom`, while `nav2_bringup`
# runs the planner + controller. Nav2's `/cmd_vel` output drives the wheels via the
# node above.
# 
# > Run this **and** the simulation loop below; `slam_toolbox`/Nav2 need `/scan`,
# > `/odom` and TF, which the loop publishes. Then open RViz and use **2D Goal
# > Pose**. You can also launch the stack manually in a terminal:
# > ```
# > source /opt/ros/jazzy/setup.bash && source ../nav2_ws/install/setup.bash
# > ros2 launch stretch_nav2 slam_nav2.launch.py
# > ```

# In[ ]:


import subprocess

# Launch SLAM (slam_toolbox) + Nav2 from the prebuilt ROS workspace. The sim
# loop below must be running so /scan, /odom and TF flow. Set a 2D Goal Pose in
# RViz to navigate; the map is built live as the robot drives.
os.environ.pop("PYTHONPATH", None)
nav_launch = '''
source /opt/ros/jazzy/setup.bash
source ../nav2_ws/install/setup.bash
ros2 launch stretch_nav2 slam_nav2.launch.py
'''
nav2_proc = subprocess.Popen(["bash", "-c", nav_launch],
                             stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
print("SLAM + Nav2 launching (slam_toolbox + nav2_bringup)...")


# ## Open `rviz2`

# In[ ]:


import subprocess

# RViz with the Nav2 config (scan, map, costmaps, robot TF, 2D Goal Pose).
os.environ.pop("PYTHONPATH", None)
launch = '''
source /opt/ros/jazzy/setup.bash
source ../nav2_ws/install/setup.bash
rviz2 -d ./stretch.rviz
'''
subprocess.Popen(["bash", "-c", launch],
                 stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)


# ## Run the simulation loop
# 
# Drag the `rqt_robot_steering` sliders (or publish to `/stretch/cmd_vel`) to drive
# the robot. Rendering and the camera publish every 3rd step to keep physics fast.

# In[ ]:


from tqdm import tqdm

steps = 30000
bar_format = "{l_bar}{bar}| {n_fmt}/{total_fmt} steps] {elapsed_s:.2f}s"

for frame in tqdm(range(steps), desc="ROS Spin", ncols=60, bar_format=bar_format, file=sys.stdout):
    stretch_node.integrate_base(my_world.get_rendering_dt())
    my_world.step(render=True)
    if "franka_mgr" in globals():
        franka_mgr.step()                 # advance the Franka pick-and-place loop
    rclpy.spin_once(stretch_node, timeout_sec=0.0)
    stretch_node.publish_joint_states()
    stretch_node.publish_tf()
    stretch_node.publish_odom()
    stretch_node.publish_camera()
    stretch_node.publish_scan()


# ## Shutdown

# In[ ]:


# Stop the SLAM + Nav2 launch if it is still running.
try:
    nav2_proc.terminate()
except NameError:
    pass

stretch_node.destroy_node()
rclpy.shutdown()
simulation_app.close()


# ## Convert notebook to Python script
# 
# ```
# jupyter nbconvert --to python apartment.ipynb
# ```
