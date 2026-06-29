"""
env_builder.py — Build the cup_to_sink simulation environment.

All pxr/isaacsim/omni imports are deferred inside build_env() so this module
can be safely imported before SimulationApp is started.
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

# Repository root: this file lives at <repo>/cup_to_sink/env_builder.py,
# so parents[1] is the repo root.
_REPO_ROOT = Path(__file__).resolve().parents[1]


@dataclass
class Env:
    """All simulation handles needed by the cup_to_sink benchmark."""

    world: Any              # isaacsim.core.api.World
    franka: Any             # SingleManipulator
    gripper: Any            # ParallelGripper
    cup: Any                # SingleRigidPrim
    sink_path: str          # USD prim path to the invisible SinkTarget Xform
    cams: dict              # name -> Camera
    stage: Any              # Usd.Stage
    cfg: dict               # raw config dict
    robot_base_pose7d: np.ndarray  # [x, y, z, qw, qx, qy, qz]


def build_env(cfg: dict, simulation_app: Any) -> Env:  # noqa: ARG001
    """Build the kitchen + Franka + cup + sink + cameras environment.

    Replicates the proven Franka setup from examples/apartment.py:
    AlternateFinger/Quality variant selection, ParallelGripper with
    joint_opened=[0.04,0.04] / joint_closed=[0.0,0.0] / action_deltas=None,
    and a /World/PhysicsMaterials/high_friction material (static/dynamic
    friction 2.0) bound to both fingers and the cup.

    Args:
        cfg: loaded config dict (from cup_to_sink.config.load).
        simulation_app: running SimulationApp instance (unused directly but
            must be started before this function is called).

    Returns:
        Env dataclass populated with all simulation handles.
    """
    import omni.usd
    from isaacsim.core.api import World
    from isaacsim.core.prims import SingleRigidPrim
    from isaacsim.core.utils.prims import define_prim, create_prim
    from isaacsim.core.utils.stage import add_reference_to_stage
    from isaacsim.storage.native import get_assets_root_path
    from isaacsim.robot.manipulators import SingleManipulator
    from isaacsim.robot.manipulators.grippers import ParallelGripper
    from omni.physx.scripts import utils as physx_utils
    from pxr import Usd, UsdGeom, UsdShade, UsdPhysics, Gf

    from cup_to_sink.cameras import setup_cameras

    # ── World ─────────────────────────────────────────────────────────────────
    world = World(
        stage_units_in_meters=1.0,
        physics_dt=1.0 / 200.0,
        rendering_dt=8.0 / 200.0,
    )
    world.reset()
    stage = omni.usd.get_context().get_stage()

    # ── Ground ────────────────────────────────────────────────────────────────
    ground_usd = str(_REPO_ROOT / "usd/Grid/default_environment.usd")
    define_prim("/World/Ground", "Xform").GetReferences().AddReference(ground_usd)

    # ── Kitchen ───────────────────────────────────────────────────────────────
    kitchen_usd = str(_REPO_ROOT / cfg["scene"]["lab_usd_path"])
    create_prim(
        usd_path=kitchen_usd,
        prim_path="/World/Kitchen",
    )

    # ── Lights (so raytraced scene is not black) ──────────────────────────────
    define_prim("/World/Lights", "Xform")
    _light_positions = [
        (1.5, -0.4, 2.5),
        (0.5,  0.0, 2.5),
        (2.0,  0.5, 2.5),
    ]
    for i, pos in enumerate(_light_positions, start=1):
        create_prim(
            prim_path=f"/World/Lights/kitchen_light_{i}",
            prim_type="SphereLight",
            attributes={"inputs:intensity": 10000},
            position=pos,
        )

    # ── Franka ────────────────────────────────────────────────────────────────
    robot_cfg = cfg["robot"]
    base_position = np.array(robot_cfg["base_position"], dtype=float)
    base_quat = np.array(robot_cfg["base_quat"], dtype=float)   # [w, x, y, z]
    franka_prim_path: str = robot_cfg["prim_path"]              # "/World/Franka"

    assets_root = get_assets_root_path()
    franka_usd = assets_root + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"

    franka_prim = add_reference_to_stage(usd_path=franka_usd, prim_path=franka_prim_path)
    # Grippy rubber fingers + quality meshes (no-op if variant sets are absent).
    franka_prim.GetVariantSet("Gripper").SetVariantSelection("AlternateFinger")
    franka_prim.GetVariantSet("Mesh").SetVariantSelection("Quality")

    # action_deltas=None so "close" commands the FULL closed target in one shot
    # and holds it — the position error against the grasped object becomes the
    # grip force.
    franka_gripper = ParallelGripper(
        end_effector_prim_path=f"{franka_prim_path}/panda_rightfinger",
        joint_prim_names=robot_cfg["gripper_joint_names"],
        joint_opened_positions=np.array([
            float(robot_cfg["finger_joint_open"]),
            float(robot_cfg["finger_joint_open"]),
        ]),
        joint_closed_positions=np.array([
            float(robot_cfg["finger_joint_closed"]),
            float(robot_cfg["finger_joint_closed"]),
        ]),
        action_deltas=None,
    )
    franka = world.scene.add(
        SingleManipulator(
            prim_path=franka_prim_path,
            name=robot_cfg["name"],
            end_effector_prim_path=f"{franka_prim_path}/panda_rightfinger",
            position=base_position,
            orientation=base_quat,
            gripper=franka_gripper,
        )
    )

    # High-friction physics material — bound to both fingers so grasped objects
    # don't slide out during the lift.  Re-used for the cup below.
    grip_mat_path = "/World/PhysicsMaterials/high_friction"
    grip_mat = UsdShade.Material.Define(stage, grip_mat_path)
    _gm = UsdPhysics.MaterialAPI.Apply(grip_mat.GetPrim())
    _gm.CreateStaticFrictionAttr().Set(2.0)
    _gm.CreateDynamicFrictionAttr().Set(2.0)
    _gm.CreateRestitutionAttr().Set(0.0)
    for _finger_path in [
        f"{franka_prim_path}/panda_leftfinger",
        f"{franka_prim_path}/panda_rightfinger",
    ]:
        UsdShade.MaterialBindingAPI.Apply(stage.GetPrimAtPath(_finger_path)).Bind(
            grip_mat,
            bindingStrength=UsdShade.Tokens.strongerThanDescendants,
            materialPurpose="physics",
        )

    # ── Cup ───────────────────────────────────────────────────────────────────
    scene_cfg = cfg["scene"]
    cup_usd = str(_REPO_ROOT / scene_cfg["cup_usd_path"])
    cup_prim_path: str = scene_cfg["cup_prim_path"]  # "/World/Objects/Cup"

    # Ensure /World/Objects Xform parent exists before adding the cup reference.
    define_prim("/World/Objects", "Xform")

    cup_prim_ref = add_reference_to_stage(usd_path=cup_usd, prim_path=cup_prim_path)

    # Compute spawn Z from the cup's bounding box so its base sits just above
    # the counter surface — same approach as apartment.py lines 532-549.
    counter_top_z = float(scene_cfg["counter_top_z"])
    _SPAWN_GAP = 0.02   # 2 cm clearance above counter before physics settle
    _bb_cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy],
    )
    _bb = _bb_cache.ComputeWorldBound(cup_prim_ref)
    _min_z = float(_bb.ComputeAlignedRange().GetMin()[2])
    if not np.isfinite(_min_z):
        _min_z = 0.0

    # Cup spawn XY: read explicit key if present, else fall back to
    # cup_spawn_base_xy (the basin centre XY used for randomisation).
    _cup_spawn = scene_cfg.get(
        "cup_spawn_position",
        [
            float(scene_cfg["cup_spawn_base_xy"][0]),
            float(scene_cfg["cup_spawn_base_xy"][1]),
            counter_top_z,
        ],
    )
    spawn_x = float(_cup_spawn[0])
    spawn_y = float(_cup_spawn[1])
    # Place cup base SPAWN_GAP above the counter (will settle by physics).
    spawn_z = counter_top_z + _SPAWN_GAP - _min_z

    # Cup.usd ships with SM_Cup already carrying a RigidBodyAPI; nesting
    # another RigidBodyAPI on the parent (/World/Objects/Cup) causes a PhysX
    # hierarchy error.  Detect the mesh child and apply setRigidBody + material
    # there; fall back to the parent prim if SM_Cup is absent.
    cup_mesh_path = cup_prim_path + "/SM_Cup"
    cup_mesh_prim = stage.GetPrimAtPath(cup_mesh_path)
    if cup_mesh_prim and cup_mesh_prim.IsValid():
        # setRigidBody on SM_Cup is idempotent — it (re-)applies RigidBodyAPI
        # and ensures a convexHull collision shape is present.
        physx_utils.setRigidBody(cup_mesh_prim, "convexHull", False)
        UsdShade.MaterialBindingAPI.Apply(cup_mesh_prim).Bind(
            grip_mat,
            bindingStrength=UsdShade.Tokens.strongerThanDescendants,
            materialPurpose="physics",
        )
        rigid_cup_path = cup_mesh_path
    else:
        physx_utils.setRigidBody(cup_prim_ref, "convexHull", False)
        UsdShade.MaterialBindingAPI.Apply(cup_prim_ref).Bind(
            grip_mat,
            bindingStrength=UsdShade.Tokens.strongerThanDescendants,
            materialPurpose="physics",
        )
        rigid_cup_path = cup_prim_path

    cup = world.scene.add(
        SingleRigidPrim(
            prim_path=rigid_cup_path,
            name="cup",
            position=np.array([spawn_x, spawn_y, spawn_z]),
            orientation=np.array([1.0, 0.0, 0.0, 0.0]),
        )
    )

    # ── Sink target Xform (invisible marker) ─────────────────────────────────
    # [x, y, z, qw, qx, qy, qz]
    sink_target_pose = scene_cfg["sink_target_pose_world"]
    sink_path = "/World/SinkTarget"
    sink_prim = define_prim(sink_path, "Xform")
    sink_xformable = UsdGeom.Xformable(sink_prim)
    sink_xformable.ClearXformOpOrder()
    sink_xformable.AddTranslateOp().Set(
        Gf.Vec3d(
            float(sink_target_pose[0]),
            float(sink_target_pose[1]),
            float(sink_target_pose[2]),
        )
    )
    # xformOp:orient defaults to float precision; use GfQuatf to match.
    _sq = sink_target_pose[3:]  # [qw, qx, qy, qz]
    sink_xformable.AddOrientOp().Set(
        Gf.Quatf(float(_sq[0]), float(_sq[1]), float(_sq[2]), float(_sq[3]))
    )
    UsdGeom.Imageable(sink_prim).MakeInvisible()

    # ── Cameras ───────────────────────────────────────────────────────────────
    enabled = cfg["dataset"]["enabled_cameras"]
    cams = setup_cameras(cfg["cameras"], enabled)

    # ── Final reset + physics settle + open gripper ───────────────────────────
    # world.reset() re-inits the physics view so all new articulations
    # (Franka) and rigid bodies (cup) are fully registered.
    world.reset()

    # Settle physics so the cup lands on the counter before any episode starts.
    for _ in range(20):
        world.step(render=False)

    # Drive the gripper to its open default state so the first grasp descends
    # with an open hand.  (set_default_state records the open target;
    # apply_action actually drives the fingers there now.)
    franka.gripper.set_default_state(franka.gripper.joint_opened_positions)
    franka.get_articulation_controller().apply_action(
        franka.gripper.forward("open")
    )

    for _ in range(10):
        world.step(render=False)

    robot_base_pose7d = np.concatenate([base_position, base_quat])

    return Env(
        world=world,
        franka=franka,
        gripper=franka_gripper,
        cup=cup,
        sink_path=sink_path,
        cams=cams,
        stage=stage,
        cfg=cfg,
        robot_base_pose7d=robot_base_pose7d,
    )
