"""
task.py -- Task class for episode reset, observations, and success checking
in the cup_to_sink benchmark.

All pxr/isaacsim/omni imports are deferred to method scope so this module
can be safely imported before SimulationApp is started.
"""
from __future__ import annotations

from typing import Any

import numpy as np


class Task:
    """Manages episode lifecycle: reset, observations, and success checking.

    Args:
        env: ``Env`` dataclass returned by ``cup_to_sink.env_builder.build_env()``.
            Must be fully initialised (world reset + physics settled).
        cfg: config dict returned by ``cup_to_sink.config.load()``.
    """

    def __init__(self, env: Any, cfg: dict) -> None:
        self.env = env
        self.cfg = cfg
        self.instruction: str = cfg["task"]["instruction"]

        # Build joint-name -> index map from the articulation DOF names.
        # dof_names is available after world.reset() initialises the articulation
        # (env_builder guarantees this before returning).
        dof_names = list(env.franka.dof_names)
        self._dof_name_to_idx: dict[str, int] = {
            name: i for i, name in enumerate(dof_names)
        }

        # Arm joint indices -- shape (7,)
        arm_joint_names: list[str] = cfg["robot"]["arm_joint_names"]
        self.arm_idx: np.ndarray = np.array(
            [self._dof_name_to_idx[n] for n in arm_joint_names], dtype=int
        )

        # Finger / gripper joint indices -- shape (2,)
        finger_joint_names: list[str] = cfg["robot"]["gripper_joint_names"]
        self.finger_idx: np.ndarray = np.array(
            [self._dof_name_to_idx[n] for n in finger_joint_names], dtype=int
        )

        # Cache EE XFormPrim once -- stage is already ready because env is fully
        # initialised before __init__ is called (env_builder guarantees this).
        from isaacsim.core.prims import SingleXFormPrim

        ee_prim_path: str = (
            f"{cfg['robot']['prim_path']}/{cfg['robot']['ee_frame']}"
        )
        self._ee_xform: SingleXFormPrim = SingleXFormPrim(ee_prim_path)

        # Persistent state set by reset(); initialised to None until first reset.
        self.sink_target_pose7d: np.ndarray | None = None
        # Episode step counter: reset() sets this to 0; get_obs() returns the
        # current value then increments it, so callers never need to pass a step.
        self._timestep: int = 0

    # --------------------------------------------------------------------------
    # Episode lifecycle
    # --------------------------------------------------------------------------

    def reset(self, seed: int) -> dict:
        """Reset the episode: randomise poses, reset robot, settle physics.

        Steps:
            1. Sample episode randomisation deterministically from ``seed``.
            2. Teleport the cup to the sampled XY/yaw/z position.
            3. Update the sink target Xform in the USD stage.
            4. Set the Franka arm to the default qpos and open the gripper.
            5. Step ``settle_steps`` physics frames (render=False).
            6. Record initial poses and store ``sink_target_pose7d``.

        Args:
            seed: Integer episode seed (different seeds give different
                cup and sink target poses).

        Returns:
            dict containing sampled randomisation values plus the cup and robot
            initial poses (for later writing to HDF5 episode metadata).
        """
        from cup_to_sink.randomize import sample_episode
        from cup_to_sink.transforms import axis_angle_to_quat
        from pxr import Gf

        cfg = self.cfg
        env = self.env
        rnd_cfg = cfg["randomization"]
        scene_cfg = cfg["scene"]
        robot_cfg = cfg["robot"]

        # -- 1. Sample randomisation -------------------------------------------
        samp = sample_episode(
            seed,
            rnd_cfg,
            np.array(scene_cfg["sink_target_pose_world"], dtype=float),
            np.array(scene_cfg["cup_spawn_position"][:2], dtype=float),
        )

        cup_xy: np.ndarray = samp["cup_xy"]
        cup_yaw: float = float(samp["cup_yaw"])
        cup_z_offset: float = float(samp["cup_z_offset"])
        sink_target_pose7d: np.ndarray = samp["sink_target_pose7d"]

        # -- 2. Set cup world pose ---------------------------------------------
        # Z = counter-top Z + cup_z_offset (offset is 0.0 in default config).
        cup_z = float(scene_cfg["cup_spawn_position"][2]) + cup_z_offset
        cup_position = np.array(
            [float(cup_xy[0]), float(cup_xy[1]), cup_z], dtype=float
        )
        cup_orientation = axis_angle_to_quat(np.array([0.0, 0.0, cup_yaw]))
        env.cup.set_world_pose(cup_position, cup_orientation)

        # -- 3. Update sink target Xform in USD stage --------------------------
        # env_builder creates the sink as an Xform with a single
        # xformOp:translate op; we update it in-place to avoid touching
        # the xformOp:orient (which stays at identity -- no rotation needed).
        from pxr import UsdGeom

        sink_prim = env.stage.GetPrimAtPath(env.sink_path)
        _sink_pos = Gf.Vec3d(
            float(sink_target_pose7d[0]),
            float(sink_target_pose7d[1]),
            float(sink_target_pose7d[2]),
        )
        sink_translate_attr = sink_prim.GetAttribute("xformOp:translate")
        if sink_translate_attr and sink_translate_attr.IsValid():
            sink_translate_attr.Set(_sink_pos)
        else:
            # Op absent -- create it so randomisation is never silently skipped.
            xformable = UsdGeom.Xformable(sink_prim)
            translate_op = xformable.AddTranslateOp()
            if translate_op is None:
                raise RuntimeError(
                    f"Sink target Xform at {env.sink_path} has no settable translate op"
                )
            translate_op.Set(_sink_pos)

        # -- 4. Set Franka to default qpos + open gripper ----------------------
        default_qpos: list[float] = list(rnd_cfg["robot_default_qpos"])  # 7 values
        open_finger_pos = float(robot_cfg["gripper_open_width"]) / 2.0

        num_dof: int = env.franka.num_dof
        full_qpos = np.zeros(num_dof, dtype=float)
        for local_i, dof_idx in enumerate(self.arm_idx):
            full_qpos[dof_idx] = default_qpos[local_i]
        for dof_idx in self.finger_idx:
            full_qpos[dof_idx] = open_finger_pos

        env.franka.set_joint_positions(full_qpos)

        # Command gripper to open so it actively holds the open position
        # during the settling period (prevents drift from compliance).
        env.franka.get_articulation_controller().apply_action(
            env.gripper.forward("open")
        )

        # -- 5. Settle physics -------------------------------------------------
        # The arm is NOT actively held here: the grasp sequence is tuned for the
        # naturally-settled start pose, and forcing the arm rigid at default
        # breaks RMPFlow reachability. The episode's intended initial robot
        # configuration is the fixed default home (recorded below), which is what
        # acceptance and replay use; the small settle sag is a transient.
        settle_steps = int(rnd_cfg["settle_steps"])
        for _ in range(settle_steps):
            env.world.step(render=False)

        # -- 6. Record initial state and store persistent target ---------------
        self.sink_target_pose7d = sink_target_pose7d.copy()
        self._timestep = 0

        cup_pos_init, cup_ori_init = env.cup.get_world_pose()
        # Record the COMMANDED default home as the episode's initial qpos (fixed
        # across episodes, per the benchmark spec). The measured post-settle pose
        # drifts slightly under gravity and is not the intended initial config.
        robot_qpos_init = full_qpos.copy()

        # -- 7. Return episode metadata dict -----------------------------------
        return {
            "seed": int(seed),
            "cup_xy": cup_xy.tolist(),
            "cup_yaw": cup_yaw,
            "cup_z_offset": cup_z_offset,
            "sink_target_pose7d": sink_target_pose7d.tolist(),
            "cup_pose_init": np.concatenate([cup_pos_init, cup_ori_init]).tolist(),
            "robot_qpos_init": robot_qpos_init.tolist(),
        }

    # --------------------------------------------------------------------------
    # Observation
    # --------------------------------------------------------------------------

    def get_obs(self) -> dict:
        """Return the unified observation dict for the current simulation state.

        Schema:
            joint_pos           np.ndarray (7,)   arm joint positions [rad]
            joint_vel           np.ndarray (7,)   arm joint velocities [rad/s]
            gripper_joint_pos   np.ndarray (2,)   finger joint positions [m]
            gripper_width       float             sum of finger positions [m]
            gripper_aperture    float             normalised [0=closed, 1=open]
            ee_pose_7d_world    np.ndarray (7,)   [x,y,z,qw,qx,qy,qz] world
            ee_pose_6d_world    np.ndarray (6,)   [x,y,z,rx,ry,rz] world
            ee_pose_7d_base     np.ndarray (7,)   EE pose in robot-base frame
            ee_pose_6d_base     np.ndarray (6,)   EE 6d in robot-base frame
            cup_pose            np.ndarray (7,)   cup world pose [x,y,z,qw,qx,qy,qz]
            sink_target_pose    np.ndarray (7,)   sampled sink target pose7d
            images              dict  name -> {"rgb": uint8 (H,W,3), "depth": uint16 (H,W)}
            language_instruction str
            timestep            int               episode step from self._timestep counter

        Returns:
            Observation dict with the keys listed above.
        """
        from cup_to_sink.gripper import finger_joints_to_width, width_to_aperture
        from cup_to_sink.transforms import pose7d_to_6d, world_to_base
        from cup_to_sink.cameras import capture

        # Capture current step index then advance the counter.
        timestep: int = self._timestep
        self._timestep += 1

        cfg = self.cfg
        env = self.env
        robot_cfg = cfg["robot"]

        # -- Joint state -------------------------------------------------------
        all_pos = env.franka.get_joint_positions()
        all_vel = env.franka.get_joint_velocities()

        joint_pos: np.ndarray = all_pos[self.arm_idx]
        joint_vel: np.ndarray = all_vel[self.arm_idx]
        finger_joints: np.ndarray = all_pos[self.finger_idx]

        gripper_width = finger_joints_to_width(finger_joints)
        open_w = float(robot_cfg["gripper_open_width"])
        closed_w = float(robot_cfg["gripper_closed_width"])
        gripper_aperture = width_to_aperture(gripper_width, open_w, closed_w)

        # -- EE pose (panda_hand in world frame) -------------------------------
        # Reuse the cached SingleXFormPrim created in __init__.
        ee_pos, ee_ori = self._ee_xform.get_world_pose()  # ori is [w, x, y, z]

        ee_pose_7d_world = np.concatenate(
            [np.asarray(ee_pos, dtype=float), np.asarray(ee_ori, dtype=float)]
        )
        ee_pose_6d_world = pose7d_to_6d(ee_pose_7d_world)
        ee_pose_7d_base = world_to_base(ee_pose_7d_world, env.robot_base_pose7d)
        ee_pose_6d_base = pose7d_to_6d(ee_pose_7d_base)

        # -- Object poses ------------------------------------------------------
        cup_pos_arr, cup_ori_arr = env.cup.get_world_pose()
        cup_pose = np.concatenate(
            [np.asarray(cup_pos_arr, dtype=float), np.asarray(cup_ori_arr, dtype=float)]
        )

        # sink_target_pose7d is set by reset(); fall back to config value if
        # get_obs is called before any reset (e.g. during testing).
        if self.sink_target_pose7d is not None:
            sink_target_pose = self.sink_target_pose7d
        else:
            sink_target_pose = np.array(
                cfg["scene"]["sink_target_pose_world"], dtype=float
            )

        # -- Camera images -----------------------------------------------------
        enabled: list[str] = cfg["dataset"]["enabled_cameras"]
        images = capture(env.cams, enabled)

        return {
            "joint_pos": joint_pos,
            "joint_vel": joint_vel,
            "gripper_joint_pos": finger_joints,
            "gripper_width": gripper_width,
            "gripper_aperture": gripper_aperture,
            "ee_pose_7d_world": ee_pose_7d_world,
            "ee_pose_6d_world": ee_pose_6d_world,
            "ee_pose_7d_base": ee_pose_7d_base,
            "ee_pose_6d_base": ee_pose_6d_base,
            "cup_pose": cup_pose,
            "sink_target_pose": sink_target_pose,
            "images": images,
            "language_instruction": self.instruction,
            "timestep": timestep,
        }

    # --------------------------------------------------------------------------
    # Success
    # --------------------------------------------------------------------------

    def check_success(self) -> tuple[bool, dict]:
        """Check whether the current episode state satisfies the success criteria.

        Success requires:
          - cup XY within ``cfg.success.xy_threshold`` of sink target XY
          - cup Z within [sink_z + z_min_offset, sink_z + z_max_offset]
          - gripper open enough to have released the cup (width > 0.5 * open_w)

        Returns:
            (success_bool, info_dict) -- see ``cup_to_sink.success.check`` for
            info_dict keys.
        """
        if self.sink_target_pose7d is None:
            raise RuntimeError("call reset() before check_success()")

        from cup_to_sink import success
        from cup_to_sink.gripper import finger_joints_to_width

        all_pos = self.env.franka.get_joint_positions()
        finger_joints = all_pos[self.finger_idx]
        gripper_width = finger_joints_to_width(finger_joints)

        cup_pos, _ = self.env.cup.get_world_pose()
        sink_pos = self.sink_target_pose7d[:3]

        return success.check(
            cup_pos,
            sink_pos,
            gripper_width,
            self.cfg["success"],
            float(self.cfg["robot"]["gripper_open_width"]),
        )

    # --------------------------------------------------------------------------
    # Helpers
    # --------------------------------------------------------------------------

    def current_qpos(self) -> np.ndarray:
        """Return current arm joint positions, shape (7,)."""
        all_pos = self.env.franka.get_joint_positions()
        return all_pos[self.arm_idx]

    def gripper_width(self) -> float:
        """Return current gripper width in metres (sum of two finger positions)."""
        from cup_to_sink.gripper import finger_joints_to_width

        all_pos = self.env.franka.get_joint_positions()
        finger_joints = all_pos[self.finger_idx]
        return finger_joints_to_width(finger_joints)

    def ee_pose_world(self) -> np.ndarray:
        """Return panda_hand world pose as [x, y, z, qw, qx, qy, qz], shape (7,)."""
        ee_pos, ee_ori = self._ee_xform.get_world_pose()
        return np.concatenate(
            [np.asarray(ee_pos, dtype=float), np.asarray(ee_ori, dtype=float)]
        )
