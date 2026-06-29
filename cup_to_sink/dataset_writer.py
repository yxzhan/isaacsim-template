"""HDF5 dataset writer for cup_to_sink task demonstrations.

Implements the full plan §7 HDF5 layout:
  /observations/{joint_pos, joint_vel, gripper_joint_pos, gripper_width,
                 gripper_aperture, ee_pose_7d, ee_pose_6d, ee_pose_7d_world,
                 ee_pose_6d_world, cup_pose, sink_target_pose}
  /observations/images/<cam>/{rgb, depth}
  /actions/{joint_pos_target, ee_pose_target_7d, ee_pose_target_6d,
            delta_ee_pose_target_6d, gripper_joint_pos_target,
            gripper_width_target, gripper_aperture_target,
            policy_action_command, policy_action_next_state,
            policy_action_ee_command, policy_action_ee_next_state}
  /debug/{sparse_ee_target_pose, grasp_pose_7d, pregrasp_pose_7d, phase}
  /meta/{task_name, language_instruction, seed, success, config_yaml, ...}
  /meta/randomization/{seed, cup_initial_pose, sink_target_pose, ...}

Conventions:
- Quaternions: [w, x, y, z]
- ee_pose_7d = [x, y, z, qw, qx, qy, qz]
- ee_pose_6d = [x, y, z, rx, ry, rz]  (rotation vector / axis-angle)
- /observations/ee_pose_* stores the robot_base frame; world frame stored under
  /observations/ee_pose_*_world
- depth: uint16, millimetres
- All array datasets use gzip compression
- Scalars and strings stored uncompressed; strings via np.bytes_
"""

from __future__ import annotations

import json
import numpy as np
import h5py


class Recorder:
    """Buffers per-step dicts and writes a complete HDF5 episode file.

    Usage::

        rec = Recorder()
        rec.append(step_dict)   # call once per control step
        rec.write("ep.hdf5", meta=episode_meta)
    """

    def __init__(self) -> None:
        self._steps: list[dict] = []

    # ------------------------------------------------------------------
    # Public interface
    # ------------------------------------------------------------------

    @property
    def num_steps(self) -> int:
        """Number of steps buffered so far."""
        return len(self._steps)

    def append(self, step: dict) -> None:
        """Buffer one control-step dict.

        Expected top-level keys in *step*:
            joint_pos, joint_vel, gripper_joint_pos, gripper_width,
            gripper_aperture, ee_pose_7d_world, ee_pose_6d_world,
            ee_pose_7d_base, ee_pose_6d_base, cup_pose, sink_target_pose,
            images  (dict cam_name -> {"rgb": uint8 HxWx3, "depth": uint16 HxW}),
            action  (sub-dict),
            debug   (sub-dict),
            phase   (int)
        """
        self._steps.append(step)

    def write(self, path: str, meta: dict) -> None:
        """Stack buffered steps and write the full §7 HDF5 layout.

        Args:
            path: Output .hdf5 file path.
            meta: Episode metadata.  Recognised keys (all optional except
                where noted):
                  task_name*, language_instruction*, seed*, success*,
                  enabled_cameras, joint_names, config_yaml,
                  randomization (sub-dict),
                  + all optional §7 /meta/* fields.
        """
        if not self._steps:
            raise ValueError("Recorder has no steps to write.")

        steps = self._steps
        T = len(steps)
        gz = {"compression": "gzip"}

        # ==============================================================
        # Stack observations
        # ==============================================================
        joint_pos = np.stack([s["joint_pos"] for s in steps])              # (T, 7)
        joint_vel = np.stack([s["joint_vel"] for s in steps])              # (T, 7)
        gripper_joint_pos = np.stack([s["gripper_joint_pos"] for s in steps])  # (T, 2)
        gripper_width = np.array([float(s["gripper_width"]) for s in steps])   # (T,)
        gripper_aperture = np.array([float(s["gripper_aperture"]) for s in steps])  # (T,)

        ee_pose_7d_world = np.stack([s["ee_pose_7d_world"] for s in steps])   # (T, 7)
        ee_pose_6d_world = np.stack([s["ee_pose_6d_world"] for s in steps])   # (T, 6)
        ee_pose_7d_base = np.stack([s["ee_pose_7d_base"] for s in steps])     # (T, 7)
        ee_pose_6d_base = np.stack([s["ee_pose_6d_base"] for s in steps])     # (T, 6)

        cup_pose = np.stack([s["cup_pose"] for s in steps])                # (T, 7)
        sink_target_pose = np.stack([s["sink_target_pose"] for s in steps])   # (T, 7)
        phase = np.array([int(s["phase"]) for s in steps])                 # (T,)

        # ==============================================================
        # Stack actions
        # ==============================================================
        a = [s["action"] for s in steps]
        act_joint_pos_target = np.stack([x["joint_pos_target"] for x in a])      # (T, 7)
        act_ee_pose_7d = np.stack([x["ee_pose_target_7d"] for x in a])           # (T, 7)
        act_ee_pose_6d = np.stack([x["ee_pose_target_6d"] for x in a])           # (T, 6)
        act_delta_ee_6d = np.stack([x["delta_ee_pose_target_6d"] for x in a])    # (T, 6)
        act_gripper_joint_pos = np.stack([x["gripper_joint_pos_target"] for x in a])  # (T, 2)
        act_gripper_width = np.array([float(x["gripper_width_target"]) for x in a])   # (T,)
        act_gripper_aperture = np.array([float(x["gripper_aperture_target"]) for x in a])  # (T,)
        act_policy_command = np.stack([x["policy_action_command"] for x in a])    # (T, 8)
        act_policy_ee_command = np.stack([x["policy_action_ee_command"] for x in a])  # (T, 7)

        # ==============================================================
        # Derived: policy_action_next_state  (T, 8)
        #   = concat( joint_pos[t+1], gripper_width[t+1] ); last step repeats
        # ==============================================================
        jp_next = np.concatenate([joint_pos[1:], joint_pos[-1:]], axis=0)       # (T, 7)
        gw_next = np.concatenate([gripper_width[1:], gripper_width[-1:]])        # (T,)
        policy_action_next_state = np.concatenate(
            [jp_next, gw_next[:, None]], axis=1
        )  # (T, 8)

        # Derived: policy_action_ee_next_state  (T, 7)
        #   = concat( ee_pose_6d_base[t+1], gripper_width[t+1] )
        ee6d_next = np.concatenate([ee_pose_6d_base[1:], ee_pose_6d_base[-1:]], axis=0)  # (T, 6)
        policy_action_ee_next_state = np.concatenate(
            [ee6d_next, gw_next[:, None]], axis=1
        )  # (T, 7)

        # ==============================================================
        # Stack debug
        # ==============================================================
        d = [s["debug"] for s in steps]
        dbg_sparse_ee = np.stack([x["sparse_ee_target_pose"] for x in d])    # (T, 7)
        dbg_grasp = np.stack([x["grasp_pose_7d"] for x in d])                # (T, 7)
        dbg_pregrasp = np.stack([x["pregrasp_pose_7d"] for x in d])          # (T, 7)

        # ==============================================================
        # Determine cameras: only write groups present in ALL steps
        # ==============================================================
        cam_sets = [set(s.get("images", {}).keys()) for s in steps]
        cameras_to_write: set[str] = cam_sets[0].copy() if cam_sets else set()
        for cs in cam_sets[1:]:
            cameras_to_write.intersection_update(cs)

        # ==============================================================
        # Write HDF5
        # ==============================================================
        with h5py.File(path, "w") as f:
            f.attrs["schema_version"] = "1.0"

            # ---- /observations ----
            obs = f.create_group("observations")
            obs.create_dataset("joint_pos",       data=joint_pos,       **gz)
            obs.create_dataset("joint_vel",        data=joint_vel,        **gz)
            obs.create_dataset("gripper_joint_pos", data=gripper_joint_pos, **gz)
            obs.create_dataset("gripper_width",    data=gripper_width,    **gz)
            obs.create_dataset("gripper_aperture", data=gripper_aperture, **gz)
            # Base-frame EE (default policy frame per spec)
            obs.create_dataset("ee_pose_7d",        data=ee_pose_7d_base, **gz)
            obs.create_dataset("ee_pose_6d",        data=ee_pose_6d_base, **gz)
            # World-frame EE also stored
            obs.create_dataset("ee_pose_7d_world",  data=ee_pose_7d_world, **gz)
            obs.create_dataset("ee_pose_6d_world",  data=ee_pose_6d_world, **gz)
            obs.create_dataset("cup_pose",         data=cup_pose,         **gz)
            obs.create_dataset("sink_target_pose", data=sink_target_pose, **gz)

            # /observations/images/<cam>/{rgb, depth}
            if cameras_to_write:
                imgs_grp = obs.create_group("images")
                for cam in cameras_to_write:
                    cg = imgs_grp.create_group(cam)
                    rgb = np.stack([s["images"][cam]["rgb"] for s in steps])          # (T,H,W,3) uint8
                    depth = np.stack(
                        [s["images"][cam]["depth"] for s in steps]
                    ).astype(np.uint16)                                                # (T,H,W) uint16
                    cg.create_dataset("rgb",   data=rgb,   **gz)
                    cg.create_dataset("depth", data=depth, **gz)

            # ---- /actions ----
            act_grp = f.create_group("actions")
            act_grp.create_dataset("joint_pos_target",         data=act_joint_pos_target,  **gz)
            act_grp.create_dataset("ee_pose_target_7d",        data=act_ee_pose_7d,         **gz)
            act_grp.create_dataset("ee_pose_target_6d",        data=act_ee_pose_6d,         **gz)
            act_grp.create_dataset("delta_ee_pose_target_6d",  data=act_delta_ee_6d,        **gz)
            act_grp.create_dataset("gripper_joint_pos_target", data=act_gripper_joint_pos,  **gz)
            act_grp.create_dataset("gripper_width_target",     data=act_gripper_width,      **gz)
            act_grp.create_dataset("gripper_aperture_target",  data=act_gripper_aperture,   **gz)
            act_grp.create_dataset("policy_action_command",        data=act_policy_command,     **gz)
            act_grp.create_dataset("policy_action_next_state",     data=policy_action_next_state,    **gz)
            act_grp.create_dataset("policy_action_ee_command",     data=act_policy_ee_command,  **gz)
            act_grp.create_dataset("policy_action_ee_next_state",  data=policy_action_ee_next_state, **gz)

            # ---- /debug ----
            dbg_grp = f.create_group("debug")
            dbg_grp.create_dataset("sparse_ee_target_pose", data=dbg_sparse_ee,  **gz)
            dbg_grp.create_dataset("grasp_pose_7d",         data=dbg_grasp,      **gz)
            dbg_grp.create_dataset("pregrasp_pose_7d",      data=dbg_pregrasp,   **gz)
            dbg_grp.create_dataset("phase",                 data=phase,          **gz)

            # ---- /meta ----
            m = f.create_group("meta")
            _write_meta(m, meta, gz)


# ==============================================================
# Helpers
# ==============================================================

def _write_str(grp: h5py.Group, name: str, value) -> None:
    """Write a scalar string as np.bytes_; skip if value is None."""
    if value is None:
        return
    grp.create_dataset(name, data=np.bytes_(str(value)))


def _write_meta(m: h5py.Group, meta: dict, gz: dict) -> None:
    """Populate /meta and /meta/randomization from the meta dict."""

    # Scalar strings
    for key in ("task_name", "language_instruction", "ee_pose_frame",
                "default_policy_ee_frame", "ee_pose_rotation_type",
                "policy_action_schema", "policy_action_sources",
                "scene_usd_path", "cup_usd_path", "cup_prim_path",
                "config_yaml"):
        _write_str(m, key, meta.get(key))

    # Scalar integers / booleans
    seed = meta.get("seed")
    if seed is not None:
        m.create_dataset("seed", data=int(seed))

    success = meta.get("success")
    if success is not None:
        m.create_dataset("success", data=bool(success))

    # Scalar floats
    for key in ("physics_dt", "control_dt", "gripper_open_width",
                "gripper_closed_width", "initial_gripper_width"):
        v = meta.get(key)
        if v is not None:
            m.create_dataset(key, data=float(v))

    # Array fields
    for key in ("robot_base_pose", "initial_robot_qpos",
                "initial_gripper_joint_pos", "initial_cup_pose"):
        v = meta.get(key)
        if v is not None:
            m.create_dataset(key, data=np.asarray(v, dtype=np.float64), **gz)

    sink_target_meta = meta.get("sink_target_pose")
    if sink_target_meta is not None:
        m.create_dataset("sink_target_pose",
                         data=np.asarray(sink_target_meta, dtype=np.float64), **gz)

    # String-list fields
    for key in ("joint_names", "gripper_joint_names", "enabled_cameras"):
        lst = meta.get(key)
        if lst is not None:
            m.create_dataset(key, data=np.array([s.encode() for s in lst]), **gz)

    # ---- /meta/randomization ----
    rnd = meta.get("randomization", {})
    if not rnd:
        return
    rnd_grp = m.create_group("randomization")
    for k, v in rnd.items():
        if v is None:
            continue
        if isinstance(v, str):
            rnd_grp.create_dataset(k, data=np.bytes_(v))
        elif isinstance(v, bool):
            rnd_grp.create_dataset(k, data=bool(v))
        elif isinstance(v, int):
            rnd_grp.create_dataset(k, data=int(v))
        elif isinstance(v, float):
            rnd_grp.create_dataset(k, data=float(v))
        elif isinstance(v, np.ndarray):
            rnd_grp.create_dataset(k, data=v, **gz)
        elif isinstance(v, (list, tuple)):
            rnd_grp.create_dataset(k, data=np.asarray(v), **gz)
        else:
            if isinstance(v, dict):
                rnd_grp.create_dataset(k, data=np.bytes_(json.dumps(v)))
            else:
                try:
                    rnd_grp.create_dataset(k, data=np.asarray(v), **gz)
                except Exception:
                    rnd_grp.create_dataset(k, data=np.bytes_(str(v)))
