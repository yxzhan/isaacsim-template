"""
planner.py — Motion planner ABC and RMPFlow implementation for cup_to_sink.

All isaacsim imports are deferred inside __init__ so this module can be
imported before SimulationApp is started.

Confirmed config keys (from policy_map.json in isaacsim.robot_motion.motion_generation):
    robot_name  = "Franka"
    policy_name = "RMPflow"   ← note lower-case 'f' in 'flow'

Import path for Isaac Sim 5.1:
    from isaacsim.robot_motion.motion_generation import RmpFlow, ArticulationMotionPolicy
    from isaacsim.robot_motion.motion_generation import interface_config_loader
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any

import numpy as np


class Planner(ABC):
    """Abstract base class for motion planners used in the cup_to_sink benchmark."""

    @abstractmethod
    def reset(self) -> None:
        """Reset planner state (re-initialise or zero internal fields)."""

    @abstractmethod
    def set_target(self, ee_pose7d_world: np.ndarray) -> None:
        """Set the EE target pose in WORLD frame.

        Args:
            ee_pose7d_world: np.ndarray of shape (7,) = [x, y, z, qw, qx, qy, qz]
        """

    @abstractmethod
    def step(self, current_joint_positions: np.ndarray) -> Any:
        """Compute the next ArticulationAction for this control step.

        Args:
            current_joint_positions: current robot joint positions (full DOF,
                accepted for interface compatibility but ArticulationMotionPolicy
                reads joint states from the Articulation internally).

        Returns:
            ArticulationAction with joint position/velocity targets for the
            active arm joints.
        """

    @abstractmethod
    def reached(
        self,
        current_ee_pose7d: np.ndarray,
        pos_tol: float = 0.01,
        rot_tol: float = 0.05,
    ) -> bool:
        """Check whether the EE has reached the stored target within tolerance.

        Args:
            current_ee_pose7d: current EE pose [x, y, z, qw, qx, qy, qz] in
                world frame.
            pos_tol: position tolerance in metres.
            rot_tol: rotation tolerance in radians (axis-angle magnitude of the
                relative quaternion).

        Returns:
            True if position error < pos_tol AND rotation error < rot_tol.
        """


class RMPFlowPlanner(Planner):
    """Lula RMPFlow planner for the Franka Panda in the cup_to_sink benchmark.

    Wraps ``isaacsim.robot_motion.motion_generation.RmpFlow`` and
    ``ArticulationMotionPolicy`` to produce per-step ``ArticulationAction``
    joint targets that drive the Franka end-effector toward a task-space goal.

    Confirmed working config keys from
    ``isaacsim.robot_motion.motion_generation/motion_policy_configs/policy_map.json``::

        robot_name  = "Franka"
        policy_name = "RMPflow"   ← lower-case 'f' is significant

    End-effector frame used by the Franka Lula config: ``"right_gripper"``
    (set in ``franka/rmpflow/config.json``).

    Args:
        env: ``Env`` dataclass returned by ``cup_to_sink.env_builder.build_env()``.
            Must already be fully initialised (world reset + physics settled).
    """

    def __init__(self, env: Any) -> None:
        # All isaacsim imports deferred — SimulationApp must be running
        from isaacsim.robot_motion.motion_generation import (
            RmpFlow,
            ArticulationMotionPolicy,
            interface_config_loader,
        )

        # ── Load Franka RMPflow config ────────────────────────────────────────
        # Confirmed key: robot="Franka", policy="RMPflow" (lower-case 'f')
        rmp_config = interface_config_loader.load_supported_motion_policy_config(
            "Franka", "RMPflow"
        )
        if rmp_config is None:
            pairs = interface_config_loader.get_supported_robot_policy_pairs()
            print(f"[RMPFlowPlanner] Available robot/policy pairs: {pairs}")
            raise RuntimeError(
                "load_supported_motion_policy_config('Franka', 'RMPflow') returned None. "
                f"Available pairs: {pairs}"
            )

        print("[RMPFlowPlanner] Config loaded: robot='Franka' policy='RMPflow'")
        print(f"[RMPFlowPlanner] Config keys: {list(rmp_config.keys())}")

        self._env = env
        self._rmp_config = rmp_config

        # Instantiate RMPflow
        self._rmpflow = RmpFlow(**rmp_config)

        # ── Set robot base pose ──────────────────────────────────────────────
        # The Franka is NOT at the world origin (base ≈ [1.325, -0.390, 0.850]).
        # Without this call RMPflow targets would be computed in robot-base frame
        # treated as origin, causing the EE to reach a completely wrong location.
        base_pos = env.robot_base_pose7d[:3]   # [x, y, z] in metres
        base_quat = env.robot_base_pose7d[3:]  # [qw, qx, qy, qz]
        self._rmpflow.set_robot_base_pose(base_pos, base_quat)

        # ── ArticulationMotionPolicy ──────────────────────────────────────────
        # physics_dt must match World(physics_dt=1/200)
        self._art_rmp = ArticulationMotionPolicy(
            env.franka,
            self._rmpflow,
            default_physics_dt=1.0 / 200.0,
        )

        self._target_pose7d: np.ndarray | None = None

    # ── Planner ABC implementation ────────────────────────────────────────────

    def reset(self) -> None:
        """Reset RMPflow internal state and re-apply the robot base pose.

        ``RmpFlow.reset()`` recreates the internal Lula policy and also calls
        ``LulaInterfaceHelper.reset()`` which zeroes ``_robot_pos``/``_robot_rot``.
        We therefore re-set the base pose afterward so RMPflow continues to
        work in world-frame coordinates.
        """
        self._rmpflow.reset()
        # Re-apply base pose (reset() zeroes it inside LulaInterfaceHelper)
        base_pos = self._env.robot_base_pose7d[:3]
        base_quat = self._env.robot_base_pose7d[3:]
        self._rmpflow.set_robot_base_pose(base_pos, base_quat)
        self._target_pose7d = None

    def set_target(self, ee_pose7d_world: np.ndarray) -> None:
        """Set EE target in world frame.

        Args:
            ee_pose7d_world: [x, y, z, qw, qx, qy, qz] — world frame, metres.
        """
        self._target_pose7d = np.asarray(ee_pose7d_world, dtype=np.float64)
        # RmpFlow.set_end_effector_target expects orientation as [qw, qx, qy, qz]
        self._rmpflow.set_end_effector_target(
            target_position=self._target_pose7d[:3],
            target_orientation=self._target_pose7d[3:],
        )

    def step(self, current_joint_positions: np.ndarray) -> Any:
        """Compute the next ArticulationAction via RMPflow.

        ``ArticulationMotionPolicy.get_next_articulation_action()`` reads the
        current joint positions and velocities directly from the Articulation,
        so ``current_joint_positions`` is accepted for interface compatibility
        only and not forwarded.

        Returns:
            ``ArticulationAction`` covering the 7 active arm joints with
            position and velocity targets for the next physics step.
        """
        return self._art_rmp.get_next_articulation_action()

    def reached(
        self,
        current_ee_pose7d: np.ndarray,
        pos_tol: float = 0.01,
        rot_tol: float = 0.05,
    ) -> bool:
        """Return True if the EE has converged to the stored target.

        Position error: L2 distance between target and current XYZ.
        Rotation error: axis-angle magnitude of the relative quaternion
            ``target_q * conj(current_q)``.

        Args:
            current_ee_pose7d: [x, y, z, qw, qx, qy, qz] world frame.
            pos_tol: position tolerance in metres.
            rot_tol: rotation tolerance in radians.

        Returns:
            True when both errors are within tolerance.
        """
        if self._target_pose7d is None:
            return False

        from cup_to_sink.transforms import quat_mul, quat_conj, quat_to_axis_angle

        current = np.asarray(current_ee_pose7d, dtype=np.float64)
        pos_err = float(np.linalg.norm(current[:3] - self._target_pose7d[:3]))

        # Relative rotation: target * conj(current)
        rel_q = quat_mul(
            self._target_pose7d[3:],
            quat_conj(current[3:]),
        )
        rot_err = float(np.linalg.norm(quat_to_axis_angle(rel_q)))

        return pos_err < pos_tol and rot_err < rot_tol

    # ── Utility ───────────────────────────────────────────────────────────────

    def get_ee_pose(self) -> np.ndarray:
        """Return current EE world pose as [x, y, z, qw, qx, qy, qz].

        Uses Lula forward kinematics over the active arm joints read from the
        Articulation — consistent with the frame that RMPflow plans in
        (``"right_gripper"`` for the Franka config).

        Returns:
            np.ndarray of shape (7,).

        Raises:
            RuntimeError: if the Articulation has not been initialised yet.
        """
        import torch

        from cup_to_sink.transforms import matrix_to_quat

        active_subset = self._art_rmp.get_active_joints_subset()
        q_active = active_subset.get_joint_positions()

        if q_active is None:
            raise RuntimeError(
                "Articulation not initialised — cannot read joint positions."
            )

        if isinstance(q_active, torch.Tensor):
            q_active = q_active.cpu().numpy()
        q_active = np.asarray(q_active, dtype=np.float64)

        # Returns (translation_world_m, rotation_matrix_world_3x3)
        pos_m, rot_mat = self._rmpflow.get_end_effector_pose(q_active)
        quat = matrix_to_quat(rot_mat)  # [w, x, y, z]

        return np.concatenate([pos_m, quat])
