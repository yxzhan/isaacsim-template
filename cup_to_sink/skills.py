"""Sparse end-effector goal skills for the cup-to-sink benchmark.

Each function returns an ordered list of action dicts. An action is one of:
  {"type": "move",    "ee_pose7d": np.ndarray(7), "phase": str}
  {"type": "gripper", "width": float,              "phase": str}

Pure numpy; no Isaac Sim imports. No side-effects on inputs.
"""

import numpy as np
from cup_to_sink.transforms import quat_rotate

# Gripper fully open width (metres — matches Franka finger_joint_open * 2)
_GRIPPER_OPEN = 0.08


def grasp_actor(
    actor,
    pre_grasp_dis: float,
    grasp_dis: float,
    gripper_width_target: float,
    contact_point_id: int = 0,
) -> list:
    """Build the action sequence to grasp ``actor`` at a contact point.

    Steps emitted:
      1. gripper open  (phase "pregrasp")
      2. move to pregrasp pose  (phase "pregrasp")
      3. move to grasp pose     (phase "grasp")
      4. gripper close          (phase "grasp_close")

    The pregrasp position is the grasp position offset by ``-pre_grasp_dis``
    along the gripper approach axis (local +Z of the grasp orientation).
    If ``grasp_dis`` is non-zero, the grasp position is further offset by
    ``-grasp_dis`` along the same axis (e.g. for surface-contact approaches).

    Args:
        actor: object with ``get_contact_point(idx) -> pose7d`` method.
        pre_grasp_dis: stand-off distance before the grasp (metres).
        grasp_dis: additional approach offset at the grasp itself (metres).
        gripper_width_target: gripper width to close to (metres).
        contact_point_id: index into the actor's contact points list.

    Returns:
        list of 4 action dicts.
    """
    grasp_pose7d = np.array(actor.get_contact_point(contact_point_id), dtype=np.float64)
    grasp_pos = grasp_pose7d[:3].copy()
    grasp_quat = grasp_pose7d[3:7].copy()

    # Approach axis = local +Z rotated into world frame
    approach_axis = quat_rotate(grasp_quat, np.array([0.0, 0.0, 1.0]))
    approach_axis = approach_axis / (np.linalg.norm(approach_axis) + 1e-12)

    # Pregrasp: stand off before the contact point
    pregrasp_pos = grasp_pos - pre_grasp_dis * approach_axis
    pregrasp_pose7d = np.concatenate([pregrasp_pos, grasp_quat])

    # Grasp: optionally offset along the approach axis
    actual_grasp_pos = grasp_pos - grasp_dis * approach_axis
    actual_grasp_pose7d = np.concatenate([actual_grasp_pos, grasp_quat])

    return [
        {"type": "gripper", "width": _GRIPPER_OPEN, "phase": "pregrasp"},
        {"type": "move",    "ee_pose7d": pregrasp_pose7d,     "phase": "pregrasp"},
        {"type": "move",    "ee_pose7d": actual_grasp_pose7d, "phase": "grasp"},
        {"type": "gripper", "width": gripper_width_target,    "phase": "grasp_close"},
    ]


def move_by_displacement(
    current_ee_pose7d: np.ndarray,
    dx: float,
    dy: float,
    dz: float,
    phase: str = "move",
) -> list:
    """Move the end-effector by a Cartesian displacement, keeping orientation.

    Args:
        current_ee_pose7d: current EE pose [px, py, pz, qw, qx, qy, qz].
        dx, dy, dz: displacement in world-frame X, Y, Z (metres).
        phase: label for the emitted action.

    Returns:
        list with a single move action.
    """
    pose = np.asarray(current_ee_pose7d, dtype=np.float64).copy()
    target = pose.copy()
    target[0] += dx
    target[1] += dy
    target[2] += dz
    return [{"type": "move", "ee_pose7d": target, "phase": phase}]


def place_actor(
    sink_target,
    target_pose7d: np.ndarray,
    pre_dis: float,
    retreat_h: float,
    is_open: bool = True,
    gripper_open_width: float = _GRIPPER_OPEN,
) -> list:
    """Build the action sequence to place the held object at ``target_pose7d``.

    Steps emitted:
      1. move to preplace pose  (target + pre_dis in Z)  phase "preplace"
      2. move to place pose     (target)                  phase "place"
      3. gripper open (if is_open)                        phase "release"
      4. move to retreat pose   (place + retreat_h in Z)  phase "retreat"

    Args:
        sink_target: unused here (kept for API symmetry with the planner); the
            placement location is given by ``target_pose7d``.
        target_pose7d: desired placement pose [px, py, pz, qw, qx, qy, qz].
        pre_dis: vertical lift above target for the preplace waypoint (metres).
        retreat_h: vertical lift above place pose for the retreat waypoint (metres).
        is_open: if True, emit a gripper-open action between place and retreat.
        gripper_open_width: gripper width for the release action.

    Returns:
        list of 4 action dicts.
    """
    target = np.asarray(target_pose7d, dtype=np.float64).copy()

    preplace_pose = target.copy()
    preplace_pose[2] += pre_dis

    place_pose = target.copy()

    retreat_pose = place_pose.copy()
    retreat_pose[2] += retreat_h

    actions = [
        {"type": "move",    "ee_pose7d": preplace_pose, "phase": "preplace"},
        {"type": "move",    "ee_pose7d": place_pose,    "phase": "place"},
    ]
    if is_open:
        actions.append({"type": "gripper", "width": gripper_open_width, "phase": "release"})
    actions.append({"type": "move", "ee_pose7d": retreat_pose, "phase": "retreat"})

    return actions
