"""Success condition checker for cup-to-sink task."""

import numpy as np


def check(cup_pos, sink_pos, gripper_width, cfg_success, open_w):
    """Check if the task is successful.

    Args:
        cup_pos: List/array [cup_x, cup_y, cup_z] (position of the cup)
        sink_pos: List/array [sink_x, sink_y, sink_z] (position of the sink)
        gripper_width: float, gripper width in meters
        cfg_success: dict with keys:
            - xy_threshold: max horizontal distance
            - z_min_offset: minimum z offset from sink
            - z_max_offset: maximum z offset from sink
        open_w: float, fully open gripper width in meters

    Returns:
        tuple: (success_bool, info_dict) where info_dict contains:
            - xy_ok: bool, horizontal distance check
            - z_ok: bool, vertical position check
            - gripper_ok: bool, gripper width check
            - xy_dist: float, actual horizontal distance
            - cup_z: float, cup z position
    """
    cup_pos = np.asarray(cup_pos)
    sink_pos = np.asarray(sink_pos)

    # Calculate horizontal distance
    dx = cup_pos[0] - sink_pos[0]
    dy = cup_pos[1] - sink_pos[1]
    xy_dist = float(np.hypot(dx, dy))

    # Check horizontal alignment
    xy_ok = xy_dist < cfg_success["xy_threshold"]

    # Check vertical alignment
    cup_z = float(cup_pos[2])
    sink_z = float(sink_pos[2])
    z_min = sink_z + cfg_success["z_min_offset"]
    z_max = sink_z + cfg_success["z_max_offset"]
    z_ok = z_min <= cup_z <= z_max

    # Check gripper is open enough to have released the cup
    gripper_ok = gripper_width > 0.5 * open_w

    # Overall success
    success = xy_ok and z_ok and gripper_ok

    info = {
        "xy_ok": xy_ok,
        "z_ok": z_ok,
        "gripper_ok": gripper_ok,
        "xy_dist": xy_dist,
        "cup_z": cup_z,
    }

    return success, info
