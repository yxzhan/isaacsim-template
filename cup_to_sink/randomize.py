"""Seeded episode randomization for cup-to-sink task."""

import numpy as np


def sample_episode(seed, rnd_cfg, sink_base_pose7d, cup_base_xy):
    """Sample a randomized episode deterministically from a seed.

    Args:
        seed: int, random seed for reproducibility
        rnd_cfg: dict with keys:
            - cup_xlim: [min, max] offsets for cup x
            - cup_ylim: [min, max] offsets for cup y
            - cup_yaw_range: [min, max] for cup yaw angle
            - cup_z_offset_range: [min, max] for cup z offset
            - sink_target_sample_xy: bool, whether to sample sink xy
            - sink_target_xlim: [min, max] offsets for sink target x
            - sink_target_ylim: [min, max] offsets for sink target y
        sink_base_pose7d: np.ndarray of shape (7,) = [x, y, z, qw, qx, qy, qz]
        cup_base_xy: np.ndarray of shape (2,) = [x, y]

    Returns:
        dict with keys:
            - cup_xy: sampled cup position [x, y]
            - cup_yaw: sampled cup yaw
            - cup_z_offset: sampled cup z offset
            - sink_target_pose7d: sampled sink target pose7d [x, y, z, qw, qx, qy, qz]
    """
    # Create deterministic RNG from seed
    rng = np.random.default_rng(seed)

    # Sample cup xy offsets and add to base
    cup_x_offset = rng.uniform(rnd_cfg["cup_xlim"][0], rnd_cfg["cup_xlim"][1])
    cup_y_offset = rng.uniform(rnd_cfg["cup_ylim"][0], rnd_cfg["cup_ylim"][1])
    cup_xy = cup_base_xy + np.array([cup_x_offset, cup_y_offset])

    # Sample cup yaw
    cup_yaw = rng.uniform(rnd_cfg["cup_yaw_range"][0], rnd_cfg["cup_yaw_range"][1])

    # Sample cup z offset
    cup_z_offset = rng.uniform(
        rnd_cfg["cup_z_offset_range"][0], rnd_cfg["cup_z_offset_range"][1]
    )

    # Build sink target pose7d (copy from sink_base_pose7d)
    sink_base_pose7d = np.asarray(sink_base_pose7d)
    sink_target_pose7d = sink_base_pose7d.copy()

    # Sample sink xy if requested
    if rnd_cfg["sink_target_sample_xy"]:
        sink_x_offset = rng.uniform(
            rnd_cfg["sink_target_xlim"][0], rnd_cfg["sink_target_xlim"][1]
        )
        sink_y_offset = rng.uniform(
            rnd_cfg["sink_target_ylim"][0], rnd_cfg["sink_target_ylim"][1]
        )
        sink_target_pose7d[0] = sink_base_pose7d[0] + sink_x_offset
        sink_target_pose7d[1] = sink_base_pose7d[1] + sink_y_offset

    return {
        "cup_xy": cup_xy,
        "cup_yaw": cup_yaw,
        "cup_z_offset": cup_z_offset,
        "sink_target_pose7d": sink_target_pose7d,
    }
