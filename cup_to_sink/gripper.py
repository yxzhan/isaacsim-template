"""Gripper width conversions."""

import numpy as np


def width_to_finger_joints(width):
    """Convert gripper width to individual finger joint positions.

    Args:
        width: Gripper width in meters [0, 0.08]

    Returns:
        np.ndarray of shape (2,): [left_finger, right_finger] = [width/2, width/2]
    """
    return np.array([width / 2.0, width / 2.0])


def finger_joints_to_width(joints):
    """Convert individual finger joint positions to gripper width.

    Args:
        joints: np.ndarray of shape (2,) with [left_finger, right_finger]

    Returns:
        float: Gripper width = sum of finger positions
    """
    return float(np.sum(joints))


def width_to_aperture(width, open_w, closed_w):
    """Convert gripper width to normalized aperture.

    Args:
        width: Gripper width in meters
        open_w: Width when fully open
        closed_w: Width when fully closed

    Returns:
        float: Normalized aperture in [0, 1], clipped to valid range
    """
    aperture = (width - closed_w) / (open_w - closed_w)
    return float(np.clip(aperture, 0.0, 1.0))


def aperture_to_width(aperture, open_w, closed_w):
    """Convert normalized aperture to gripper width.

    Args:
        aperture: Normalized aperture in [0, 1]
        open_w: Width when fully open
        closed_w: Width when fully closed

    Returns:
        float: Gripper width in meters
    """
    return closed_w + aperture * (open_w - closed_w)
