"""
cameras.py — Camera setup and RGB-D capture for the cup_to_sink benchmark.

All pxr/isaacsim/omni imports are deferred to function scope so this module
can be imported before SimulationApp is started.
"""
from __future__ import annotations

from math import radians, tan

import numpy as np


def setup_cameras(cfg_cameras: dict, enabled: list) -> dict:
    """Create and configure cameras from the cameras section of the config.

    For each enabled camera:
      - Creates a Camera prim at the configured path/pose/resolution.
      - Sets the horizontal FOV via focal length.
      - Attaches a depth-to-image-plane annotator so depth is available.

    World cameras (front / left / right) receive a world-space pose directly
    from the constructor.  The wrist camera lives under the ``parent_prim_path``
    (panda_hand) and its ``position``/``quat`` are a LOCAL offset — they are
    applied after construction via ``set_local_pose`` so the camera rides the
    end-effector.

    Args:
        cfg_cameras: cameras sub-dict from config (``cfg["cameras"]``).
        enabled: list of camera names to create, e.g. ``["front", "left",
                 "right", "wrist"]``.

    Returns:
        dict mapping name -> initialised Camera instance.
    """
    from isaacsim.sensors.camera import Camera

    cameras: dict = {}

    for name in enabled:
        if name not in cfg_cameras:
            continue
        ccfg = cfg_cameras[name]

        has_parent = "parent_prim_path" in ccfg

        if has_parent:
            # Wrist camera: create WITHOUT world position; we will set the
            # LOCAL offset afterwards so it follows the parent prim.
            cam = Camera(
                prim_path=ccfg["prim_path"],
                resolution=(ccfg["width"], ccfg["height"]),
            )
        else:
            # World-space cameras (front, left, right).
            cam = Camera(
                prim_path=ccfg["prim_path"],
                position=np.array(ccfg["position"], dtype=float),
                orientation=np.array(ccfg["quat"], dtype=float),
                resolution=(ccfg["width"], ccfg["height"]),
            )

        cam.initialize()

        if has_parent:
            # Apply LOCAL offset so the camera is rigidly attached to its
            # parent prim (e.g. panda_hand).
            cam.set_local_pose(
                translation=np.array(ccfg["position"], dtype=float),
                orientation=np.array(ccfg["quat"], dtype=float),
            )

        # Match horizontal FOV: f = (aperture/2) / tan(fov/2)
        aperture = cam.get_horizontal_aperture()
        fov_rad = radians(float(ccfg["fov"]))
        focal_length = (aperture / 2.0) / tan(fov_rad / 2.0)
        cam.set_focal_length(focal_length)

        # Attach depth annotator (distance-to-image-plane, in metres).
        cam.add_distance_to_image_plane_to_frame()

        cameras[name] = cam

    return cameras


def capture(cams: dict, enabled: list) -> dict:
    """Capture an RGB-D frame from each enabled camera.

    RGB  : ``cam.get_rgba()[:, :, :3]`` cast to uint8.
    Depth: ``cam.get_depth()`` (metres) → NaN→0 → clip [0, 65.535] →
           ×1000 → uint16 (millimetres, max ≈ 65 m).

    Cameras whose frame buffer is not yet populated (returns None) are
    silently skipped so the caller can retry after more render steps.

    Args:
        cams: dict of name -> Camera (as returned by setup_cameras).
        enabled: list of camera names to capture.

    Returns:
        dict of name -> ``{"rgb": np.ndarray uint8 (H,W,3),
                            "depth": np.ndarray uint16 (H,W) mm}``.
    """
    result: dict = {}

    for name in enabled:
        if name not in cams:
            continue
        cam = cams[name]

        rgba = cam.get_rgba()
        depth_m = cam.get_depth()

        if rgba is None or depth_m is None:
            continue

        # Drop alpha channel; ensure uint8.
        rgb = rgba[:, :, :3].astype(np.uint8)

        # NaN / inf → 0, clip to uint16 range (65535 mm ≈ 65.5 m), then
        # convert metres → millimetres and cast to uint16.
        depth_clipped = np.clip(
            np.nan_to_num(depth_m, nan=0.0, posinf=0.0, neginf=0.0),
            0.0,
            65.535,
        )
        depth_mm = (depth_clipped * 1000.0).astype(np.uint16)

        result[name] = {"rgb": rgb, "depth": depth_mm}

    return result
