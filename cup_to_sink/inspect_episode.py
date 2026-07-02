"""
inspect_episode.py -- Inspect a collected cup_to_sink demonstration HDF5.

Run with the Isaac Sim python (it has h5py + PIL + cv2):

    # Print a summary (meta + obs/action shapes):
    /isaac-sim/python.sh -m cup_to_sink.inspect_episode \
        --episode datasets/cup_to_sink/episode_0000.hdf5

    # Print the full HDF5 tree (every group/dataset with shape+dtype):
    /isaac-sim/python.sh -m cup_to_sink.inspect_episode \
        --episode datasets/cup_to_sink/episode_0000.hdf5 --tree

    # Save a 4-camera RGB+depth montage PNG at timestep t:
    /isaac-sim/python.sh -m cup_to_sink.inspect_episode \
        --episode datasets/cup_to_sink/episode_0000.hdf5 --frame 200 --out /tmp/frame.png

    # Export a camera's RGB stream to an MP4 video:
    /isaac-sim/python.sh -m cup_to_sink.inspect_episode \
        --episode datasets/cup_to_sink/episode_0000.hdf5 --video front --out /tmp/front.mp4
"""
from __future__ import annotations

import argparse
import sys

import numpy as np


def _decode(v):
    return v.decode("utf-8", "replace") if isinstance(v, (bytes, np.bytes_)) else v


def print_tree(f) -> None:
    """Print every group/dataset with shape and dtype."""
    def visit(name, obj):
        import h5py
        if isinstance(obj, h5py.Dataset):
            print(f"  {name:52s} {str(obj.shape):18s} {obj.dtype}")
        else:
            print(f"  {name}/")
    f.visititems(visit)


def print_summary(f) -> None:
    """Print a human-readable summary of the episode."""
    m = f["meta"]
    print("=== META ===")
    for k in ("task_name", "language_instruction", "seed", "success",
              "ee_pose_frame", "default_policy_ee_frame", "ee_pose_rotation_type"):
        if k in m:
            print(f"  {k:26s} = {_decode(m[k][()])}")
    for k in ("grasp_attach_used", "grasp_snap_used"):
        if k in m:
            print(f"  {k:26s} = {_decode(m[k][()])}")
    if "enabled_cameras" in m:
        cams = [_decode(x) for x in m["enabled_cameras"][:]]
        print(f"  {'enabled_cameras':26s} = {cams}")
    for k in ("initial_cup_pose", "sink_target_pose", "initial_robot_qpos"):
        if k in m:
            print(f"  {k:26s} = {np.round(m[k][:], 3)}")

    n = f["actions/joint_pos_target"].shape[0]
    print(f"\n=== SHAPES (T={n}) ===")
    print("  observations:")
    for k in f["observations"]:
        obj = f["observations"][k]
        if hasattr(obj, "shape"):
            print(f"    {k:24s} {obj.shape} {obj.dtype}")
    print("  images:")
    for cam in f["observations/images"]:
        rgb = f[f"observations/images/{cam}/rgb"]
        dep = f[f"observations/images/{cam}/depth"]
        print(f"    {cam:8s} rgb {rgb.shape} {rgb.dtype} | depth {dep.shape} {dep.dtype}")
    print("  actions:")
    for k in f["actions"]:
        obj = f["actions"][k]
        if hasattr(obj, "shape"):
            print(f"    {k:28s} {obj.shape} {obj.dtype}")


def save_frame_montage(f, t: int, out: str) -> None:
    """Save a montage PNG: each enabled camera's RGB (top) + depth (bottom) at step t."""
    from PIL import Image

    cams = list(f["observations/images"].keys())
    tiles = []
    for cam in cams:
        rgb = f[f"observations/images/{cam}/rgb"][t]                 # (H,W,3) uint8
        depth = f[f"observations/images/{cam}/depth"][t].astype(np.float32)  # (H,W) mm
        # Normalize depth (ignore the 65535 background) to a grayscale image.
        valid = depth[(depth > 0) & (depth < 65535)]
        dmax = float(valid.max()) if valid.size else 1.0
        dnorm = np.clip(depth / max(dmax, 1.0), 0, 1)
        dnorm[depth >= 65535] = 0
        depth_img = (dnorm * 255).astype(np.uint8)
        depth_rgb = np.stack([depth_img] * 3, axis=-1)
        col = np.concatenate([rgb, depth_rgb], axis=0)  # stack rgb over depth
        tiles.append(col)
    montage = np.concatenate(tiles, axis=1)  # cameras side by side
    Image.fromarray(montage).save(out)
    print(f"[inspect] saved montage ({cams}, RGB top / depth bottom) at t={t} -> {out}")


def export_video(f, cam: str, out: str, fps: int = 25) -> None:
    """Export a camera's RGB stream to a browser-playable H.264 MP4.

    Browsers (and JupyterLab's preview) need H.264/AVC in MP4; the old cv2 'mp4v'
    (MPEG-4 Part 2) only plays in desktop players like VLC. We write H.264 via
    imageio + the bundled ffmpeg (imageio-ffmpeg). Fallbacks: cv2 mp4v, then GIF.
    """
    import os
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)

    rgb = f[f"observations/images/{cam}/rgb"]        # (T,H,W,3) uint8
    T = rgb.shape[0]
    frames = [rgb[t] for t in range(T)]

    # 1) Preferred: H.264 MP4 (browser-compatible) via imageio-ffmpeg.
    try:
        import imageio.v2 as imageio
        imageio.mimwrite(
            out, frames, fps=fps, codec="libx264", quality=8,
            macro_block_size=1,           # allow non-multiple-of-16 sizes (256 is fine)
            output_params=["-pix_fmt", "yuv420p"],  # required for broad browser support
        )
        print(f"[inspect] saved {cam} H.264 MP4 ({T} frames @ {fps}fps) -> {out}")
        return
    except Exception as exc:
        print(f"[inspect] H.264 export unavailable ({exc}); trying cv2 mp4v")

    # 2) Fallback: cv2 mp4v (plays in VLC, not most browsers).
    try:
        import cv2
        H, W = rgb.shape[1], rgb.shape[2]
        writer = cv2.VideoWriter(out, cv2.VideoWriter_fourcc(*"mp4v"), fps, (W, H))
        if writer.isOpened():
            for fr in frames:
                writer.write(cv2.cvtColor(fr, cv2.COLOR_RGB2BGR))
            writer.release()
            print(f"[inspect] saved {cam} mp4v MP4 (VLC only) -> {out}")
            return
        writer.release()
    except Exception as exc:
        print(f"[inspect] cv2 mp4v failed ({exc}); falling back to GIF")

    # 3) Last resort: animated GIF (works everywhere, larger).
    import imageio.v2 as imageio
    gif = out.rsplit(".", 1)[0] + ".gif"
    imageio.mimsave(gif, frames, fps=fps)
    print(f"[inspect] saved {cam} GIF -> {gif}")


def main() -> int:
    p = argparse.ArgumentParser(description="Inspect a cup_to_sink episode HDF5.")
    p.add_argument("--episode", required=True)
    p.add_argument("--tree", action="store_true", help="Print full HDF5 tree.")
    p.add_argument("--frame", type=int, default=None,
                   help="Save a 4-cam RGB+depth montage PNG at this timestep.")
    p.add_argument("--video", default=None,
                   help="Camera name (front/left/right/wrist) to export as MP4.")
    p.add_argument("--out", default=None, help="Output path for --frame / --video.")
    args = p.parse_args()

    import h5py
    with h5py.File(args.episode, "r") as f:
        if args.tree:
            print(f"=== TREE: {args.episode} ===")
            print_tree(f)
        elif args.frame is not None:
            save_frame_montage(f, args.frame, args.out or "/tmp/episode_frame.png")
        elif args.video is not None:
            export_video(f, args.video, args.out or f"/tmp/episode_{args.video}.mp4")
        else:
            print(f"=== {args.episode} ===")
            print_summary(f)
    return 0


if __name__ == "__main__":
    sys.exit(main())
