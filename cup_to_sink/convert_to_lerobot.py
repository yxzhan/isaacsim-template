"""Convert cup_to_sink HDF5 episodes (dataset_writer.py sec7 layout) to LeRobot format.

Mapping:
  observation.state  = concat(observations/joint_pos, observations/gripper_width)  (8,)
  observation.images.<cam> = observations/images/<cam>/rgb   (256x256x3, video)
  action             = actions/policy_action_command          (8,) joint targets + gripper width target
  task               = meta/language_instruction
  fps                = 1 / (control_dt * record_every) = 25

Depth images are skipped (LeRobot video features are uint8 RGB only).

Usage:
    python cup_to_sink/convert_to_lerobot.py \
        --input datasets/cup_to_sink \
        --output datasets/cup_to_sink_lerobot \
        --repo-id yxzhan/cup_to_sink
"""

from __future__ import annotations

import argparse
import shutil
from pathlib import Path

import h5py
import numpy as np

from lerobot.datasets.lerobot_dataset import LeRobotDataset

STATE_NAMES = [
    "panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
    "panda_joint5", "panda_joint6", "panda_joint7", "gripper_width",
]


def build_features(camera_shapes: dict[str, tuple[int, int, int]]) -> dict:
    features = {
        "observation.state": {
            "dtype": "float32",
            "shape": (len(STATE_NAMES),),
            "names": STATE_NAMES,
        },
        "action": {
            "dtype": "float32",
            "shape": (len(STATE_NAMES),),
            "names": STATE_NAMES,
        },
    }
    for cam, (h, w, c) in camera_shapes.items():
        features[f"observation.images.{cam}"] = {
            "dtype": "video",
            "shape": (h, w, c),
            "names": ["height", "width", "channels"],
        }
    return features


def convert(input_dir: Path, output_dir: Path, repo_id: str, fps: int) -> None:
    episodes = sorted(input_dir.glob("episode_*.hdf5"))
    if not episodes:
        raise SystemExit(f"No episode_*.hdf5 files found in {input_dir}")

    # Probe the first episode for cameras / per-camera shapes / instruction.
    with h5py.File(episodes[0], "r") as f:
        cameras = sorted(f["observations/images"].keys())
        camera_shapes = {
            cam: f[f"observations/images/{cam}/rgb"].shape[1:] for cam in cameras
        }
        control_dt = float(f["meta/control_dt"][()])

    if output_dir.exists():
        raise SystemExit(f"Output dir {output_dir} already exists; remove it first.")

    dataset = LeRobotDataset.create(
        repo_id=repo_id,
        fps=fps,
        root=output_dir,
        robot_type="franka_panda",
        features=build_features(camera_shapes),
        use_videos=True,
    )

    for ep_path in episodes:
        with h5py.File(ep_path, "r") as f:
            joint_pos = f["observations/joint_pos"][:].astype(np.float32)          # (T, 7)
            gripper_width = f["observations/gripper_width"][:].astype(np.float32)  # (T,)
            action = f["actions/policy_action_command"][:].astype(np.float32)      # (T, 8)
            rgbs = {cam: f[f"observations/images/{cam}/rgb"][:] for cam in cameras}
            task = f["meta/language_instruction"][()].decode("utf-8")

            state = np.concatenate([joint_pos, gripper_width[:, None]], axis=1)    # (T, 8)
            T = state.shape[0]
            for t in range(T):
                frame = {
                    "observation.state": state[t],
                    "action": action[t],
                    "task": task,
                }
                for cam in cameras:
                    frame[f"observation.images.{cam}"] = rgbs[cam][t]
                dataset.add_frame(frame)
        dataset.save_episode()
        print(f"converted {ep_path.name} ({T} frames)")

    print(f"\nDone: {len(episodes)} episodes -> {output_dir}")
    print(f"fps={fps} (control_dt={control_dt}, record_every=8)")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path,
                        default=Path("datasets/cup_to_sink"))
    parser.add_argument("--output", type=Path,
                        default=Path("datasets/cup_to_sink_lerobot"))
    parser.add_argument("--repo-id", default="yxzhan/cup_to_sink")
    parser.add_argument("--fps", type=int, default=25,
                        help="Recorded control rate: 200 Hz physics / record_every=8 = 25 Hz")
    args = parser.parse_args()
    convert(args.input.resolve(), args.output.resolve(), args.repo_id, args.fps)


if __name__ == "__main__":
    main()
