"""
check_distribution.py -- Verify the collected demo set has real variation.

Opens every datasets/cup_to_sink/episode_*.hdf5 and checks:
  - cup initial XY and sink-target XY vary across episodes (std > 0),
  - Franka initial qpos is identical across episodes (default home),
  - every saved episode has meta/success == True.

Run with the Isaac Sim python (needs h5py):
    /isaac-sim/python.sh -m cup_to_sink.tests.check_distribution \
        [--dir datasets/cup_to_sink]
"""
from __future__ import annotations

import argparse
import glob
import sys
from pathlib import Path

import numpy as np


def main() -> int:
    parser = argparse.ArgumentParser(description="Check demo-set distribution.")
    parser.add_argument("--dir", default="datasets/cup_to_sink",
                        help="Directory of episode_*.hdf5 files.")
    args = parser.parse_args()

    import h5py

    paths = sorted(glob.glob(str(Path(args.dir) / "episode_*.hdf5")))
    if not paths:
        print(f"[check] no episodes found in {args.dir}")
        return 1

    cup_xy, sink_xy, qpos_list, successes = [], [], [], []
    for p in paths:
        with h5py.File(p, "r") as f:
            cup_xy.append(np.asarray(f["meta/randomization/cup_initial_pose"][:])[:2])
            sink_xy.append(np.asarray(f["meta/randomization/sink_target_pose"][:])[:2])
            qpos_list.append(np.asarray(f["meta/initial_robot_qpos"][:]))
            successes.append(bool(f["meta/success"][()]))

    cup_xy = np.array(cup_xy)
    sink_xy = np.array(sink_xy)
    qpos = np.array(qpos_list)

    cup_std = cup_xy.std(axis=0)
    sink_std = sink_xy.std(axis=0)
    qpos_range = qpos.max(axis=0) - qpos.min(axis=0)

    print(f"[check] episodes: {len(paths)}")
    print(f"[check] all success: {all(successes)} ({sum(successes)}/{len(successes)})")
    print(f"[check] cup_initial_xy  std = {cup_std}")
    print(f"[check] sink_target_xy  std = {sink_std}")
    print(f"[check] robot_initial_qpos max-min = {qpos_range}")

    ok = True
    if not (cup_std > 1e-6).any():
        print("[check] FAIL: cup initial XY does not vary across episodes")
        ok = False
    if not (sink_std > 1e-6).any():
        print("[check] FAIL: sink target XY does not vary across episodes")
        ok = False
    if (qpos_range > 1e-6).any():
        print("[check] FAIL: robot initial qpos is NOT constant across episodes")
        ok = False
    if not all(successes):
        print("[check] FAIL: not all saved episodes have success==True")
        ok = False

    print(f"[check] RESULT: {'PASS' if ok else 'FAIL'}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
