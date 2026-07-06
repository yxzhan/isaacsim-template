"""eval_policy.py -- Closed-loop policy evaluation for the cup-to-sink benchmark.

Runs the trained imitation policy (served by cup_to_sink/policy_server.py over
a local TCP socket) inside the Isaac Sim environment and measures the task
success rate over N randomized episodes.

Two-process architecture: lerobot is not installed in the Isaac Sim
interpreter, so the policy runs in the system python3 (policy_server.py) and
this script -- running under /isaac-sim/python.sh -- streams observations to
it and applies the returned joint-position actions.

Control loop (matches the training data):
    physics 200 Hz, one policy action every `dataset.record_every` (8) physics
    ticks -> 25 Hz control. Observation = 7 arm joint positions + gripper
    width (8,) plus the 4 RGB cameras (front/left/right/wrist, 256x256).
    Action = 7 arm joint position targets + gripper width, applied via the
    articulation's joint position controller.

Usage::

    # Terminal 1 (system python3, lerobot env):
    python3 -m cup_to_sink.policy_server \
        --checkpoint kitchen_imitation_learning/checkpoints/act_100000/pretrained_model

    # Terminal 2 (Isaac Sim python):
    /isaac-sim/python.sh -m cup_to_sink.eval_policy \
        --config cup_to_sink/configs/cup_to_sink.yaml \
        --num-episodes 50 --seed-start 10000

Outputs (default datasets/eval_policy/):
    eval_log.json           per-episode results + summary success rate
    episode_<seed>.mp4      2x2 camera-grid rollout video (front/left/right/wrist)
"""
from __future__ import annotations

import argparse
import json
import socket
import time
from pathlib import Path

import numpy as np

_REPO_ROOT = Path(__file__).resolve().parents[1]


# -- Policy client ---------------------------------------------------------------

class PolicyClient:
    """Thin blocking client for policy_server.py."""

    def __init__(self, host: str, port: int):
        from cup_to_sink.policy_ipc import recv_msg, send_msg
        self._recv = recv_msg
        self._send = send_msg
        self.sock = socket.create_connection((host, port), timeout=60.0)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.info = self._request({"cmd": "ping"})

    def _request(self, msg: dict) -> dict:
        self._send(self.sock, msg)
        reply = self._recv(self.sock)
        if reply is None:
            raise ConnectionError("policy server closed the connection")
        if not reply.get("ok"):
            raise RuntimeError(f"policy server error: {reply.get('error')}")
        return reply

    def reset(self) -> None:
        self._request({"cmd": "reset"})

    def act(self, state: np.ndarray, images: dict[str, np.ndarray]) -> np.ndarray:
        reply = self._request({
            "cmd": "act",
            "state": np.asarray(state, dtype=np.float32),
            "images": images,
        })
        return np.asarray(reply["action"], dtype=np.float64)

    def close(self) -> None:
        try:
            self.sock.close()
        except OSError:
            pass


# -- Video helper ----------------------------------------------------------------

def _grid_frame(images: dict, order: list[str]) -> np.ndarray:
    """Tile up to 4 camera RGB frames into a 2x2 grid (uint8, RGB)."""
    tiles = [images[n]["rgb"] for n in order if n in images]
    while len(tiles) < 4:
        tiles.append(np.zeros_like(tiles[0]))
    top = np.concatenate(tiles[:2], axis=1)
    bottom = np.concatenate(tiles[2:4], axis=1)
    return np.concatenate([top, bottom], axis=0)


def _write_video(path: Path, frames: list[np.ndarray], fps: float) -> bool:
    """Write RGB frames to browser-playable H.264 mp4; return False on failure.

    Prefers imageio-ffmpeg (libx264 + yuv420p + faststart -- plays in any
    browser, matching the LeRobot training videos). Falls back to OpenCV
    mp4v, which desktop players open but browsers do NOT.
    """
    if not frames:
        return False
    h, w = frames[0].shape[:2]
    try:
        import imageio_ffmpeg
        writer = imageio_ffmpeg.write_frames(
            str(path), (w, h), fps=fps,
            codec="libx264", pix_fmt_in="rgb24", pix_fmt_out="yuv420p",
            output_params=["-movflags", "+faststart"],
        )
        writer.send(None)
        for f in frames:
            writer.send(np.ascontiguousarray(f))
        writer.close()
        return True
    except Exception as exc:
        print(f"[eval_policy] WARNING: imageio-ffmpeg failed ({exc}); "
              f"falling back to OpenCV mp4v (not browser-playable). "
              f"Fix with: /isaac-sim/python.sh -m pip install imageio-ffmpeg")
    try:
        import cv2
        writer = cv2.VideoWriter(
            str(path), cv2.VideoWriter_fourcc(*"mp4v"), fps, (w, h)
        )
        for f in frames:
            writer.write(cv2.cvtColor(f, cv2.COLOR_RGB2BGR))
        writer.release()
        return True
    except Exception as exc:
        print(f"[eval_policy] WARNING: could not write video {path}: {exc}")
        return False


# -- Episode rollout ---------------------------------------------------------------

def run_episode(env, task, client: PolicyClient, cfg: dict, seed: int,
                max_steps: int, render: bool, video_frames: list | None) -> dict:
    """Roll out the policy for one episode; return a result-log dict."""
    from isaacsim.core.utils.types import ArticulationAction
    from cup_to_sink.gripper import width_to_finger_joints

    robot_cfg = cfg["robot"]
    open_w = float(robot_cfg["gripper_open_width"])
    closed_w = float(robot_cfg["gripper_closed_width"])
    record_every = max(1, int(cfg.get("dataset", {}).get("record_every", 8)))
    enabled_cams = list(cfg["dataset"]["enabled_cameras"])
    joint_indices = np.concatenate([task.arm_idx, task.finger_idx])

    rnd = task.reset(seed)
    client.reset()
    # One rendered frame so the cameras see the post-reset scene.
    env.world.step(render=True)

    t0 = time.time()
    steps_taken = 0
    early_success_step: int | None = None

    for t in range(max_steps):
        obs = task.get_obs()
        state = np.concatenate(
            [np.asarray(obs["joint_pos"], dtype=np.float32),
             [np.float32(obs["gripper_width"])]]
        )
        images = {n: obs["images"][n]["rgb"] for n in enabled_cams}
        if video_frames is not None:
            video_frames.append(_grid_frame(obs["images"], enabled_cams))

        action = client.act(state, images)
        arm_targets = np.asarray(action[:7], dtype=np.float64)
        width = float(np.clip(action[7], closed_w, open_w))
        fingers = width_to_finger_joints(width)

        env.franka.apply_action(ArticulationAction(
            joint_positions=np.concatenate([arm_targets, fingers]),
            joint_indices=joint_indices,
        ))
        # Hold the action for one control period; render only the last tick
        # (the frame the next get_obs() will capture).
        for k in range(record_every):
            env.world.step(render=render or (k == record_every - 1))
        steps_taken = t + 1

        success_now, _ = task.check_success()
        if success_now:
            early_success_step = steps_taken
            break

    # Let physics settle, then take the authoritative success reading.
    for _ in range(50):
        env.world.step(render=render)
    success, info = task.check_success()

    cup_final, _ = env.cup.get_world_pose()
    cup_final = np.asarray(cup_final, dtype=float)
    sink_xyz = np.asarray(task.sink_target_pose7d[:3], dtype=float)
    xy_dist = float(np.hypot(cup_final[0] - sink_xyz[0], cup_final[1] - sink_xyz[1]))

    return {
        "seed": int(seed),
        "success": bool(success),
        "cup_init_xy": [float(v) for v in rnd["cup_xy"]],
        "cup_init_yaw": float(rnd["cup_yaw"]),
        "success_info": {k: (v.tolist() if isinstance(v, np.ndarray) else v)
                         for k, v in (info or {}).items()},
        "steps": int(steps_taken),
        "early_success_step": early_success_step,
        "timed_out": early_success_step is None,
        "cup_final_xyz": cup_final.tolist(),
        "sink_target_xyz": sink_xyz.tolist(),
        "xy_dist_m": round(xy_dist, 4),
        "elapsed_s": round(time.time() - t0, 1),
    }


# -- main --------------------------------------------------------------------------

def main() -> int:
    from cup_to_sink.policy_ipc import DEFAULT_HOST, DEFAULT_PORT

    parser = argparse.ArgumentParser(
        description="Evaluate a policy (served by policy_server.py) on cup_to_sink."
    )
    parser.add_argument("--config", required=True,
                        help="YAML config (use the SAME config the training data "
                             "was collected with).")
    parser.add_argument("--num-episodes", type=int, default=20)
    parser.add_argument("--seed-start", type=int, default=10000,
                        help="First episode seed. Default 10000 so eval initial "
                             "conditions do not repeat the training seeds (0..N).")
    parser.add_argument("--max-steps", type=int, default=600,
                        help="Max control steps (25 Hz) per episode; 600 = 24 s. "
                             "Expert demos average ~340 steps.")
    parser.add_argument("--host", default=DEFAULT_HOST)
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--out", default=None,
                        help="Output dir (default datasets/eval_policy).")
    parser.add_argument("--no-video", action="store_true",
                        help="Skip writing per-episode mp4 rollout videos.")
    parser.add_argument("--viewer", action="store_true",
                        help="Run non-headless and render every physics tick "
                             "(slow; for watching, not batch eval).")
    args = parser.parse_args()

    from cup_to_sink.sim_app import _force_utf8_stdio
    _force_utf8_stdio()

    from cup_to_sink import config as cfg_mod
    cfg = cfg_mod.load(args.config)

    out_dir = Path(args.out) if args.out else (_REPO_ROOT / "datasets" / "eval_policy")
    out_dir.mkdir(parents=True, exist_ok=True)
    record_every = max(1, int(cfg.get("dataset", {}).get("record_every", 8)))
    control_hz = 200.0 / record_every

    # Connect BEFORE booting Isaac Sim so a missing server fails in seconds.
    print(f"[eval_policy] connecting to policy server {args.host}:{args.port} ...")
    try:
        client = PolicyClient(args.host, args.port)
    except OSError as exc:
        print(f"[eval_policy] ERROR: cannot reach policy server ({exc}).\n"
              f"  Start it first:  python3 -m cup_to_sink.policy_server")
        return 2
    print(f"[eval_policy] server ready: {client.info}")

    from cup_to_sink import sim_app as sim_app_mod
    app = sim_app_mod.start(headless=not args.viewer)

    from cup_to_sink import env_builder, task as task_mod

    print("[eval_policy] building env ...")
    env = env_builder.build_env(cfg, app)
    task = task_mod.Task(env, cfg)
    print("[eval_policy] env+task ready\n")

    results: list[dict] = []
    n_success = 0
    seeds = range(args.seed_start, args.seed_start + args.num_episodes)

    for i, seed in enumerate(seeds):
        print(f"\n{'='*64}\n[eval_policy] episode {i+1}/{args.num_episodes} "
              f"(seed={seed}, {n_success} successes so far)\n{'='*64}")
        video_frames: list | None = None if args.no_video else []
        try:
            entry = run_episode(env, task, client, cfg, seed,
                                args.max_steps, args.viewer, video_frames)
        except Exception as exc:
            import traceback
            traceback.print_exc()
            entry = {"seed": int(seed), "success": False, "error": str(exc)}
            results.append(entry)
            _flush(results, out_dir, client.info, args)
            continue

        if video_frames:
            video_path = out_dir / f"episode_{seed}.mp4"
            if _write_video(video_path, video_frames, fps=control_hz):
                entry["video"] = str(video_path)

        n_success += bool(entry["success"])
        print(f"[eval_policy] seed={seed}: success={entry['success']}, "
              f"steps={entry['steps']}, xy_dist={entry.get('xy_dist_m')}m, "
              f"elapsed={entry.get('elapsed_s')}s")

        results.append(entry)
        _flush(results, out_dir, client.info, args)

    n_done = len(results)
    rate = n_success / max(n_done, 1)
    print(f"\n{'='*64}\n[eval_policy] DONE: {n_success}/{n_done} succeeded "
          f"(success rate {rate:.0%})\n  log: {out_dir / 'eval_log.json'}\n{'='*64}")

    client.close()
    app.close()
    return 0


def _flush(results: list, out_dir: Path, server_info: dict, args) -> None:
    n_success = sum(1 for r in results if r.get("success"))
    payload = {
        "checkpoint": server_info.get("checkpoint"),
        "policy_type": server_info.get("policy_type"),
        "config": str(args.config),
        "seed_start": args.seed_start,
        "num_episodes": args.num_episodes,
        "max_steps": args.max_steps,
        "episodes_run": len(results),
        "successes": n_success,
        "success_rate": round(n_success / max(len(results), 1), 4),
        "episodes": results,
    }
    with open(out_dir / "eval_log.json", "w") as f:
        json.dump(payload, f, indent=2)


if __name__ == "__main__":
    import sys
    sys.exit(main())
