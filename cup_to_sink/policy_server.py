"""policy_server.py -- LeRobot policy inference server for cup_to_sink eval.

Runs under the SYSTEM python3 (the one with lerobot installed), NOT under
/isaac-sim/python.sh. It loads a trained checkpoint (ACT or Diffusion) and
serves actions over a local TCP socket to eval_policy.py, which runs inside
the Isaac Sim interpreter. This two-process split exists because lerobot and
Isaac Sim cannot share one interpreter here.

Usage::

    python3 -m cup_to_sink.policy_server \\
        --checkpoint kitchen_imitation_learning/checkpoints/act_100000/pretrained_model

Then, in another terminal, run eval_policy.py (see its docstring). Stop the
server with Ctrl-C.

Protocol: see cup_to_sink/policy_ipc.py.
"""
from __future__ import annotations

import argparse
import json
import socket
import traceback
from pathlib import Path

import numpy as np
import torch

from cup_to_sink.policy_ipc import DEFAULT_HOST, DEFAULT_PORT, recv_msg, send_msg

_REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CHECKPOINT = str(
    _REPO_ROOT / "kitchen_imitation_learning" / "checkpoints" / "act_100000" / "pretrained_model"
)
DEFAULT_TASK = "put the cup into the sink"
DEFAULT_ROBOT_TYPE = "franka_panda"


def load_policy(checkpoint: Path, device: torch.device):
    """Load an ACT or Diffusion policy + pre/post processors from a checkpoint dir."""
    with open(checkpoint / "config.json") as f:
        policy_type = json.load(f).get("type", "act")

    if policy_type == "act":
        from lerobot.policies.act.modeling_act import ACTPolicy as PolicyCls
    elif policy_type == "diffusion":
        from lerobot.policies.diffusion.modeling_diffusion import DiffusionPolicy as PolicyCls
    else:
        raise ValueError(f"Unsupported policy type in config.json: {policy_type!r}")

    from lerobot.policies.factory import make_pre_post_processors

    policy = PolicyCls.from_pretrained(checkpoint)
    policy.to(device)
    policy.eval()
    preprocessor, postprocessor = make_pre_post_processors(
        policy.config, pretrained_path=str(checkpoint)
    )
    return policy, preprocessor, postprocessor, policy_type


def serve(args) -> None:
    device = torch.device(args.device)
    checkpoint = Path(args.checkpoint)
    print(f"[policy_server] loading {checkpoint} on {device} ...")
    policy, preprocessor, postprocessor, policy_type = load_policy(checkpoint, device)
    print(f"[policy_server] policy type: {policy_type}")

    from lerobot.utils.control_utils import predict_action

    def handle(msg: dict) -> dict:
        cmd = msg.get("cmd")
        if cmd == "ping":
            return {
                "ok": True,
                "policy_type": policy_type,
                "checkpoint": str(checkpoint),
                "device": str(device),
            }
        if cmd == "reset":
            policy.reset()
            return {"ok": True}
        if cmd == "act":
            obs = {"observation.state": np.asarray(msg["state"], dtype=np.float32)}
            for name, rgb in msg["images"].items():
                obs[f"observation.images.{name}"] = np.ascontiguousarray(rgb)
            action = predict_action(
                obs,
                policy,
                device,
                preprocessor,
                postprocessor,
                use_amp=False,
                task=args.task,
                robot_type=args.robot_type,
            )
            action_np = action.squeeze(0).to("cpu").numpy().astype(np.float32)
            return {"ok": True, "action": action_np}
        if cmd == "shutdown":
            return {"ok": True, "shutdown": True}
        return {"ok": False, "error": f"unknown cmd {cmd!r}"}

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((args.host, args.port))
    server.listen(1)
    print(f"[policy_server] listening on {args.host}:{args.port} -- Ctrl-C to stop")

    try:
        while True:
            conn, addr = server.accept()
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            print(f"[policy_server] client connected: {addr}")
            with conn:
                while True:
                    msg = recv_msg(conn)
                    if msg is None:
                        print("[policy_server] client disconnected")
                        break
                    try:
                        reply = handle(msg)
                    except Exception as exc:
                        traceback.print_exc()
                        reply = {"ok": False, "error": str(exc)}
                    send_msg(conn, reply)
                    if reply.get("shutdown"):
                        print("[policy_server] shutdown requested")
                        return
    except KeyboardInterrupt:
        print("\n[policy_server] interrupted; exiting")
    finally:
        server.close()


def main() -> None:
    parser = argparse.ArgumentParser(description="LeRobot policy inference server.")
    parser.add_argument("--checkpoint", default=DEFAULT_CHECKPOINT,
                        help="Path to a pretrained_model directory (ACT or Diffusion).")
    parser.add_argument("--host", default=DEFAULT_HOST)
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    parser.add_argument("--task", default=DEFAULT_TASK)
    parser.add_argument("--robot-type", default=DEFAULT_ROBOT_TYPE)
    serve(parser.parse_args())


if __name__ == "__main__":
    main()
