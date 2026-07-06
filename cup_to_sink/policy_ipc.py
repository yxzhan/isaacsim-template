"""policy_ipc.py -- Tiny length-prefixed pickle protocol shared by
policy_server.py (system python3 + lerobot) and eval_policy.py
(/isaac-sim/python.sh).

Kept minimal (stdlib + numpy) so both interpreters can import it.

Wire format: 4-byte big-endian unsigned length, then a pickled dict.
np.ndarray values are converted to (dtype-str, shape, bytes) triples before
pickling: the two interpreters run DIFFERENT numpy major versions (Isaac Sim
numpy 1.x vs lerobot numpy 2.x), and a numpy-2 pickled ndarray fails to load
under numpy 1 (`No module named 'numpy._core'`).

Messages (client -> server):
    {"cmd": "ping"}
    {"cmd": "reset"}                                   # clear ACT action queue
    {"cmd": "act", "state": np(8,), "images": {name: np(H,W,3) uint8}}
    {"cmd": "shutdown"}

Responses (server -> client):
    {"ok": True, ...}          # ping/reset/shutdown
    {"ok": True, "action": np(8,) float32}             # act
    {"ok": False, "error": str}
"""
from __future__ import annotations

import pickle
import socket
import struct

import numpy as np

DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 8765

_LEN = struct.Struct(">I")
_ND_KEY = "__ndarray__"


def _encode(value):
    """Recursively replace ndarrays with version-agnostic triples."""
    if isinstance(value, np.ndarray):
        a = np.ascontiguousarray(value)
        return {_ND_KEY: (a.dtype.str, a.shape, a.tobytes())}
    if isinstance(value, dict):
        return {k: _encode(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return type(value)(_encode(v) for v in value)
    return value


def _decode(value):
    if isinstance(value, dict):
        if set(value.keys()) == {_ND_KEY}:
            dtype, shape, data = value[_ND_KEY]
            return np.frombuffer(data, dtype=np.dtype(dtype)).reshape(shape).copy()
        return {k: _decode(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return type(value)(_decode(v) for v in value)
    return value


def send_msg(sock: socket.socket, obj: dict) -> None:
    payload = pickle.dumps(_encode(obj), protocol=pickle.HIGHEST_PROTOCOL)
    sock.sendall(_LEN.pack(len(payload)) + payload)


def recv_msg(sock: socket.socket) -> dict | None:
    """Receive one message; None on clean EOF."""
    header = _recv_exact(sock, _LEN.size)
    if header is None:
        return None
    (length,) = _LEN.unpack(header)
    payload = _recv_exact(sock, length)
    if payload is None:
        return None
    return _decode(pickle.loads(payload))


def _recv_exact(sock: socket.socket, n: int) -> bytes | None:
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            return None
        buf += chunk
    return buf
