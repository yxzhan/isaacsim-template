"""Affordance actors: reference point accessors for cup and sink objects.

Pure numpy; no Isaac Sim imports. Usable in plain python3 for testing.
"""

import numpy as np
from cup_to_sink.transforms import make_T, compose, T_to_pose7d


class CupActor:
    """Wraps a cup prim and its config to provide world-space reference points.

    Args:
        cup_prim: object with a ``get_world_pose() -> (position(3), quat(4))`` method.
            In the real env this is ``env.cup``; in tests it can be a fake.
        cfg_cup: dict with keys ``contact_points`` and ``functional_points``, each a
            list of dicts with ``position_obj``, ``quat_obj``, and (for contact points)
            ``pregrasp_distance``.
    """

    def __init__(self, cup_prim, cfg_cup: dict):
        self._prim = cup_prim
        self._cfg = cfg_cup

    def get_world_pose(self) -> np.ndarray:
        """Return the cup world pose as a 7D array [px, py, pz, qw, qx, qy, qz]."""
        pos, quat = self._prim.get_world_pose()
        return np.concatenate([np.asarray(pos, dtype=np.float64),
                               np.asarray(quat, dtype=np.float64)])

    def _compose_point(self, point_cfg: dict) -> np.ndarray:
        """Compose the cup world transform with a local point config to get world pose7d."""
        pos, quat = self._prim.get_world_pose()
        T_world_cup = make_T(pos, quat)
        cp_pos = np.asarray(point_cfg["position_obj"], dtype=np.float64)
        cp_quat = np.asarray(point_cfg["quat_obj"], dtype=np.float64)
        T_cup_point = make_T(cp_pos, cp_quat)
        T_world_point = compose(T_world_cup, T_cup_point)
        return T_to_pose7d(T_world_point)

    def get_contact_point(self, idx: int = 0) -> np.ndarray:
        """Return the world-space pose7d of contact point ``idx``."""
        return self._compose_point(self._cfg["contact_points"][idx])

    def get_functional_point(self, idx: int = 0) -> np.ndarray:
        """Return the world-space pose7d of functional point ``idx``."""
        return self._compose_point(self._cfg["functional_points"][idx])

    def pregrasp_distance(self, idx: int = 0) -> float:
        """Return the pregrasp stand-off distance for contact point ``idx``."""
        return self._cfg["contact_points"][idx]["pregrasp_distance"]


class SinkTarget:
    """Wraps a sampled sink target pose.

    Args:
        sampled_pose7d: np.ndarray of shape (7,) -- [px, py, pz, qw, qx, qy, qz]
            representing the sampled placement location inside the sink.
    """

    def __init__(self, sampled_pose7d: np.ndarray):
        self._pose7d = np.asarray(sampled_pose7d, dtype=np.float64).copy()

    def get_functional_point(self) -> np.ndarray:
        """Return the stored sink target pose as a 7D array."""
        return self._pose7d.copy()
