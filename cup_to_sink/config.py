"""Config loader for cup_to_sink task.

Usage::

    from cup_to_sink.config import load
    cfg = load("cup_to_sink/configs/cup_to_sink.yaml")
    # cfg is a nested dict; raw YAML text is at cfg["__raw_yaml__"]
"""

from __future__ import annotations

import yaml
from pathlib import Path


def load(path: str | Path) -> dict:
    """Load a YAML config file and return a nested dict.

    The raw YAML text is stored under the special key ``__raw_yaml__``
    so callers can write it verbatim into HDF5 metadata.

    Args:
        path: Path to the YAML file.

    Returns:
        Nested dict with an extra ``__raw_yaml__`` key containing the
        raw file text.
    """
    path = Path(path)
    raw = path.read_text(encoding="utf-8")
    cfg: dict = yaml.safe_load(raw)
    if cfg is None:
        cfg = {}
    cfg["__raw_yaml__"] = raw
    return cfg
