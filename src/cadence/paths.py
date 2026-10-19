"""Repository paths and SUMO executable lookup."""

from __future__ import annotations

import json
import os
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
R1_DIR = REPO_ROOT / "networks" / "corridor_r1"
SPEC_PATH = R1_DIR / "spec.json"


def load_spec(path: Path = SPEC_PATH) -> dict:
    with open(path, encoding="utf-8") as fh:
        return json.load(fh)


def sumo_home() -> Path:
    """SUMO installation directory, from SUMO_HOME or the eclipse-sumo wheel."""
    env = os.environ.get("SUMO_HOME")
    if env:
        return Path(env)
    import sumo  # eclipse-sumo wheel

    return Path(sumo.SUMO_HOME)


def sumo_env() -> dict:
    """Process environment with SUMO_HOME set, for SUMO subprocesses."""
    env = dict(os.environ)
    env["SUMO_HOME"] = str(sumo_home())
    return env


def sumo_bin(name: str) -> str:
    path = sumo_home() / "bin" / name
    if not path.exists():
        raise FileNotFoundError(f"SUMO executable not found: {path}")
    return str(path)
