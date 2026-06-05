"""Archived raw-pybullet URDF loader for UR10e.

This helper is retained for reference only. The active runtime path for
simulation and collision checking uses the curated compas_fab scene
JSON under `src/subsystems/robot/assets/` via `shared_compas_scene.py`.
The raw URDF + mesh bundle this helper expects now lives in
`archive/robot_assets/`.
"""

from __future__ import annotations

import os
import re
from pathlib import Path
from typing import Iterable

# Joint order used everywhere on the bus (BUS.md §6.4: 6 elements in
# URDF joint order for UR10e).
UR10E_JOINT_NAMES = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)

_ASSETS_DIR = Path(__file__).resolve().parent / "robot_assets"
_ASSETS_DIR = _ASSETS_DIR.resolve()
_URDF_SRC = _ASSETS_DIR / "urdf" / "robot_description.urdf"

# Pybullet search path: meshes referenced as `ur_description/...` will
# resolve to `<assets>/ur_description/...`.
PYBULLET_SEARCH_PATH = str(_ASSETS_DIR)


_PACKAGE_RE = re.compile(r'filename="package://')


def patched_urdf_path() -> str:
    """Return a path to the URDF with `package://` URIs stripped.

    Writes the patched URDF next to the meshes inside `assets/` so
    pybullet's URDF-relative search resolves `ur_description/meshes/...`
    correctly. Multiple processes call this at startup; we write
    atomically (temp + replace) and skip writing entirely when the
    on-disk file is already the latest version, so concurrent workers
    don't corrupt each other.
    """
    cache = _ASSETS_DIR / "urdf" / "robot_description.patched.urdf"
    src_text = _URDF_SRC.read_text(encoding="utf-8")
    patched_text = _PACKAGE_RE.sub('filename="../', src_text)
    if cache.exists():
        existing = cache.read_text(encoding="utf-8")
        if existing == patched_text:
            return str(cache)
    tmp = cache.with_suffix(f".tmp.{os.getpid()}")
    tmp.write_text(patched_text, encoding="utf-8")
    try:
        os.replace(tmp, cache)
    except OSError:
        try:
            tmp.unlink()
        except OSError:
            pass
    return str(cache)


def load_into_pybullet(p, *, base_position=(0.0, 0.0, 0.0),
                       use_fixed_base: bool = True,
                       use_self_collision: bool = True):
    """Load the UR10e into an existing pybullet connection `p` and return
    `(body_id, joint_indices)` where `joint_indices` lines up with
    `UR10E_JOINT_NAMES`. The caller is responsible for `p.connect()`
    before and `p.disconnect()` after.
    """
    p.setAdditionalSearchPath(PYBULLET_SEARCH_PATH)
    flags = p.URDF_USE_INERTIA_FROM_FILE
    if use_self_collision:
        flags |= p.URDF_USE_SELF_COLLISION
    body_id = p.loadURDF(
        patched_urdf_path(),
        basePosition=list(base_position),
        useFixedBase=use_fixed_base,
        flags=flags,
    )
    joint_indices = _resolve_joint_indices(p, body_id, UR10E_JOINT_NAMES)
    return body_id, joint_indices


def _resolve_joint_indices(p, body_id: int, names: Iterable[str]) -> list[int]:
    name_to_idx: dict[str, int] = {}
    for i in range(p.getNumJoints(body_id)):
        info = p.getJointInfo(body_id, i)
        name_to_idx[info[1].decode("utf-8")] = i
    return [name_to_idx[n] for n in names]