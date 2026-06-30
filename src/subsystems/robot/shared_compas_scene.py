"""compas_fab RobotCell + RobotCellState loader (shared by sim + collision).

The scene assets follow the convention from
`archive/bullet_collision_keyboard_explorer.py`:

  - `assets/robot_cell_and_state.json` is a compas `json_load`-able dict
    with `{robot_cell, robot_cell_state}` (RobotCell + RobotCellState).
  - `assets/bullet_collision_pair_discovery.json` carries per-body /
    per-tool `touch_links_candidates` + `touch_bodies_candidates` lists
    produced by the offline discovery pass. Patching these into the
    RobotCellState is what makes collision results match the curated
    explorer behavior (without them, every static body-vs-link contact
    that should be ignored shows up as a hit).

All P2 processes that touch collision (collision_worker, the GUI sim,
later swept-volume planners) go through here so they all see the same
scene.

Joint order on the bus is always the UR10e URDF order; we expose it as
`UR10E_JOINT_NAMES` so callers don't have to re-derive it from
`robot_cell.robot_model`.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Tuple

from compas.data import json_load
from compas_fab.backends import PyBulletClient, PyBulletPlanner
from compas_fab.backends.exceptions import BackendError
import compas_fab.robots  # noqa: F401  # Registers RobotCell / RobotCellState for json_load.


_ASSETS_DIR = Path(__file__).resolve().parent / "assets"
SCENE_JSON_PATH = _ASSETS_DIR / "robot_cell_and_state.json"
DISCOVERY_JSON_PATH = _ASSETS_DIR / "bullet_collision_pair_discovery.json"

UR10E_JOINT_NAMES: Tuple[str, ...] = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)


def _load_discovery() -> dict:
    with open(DISCOVERY_JSON_PATH, "r", encoding="utf-8") as f:
        return json.load(f)


def _apply_touch_lists(robot_cell_state, discovery: dict) -> dict:
    """Patch curated touch-link / touch-body lists onto a RobotCellState.

    Returns a small stats dict useful for banner-printing on startup
    (how many bodies + tools were patched, total skip counts).
    """
    per_body = discovery.get("per_rigid_body", {})
    per_tool = discovery.get("per_tool", {})
    n_b = n_t = tl_total = tb_total = 0
    for key, info in per_body.items():
        state = robot_cell_state.rigid_body_states.get(key)
        if state is None:
            continue
        tl = list(info.get("touch_links_candidates", []))
        tb = list(info.get("touch_bodies_candidates", []))
        state.touch_links = tl
        state.touch_bodies = tb
        n_b += 1
        tl_total += len(tl)
        tb_total += len(tb)
    if hasattr(robot_cell_state, "tool_states"):
        for key, info in per_tool.items():
            state = robot_cell_state.tool_states.get(key)
            if state is None:
                continue
            tl = list(info.get("touch_links_candidates", []))
            tb = list(info.get("touch_bodies_candidates", []))
            if hasattr(state, "touch_links"):
                state.touch_links = tl
            if hasattr(state, "touch_bodies"):
                state.touch_bodies = tb
            n_t += 1
            tl_total += len(tl)
            tb_total += len(tb)
    return {
        "n_bodies_patched": n_b,
        "n_tools_patched": n_t,
        "total_touch_links": tl_total,
        "total_touch_bodies": tb_total,
    }


def load_scene(*, apply_touch: bool = True):
    """Load RobotCell + RobotCellState from disk, ready for a planner.

    Returns `(robot_cell, robot_cell_state, patch_stats)` where
    `patch_stats` is an empty dict when `apply_touch=False`.

    The `transmission` attribute is popped because compas_fab's
    PyBulletClient chokes on it (same workaround the explorer uses).
    """
    data = json_load(str(SCENE_JSON_PATH))
    robot_cell = data["robot_cell"]
    robot_cell_state = data["robot_cell_state"]
    robot_cell.robot_model.attr.pop("transmission", None)
    stats: dict = {}
    if apply_touch:
        stats = _apply_touch_lists(robot_cell_state, _load_discovery())
    return robot_cell, robot_cell_state, stats


def make_planner(*, connection_type: str = "direct", verbose: bool = False
                 ) -> Tuple[PyBulletClient, PyBulletPlanner, object, object, dict]:
    """One-stop helper: open a PyBullet client, load the scene, return
    `(client, planner, robot_cell, robot_cell_state, patch_stats)`.

    Caller owns the client lifecycle (must call `client.__exit__(...)`
    on shutdown). A throwaway `check_collision` is run on startup the
    same way the explorer does it -- this primes pybullet's internal
    contact caches so the first request from a real client doesn't pay
    a one-shot warmup cost.

    `connection_type` is passed through to compas_fab's PyBulletClient:
      - "direct" -- headless (collision workers, CI)
      - "gui"    -- OpenGL viewer (manual P2 demo)
    """
    robot_cell, robot_cell_state, stats = load_scene(apply_touch=True)
    client = PyBulletClient(connection_type=connection_type, verbose=verbose)
    client.__enter__()
    planner = PyBulletPlanner(client)
    planner.set_robot_cell(robot_cell)
    planner.set_robot_cell_state(robot_cell_state)
    try:
        planner.check_collision(robot_cell_state, options={"verbose": False})
    except BackendError:
        # Startup pose may already be in collision (curated scene); fine.
        pass
    return client, planner, robot_cell, robot_cell_state, stats


__all__ = [
    "UR10E_JOINT_NAMES",
    "SCENE_JSON_PATH",
    "DISCOVERY_JSON_PATH",
    "load_scene",
    "make_planner",
]