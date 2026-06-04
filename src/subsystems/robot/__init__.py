"""src/subsystems/robot — UR10e impls and shared assets.

Modules:

- `scene.py` — shared loader for the curated compas_fab RobotCell +
  RobotCellState (`assets/robot_cell_and_state.json`) and the
  per-body touch-list whitelists
  (`assets/bullet_collision_pair_discovery.json`). Used by both the
  collision worker and the GUI sim so they see the same world.
- `sim_pybullet.py` — compas_fab `PyBulletPlanner`-backed `RobotIO`
  (P2). Supports both a GUI viewer (default in `dev_keyboard.yaml`)
  and a headless DIRECT mode (used by tests).
- `urdf_loader.py` — legacy raw-pybullet URDF loader. Kept on disk for
  diff-history; nothing imports it any more.
"""
