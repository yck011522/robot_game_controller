"""src/subsystems/robot — UR10e impls and shared assets.

Modules:

- `shared_compas_scene.py` — shared loader for the curated compas_fab RobotCell +
  RobotCellState (`assets/robot_cell_and_state.json`) and the
  per-body touch-list whitelists
  (`assets/bullet_collision_pair_discovery.json`). Used by both the
  collision worker and the GUI sim so they see the same world.
- `robot_sim_pybullet.py` — compas_fab `PyBulletPlanner`-backed `RobotIO`
  (P2). Supports both a GUI viewer (default in `dev_keyboard.yaml`)
  and a headless DIRECT mode (used by tests).
- `robot_real_rtde.py` — real UR10e `RobotIO` using `ur_rtde` with the
  startup-sync and `servoJ` behavior proven in the archived keyboard
  explorer.
- `rtde_helpers.py` — thin connectivity and preflight helpers shared by
  the RTDE-backed RobotIO implementation.

Archived reference-only files live under `archive/` and are not part of
the active import graph.
"""
