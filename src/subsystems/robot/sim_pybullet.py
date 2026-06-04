"""compas_fab PyBullet UR10e simulator (P2).

Owns a compas_fab `PyBulletClient` (gui or direct) loaded with the
curated RobotCell + RobotCellState from
[scene.make_planner](./scene.py). On every `set_target(q)` we teleport
the robot's `robot_configuration` to `q` and push the updated
RobotCellState into the planner -- this both moves the visible robot
in the GUI viewer and (because compas_fab also tracks contact state on
the same client) keeps the scene's collision world consistent with
what the GUI shows.

We deliberately use teleport, not pybullet POSITION_CONTROL, for the
P2 demo. The "follow the dial" feel is what the game needs; PD tuning
to match real RTDE behavior lives in P3 where we wire up real-robot
output.

`maybe_step()` is a no-op (no physics to integrate when we teleport);
it stays in the API so the I/O loop in `apps/robot_io` doesn't need a
mode switch when a future driver re-introduces stepping.

`read_state()` reads back q from the RobotCellState we just wrote. We
don't sample the pybullet body directly because, with compas_fab, the
RobotCellState is the source of truth -- the underlying pybullet body
indices are an implementation detail of the planner.
"""

from __future__ import annotations

from typing import List, Tuple


class SimPybulletRobot:
    def __init__(self, *, headless: bool, initial_pose_rad: List[float] | None = None):
        # Late import so a tooling import of this module (e.g. for
        # --help) doesn't drag in compas_fab + pybullet.
        from subsystems.robot.scene import make_planner, UR10E_JOINT_NAMES
        connection_type = "direct" if headless else "gui"
        client, planner, robot_cell, rcs, stats = make_planner(
            connection_type=connection_type, verbose=False
        )
        self._client = client
        self._planner = planner
        self._robot_cell = robot_cell
        self._rcs = rcs
        self._cfg = rcs.robot_configuration.copy()
        self._joint_names = UR10E_JOINT_NAMES
        self._n = len(UR10E_JOINT_NAMES)
        # Seed from caller-supplied home pose (profile's
        # tuning.robot.initial_pose_deg). The scene JSON itself sits at
        # all-zero, which is inside the pedestal -- starting there
        # would flag a collision on tick 0.
        if initial_pose_rad is not None and len(initial_pose_rad) >= self._n:
            self._q = [float(v) for v in initial_pose_rad[: self._n]]
        else:
            self._q = list(self._cfg.joint_values) + [0.0] * (self._n - len(self._cfg.joint_values))
            self._q = self._q[: self._n]
        self._qd = [0.0] * self._n
        self._scene_stats = stats
        # Push the initial pose into the planner so the GUI opens at
        # the home pose, not at the scene's stored zeros.
        self._cfg.joint_values = list(self._q)
        self._rcs.robot_configuration = self._cfg
        try:
            self._planner.set_robot_cell_state(self._rcs)
        except Exception:
            pass

    @property
    def scene_stats(self) -> dict:
        return self._scene_stats

    def set_target(self, q: List[float]) -> None:
        if len(q) != self._n:
            return
        # Velocity estimate is just the per-call delta; the I/O loop
        # publishes telem at TELEM_HZ so this is good enough for the
        # game_controller's "how fast is the robot actually moving"
        # readout.
        self._qd = [float(b) - float(a) for a, b in zip(self._q, q)]
        self._q = [float(v) for v in q]
        self._cfg.joint_values = list(self._q)
        self._rcs.robot_configuration = self._cfg
        try:
            # set_robot_cell_state updates the underlying pybullet body
            # transforms; in GUI mode this re-draws the arm.
            self._planner.set_robot_cell_state(self._rcs)
        except Exception:
            # Don't let a transient draw error kill the I/O loop.
            pass

    def maybe_step(self) -> None:
        # Teleport mode: nothing to integrate.
        return

    def read_state(self) -> Tuple[List[float], List[float]]:
        return list(self._q), list(self._qd)

    def close(self) -> None:
        try:
            self._client.__exit__(None, None, None)
        except Exception:
            pass
