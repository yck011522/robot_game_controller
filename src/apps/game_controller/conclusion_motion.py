"""Conclusion-show collision certification and SegmentMover driving.

This module owns the two pieces of the end-of-game choreography that need bus /
collision I/O, keeping ``__main__`` orchestration-only:

* :class:`ConclusionCertifier` — a one-shot background check that the fixed
  pose-to-pose path each team will follow during the conclusion show is
  collision-free. It runs on a worker thread (so the game loop never blocks),
  reuses the existing collision-worker pool, and finishes within a configured
  budget. A team whose path is not certified in time is hard-stopped.
* :func:`drive_conclusion_team_motion` — called once per running tick per team
  during the conclusion stage. It gates on the certifier, seeds / advances that
  team's :class:`~subsystems.motion_planning.trajectory_timing.SegmentMover` for
  the active motion phase, and publishes ``cmd.robot.target.<team>``.

The phase machine in :mod:`apps.game_controller.stages` decides *which* pose to
move to and when (via the ``conclusion_move_pending`` / ``conclusion_move_arrived``
handshake); this module only realises that motion on the bus.
"""

from __future__ import annotations

import math
import threading
import time
from typing import Any

from core import bus
from subsystems.motion_planning.collision_client import CollisionWorkerClient
from subsystems.motion_planning.planner_core import discretize_joint_line

from apps.game_controller.context import DEFAULT_LOOK_POSE_DEG
from apps.game_controller.haptics import _reset_team_motion_outputs

# Conclusion phases that command a real robot move (as opposed to a hold). These
# must match the motion phase names set in ``stages._tick_conclusion_team``.
CONCLUSION_MOTION_PHASES = frozenset(
    {
        "move_to_bucket_pose",
        "move_to_announcement",
        "move_to_winner_pose",
        "move_to_begin",
    }
)

# Pose names (in ``robot_show_poses``) that make up the fixed conclusion path.
# The win/lose branch is certified for BOTH outcomes so the winner is free to be
# resolved late without re-certifying.
_LOOK_POSE_NAMES = ("robot_lookb1_pose", "robot_lookb2_pose", "robot_lookb3_pose")
_ANNOUNCEMENT_POSE_NAME = "robot_announcement_pose"
_WIN_POSE_NAME = "robot_win_pose"
_LOSE_POSE_NAME = "robot_lose_pose"
_BEGIN_POSE_NAME = "robot_begin_pose"


def _pose_rad(poses_deg: dict[str, list[float]], name: str) -> list[float]:
    """Return one named show pose converted from degrees to radians.

    Args:
        poses_deg: This team's ``robot_show_poses`` mapping (name -> degrees).
        name: Pose key to look up; falls back to ``DEFAULT_LOOK_POSE_DEG``.

    Returns:
        Six-joint configuration in radians.
    """

    deg = poses_deg.get(name, list(DEFAULT_LOOK_POSE_DEG))
    return [math.radians(float(v)) for v in list(deg)[:6]]


def build_conclusion_team_edges(
    start_rad: list[float],
    poses_deg: dict[str, list[float]],
) -> list[list[list[float]]]:
    """Build the fixed conclusion path as a list of straight ``[from, to]`` edges.

    The path starts at the measured pose where reset left the robot and walks
    the three bucket-look poses, the announcement pose, then both win→begin and
    lose→begin branches (so either outcome is pre-certified).

    Args:
        start_rad: Measured six-joint start configuration (radians) at the
            moment the conclusion stage is entered.
        poses_deg: This team's ``robot_show_poses`` mapping (name -> degrees).

    Returns:
        Ordered list of sparse edges, each ``[q_from, q_to]`` in radians.
    """

    look = [_pose_rad(poses_deg, name) for name in _LOOK_POSE_NAMES]
    ann = _pose_rad(poses_deg, _ANNOUNCEMENT_POSE_NAME)
    win = _pose_rad(poses_deg, _WIN_POSE_NAME)
    lose = _pose_rad(poses_deg, _LOSE_POSE_NAME)
    begin = _pose_rad(poses_deg, _BEGIN_POSE_NAME)
    return [
        [list(start_rad), look[0]],
        [look[0], look[1]],
        [look[1], look[2]],
        [look[2], ann],
        [ann, win],
        [win, begin],
        [ann, lose],
        [lose, begin],
    ]


class ConclusionCertifier:
    """One-shot, background collision check of every team's conclusion path.

    Created once in ``main()``. On entering the conclusion stage, ``__main__``
    calls :meth:`start` with each team's sparse edge list; a single worker thread
    densifies all edges and checks them through the shared collision pool with a
    wall-clock deadline. Results are read per team via :meth:`result`.

    When collision workers are disabled in the profile the certifier is a
    pass-through (every team is reported certified), because there is nothing to
    check against.
    """

    def __init__(
        self,
        *,
        collision_enabled: bool,
        collision_step_rad: float,
        collision_batch_size: int,
        worker_limit: int,
        budget_s: float,
        endpoint: str = bus.COLLISION_ROUTER_ENDPOINT,
    ) -> None:
        """Configure the certifier (no work happens until :meth:`start`).

        Args:
            collision_enabled: Whether the profile runs collision workers. When
                False the certifier passes every team (nothing to check).
            collision_step_rad: Maximum joint-space step (radians) between
                densified collision samples on each edge. Smaller = finer/slower.
            collision_batch_size: Configs per collision-worker request.
            worker_limit: Global outstanding-request ceiling (the collision pool
                size); bounds the parallel fan-out.
            budget_s: Wall-clock seconds allowed for certification. Edges still
                unresolved at the deadline count as NOT certified.
            endpoint: Collision broker ROUTER endpoint to connect to.
        """

        self._collision_enabled = bool(collision_enabled)
        self._collision_step_rad = max(1e-4, float(collision_step_rad))
        self._collision_batch_size = max(1, int(collision_batch_size))
        self._worker_limit = max(1, int(worker_limit))
        self._budget_s = max(0.1, float(budget_s))
        self._endpoint = endpoint
        # team -> True (certified) / False (collision or timed out) / None
        # (still running). Guarded by ``_lock`` because the worker thread writes
        # it while the game loop reads it.
        self._results: dict[str, bool | None] = {}
        self._lock = threading.Lock()
        self._thread: threading.Thread | None = None

    def start(self, team_edges: dict[str, list[list[list[float]]]]) -> None:
        """Begin certifying the given per-team edge lists on a worker thread.

        Any previous run is abandoned (its results are discarded). Each team's
        result starts as ``None`` (pending) and resolves to True/False.

        Args:
            team_edges: Mapping ``team -> [[q_from, q_to], ...]`` in radians, as
                produced by :func:`build_conclusion_team_edges`.
        """

        with self._lock:
            self._results = {team: None for team in team_edges}
        if not team_edges:
            return
        if not self._collision_enabled:
            # No collision pool to check against: certify everything so the show
            # still runs (matches the rest of the controller treating collision
            # checking as an optional subsystem).
            with self._lock:
                self._results = {team: True for team in team_edges}
            return
        self._thread = threading.Thread(
            target=self._run,
            args=({team: [list(map(list, edge)) for edge in edges]
                   for team, edges in team_edges.items()},),
            name="conclusion-certify",
            daemon=True,
        )
        self._thread.start()

    def result(self, team: str) -> bool | None:
        """Return a team's certification result.

        Returns:
            ``True`` certified collision-free, ``False`` collision found or the
            budget expired before it resolved, or ``None`` while still running /
            never started.
        """

        with self._lock:
            return self._results.get(team)

    def _run(self, team_edges: dict[str, list[list[list[float]]]]) -> None:
        """Worker-thread body: densify + check all edges within the budget."""

        deadline_s = time.perf_counter() + self._budget_s
        client: CollisionWorkerClient | None = None
        try:
            client = CollisionWorkerClient(
                endpoint=self._endpoint,
                producer="conclusion_certifier",
                timeout_s=self._budget_s,
            )
            # Flatten every team's edges into one densified batch, remembering
            # which flat indices belong to each team so results can be split.
            flat_edges: list[list[list[float]]] = []
            team_slices: dict[str, range] = {}
            for team, edges in team_edges.items():
                start_index = len(flat_edges)
                for q_from, q_to in edges:
                    flat_edges.append(
                        discretize_joint_line(q_from, q_to, self._collision_step_rad)
                    )
                team_slices[team] = range(start_index, len(flat_edges))

            check = client.check_edges_parallel_until_collision(
                flat_edges,
                batch_size=self._collision_batch_size,
                max_in_flight=self._worker_limit,
                deadline_s=deadline_s,
            )
            free = list(check.free)
            with self._lock:
                for team, indices in team_slices.items():
                    self._results[team] = all(
                        free[i] is True for i in indices
                    )
        except BaseException as exc:  # noqa: BLE001 - thread must not crash loop
            # On any failure (timeout, transport error) mark every pending team
            # as NOT certified so __main__ hard-stops them rather than moving
            # an uncertified arm.
            print(
                f"[conclusion-certify] failed: {type(exc).__name__}: {exc}",
                flush=True,
            )
            with self._lock:
                for team in list(self._results):
                    if self._results[team] is None:
                        self._results[team] = False
        finally:
            if client is not None:
                client.close()


def drive_conclusion_team_motion(
    pub: Any,
    proc: Any,
    team: str,
    st: dict[str, Any],
    dt: float,
    certifier: ConclusionCertifier,
) -> bool:
    """Realise the active conclusion motion phase for one team on the bus.

    Called once per running tick while the stage is ``conclusion``, AFTER the
    phase machine (``_tick_conclusion_team``) has run. For motion phases it gates
    on the certifier, (re)seeds the team's ``SegmentMover`` on a pending request,
    advances it, publishes ``cmd.robot.target.<team>``, and reports arrival back
    to the phase machine. Non-motion (hold) phases are left to the caller.

    Args:
        pub: Bus PUB socket.
        proc: Owning :class:`core.proc.Proc` (for envelope metadata).
        team: Team id (``"a"`` / ``"b"``).
        st: Per-team state dict (mutated in place).
        dt: Seconds since the previous running tick.
        certifier: Shared :class:`ConclusionCertifier` for this conclusion.

    Returns:
        True if a motion target was published (caller must NOT also hold); False
        if this phase is a hold / blocked on certification (caller should hold
        the current pose as usual).
    """

    phase = st.get("conclusion_phase")
    if phase not in CONCLUSION_MOTION_PHASES:
        return False

    cert = certifier.result(team)
    if cert is None:
        # Certification still running (within budget): hold until it resolves.
        return False
    if cert is False:
        # Path could not be certified collision-free. Hard-stop this team: hold
        # its pose, abandon the rest of the show, and let the game still finish.
        if not st.get("conclusion_hardstopped"):
            st["conclusion_hardstopped"] = True
            print(
                f"[conclusion] team={team} path NOT collision-free; "
                f"hard-stopping conclusion motion at phase={phase}",
                flush=True,
            )
        st["conclusion_phase"] = None
        st["conclusion_target_pose_name"] = None
        st["conclusion_target_pose_deg"] = None
        st["conclusion_done"] = True
        return False

    mover = st.get("conclusion_mover")
    start_rad = st.get("last_q")
    if mover is None or start_rad is None:
        # No mover wired or no measured pose yet: cannot move safely, hold.
        return False

    if st.get("conclusion_move_pending"):
        goal_deg = st.get("conclusion_target_pose_deg") or []
        goal_rad = [math.radians(float(v)) for v in list(goal_deg)[:6]]
        if len(goal_rad) < 6:
            # Malformed target pose: do not move, advance as if arrived so the
            # show does not stall forever on bad config.
            st["conclusion_move_pending"] = False
            st["conclusion_move_arrived"] = True
            return False
        mover.begin(list(start_rad), goal_rad)
        st["conclusion_move_pending"] = False

    q_target = mover.advance(dt)
    if not q_target:
        return False

    _reset_team_motion_outputs(st, q_target_rad=list(q_target))
    env = bus.make_envelope(proc)
    env.update(
        {
            "team": team,
            "q_target_rad": list(q_target),
            "clamps": {"path": 1.0, "prox": 1.0, "final": 1.0},
        }
    )
    bus.publish(pub, f"cmd.robot.target.{team}", env)

    if mover.arrived:
        st["conclusion_move_arrived"] = True
    return True
