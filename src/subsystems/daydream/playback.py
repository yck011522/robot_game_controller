"""Load a recorded game's joint trajectory and play it back during daydream.

Daydream attract mode replays the most recent recorded game: each team's robot
re-walks the actual joints it traced during the first ``play`` stage of a
display-broadcast recording, then rewinds (smoothed) to the play-entry pose and
loops. Forward playback follows the recording verbatim (no smoothing); the
rewind is velocity-retimed and optionally collision-shortcut-smoothed by the
shared :class:`RewindController`.

Layering: this module is hardware-free. ``game_controller.__main__`` owns the
bus and feeds measured poses / ``dt`` in, then publishes the returned targets.

Recording format: ``logs/display_broadcast_recording/*.jsonl.gz`` -- see
``core.state_recording``. Each frame's ``state`` carries a top-level ``stage``
(or ``active_stage``) and ``teams[<id>].robot.q_rad`` (six joints, radians).
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any

from core.state_recording import iter_frames
from subsystems.motion_planning.trajectory_timing import sample_path_with_index
from subsystems.rewind.in_process import RewindController
from subsystems.rewind.shortcut import ShortcutSettings


_AXES = 6


def find_latest_recording(directory: str | Path) -> Path | None:
    """Return the newest ``*.jsonl.gz`` recording under ``directory``.

    The daydream loader calls this once at startup to pick the most recently
    captured session. Returns ``None`` when the folder is missing or empty so
    callers can fall back to the LED-breathing-only attract mode.
    """

    folder = Path(directory)
    if not folder.is_dir():
        return None
    candidates = sorted(
        folder.glob("*.jsonl.gz"), key=lambda p: p.stat().st_mtime, reverse=True
    )
    return candidates[0] if candidates else None


def load_first_play_segment(
    recording_path: str | Path, teams: list[str]
) -> dict[str, list[tuple[float, list[float]]]]:
    """Extract each team's first contiguous ``play`` joint trajectory.

    Scans frames in file order, keeps only the first run of consecutive frames
    whose stage is ``play``, and returns per-team ``[(t_s, q_rad6), ...]`` with
    ``t_s`` measured from the first play frame. ``teams`` selects which team
    blocks to extract. Teams with too few play samples are omitted.

    Returns an empty dict when no play segment exists, so daydream falls back to
    breathing-only attract mode.
    """

    out: dict[str, list[tuple[float, list[float]]]] = {t: [] for t in teams}
    seen_play = False
    t0_ns: int | None = None
    for frame in iter_frames(recording_path):
        state = frame.get("state") or {}
        stage = str(state.get("active_stage") or state.get("stage") or "")
        if stage != "play":
            if seen_play:
                break  # first play segment ended; stop at the first game
            continue
        seen_play = True
        ts = int(frame.get("ts_wall_ns") or 0)
        if t0_ns is None:
            t0_ns = ts
        t_s = max(0.0, (ts - t0_ns) / 1e9)
        team_blocks = state.get("teams") or {}
        for team in teams:
            q = _team_q_rad(team_blocks.get(team))
            if q is not None:
                out[team].append((t_s, q))
    return {team: samples for team, samples in out.items() if len(samples) >= 2}


def _team_q_rad(team_block: Any) -> list[float] | None:
    """Pull a six-joint ``robot.q_rad`` list (radians) from one team block."""

    if not isinstance(team_block, dict):
        return None
    robot = team_block.get("robot")
    if not isinstance(robot, dict):
        return None
    q = robot.get("q_rad")
    if not isinstance(q, list) or len(q) < _AXES:
        return None
    try:
        return [float(v) for v in q[:_AXES]]
    except (TypeError, ValueError):
        return None


def _valid_q_rad(value: list[float] | None) -> list[float] | None:
    """Return a six-joint float copy for daydream playback internals.

    Called when the runtime supplies live robot telemetry for an interrupted
    rewind. Malformed or missing telemetry returns ``None`` so the player falls
    back to the already-sampled recorded path.
    """

    if not isinstance(value, list) or len(value) < _AXES:
        return None
    try:
        return [float(v) for v in value[:_AXES]]
    except (TypeError, ValueError):
        return None


class DaydreamPlayer:
    """Replay one team's recorded play path, then smoothed rewind, then loop.

    The controller is open-loop in time for the forward pass (it follows the
    recorded timestamps verbatim) and reuses :class:`RewindController` for a
    velocity-retimed, optionally collision-smoothed return to the play-entry
    pose. Drive it once per tick with :meth:`forward_target` while idle and with
    :meth:`rewind_target` once :meth:`return_complete` style flags say so.

    Parameters
    ----------
    samples:
        ``[(t_s, q_rad6), ...]`` for the recorded play segment (forward source).
    max_velocity_rad_s:
        Per-joint configured max velocities used to retime the rewind.
    rewind_speed_fraction:
        Fraction of those maxima used for the smoothed rewind.
    arrival_tolerance_rad:
        Per-joint completion tolerance for the rewind.
    shortcut_settings:
        Collision-shortcut smoothing config for the rewind (disabled = none).
    team:
        Team id, for logging.
    """

    def __init__(
        self,
        *,
        samples: list[tuple[float, list[float]]],
        max_velocity_rad_s: list[float],
        rewind_speed_fraction: float,
        arrival_tolerance_rad: float,
        shortcut_settings: ShortcutSettings | None,
        team: str = "a",
    ) -> None:
        self.team = str(team)
        self._times = [float(t) for t, _ in samples]
        self._path = [[float(v) for v in q[:_AXES]] for _, q in samples]
        self.available = len(self._path) >= 2
        self._max_velocity_rad_s = list(max_velocity_rad_s)
        self._rewind_speed_fraction = float(rewind_speed_fraction)
        self._arrival_tolerance_rad = float(arrival_tolerance_rad)
        self._shortcut_settings = shortcut_settings or ShortcutSettings()
        # "forward" -> following recording; "rewinding" -> smoothed return;
        # "loop_waiting" -> holding at play-entry until all teams are ready;
        # "idle" until first started.
        self._phase = "idle"
        self._forward_elapsed_s = 0.0
        self._forward_index = 0
        self._rewind: RewindController | None = None

    @property
    def phase(self) -> str:
        """Current playback phase: ``idle`` / ``forward`` / ``rewinding`` / ``loop_waiting``."""

        return self._phase

    def start_forward(self) -> None:
        """Begin (or loop back to) the forward recorded pass at t=0."""

        self._phase = "forward"
        self._forward_elapsed_s = 0.0
        self._forward_index = 0

    def wait_for_loop_restart(self) -> None:
        """Hold at play-entry until the controller restarts every team together.

        Called by the game controller after a natural daydream rewind completes.
        The player keeps its completed rewind controller so
        :meth:`rewind_target` can continue returning the play-entry target while
        other teams finish their own rewind.
        """

        self._phase = "loop_waiting"

    def forward_target(self, dt_s: float) -> tuple[list[float], bool]:
        """Advance the forward pass and return ``(q_target, finished)``.

        ``finished`` is True once the recorded timeline is exhausted; the caller
        should then begin the smoothed rewind via :meth:`begin_rewind`.
        """

        if not self.available:
            return [], True
        if self._phase != "forward":
            self.start_forward()
        self._forward_elapsed_s = min(
            self._times[-1], self._forward_elapsed_s + max(0.0, float(dt_s))
        )
        target, self._forward_index = sample_path_with_index(
            self._path, self._times, self._forward_elapsed_s
        )
        finished = self._forward_elapsed_s >= self._times[-1]
        return target, finished

    def begin_rewind(
        self, *, now_s: float, current_q_rad: list[float] | None = None
    ) -> bool:
        """Build a smoothed rewind from the current forward point to the start.

        Reverses only the already-played portion of the recording (so an
        interrupt rewinds from where the robot currently is back to the
        play-entry pose). ``current_q_rad`` is supplied by the game controller
        for interrupt rewinds so the optimized path starts at the measured pose
        being held while the shortcut search runs. Returns True when a rewind
        path was armed.
        """

        played = self._path[: self._forward_index + 1]
        current_q = _valid_q_rad(current_q_rad)
        if current_q is not None:
            if played:
                max_delta = max(
                    abs(current_q[axis] - played[-1][axis]) for axis in range(_AXES)
                )
                if max_delta > 1e-9:
                    played.append(current_q)
            else:
                played = [current_q]
        if len(played) < 2:
            self._phase = "rewinding"
            self._rewind = None
            return False
        ctrl = RewindController(
            enabled=True,
            max_velocity_rad_s=self._max_velocity_rad_s,
            speed_fraction=self._rewind_speed_fraction,
            arrival_tolerance_rad=self._arrival_tolerance_rad,
            team=self.team,
            shortcut_settings=self._shortcut_settings,
        )
        ctrl.start_recording(list(played[0]), now_s=now_s)
        for q in played[1:]:
            ctrl.record_target(list(q), now_s=now_s)
        ctrl.start_rewind()
        self._rewind = ctrl
        self._phase = "rewinding"
        return True

    def rewind_target(
        self, *, dt_s: float, q_actual_rad: list[float] | None
    ) -> list[float] | None:
        """Return the next smoothed-rewind target, or None while not ready."""

        if self._rewind is None:
            return None
        return self._rewind.next_target(dt_s=dt_s, q_actual_rad=q_actual_rad)

    @property
    def rewind_complete(self) -> bool:
        """True once the smoothed rewind has reached the play-entry pose."""

        return self._rewind is None or bool(self._rewind.complete)

    def close(self) -> None:
        """Stop any background shortcut search on teardown."""

        if self._rewind is not None:
            self._rewind.close()
