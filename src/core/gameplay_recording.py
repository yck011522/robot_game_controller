"""Per-game gameplay recording: Parquet traces + a CSV ledger for analysis.

A *gameplay recording* covers exactly one game, from Tutorial entry through
the end of Play (Conclusion, Reset, and rewind motion are intentionally
excluded). It is unrelated to ``core.state_recording`` (the
``display_broadcast_recording`` "daydream" replay tool): that one captures the
verbatim ``state.full`` UDP feed for offline display-UI development; this one
captures a curated, purpose-built set of fields for gameplay analysis and
future rewind-shortcut tuning.

On-disk layout
--------------
::

    <root_dir>/
      games_index.csv                 # one row per completed game, permanent
      games/
        <date>/                       # local time, e.g. 2026-07-07
          <time>/                     # local time, e.g. 14-32-05
            state_global.parquet      # shared, not per-team
            a/
              game_controller.parquet
              haptic.parquet
              robot_actual.parquet
              weight.parquet
            b/
              game_controller.parquet
              haptic.parquet
              robot_actual.parquet
              weight.parquet

See ``GAMEPLAY_RECORDER_PLAN.md`` at the repo root for the full field-by-field
schema and the design rationale (source bus topic, unit, why each field is or
isn't included).

This module owns only the in-memory buffering + on-disk writing; the bus
wiring (subscribing to topics, deciding when a game starts/ends, mapping bus
message bodies to the ``record_*`` calls below) lives in
``apps.gameplay_recorder``.
"""

from __future__ import annotations

import csv
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any

import pyarrow as pa
import pyarrow.parquet as pq

# Bump when the on-disk column layout changes in a backward-incompatible way,
# so a reader can tell which schema version a given recording used.
GAMEPLAY_RECORDING_SCHEMA_VERSION = 1

# Default recordings root when a profile omits an explicit `dir` in its
# `gameplay_recording` block (see `resolve_gameplay_recording_config`).
DEFAULT_RECORDINGS_ROOT = "recordings"

# Permanent per-game ledger CSV filename, written directly under the
# recordings root (NOT inside a per-game folder, so it survives even if that
# game's Parquet folder is later deleted for disk space).
LEDGER_FILENAME = "games_index.csv"

# Column order for one games_index.csv row. Kept as an explicit tuple (rather
# than deriving it from a dict) so the on-disk column order is stable and
# reviewable in one place.
LEDGER_FIELDNAMES = (
    "date",
    "time",
    "profile_name",
    "tutorial_entered_at",
    "play_entered_at",
    "play_ended_at",
    "total_game_time_s",
    "score_a",
    "score_b",
    *[f"a_joint{j}_distance_rad" for j in range(1, 7)],
    *[f"b_joint{j}_distance_rad" for j in range(1, 7)],
)


# ---- local-time folder naming + timestamp formatting ----------------------


def local_folder_names(ts_wall_ns: int) -> tuple[str, str]:
    """Return ``(date_str, time_str)`` folder names for one game.

    Args:
        ts_wall_ns: Wall-clock nanoseconds since the Unix epoch (the game's
            Tutorial-entry ``state.full.ts_wall_ns``).

    Returns:
        ``(date_str, time_str)`` as ``("YYYY-MM-DD", "HH-MM-SS")``, using the
        machine's local time zone. The deployment PC's local clock is Hong
        Kong time, so no explicit timezone conversion is done here -- this
        just formats whatever the OS considers "now" for that instant.
    """
    dt = datetime.fromtimestamp(ts_wall_ns / 1e9)
    return dt.strftime("%Y-%m-%d"), dt.strftime("%H-%M-%S")


def iso_local(ts_wall_ns: int | None) -> str:
    """Format a wall-clock ns timestamp as a local ISO-8601 string with offset.

    Args:
        ts_wall_ns: Wall-clock nanoseconds since the Unix epoch, or ``None``.

    Returns:
        e.g. ``"2026-07-07T14:32:05.123456+08:00"``, both machine-parseable
        (standard ISO-8601) and human-readable. Returns ``""`` when
        ``ts_wall_ns`` is ``None`` (e.g. Play was never entered).
    """
    if ts_wall_ns is None:
        return ""
    # astimezone() with no argument attaches the OS's current local UTC
    # offset, so this stays correct without a zoneinfo dependency.
    dt = datetime.fromtimestamp(ts_wall_ns / 1e9).astimezone()
    return dt.isoformat(timespec="microseconds")


def unique_game_folder(root_dir: Path, date_str: str, time_str: str) -> Path:
    """Return a fresh ``<root_dir>/games/<date_str>/<time_str>[_N]`` path.

    Args:
        root_dir: Recordings root.
        date_str: Date folder name, e.g. ``"2026-07-07"``.
        time_str: Time folder name, e.g. ``"14-32-05"``.

    Returns:
        A path guaranteed not to already exist on disk. A ``_2``, ``_3``, ...
        suffix is appended only in the (practically impossible, since games
        take minutes) event that a folder for this exact second already
        exists, e.g. across a process restart.
    """
    base = root_dir / "games" / date_str
    candidate = base / time_str
    suffix = 1
    while candidate.exists():
        suffix += 1
        candidate = base / f"{time_str}_{suffix}"
    return candidate


# ---- per-stream column layouts --------------------------------------------
# Each tuple's element order matches the tuple order every `record_*` method
# below appends to its buffer, so `_write_parquet` can zip them positionally.

STATE_GLOBAL_COLUMNS = ("ts_wall_ns", "stage", "paused", "countdown_s")

GAME_CONTROLLER_COLUMNS = (
    "ts_wall_ns",
    "in_collision",
    "first_hit_detail",
    "prox_zones",
    "q_target_rad",
    "v_cmd_rad_s",
    "v_out_rad_s",
    "clamp_path",
    "clamp_prox",
    "clamp_final",
    "practice_player",
)

HAPTIC_COLUMNS = (
    "ts_wall_ns",
    "dial_pos_rad",
    "dial_vel_rad_s",
    "torque_ma",
    "dial_robot_deg",
)

ROBOT_ACTUAL_COLUMNS = (
    "ts_wall_ns",
    "q_rad",
    "qd_rad_s",
    "fault_active",
    "fault_reason",
)

WEIGHT_COLUMNS = ("ts_wall_ns", "bucket_1_g", "bucket_2_g", "bucket_3_g")

# Field names inside one `prox_zones` struct entry (one per robot joint, 6
# entries per row). Mirrors `teams.<t>.collision.prox_zones` in state.full
# (see docs/DISPLAY_BROADCAST_PROTOCOL.md).
PROX_ZONE_FIELDS = (
    "valid",
    "free_min_deg",
    "free_max_deg",
    "blocked_above_till_deg",
    "blocked_below_till_deg",
)


def _prox_zone_list_type() -> pa.DataType:
    """Return the pyarrow ``list<struct<...>>`` type for one `prox_zones` cell.

    Stored as a nested column (6 structs per row) rather than 30 flat
    columns (6 joints x 5 fields) -- this matches the natural shape of the
    data and keeps the Parquet schema compact; pandas/pyarrow both read
    nested columns back fine for later analysis.
    """
    return pa.list_(
        pa.struct(
            [
                pa.field("valid", pa.bool_()),
                pa.field("free_min_deg", pa.float64()),
                pa.field("free_max_deg", pa.float64()),
                pa.field("blocked_above_till_deg", pa.float64()),
                pa.field("blocked_below_till_deg", pa.float64()),
            ]
        )
    )


@dataclass
class _TeamBuffers:
    """One team's in-memory row buffers for one in-progress game recording.

    Each list holds one tuple per received sample, in arrival order; a
    tuple's element order matches the corresponding ``*_COLUMNS`` tuple above.
    The whole game (a few minutes) is buffered in memory and written to
    Parquet exactly once, at :meth:`GameRecording.finalize` -- crash safety
    via periodic flushing was explicitly not required (see
    GAMEPLAY_RECORDER_PLAN.md 2).
    """

    game_controller: list[tuple] = field(default_factory=list)
    haptic: list[tuple] = field(default_factory=list)
    robot_actual: list[tuple] = field(default_factory=list)
    weight: list[tuple] = field(default_factory=list)


class GameRecording:
    """Buffers one game's samples in memory and writes them out at Play end.

    One instance covers exactly one game, from Tutorial entry to Play end.
    The owning process (``apps.gameplay_recorder``) constructs one instance
    per detected Tutorial-entry edge, calls the ``record_*`` methods once per
    matching bus message it receives, and calls :meth:`finalize` exactly once
    on the edge out of the Play stage.

    Args:
        root_dir: Recordings root (``recordings`` by default); the ledger CSV
            lives directly under it and this game's Parquet files live under
            ``<root_dir>/games/<date>/<time>/`` (local time).
        profile_name: Loaded profile name, written to the ledger row only.
            Tuning/gear/rewind parameters are intentionally NOT duplicated
            here -- look them up from the named profile file instead.
        active_teams: Teams to create per-team subfolders/buffers for, e.g.
            ``["a"]``, ``["b"]``, or ``["a", "b"]``.
        tutorial_entered_wall_ns: Wall-clock ns when this game's Tutorial
            stage began. Used both as the ledger's ``tutorial_entered_at``
            and to derive the ``date``/``time`` folder names.
    """

    def __init__(
        self,
        *,
        root_dir: str | Path,
        profile_name: str,
        active_teams: list[str],
        tutorial_entered_wall_ns: int,
    ) -> None:
        self.root_dir = Path(root_dir)
        self.profile_name = str(profile_name)
        self.active_teams = [t for t in active_teams if t in ("a", "b")]
        self.tutorial_entered_wall_ns = int(tutorial_entered_wall_ns)
        # Set once by `mark_play_entered`; stays None if Play is never
        # reached (e.g. the game is abandoned mid-Tutorial).
        self.play_entered_wall_ns: int | None = None

        date_str, time_str = local_folder_names(self.tutorial_entered_wall_ns)
        self.date_str = date_str
        self.time_str = time_str
        self.folder = unique_game_folder(self.root_dir, date_str, time_str)

        self._state_global: list[tuple] = []
        self._teams: dict[str, _TeamBuffers] = {
            team: _TeamBuffers() for team in self.active_teams
        }
        self._closed = False

    # -- ingest -------------------------------------------------------------

    def mark_play_entered(self, ts_wall_ns: int) -> None:
        """Record the wall-clock instant Play began, for the ledger's `play_entered_at`.

        Called once by the owning process on the stage edge into `"play"`.
        Idempotent: only the first call takes effect.
        """
        if self.play_entered_wall_ns is None:
            self.play_entered_wall_ns = int(ts_wall_ns)

    def record_state_global(
        self, *, ts_wall_ns: int, stage: str, paused: bool, countdown_s: float
    ) -> None:
        """Buffer one shared (not per-team) `state.full`-derived row.

        Called once per received `state.full` message while a game is
        in progress.
        """
        self._state_global.append(
            (int(ts_wall_ns), str(stage), bool(paused), float(countdown_s))
        )

    def record_game_controller(
        self,
        team: str,
        *,
        ts_wall_ns: int,
        in_collision: bool,
        first_hit_detail: str | None,
        prox_zones: list[dict[str, Any]],
        q_target_rad: list[float],
        v_cmd_rad_s: list[float],
        v_out_rad_s: list[float],
        clamp_path: float,
        clamp_prox: float,
        clamp_final: float,
        practice_player: int,
    ) -> None:
        """Buffer one `state.full`-derived per-team planner/collision row.

        Called once per received `state.full` message, per active team,
        while a game is in progress. Silently ignored for a team this
        recording was not constructed with (``team not in active_teams``).
        """
        buf = self._teams.get(team)
        if buf is None:
            return
        buf.game_controller.append(
            (
                int(ts_wall_ns),
                bool(in_collision),
                first_hit_detail,
                list(prox_zones),
                [float(v) for v in q_target_rad[:6]],
                [float(v) for v in v_cmd_rad_s[:6]],
                [float(v) for v in v_out_rad_s[:6]],
                float(clamp_path),
                float(clamp_prox),
                float(clamp_final),
                int(practice_player),
            )
        )

    def record_haptic(
        self,
        team: str,
        *,
        ts_wall_ns: int,
        dial_pos_rad: list[float],
        dial_vel_rad_s: list[float],
        torque_ma: list[float],
        dial_robot_deg: list[float],
    ) -> None:
        """Buffer one `telem.haptic.<team>`-derived row.

        Called once per received `telem.haptic.<team>` message while a game
        is in progress. Silently ignored for an inactive team.
        """
        buf = self._teams.get(team)
        if buf is None:
            return
        buf.haptic.append(
            (
                int(ts_wall_ns),
                [float(v) for v in dial_pos_rad[:6]],
                [float(v) for v in dial_vel_rad_s[:6]],
                [float(v) for v in torque_ma[:6]],
                [float(v) for v in dial_robot_deg[:6]],
            )
        )

    def record_robot_actual(
        self,
        team: str,
        *,
        ts_wall_ns: int,
        q_rad: list[float],
        qd_rad_s: list[float],
        fault_active: bool,
        fault_reason: str | None,
    ) -> None:
        """Buffer one `telem.robot.actual.<team>`-derived row.

        Called once per received `telem.robot.actual.<team>` message while a
        game is in progress. Silently ignored for an inactive team.
        """
        buf = self._teams.get(team)
        if buf is None:
            return
        buf.robot_actual.append(
            (
                int(ts_wall_ns),
                [float(v) for v in q_rad[:6]],
                [float(v) for v in qd_rad_s[:6]],
                bool(fault_active),
                fault_reason,
            )
        )

    def record_weight(
        self,
        team: str,
        *,
        ts_wall_ns: int,
        bucket_1_g: float,
        bucket_2_g: float,
        bucket_3_g: float,
    ) -> None:
        """Buffer one `telem.weight`-derived row for this team's 3 buckets.

        `telem.weight` carries all 12 cells (both teams) in one message; the
        owning process splits it per team (see the static cell-id convention
        in `apps.gameplay_recorder`) and calls this once per team, per
        received message, while a game is in progress. Silently ignored for
        an inactive team.
        """
        buf = self._teams.get(team)
        if buf is None:
            return
        buf.weight.append(
            (int(ts_wall_ns), float(bucket_1_g), float(bucket_2_g), float(bucket_3_g))
        )

    # -- finalize -------------------------------------------------------------

    def finalize(self, *, play_ended_wall_ns: int, final_score: dict[str, int]) -> Path:
        """Write every Parquet file, append the ledger row, and return the folder.

        Args:
            play_ended_wall_ns: Wall-clock ns at the stage edge out of
                `"play"` (this recording's stop instant).
            final_score: ``{"a": <int>, "b": <int>}`` (only active teams need
                be present), the last-seen per-team score before Play ended.

        Returns:
            The per-game folder path that was written to.

        Raises:
            RuntimeError: If called more than once on the same instance.
        """
        if self._closed:
            raise RuntimeError("GameRecording.finalize() called twice")
        self._closed = True

        self.folder.mkdir(parents=True, exist_ok=True)
        _write_parquet(
            self.folder / "state_global.parquet",
            STATE_GLOBAL_COLUMNS,
            self._state_global,
        )

        distances: dict[str, list[float]] = {}
        for team, buf in self._teams.items():
            team_dir = self.folder / team
            team_dir.mkdir(parents=True, exist_ok=True)
            _write_game_controller_parquet(
                team_dir / "game_controller.parquet", buf.game_controller
            )
            _write_parquet(team_dir / "haptic.parquet", HAPTIC_COLUMNS, buf.haptic)
            _write_parquet(
                team_dir / "robot_actual.parquet",
                ROBOT_ACTUAL_COLUMNS,
                buf.robot_actual,
                dtypes={"fault_reason": pa.string()},
            )
            _write_parquet(team_dir / "weight.parquet", WEIGHT_COLUMNS, buf.weight)
            distances[team] = _per_joint_distance_rad(buf.robot_actual)

        total_game_time_s = (
            int(play_ended_wall_ns) - self.tutorial_entered_wall_ns
        ) / 1e9
        row: dict[str, Any] = {
            "date": self.date_str,
            "time": self.time_str,
            "profile_name": self.profile_name,
            "tutorial_entered_at": iso_local(self.tutorial_entered_wall_ns),
            "play_entered_at": iso_local(self.play_entered_wall_ns),
            "play_ended_at": iso_local(int(play_ended_wall_ns)),
            "total_game_time_s": f"{total_game_time_s:.3f}",
            "score_a": final_score.get("a", ""),
            "score_b": final_score.get("b", ""),
        }
        for team in ("a", "b"):
            joint_distances = distances.get(team, [0.0] * 6)
            for j in range(6):
                row[f"{team}_joint{j + 1}_distance_rad"] = f"{joint_distances[j]:.6f}"

        append_ledger_row(self.root_dir, row)
        return self.folder


def _per_joint_distance_rad(robot_actual_rows: list[tuple]) -> list[float]:
    """Sum ``|delta|`` per joint across one team's buffered robot_actual rows.

    Args:
        robot_actual_rows: The same tuple list buffered by
            `GameRecording.record_robot_actual`; each tuple's index 1 is the
            6-element `q_rad` list (see `ROBOT_ACTUAL_COLUMNS`).

    Returns:
        Six cumulative absolute-delta sums in radians, one per joint. A
        buffer with 0 or 1 samples returns all zeros (no deltas to sum).
    """
    totals = [0.0] * 6
    previous: list[float] | None = None
    for row in robot_actual_rows:
        q_rad = row[1]
        if previous is not None:
            for j in range(6):
                totals[j] += abs(q_rad[j] - previous[j])
        previous = q_rad
    return totals


# ---- Parquet / CSV writing --------------------------------------------------


def _write_parquet(
    path: Path,
    columns: tuple[str, ...],
    rows: list[tuple],
    *,
    dtypes: dict[str, pa.DataType] | None = None,
) -> None:
    """Write one flat Parquet file from buffered row tuples.

    Args:
        path: Destination ``.parquet`` file (parent directory must already
            exist; ``GameRecording.finalize`` creates it beforehand).
        columns: Column names, in the same order as each row tuple's elements.
        rows: Buffered row tuples (e.g. one team's ``buf.haptic`` list).
        dtypes: Optional explicit pyarrow type per column name. Needed for
            nullable columns (e.g. ``fault_reason``) that could otherwise be
            inferred as pyarrow's ``null`` type when every value in one game
            happens to be ``None`` (e.g. no faults occurred), which would
            make that game's file schema-incompatible with a game that *did*
            see a string value there. Columns not listed keep pyarrow's
            normal type inference.

    Columns are built directly with pyarrow (not through pandas) so
    fixed-length float arrays (e.g. the 6-element joint arrays) get a clean
    ``list<double>`` column type instead of pandas' less predictable
    object-column inference. Writing zero rows (e.g. a team that never
    reported telemetry) still produces a valid, empty, correctly-typed file.
    """
    dtypes = dtypes or {}
    arrays = {}
    for idx, name in enumerate(columns):
        values = [row[idx] for row in rows]
        arrays[name] = pa.array(values, type=dtypes.get(name))
    table = pa.table(arrays)
    pq.write_table(table, path)


def _write_game_controller_parquet(path: Path, rows: list[tuple]) -> None:
    """Write one team's ``game_controller.parquet``, including nested `prox_zones`.

    Args:
        path: Destination ``.parquet`` file.
        rows: Buffered ``game_controller`` row tuples (see
            `GAME_CONTROLLER_COLUMNS` for element order).

    Every column except `prox_zones` is a plain scalar/list-of-float column
    built the same way as `_write_parquet`; `prox_zones` additionally needs
    an explicit `list<struct<...>>` pyarrow type (see `_prox_zone_list_type`)
    since pyarrow cannot reliably infer a nested struct type from plain
    Python dicts on its own.
    """
    columns = GAME_CONTROLLER_COLUMNS
    prox_idx = columns.index("prox_zones")
    dtypes = {"first_hit_detail": pa.string()}
    arrays: dict[str, Any] = {
        name: pa.array([row[idx] for row in rows], type=dtypes.get(name))
        for idx, name in enumerate(columns)
        if name != "prox_zones"
    }
    arrays["prox_zones"] = pa.array(
        [row[prox_idx] for row in rows], type=_prox_zone_list_type()
    )
    table = pa.table(arrays)
    pq.write_table(table, path)


def append_ledger_row(root_dir: Path, row: dict[str, Any]) -> None:
    """Append one row to ``<root_dir>/games_index.csv``, writing the header once.

    Args:
        root_dir: Recordings root (created if missing).
        row: Mapping covering every name in `LEDGER_FIELDNAMES`.

    Only the `gameplay_recorder` process ever calls this (one writer), so a
    plain text-mode append is safe without extra file locking.
    """
    root_dir.mkdir(parents=True, exist_ok=True)
    path = root_dir / LEDGER_FILENAME
    is_new = not path.exists()
    with path.open("a", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=list(LEDGER_FIELDNAMES))
        if is_new:
            writer.writeheader()
        writer.writerow(row)


# ---- profile config -----------------------------------------------------


def resolve_gameplay_recording_config(profile: Any) -> tuple[bool, str]:
    """Return ``(enabled, root_dir)`` for the gameplay recorder.

    Args:
        profile: A loaded `core.config.Profile` (typed as `Any` here to
            avoid a hard import dependency on `core.config` from this
            module).

    Reads the optional top-level ``gameplay_recording`` mapping from the
    profile (mirrors the ``display_broadcast_recording`` block used by
    ``core.state_recording``). Unlike that block, this one defaults to
    **enabled** when the block is absent entirely, matching the "on by
    default, adjustable per profile" requirement; a profile opts out with
    an explicit ``gameplay_recording: {enabled: false}``.
    """
    node = profile.raw.get("gameplay_recording")
    if not isinstance(node, dict):
        return True, DEFAULT_RECORDINGS_ROOT
    enabled = bool(node.get("enabled", True))
    root_dir = str(node.get("dir") or DEFAULT_RECORDINGS_ROOT)
    return enabled, root_dir
