"""gameplay_recorder - per-game Parquet/CSV gameplay recording.

Subscribes directly to the main bus (not owned by ``GameController``) and
records exactly one ``core.gameplay_recording.GameRecording`` per game, from
the Tutorial-entry stage edge through the stage edge out of Play (Conclusion,
Reset, and rewind motion are intentionally excluded). See
``GAMEPLAY_RECORDER_PLAN.md`` at the repo root for the full field-by-field
schema and design rationale.

This is unrelated to ``apps.state_broadcaster``'s optional
``display_broadcast_recording`` ("daydream" replay) session recording, which
captures the verbatim ``state.full`` UDP feed for offline display-UI
development instead of curated per-team analysis fields.

Configuration
-------------
* Recordings root + on/off switch: optional ``gameplay_recording`` block in
  the loaded **profile** (``enabled: true|false`` + ``dir``). Defaults to
  **enabled** with ``dir: recordings`` when the block is absent entirely, so
  every profile records by default; add ``gameplay_recording: {enabled:
  false}`` to opt a profile out (e.g. synthetic batch-validation runs).
* Haptic gear ratio (for the recorded ``dial_robot_deg`` column): profile
  ``tuning.haptic.gear_ratio``.
* Poll rate: this process's own wake-up rate in `config/runtime.yaml`
  (``subsystems.gameplay_recorder.fps_target``). The process is event-driven
  -- it drains every queued bus message each tick regardless of this rate --
  so this only bounds worst-case latency between a bus message arriving and
  it landing in the in-memory buffer.

Run standalone (the launcher also spawns it by default, tier 9):

    $env:PYTHONPATH = "src"
    & C:/Users/leungp/anaconda3/envs/game/python.exe -m apps.gameplay_recorder \
        --profile config/profiles/two_teams.yaml --proc gameplay_recorder
"""

from __future__ import annotations

import math
import sys
from pathlib import Path
from typing import Any

_SRC = Path(__file__).resolve().parents[2]
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

import zmq  # noqa: E402

from core import bus  # noqa: E402
from core.config import default_runtime_setting, load as load_profile  # noqa: E402
from core.gameplay_recording import (  # noqa: E402
    GameRecording,
    resolve_gameplay_recording_config,
)
from core.proc import Proc, banner, parse_proc_args  # noqa: E402

# Fallback poll rate if runtime.yaml lacks an entry (Hz). The process is
# event-driven (drains the whole queue every tick), so this only bounds
# worst-case latency, not throughput.
DEFAULT_TARGET_HZ = 120.0

# Static team -> bucket-cell-id mapping for splitting one global `telem.weight`
# message into each team's 3 cells. Mirrors
# `apps.game_controller.context.TEAM_BUCKET_IDS`; duplicated here (rather than
# imported) so this process's dependency footprint stays independent of the
# game_controller app package.
_TEAM_BUCKET_CELL_IDS: dict[str, list[str]] = {
    "a": ["11", "12", "13"],
    "b": ["21", "22", "23"],
}


def main(argv: list[str] | None = None) -> int:
    """Run the gameplay recorder: subscribe to the bus, record one game at a time."""

    args, _ = parse_proc_args(argv, default_proc="gameplay_recorder")
    profile = load_profile(args.profile_path)
    enabled, root_dir = resolve_gameplay_recording_config(profile)
    active_teams = [t for t in profile.active_teams if t in ("a", "b")]
    gear_ratio = _team_gear_ratio(profile)

    target_hz = default_runtime_setting(
        "gameplay_recorder", "fps_target", DEFAULT_TARGET_HZ
    )
    proc = Proc(args, profile, target_hz=target_hz or DEFAULT_TARGET_HZ)

    topics = ["state.full", "telem.weight"]
    topics += [f"telem.haptic.{team}" for team in active_teams]
    topics += [f"telem.robot.actual.{team}" for team in active_teams]
    sub = bus.make_sub(proc.ctx, topics=topics)
    pub = bus.make_pub(proc.ctx)
    proc.use_heartbeat_pub(pub)

    banner(
        proc.proc,
        f"enabled={enabled} root={root_dir} teams={active_teams or '(none)'}",
    )

    # Per-process state held in a dict so the nested tick() can mutate it.
    rec_state: dict[str, Any] = {
        "recording": None,  # GameRecording when a game is in progress, else None
        "prev_stage": None,  # last-seen state.full.active_stage, for edge detection
        "last_score": {},  # most recently seen {"a": int, "b": int} score
    }

    def tick(_p: Proc) -> None:
        """Drain every queued bus message and route it into the active recording."""

        if not enabled:
            _drain_all(sub)  # keep draining so the SUB queue never backs up
            return
        for topic, body in _drain_all(sub):
            _handle_message(
                rec_state, topic, body, root_dir, profile.name, active_teams, gear_ratio
            )

    def teardown(_p: Proc) -> None:
        """Close the subscription. An in-progress recording is intentionally
        discarded here: it never reached Play end, so there is nothing valid
        to finalize (see module docstring)."""

        sub.close(0)

    return proc.run(tick, teardown=teardown)


def _team_gear_ratio(profile) -> list[float]:
    """Return the 6-element per-axis dial->robot gear ratio from the profile.

    Used to compute the recorded ``dial_robot_deg`` column from each raw
    ``dial_pos_rad`` haptic sample. Falls back to ``1.0`` per axis (and pads
    to 6 entries) if the profile's ``tuning.haptic.gear_ratio`` is missing or
    short.
    """
    gear = list(profile.tuning.get("haptic", {}).get("gear_ratio") or [1.0] * 6)
    while len(gear) < 6:
        gear.append(1.0)
    return [float(v) for v in gear[:6]]


def _drain_all(sub: zmq.Socket) -> list[tuple[str, dict]]:
    """Return every queued ``(topic, body)`` pair in arrival order.

    Unlike most bus consumers (which only need the freshest sample per topic
    and drain latest-wins), the recorder must not lose any message -- every
    frame is a genuine analysis data point -- so this drains the whole queue
    every tick instead of keeping only the last message per topic.
    """
    out: list[tuple[str, dict]] = []
    while True:
        try:
            topic, body = bus.recv(sub, flags=zmq.NOBLOCK)
        except zmq.Again:
            return out
        if isinstance(body, dict):
            out.append((topic, body))


def _handle_message(
    rec_state: dict[str, Any],
    topic: str,
    body: dict[str, Any],
    root_dir: str,
    profile_name: str,
    active_teams: list[str],
    gear_ratio: list[float],
) -> None:
    """Route one drained bus message to the right handler by topic."""

    if topic == "state.full":
        _handle_state_full(rec_state, body, root_dir, profile_name, active_teams)
        return
    if topic == "telem.weight":
        _handle_weight(rec_state, body)
        return
    for team in active_teams:
        if topic == f"telem.haptic.{team}":
            _handle_haptic(rec_state, team, body, gear_ratio)
            return
        if topic == f"telem.robot.actual.{team}":
            _handle_robot_actual(rec_state, team, body)
            return


def _handle_state_full(
    rec_state: dict[str, Any],
    body: dict[str, Any],
    root_dir: str,
    profile_name: str,
    active_teams: list[str],
) -> None:
    """Advance the recording lifecycle and buffer one state.full-derived tick.

    Stage edges drive everything:
    * ``-> "tutorial"``: start a new `GameRecording` (any previous unfinished
      one never reached Play end and is silently discarded -- see the module
      docstring's teardown note).
    * ``-> "play"``: mark this game's `play_entered_at`.
    * ``"play" ->``: finalize (write Parquet + append the ledger row) and
      clear the active recording.
    """

    stage = body.get("active_stage")
    ts_wall_ns = body.get("ts_wall_ns")
    prev_stage = rec_state["prev_stage"]

    if prev_stage != "tutorial" and stage == "tutorial" and isinstance(ts_wall_ns, int):
        rec_state["recording"] = GameRecording(
            root_dir=root_dir,
            profile_name=profile_name,
            active_teams=active_teams,
            tutorial_entered_wall_ns=ts_wall_ns,
        )
        rec_state["last_score"] = {}

    recording: GameRecording | None = rec_state["recording"]

    if (
        recording is not None
        and prev_stage != "play"
        and stage == "play"
        and isinstance(ts_wall_ns, int)
    ):
        recording.mark_play_entered(ts_wall_ns)

    if recording is not None and isinstance(ts_wall_ns, int):
        recording.record_state_global(
            ts_wall_ns=ts_wall_ns,
            stage=str(stage),
            paused=bool(body.get("paused", False)),
            countdown_s=float(body.get("countdown_s", 0.0)),
        )
        teams_body = body.get("teams")
        if isinstance(teams_body, dict):
            for team, team_body in teams_body.items():
                if not isinstance(team_body, dict):
                    continue
                _record_game_controller_row(recording, team, ts_wall_ns, team_body)
                score = team_body.get("score")
                if isinstance(score, int):
                    rec_state["last_score"][team] = score

    if (
        recording is not None
        and prev_stage == "play"
        and stage != "play"
        and isinstance(ts_wall_ns, int)
    ):
        recording.finalize(
            play_ended_wall_ns=ts_wall_ns, final_score=dict(rec_state["last_score"])
        )
        rec_state["recording"] = None

    rec_state["prev_stage"] = stage


def _record_game_controller_row(
    recording: GameRecording, team: str, ts_wall_ns: int, team_body: dict[str, Any]
) -> None:
    """Extract one team's `game_controller.parquet` row out of a `state.full` team block.

    All of these fields are already published on `state.full` today (see
    docs/DISPLAY_BROADCAST_PROTOCOL.md) -- no new bus plumbing is needed for
    this file, unlike `haptic.parquet`'s `torque_ma`.
    """

    robot = team_body.get("robot") if isinstance(team_body.get("robot"), dict) else {}
    collision = (
        team_body.get("collision")
        if isinstance(team_body.get("collision"), dict)
        else {}
    )
    planner = (
        team_body.get("planner") if isinstance(team_body.get("planner"), dict) else {}
    )
    practice = (
        team_body.get("practice") if isinstance(team_body.get("practice"), dict) else {}
    )

    first_hit = collision.get("first_hit")
    first_hit_detail = first_hit.get("detail") if isinstance(first_hit, dict) else None
    practice_player = practice.get("active_player")

    recording.record_game_controller(
        team,
        ts_wall_ns=ts_wall_ns,
        in_collision=bool(collision.get("in_collision", False)),
        first_hit_detail=(
            first_hit_detail if isinstance(first_hit_detail, str) else None
        ),
        prox_zones=_coerce_prox_zones(collision.get("prox_zones")),
        q_target_rad=_coerce_floats(robot.get("q_target_rad")),
        v_cmd_rad_s=_coerce_floats(planner.get("v_cmd_rad_s")),
        v_out_rad_s=_coerce_floats(planner.get("v_out_rad_s")),
        clamp_path=float(collision.get("path_scalar", 1.0)),
        clamp_prox=float(collision.get("prox_scalar", 1.0)),
        clamp_final=float(collision.get("final_scalar", 1.0)),
        practice_player=int(practice_player) if isinstance(practice_player, int) else 0,
    )


def _coerce_prox_zones(raw: Any) -> list[dict[str, Any]]:
    """Return exactly 6 prox-zone dicts with the fields `GameRecording` expects.

    Args:
        raw: The `state.full` `teams.<t>.collision.prox_zones` value (a list
            of 6 dicts in normal operation).

    Returns:
        A 6-entry list; a missing/malformed source list is padded with
        ``valid: False`` placeholders so the recorded column is always
        exactly 6 entries long, matching `core.gameplay_recording`'s nested
        `prox_zones` schema.
    """

    zones: list[dict[str, Any]] = []
    if isinstance(raw, list):
        for item in raw[:6]:
            if isinstance(item, dict):
                zones.append(
                    {
                        "valid": bool(item.get("valid", False)),
                        "free_min_deg": item.get("free_min_deg"),
                        "free_max_deg": item.get("free_max_deg"),
                        "blocked_above_till_deg": item.get("blocked_above_till_deg"),
                        "blocked_below_till_deg": item.get("blocked_below_till_deg"),
                    }
                )
    while len(zones) < 6:
        zones.append(
            {
                "valid": False,
                "free_min_deg": None,
                "free_max_deg": None,
                "blocked_above_till_deg": None,
                "blocked_below_till_deg": None,
            }
        )
    return zones[:6]


def _coerce_floats(raw: Any, n: int = 6) -> list[float]:
    """Return exactly ``n`` floats from a possibly-missing/short source list."""

    values = [float(v) for v in raw[:n]] if isinstance(raw, list) else []
    while len(values) < n:
        values.append(0.0)
    return values[:n]


def _handle_haptic(
    rec_state: dict[str, Any], team: str, body: dict[str, Any], gear_ratio: list[float]
) -> None:
    """Buffer one `telem.haptic.<team>` sample, computing `dial_robot_deg`."""

    recording: GameRecording | None = rec_state["recording"]
    if recording is None:
        return
    ts_wall_ns = body.get("ts_wall_ns")
    if not isinstance(ts_wall_ns, int):
        return
    dial_pos_rad = _coerce_floats(body.get("dial_pos_rad"))
    recording.record_haptic(
        team,
        ts_wall_ns=ts_wall_ns,
        dial_pos_rad=dial_pos_rad,
        dial_vel_rad_s=_coerce_floats(body.get("dial_vel_rad_s")),
        torque_ma=_coerce_floats(body.get("torque_ma")),
        dial_robot_deg=[
            math.degrees(dial_pos_rad[i] * gear_ratio[i]) for i in range(6)
        ],
    )


def _handle_robot_actual(
    rec_state: dict[str, Any], team: str, body: dict[str, Any]
) -> None:
    """Buffer one `telem.robot.actual.<team>` sample."""

    recording: GameRecording | None = rec_state["recording"]
    if recording is None:
        return
    ts_wall_ns = body.get("ts_wall_ns")
    if not isinstance(ts_wall_ns, int):
        return
    robot_status = body.get("robot_status")
    if not isinstance(robot_status, dict):
        robot_status = {}
    fault_reason = robot_status.get("fault_reason")
    recording.record_robot_actual(
        team,
        ts_wall_ns=ts_wall_ns,
        q_rad=_coerce_floats(body.get("q_rad")),
        qd_rad_s=_coerce_floats(body.get("qd_rad_s")),
        fault_active=bool(robot_status.get("fault_active", False)),
        fault_reason=fault_reason if isinstance(fault_reason, str) else None,
    )


def _handle_weight(rec_state: dict[str, Any], body: dict[str, Any]) -> None:
    """Split one global `telem.weight` sample into each active team's 3 buckets."""

    recording: GameRecording | None = rec_state["recording"]
    if recording is None:
        return
    ts_wall_ns = body.get("ts_wall_ns")
    cells_g = body.get("cells_g")
    if not isinstance(ts_wall_ns, int) or not isinstance(cells_g, dict):
        return
    for team, cell_ids in _TEAM_BUCKET_CELL_IDS.items():
        values = [float(cells_g.get(cid, 0.0)) for cid in cell_ids]
        recording.record_weight(
            team,
            ts_wall_ns=ts_wall_ns,
            bucket_1_g=values[0],
            bucket_2_g=values[1],
            bucket_3_g=values[2],
        )


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
