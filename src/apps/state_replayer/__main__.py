"""state_replayer - replay a recorded display-broadcast session over UDP.

Plays a ``.jsonl.gz`` session recording captured by ``apps.state_broadcaster``
(when a profile enables ``display_broadcast_recording``) back onto the network
as the same UDP datagrams the live broadcaster emits. Point it at a single Pi,
the subnet broadcast address, or localhost + ``apps.display_viewer`` to develop
the player-display UI offline -- no game controller, robots, or haptics needed.

It is a standalone tool: it does NOT join the ZMQ bus and is NOT spawned by the
launcher. The wire format and the file format are the shared modules
``core.display_protocol`` and ``core.state_recording``.

By default frames are sent at their originally recorded cadence (reconstructed
from each frame's ``ts_wall_ns``), so a replayed session looks identical in
timing to the live feed, including stage transitions, e-stops and pauses.

Run examples
------------
    $env:PYTHONPATH = "src"

    # Replay to a local display_viewer (start the viewer with --dest 127.0.0.1):
    & C:/Users/yck01/miniconda3/envs/game/python.exe -m apps.state_replayer \
        --file logs/display_broadcast_recording/20260621_184857_record_two_teams_session.jsonl.gz \
        --dest 127.0.0.1 --port 49200

    # Replay to the real Pi subnet at 2x speed, looping forever:
    & C:/Users/yck01/miniconda3/envs/game/python.exe -m apps.state_replayer \
        --file <recording.jsonl.gz> --dest 192.168.0.255 --speed 2.0 --loop

    # Use the dest/port from config/device_ports_and_addr.yaml (omit --dest/--port):
    & C:/Users/yck01/miniconda3/envs/game/python.exe -m apps.state_replayer \
        --file <recording.jsonl.gz>

Arguments
---------
--file   : path to the recording (required).
--dest   : UDP destination IP; default = display_broadcast.dest from the device
           config. Use 127.0.0.1 for a localhost viewer or a subnet broadcast
           address (e.g. 192.168.0.255) for the real Pis.
--port   : UDP port; default = display_broadcast.port from the device config.
--speed  : playback speed multiplier (1.0 = realtime, 2.0 = twice as fast,
           0.5 = half). Frame delays are divided by this value.
--max-gap-s : cap on the sleep between two frames (s); stops a long recording
           pause (operator away) from stalling playback. Default 1.0.
--start-at-s : start playback from this session-relative timestamp in seconds
           (for example 120.0 starts at t=+120 s in the recording).
--end-at-s : stop playback at this session-relative timestamp in seconds.
           When used with --loop, each pass replays only [start_at_s, end_at_s].
--player : player selector for stage-specific status details. Default is A1
           (aliases like team1player1 and t1p1 are accepted).
--loop   : restart from the beginning when the recording ends (Ctrl-C to stop).
"""

from __future__ import annotations

import argparse
import math
import re
import shutil
import signal
import socket
import sys
import time
from pathlib import Path
from typing import Any

_SRC = Path(__file__).resolve().parents[2]
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

from core.device_connection import load_display_broadcast  # noqa: E402
from core.display_protocol import encode_datagram  # noqa: E402
from core.state_recording import iter_frames, read_header  # noqa: E402


def _parse_args(argv: list[str] | None) -> argparse.Namespace:
    """Parse CLI arguments for the UDP state replayer."""

    ap = argparse.ArgumentParser(
        description="Replay a recorded display-broadcast session over UDP."
    )
    ap.add_argument("--file", required=True, help="Path to the .jsonl.gz recording.")
    ap.add_argument(
        "--dest",
        default=None,
        help="UDP destination IP (default: display_broadcast.dest from device config).",
    )
    ap.add_argument(
        "--port",
        type=int,
        default=None,
        help="UDP destination port (default: display_broadcast.port from device config).",
    )
    ap.add_argument(
        "--speed",
        type=float,
        default=1.0,
        help="Playback speed multiplier (1.0 = realtime).",
    )
    ap.add_argument(
        "--max-gap-s",
        type=float,
        default=1.0,
        help="Maximum sleep between frames in seconds (caps long recorded pauses).",
    )
    ap.add_argument(
        "--start-at-s",
        type=float,
        default=0.0,
        help="Start playback from this session-relative timestamp in seconds.",
    )
    ap.add_argument(
        "--end-at-s",
        type=float,
        default=None,
        help="Stop playback at this session-relative timestamp in seconds.",
    )
    ap.add_argument(
        "--player",
        default="team1player1",
        help="Player selector for status details (examples: a1, b4, team1player1, t2p3).",
    )
    ap.add_argument(
        "--loop",
        action="store_true",
        help="Loop the recording until interrupted.",
    )
    return ap.parse_args(argv)


def _resolve_endpoint(ns: argparse.Namespace) -> tuple[str, int]:
    """Resolve the UDP destination, preferring CLI flags over the device config."""

    dest = ns.dest
    port = ns.port
    if dest is None or port is None:
        endpoint = load_display_broadcast()
        dest = dest or endpoint.dest
        port = port or endpoint.port
    return str(dest), int(port)


def _parse_player(player_text: str) -> tuple[str, int, str]:
    """Return (team_key, zero_based_joint_index, normalized_label)."""

    text = str(player_text or "").strip().lower()
    m = re.fullmatch(r"([ab])\s*([1-6])", text)
    if m:
        team = m.group(1)
        number = int(m.group(2))
        return team, number - 1, f"{team}{number}"

    m = re.fullmatch(r"(?:team)?\s*([12])\s*(?:player|p)?\s*([1-6])", text)
    if m:
        team = "a" if m.group(1) == "1" else "b"
        number = int(m.group(2))
        return team, number - 1, f"{team}{number}"

    # Fallback required by request: default to Team A Player 1.
    return "a", 0, "a1"


def _team_block(state: dict[str, Any], team: str) -> dict[str, Any]:
    """Return the selected team block, tolerating missing/partial payloads."""

    teams = state.get("teams")
    if not isinstance(teams, dict):
        return {}
    block = teams.get(team)
    return block if isinstance(block, dict) else {}


def _array_value(seq: Any, index: int) -> Any:
    """Read one list/tuple value by index, returning None on mismatch."""

    if isinstance(seq, (list, tuple)) and 0 <= index < len(seq):
        return seq[index]
    return None


def _dial_deg(team_block: dict[str, Any], index: int) -> float | None:
    """Return dial angle in degrees for one player index, if present."""

    haptic = team_block.get("haptic")
    if not isinstance(haptic, dict):
        return None
    dial_deg = _array_value(haptic.get("dial_deg"), index)
    if isinstance(dial_deg, (int, float)):
        return float(dial_deg)
    dial_rad = _array_value(haptic.get("dial_pos_rad"), index)
    if isinstance(dial_rad, (int, float)):
        return math.degrees(float(dial_rad))
    return None


def _robot_joint_deg(team_block: dict[str, Any], index: int) -> float | None:
    """Return robot joint angle in degrees for one player index, if present."""

    robot = team_block.get("robot")
    if not isinstance(robot, dict):
        return None
    q_rad = _array_value(robot.get("q_rad"), index)
    if isinstance(q_rad, (int, float)):
        return math.degrees(float(q_rad))
    return None


def _fmt_num(value: Any, *, decimals: int = 1) -> str:
    """Format numeric values compactly for one-line status output."""

    if isinstance(value, (int, float)):
        return f"{float(value):.{decimals}f}"
    return "--"


def _fmt_buckets(team_block: dict[str, Any]) -> str:
    """Format first three bucket values as b=[x,y,z], tolerant of short arrays."""

    buckets = team_block.get("buckets")
    if not isinstance(buckets, list):
        return "b=[--,--,--]"
    vals: list[str] = []
    for i in range(3):
        vals.append(_fmt_num(_array_value(buckets, i), decimals=0))
    return f"b=[{vals[0]},{vals[1]},{vals[2]}]"


def _print_status(elapsed_s: float, state: dict[str, Any], player_label: str) -> None:
    """Overwrite one status line with selected player details by stage."""

    team, joint_index, normalized = _parse_player(player_label)
    team_block = _team_block(state, team)
    # active_stage is the lifecycle stage; stage may read "paused".
    active_stage = str(state.get("active_stage") or state.get("stage") or "?")
    timer = state.get("countdown_s")
    timer_str = f"{float(timer):4.0f}s" if isinstance(timer, (int, float)) else "  --"

    detail = ""
    if active_stage == "idle":
        detail = f"dial_deg={_fmt_num(_dial_deg(team_block, joint_index))}"
    elif active_stage == "tutorial":
        haptic = team_block.get("haptic") if isinstance(team_block.get("haptic"), dict) else {}
        progress = _array_value(haptic.get("tutorial_progress_pct") if isinstance(haptic, dict) else None, joint_index)
        detail = f"tutorial_pct={_fmt_num(progress)}"
    elif active_stage == "play":
        detail = (
            f"dial_deg={_fmt_num(_dial_deg(team_block, joint_index))} "
            f"joint_deg={_fmt_num(_robot_joint_deg(team_block, joint_index))} "
            f"{_fmt_buckets(team_block)}"
        )
    elif active_stage == "conclusion":
        total = team_block.get("summed_score")
        detail = f"{_fmt_buckets(team_block)} total={_fmt_num(total, decimals=0)}"

    line = (
        f"[state_replayer] t=+{elapsed_s:7.1f}s  stage={active_stage:<12} "
        f"player={normalized:<2} timer={timer_str} {detail}".rstrip()
    )

    # Clamp to terminal width so long status text does not soft-wrap into new
    # lines (which defeats carriage-return single-line updates in cmd.exe).
    width = shutil.get_terminal_size(fallback=(120, 30)).columns
    max_body = max(20, width - 1)
    if len(line) > max_body:
        line = line[: max(0, max_body - 3)] + "..."

    # Clear to end of line in a width-aware way before rewriting the status.
    clear_pad = " " * max(1, width - len(line))
    print("\r" + line + clear_pad, end="", flush=True)


def _play_once(
    udp: socket.socket,
    path: Path,
    dest: str,
    port: int,
    *,
    speed: float,
    max_gap_s: float,
    start_at_s: float,
    end_at_s: float | None,
    player: str,
    start_seq: int,
    stop: dict,
) -> tuple[int, int]:
    """Send every frame of one pass through the recording over UDP.

    Frames are paced from their recorded ``ts_wall_ns`` (scaled by ``speed``)
    against a monotonic clock anchored at the first frame, so cumulative timing
    does not drift even if individual sends run slightly long. Returns the next
    free datagram sequence number so a looped replay keeps the receiver's
    reorder guard monotonic across passes.
    """

    seq = start_seq
    base_wall_ns: int | None = None  # ts_wall_ns of first sent frame this pass
    base_mono: float | None = None  # perf_counter anchor for first sent frame
    record_origin_wall_ns: int | None = None  # ts_wall_ns of first frame in file
    speed = max(1e-6, speed)
    start_at_s = max(0.0, float(start_at_s))
    end_at_s = max(0.0, float(end_at_s)) if end_at_s is not None else None
    printed_status = False  # whether any status line was painted this pass
    sent_count = 0

    for frame in iter_frames(path):
        if stop["requested"]:
            break
        state = frame.get("state")
        if not isinstance(state, dict):
            continue
        ts_wall_ns = frame.get("ts_wall_ns")

        if record_origin_wall_ns is None and isinstance(ts_wall_ns, int):
            record_origin_wall_ns = ts_wall_ns

        record_elapsed_s = (
            (ts_wall_ns - record_origin_wall_ns) / 1e9
            if isinstance(ts_wall_ns, int) and record_origin_wall_ns is not None
            else 0.0
        )
        if record_elapsed_s < start_at_s:
            continue
        if end_at_s is not None and record_elapsed_s > end_at_s:
            # Frames are recorded in chronological order, so once we pass the
            # requested end bound this pass is complete.
            break

        if base_wall_ns is None or not isinstance(ts_wall_ns, int):
            base_wall_ns = ts_wall_ns if isinstance(ts_wall_ns, int) else None
            base_mono = time.perf_counter()
        else:
            # Target wall time for this frame relative to the pass start.
            target_offset_s = (ts_wall_ns - base_wall_ns) / 1e9 / speed
            assert base_mono is not None
            sleep_s = (base_mono + target_offset_s) - time.perf_counter()
            if sleep_s > max_gap_s:
                # A long recorded pause (e.g. operator away): advance the anchor
                # so the cap does not accumulate a permanent timing offset.
                sleep_s = max_gap_s
                base_mono = time.perf_counter() + max_gap_s - target_offset_s
            if sleep_s > 0:
                # Poll in small slices so Ctrl-C is honored within ~50 ms even
                # across a long inter-frame gap.
                end = time.perf_counter() + sleep_s
                while not stop["requested"] and time.perf_counter() < end:
                    time.sleep(min(0.05, max(0.0, end - time.perf_counter())))

        seq += 1
        payload = encode_datagram(state, seq, ts_wall_ns=time.time_ns())
        try:
            udp.sendto(payload, (dest, port))
        except OSError as exc:
            print(f"\n[state_replayer] WARNING: UDP send failed: {exc}", flush=True)

        # Session-relative timestamp = this frame's recorded time minus the
        # first frame's; 0 when the recording carried no wall clock.
        _print_status(record_elapsed_s, state, player)
        printed_status = True
        sent_count += 1

    if printed_status:
        # End the overwriting status line so later messages start fresh.
        print(flush=True)
    return seq, sent_count


def main(argv: list[str] | None = None) -> int:
    """Replay a recorded session to UDP, optionally looping, until interrupted."""

    ns = _parse_args(argv)
    path = Path(ns.file)
    if not path.exists():
        print(f"[state_replayer] recording not found: {path}", file=sys.stderr, flush=True)
        return 2

    dest, port = _resolve_endpoint(ns)
    start_at_s = max(0.0, float(ns.start_at_s))
    end_at_s = max(0.0, float(ns.end_at_s)) if ns.end_at_s is not None else None
    if end_at_s is not None and end_at_s < start_at_s:
        print(
            f"[state_replayer] ERROR: --end-at-s ({end_at_s}) must be >= --start-at-s ({start_at_s})",
            file=sys.stderr,
            flush=True,
        )
        return 2

    header = read_header(path)
    print(
        f"[state_replayer] file={path} profile={header.get('profile', '?')} "
        f"dest={dest}:{port} speed={ns.speed} start_at_s={start_at_s} "
        f"end_at_s={end_at_s if end_at_s is not None else '-'} "
        f"player={_parse_player(ns.player)[2]} loop={ns.loop}",
        flush=True,
    )

    stop = {"requested": False}

    def _request_stop(*_: object) -> None:
        stop["requested"] = True

    signal.signal(signal.SIGINT, _request_stop)
    signal.signal(signal.SIGTERM, _request_stop)
    if hasattr(signal, "SIGBREAK"):
        signal.signal(signal.SIGBREAK, _request_stop)  # type: ignore[attr-defined]

    # SO_BROADCAST lets a subnet broadcast destination work; harmless otherwise.
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    seq = 0
    passes = 0
    try:
        while not stop["requested"]:
            seq, sent_count = _play_once(
                udp,
                path,
                dest,
                port,
                speed=ns.speed,
                max_gap_s=ns.max_gap_s,
                start_at_s=start_at_s,
                end_at_s=end_at_s,
                player=ns.player,
                start_seq=seq,
                stop=stop,
            )
            if sent_count <= 0:
                print(
                    f"[state_replayer] WARNING: no frames in requested window start_at_s={start_at_s} end_at_s={end_at_s if end_at_s is not None else '-'}; stopping",
                    flush=True,
                )
                break
            passes += 1
            print(f"[state_replayer] pass {passes} complete ({seq} datagrams sent)", flush=True)
            if not ns.loop:
                break
    finally:
        udp.close()
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
