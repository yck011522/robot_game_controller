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
--loop   : restart from the beginning when the recording ends (Ctrl-C to stop).
"""

from __future__ import annotations

import argparse
import signal
import socket
import sys
import time
from pathlib import Path

_SRC = Path(__file__).resolve().parents[2]
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

from core.device_connection import load_display_broadcast  # noqa: E402
from core.display_protocol import encode_datagram  # noqa: E402
from core.state_recording import iter_frames, read_header  # noqa: E402


def _parse_args(argv: list[str] | None) -> argparse.Namespace:
    """Parse the replayer CLI arguments."""

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


def _print_status(elapsed_s: float, state: dict) -> None:
    """Overwrite a single console line with the latest replayed frame.

    Shows the session-relative timestamp (seconds since the recording's first
    frame), the current game state, and the stage timer pulled from the
    ``state.full`` body. Uses a carriage return (no newline) so successive
    frames repaint the same line instead of scrolling the console.
    """

    # `stage` is "paused" while paused, otherwise the active stage name;
    # fall back to active_stage / "?" so a partial frame still prints.
    stage = state.get("stage") or state.get("active_stage") or "?"
    timer = state.get("countdown_s")
    timer_str = f"{float(timer):4.0f}s" if isinstance(timer, (int, float)) else "  --"
    line = (
        f"[state_replayer] t=+{elapsed_s:7.1f}s  stage={str(stage):<12} timer={timer_str}"
    )
    # Trailing spaces clear any leftover characters from a longer prior line.
    print("\r" + line + "    ", end="", flush=True)


def _play_once(
    udp: socket.socket,
    path: Path,
    dest: str,
    port: int,
    *,
    speed: float,
    max_gap_s: float,
    start_at_s: float,
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
        _print_status(record_elapsed_s, state)
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
    header = read_header(path)
    print(
        f"[state_replayer] file={path} profile={header.get('profile', '?')} "
        f"dest={dest}:{port} speed={ns.speed} start_at_s={ns.start_at_s} loop={ns.loop}",
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
                start_at_s=ns.start_at_s,
                start_seq=seq,
                stop=stop,
            )
            if sent_count <= 0:
                print(
                    f"[state_replayer] WARNING: no frames at/after start_at_s={ns.start_at_s}; stopping",
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
