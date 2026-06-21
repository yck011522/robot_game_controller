"""state_broadcaster - UDP fan-out of the fat game state to player displays.

This process subscribes to ``state.full`` on the ZMQ bus and re-emits the most
recent snapshot as a single UDP datagram once per tick. Six Raspberry Pis (each
driving two player screens) listen on the configured broadcast address/port and
render their two player panels. UDP is best-effort: a dropped packet just means
a Pi reuses its previous frame, which matches the installation's loss tolerance.

The datagram wire format lives in ``core.display_protocol`` and is shared with
``apps.display_viewer`` (the local mock receiver / on-Pi client).

Configuration
-------------
* Destination address + UDP port: ``display_broadcast`` block in
  ``config/device_ports_and_addr.yaml`` (overridable per run with --dest/--port).
* Poll rate: ``subsystems.state_broadcaster.fps_target`` in
  ``config/runtime.yaml``. The broadcaster is event-driven -- it drains every
  new ``state.full`` and emits one datagram per state, so this only sets how
  often it wakes to check the bus (kept above the 60 Hz state rate for low
  latency). Unchanged duplicates are never re-sent.
* Session recording: optional ``display_broadcast_recording`` block in the
  loaded **profile** (``enabled: true`` + ``dir``). When present the broadcaster
  writes every broadcast frame to a timestamped ``.jsonl.gz`` under that folder for
  the whole run, then seals it on shutdown (e.g. ESC in the dashboard). Replay
  it offline over UDP with ``apps.state_replayer``.

Run standalone (the launcher also spawns it as tier 8):

    $env:PYTHONPATH = "src"
    # Use config defaults (broadcast to the robot/Pi subnet):
    & C:/Users/yck01/miniconda3/envs/game/python.exe -m apps.state_broadcaster \
        --profile config/profiles/dev_team_a_led_integration.yaml \
        --proc state_broadcaster

    # Localhost dev loop with a local display_viewer:
    & C:/Users/yck01/miniconda3/envs/game/python.exe -m apps.state_broadcaster \
        --profile config/profiles/dev_team_a_led_integration.yaml \
        --proc state_broadcaster --dest 127.0.0.1 --port 49200
"""

from __future__ import annotations

import argparse
import socket
import sys
import time
from pathlib import Path

_SRC = Path(__file__).resolve().parents[2]
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

import zmq  # noqa: E402

from core import bus  # noqa: E402
from core.config import Profile, default_runtime_setting, load as load_profile  # noqa: E402
from core.device_connection import load_display_broadcast  # noqa: E402
from core.display_protocol import encode_datagram  # noqa: E402
from core.proc import Proc, banner, parse_proc_args  # noqa: E402
from core.state_recording import RecordingWriter, default_recording_path  # noqa: E402

# Topic carrying the fat game state we fan out verbatim.
STATE_TOPIC = "state.full"
# Fallback poll rate if runtime.yaml lacks an entry (Hz). Kept above the 60 Hz
# state.full rate so each new state is picked up within a few milliseconds.
DEFAULT_TARGET_HZ = 120.0
# Default folder for session recordings when the profile omits an explicit dir.
DEFAULT_RECORDING_DIR = "logs/display_broadcast_recording"
# Warn once if a datagram exceeds this size; large payloads still send but are
# more likely to be dropped because they span several IP fragments. 8 KB keeps
# us within a handful of 1500-byte-MTU fragments on the LAN.
_SIZE_WARN_BYTES = 8192


def _add_cli_overrides(ap: argparse.ArgumentParser) -> None:
    """Register the optional --dest/--port overrides for ad-hoc runs."""

    ap.add_argument(
        "--dest",
        default=None,
        help="Override the UDP destination address from device_ports_and_addr.yaml.",
    )
    ap.add_argument(
        "--port",
        type=int,
        default=None,
        help="Override the UDP destination port from device_ports_and_addr.yaml.",
    )
    ap.add_argument(
        "--no-record",
        action="store_true",
        help="Disable session recording even if the profile enables it.",
    )


def _resolve_recording_config(profile: Profile) -> tuple[bool, str]:
    """Return ``(enabled, output_dir)`` from the profile recording block.

    Reads the optional top-level ``display_broadcast_recording`` mapping from
    the loaded profile. A missing block (or ``enabled: false``) means "do not
    record", so normal play profiles are unaffected and only a profile that
    intentionally opts in captures a session.
    """

    node = profile.raw.get("display_broadcast_recording")
    if not isinstance(node, dict):
        return False, DEFAULT_RECORDING_DIR
    enabled = bool(node.get("enabled", False))
    directory = str(node.get("dir") or DEFAULT_RECORDING_DIR)
    return enabled, directory


def main(argv: list[str] | None = None) -> int:
    """Subscribe to state.full and broadcast each tick as a UDP datagram."""

    args, ns = parse_proc_args(
        argv, default_proc="state_broadcaster", extra=_add_cli_overrides
    )
    profile = load_profile(args.profile_path)

    # Resolve the broadcast endpoint; CLI flags win over the device file so a
    # developer can redirect the feed to localhost without editing config.
    endpoint = load_display_broadcast()
    dest_addr = ns.dest or endpoint.dest  # destination IP for sendto()
    dest_port = ns.port or endpoint.port  # destination UDP port

    target_hz = default_runtime_setting(
        "state_broadcaster", "fps_target", DEFAULT_TARGET_HZ
    )
    proc = Proc(args, profile, target_hz=target_hz or DEFAULT_TARGET_HZ)

    state_sub = bus.make_sub(proc.ctx, topics=[STATE_TOPIC])

    # UDP sender. SO_BROADCAST is required to send to a (subnet-directed or
    # limited) broadcast address; it is harmless for unicast/loopback targets.
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    record_enabled, record_dir = _resolve_recording_config(profile)
    if ns.no_record:
        record_enabled = False

    # Per-process state held in a dict so the nested tick() can mutate it.
    send_state = {
        "seq": 0,  # monotonic datagram counter handed to the receiver
        "last_state_seq": -1,  # highest state.full envelope seq already sent
        "warned_size": False,  # one-shot oversize-datagram warning latch
        "recorder": None,  # RecordingWriter when recording is enabled, else None
    }

    banner(
        proc.proc,
        f"udp dest={dest_addr}:{dest_port} target_hz={proc.target_hz:.1f} "
        f"record={'on' if record_enabled else 'off'}",
    )

    def setup(_p: Proc) -> None:
        """Open the session recording file when recording is enabled."""

        if not record_enabled:
            return
        path = default_recording_path(record_dir, profile.name)
        meta = {
            "profile": profile.name,
            "dest": dest_addr,
            "port": dest_port,
            "target_hz": proc.target_hz,
            "state_topic": STATE_TOPIC,
        }
        try:
            send_state["recorder"] = RecordingWriter(path, meta=meta)
            banner(proc.proc, f"recording session to {path}")
        except OSError as exc:
            # A failed recorder must not take the live broadcast down; log and
            # continue fanning out state to the displays.
            banner(proc.proc, f"WARNING: could not open recording: {exc}")
            send_state["recorder"] = None

    def tick(_p: Proc) -> None:
        """Broadcast (and optionally record) every new state.full as a datagram."""

        for body in _drain_states(state_sub):
            state_seq = body.get("seq")
            if isinstance(state_seq, int):
                if state_seq <= send_state["last_state_seq"]:
                    # Duplicate or out-of-order envelope; skip so the displays
                    # and the recording stay 1:1 with the game controller.
                    continue
                send_state["last_state_seq"] = state_seq
            send_state["seq"] += 1
            now_ns = time.time_ns()
            payload = encode_datagram(body, send_state["seq"], ts_wall_ns=now_ns)
            if len(payload) > _SIZE_WARN_BYTES and not send_state["warned_size"]:
                banner(
                    proc.proc,
                    f"WARNING: datagram is {len(payload)} bytes (> {_SIZE_WARN_BYTES}); "
                    "fragmentation raises drop probability",
                )
                send_state["warned_size"] = True
            try:
                udp.sendto(payload, (dest_addr, dest_port))
            except OSError as exc:
                # Never crash on a transient network error; the next state is
                # broadcast on the following tick.
                banner(proc.proc, f"WARNING: UDP send failed: {exc}")
            recorder = send_state["recorder"]
            if recorder is not None:
                # Record the controller's own seq/wall-clock so replay can
                # reproduce the exact original cadence.
                src_seq = state_seq if isinstance(state_seq, int) else send_state["seq"]
                src_ts = body.get("ts_wall_ns")
                recorder.append(
                    body, src_seq, int(src_ts) if isinstance(src_ts, int) else now_ns
                )

    def teardown(_p: Proc) -> None:
        """Seal the recording, then close the UDP socket and subscription."""

        recorder = send_state["recorder"]
        if recorder is not None:
            recorder.close()
            banner(proc.proc, f"recording closed ({recorder.frame_count} frames)")
        udp.close()
        state_sub.close(0)

    return proc.run(tick, setup=setup, teardown=teardown)


def _drain_states(sub: zmq.Socket) -> list[dict]:
    """Return every queued ``state.full`` body in arrival order.

    Draining the whole queue each tick (instead of keeping only the latest)
    guarantees no state is dropped between wakeups, so both the display feed and
    the recording stay 1:1 with the game controller's publish stream.
    """

    bodies: list[dict] = []
    while True:
        try:
            _, body = bus.recv(sub, flags=zmq.NOBLOCK)
        except zmq.Again:
            return bodies
        if isinstance(body, dict):
            bodies.append(body)


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
