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
* Send rate: ``subsystems.state_broadcaster.fps_target`` in
  ``config/runtime.yaml`` (defaults to 60 Hz, matching state.full).

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
from core.config import default_runtime_setting, load as load_profile  # noqa: E402
from core.device_connection import load_display_broadcast  # noqa: E402
from core.display_protocol import encode_datagram  # noqa: E402
from core.proc import Proc, banner, parse_proc_args  # noqa: E402

# Topic carrying the fat game state we fan out verbatim.
STATE_TOPIC = "state.full"
# Fallback send rate if runtime.yaml lacks an entry (Hz).
DEFAULT_TARGET_HZ = 60.0
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

    # Per-process state held in a dict so the nested tick() can mutate it.
    send_state = {
        "seq": 0,  # monotonic datagram counter handed to the receiver
        "latest": None,  # most recent state.full body, or None until first rx
        "warned_size": False,  # one-shot oversize-datagram warning latch
    }

    banner(
        proc.proc,
        f"udp dest={dest_addr}:{dest_port} target_hz={proc.target_hz:.1f}",
    )

    def tick(_p: Proc) -> None:
        """Drain to the newest state.full and broadcast it as one datagram."""

        latest = _drain_latest_state(state_sub)
        if latest is not None:
            send_state["latest"] = latest
        body = send_state["latest"]
        if body is None:
            # No state.full seen yet (controller still booting); nothing to send.
            return
        send_state["seq"] += 1
        payload = encode_datagram(body, send_state["seq"], ts_wall_ns=time.time_ns())
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
            # Never crash the process on a transient network error; the next
            # tick retries with the latest snapshot.
            banner(proc.proc, f"WARNING: UDP send failed: {exc}")

    def teardown(_p: Proc) -> None:
        """Close the UDP socket and the state subscription."""

        udp.close()
        state_sub.close(0)

    return proc.run(tick, teardown=teardown)


def _drain_latest_state(sub: zmq.Socket) -> dict | None:
    """Return the most recent ``state.full`` body, discarding older queued ones."""

    latest: dict | None = None
    while True:
        try:
            _, body = bus.recv(sub, flags=zmq.NOBLOCK)
        except zmq.Again:
            return latest
        if isinstance(body, dict):
            latest = body


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
