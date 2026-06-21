"""Per-bucket LED scoreboard I/O process entry point.

Subscribes to ``state.full`` from the game controller and drives the six
per-bucket LED text panels over a single RS485/USB-serial port (COM40). Each
tick:

1. drain to the most recent ``state.full`` snapshot,
2. recompute desired per-panel text for the current game stage (memory only),
3. flush any changed text commands to the serial port.

The panels are NVS-backed and hold their last content, so the controller only
sends a command when a panel's desired text/mode/enable actually changes.

Run standalone for bring-up (normally the launcher spawns it):

    # PowerShell, from the repo root, with the project's conda env:
    $env:PYTHONPATH = "src"
    & C:/Users/yck01/miniconda3/envs/game/python.exe -m apps.scoreboard_broadcaster `
        --profile config/profiles/dev_two_teams_led_integration.yaml `
        --proc scoreboard_broadcaster

Typical invocations:
    # Two-team real-hardware profile (default launch profile):
    ... -m apps.scoreboard_broadcaster --profile config/profiles/dev_two_teams_led_integration.yaml --proc scoreboard_broadcaster
    # Single-team (Team A) LED-integration profile:
    ... -m apps.scoreboard_broadcaster --profile config/profiles/dev_team_a_led_integration.yaml --proc scoreboard_broadcaster
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

_SRC = Path(__file__).resolve().parents[2]
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

import zmq  # noqa: E402

from core import bus  # noqa: E402
from core.config import default_runtime_setting, load as load_profile  # noqa: E402
from core.proc import Proc, banner, parse_proc_args  # noqa: E402
from subsystems.scoreboard.controller import (  # noqa: E402
    ScoreboardConfig,
    ScoreboardController,
)
from subsystems.scoreboard.layout import load_scoreboard_layout  # noqa: E402
from subsystems.scoreboard.transport import ScoreboardTransport  # noqa: E402

# Bus topic carrying the authoritative game snapshot we consume.
STATE_TOPIC = "state.full"
# Fallback tick rate if config/runtime.yaml has no scoreboard_broadcaster entry.
DEFAULT_TARGET_HZ = 30.0


def main(argv: list[str] | None = None) -> int:
    """Run the scoreboard controller against the configured RS485 panel string."""

    args, _ = parse_proc_args(argv, default_proc="scoreboard_broadcaster")
    profile = load_profile(args.profile_path)

    impl = profile.subsystem_impl("scoreboard_broadcaster")
    if impl is None:
        print(
            f"[{args.proc}] scoreboard_broadcaster not enabled in profile",
            file=sys.stderr,
            flush=True,
        )
        return 2
    if impl != "real":
        print(
            f"[{args.proc}] unsupported scoreboard_broadcaster impl {impl!r} (only 'real')",
            file=sys.stderr,
            flush=True,
        )
        return 2

    target_hz = default_runtime_setting(
        "scoreboard_broadcaster", "fps_target", DEFAULT_TARGET_HZ
    )
    proc = Proc(args, profile, target_hz=target_hz or DEFAULT_TARGET_HZ)

    layout = load_scoreboard_layout()
    config = ScoreboardConfig.from_profile(profile)
    transport = ScoreboardTransport(layout)
    controller = ScoreboardController(transport, layout, config)

    state_sub = bus.make_sub(proc.ctx, topics=[STATE_TOPIC])
    banner(
        proc.proc,
        f"impl=real target_hz={proc.target_hz:.1f} port={layout.port} "
        f"displays={list(layout.all_displays)}",
    )

    def setup(_: Proc) -> None:
        """Open the COM port and force every panel to a known startup state.

        After opening, ``initialize()`` queues a max-brightness + static-mode +
        blank baseline for all panels (the panels are NVS-backed, so a prior run
        could leave them dim or scrolling); the queued lines go out on the first
        ``pump`` in the tick below.
        """

        if not transport.open():
            banner(proc.proc, "WARNING: scoreboard port not opened; commands dropped")
        controller.initialize()

    def tick(p: Proc) -> None:
        """Drain to the latest state, recompute panel text, and flush commands."""

        latest = _drain_latest_state(state_sub)
        if latest is not None:
            controller.set_state(latest)
        now_mono = time.perf_counter()
        controller.update(now_mono)
        controller.pump(now_mono)

    def teardown(_: Proc) -> None:
        """Release the serial port and the state subscription."""

        transport.close()
        state_sub.close(0)

    return proc.run(tick, setup=setup, teardown=teardown)


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
