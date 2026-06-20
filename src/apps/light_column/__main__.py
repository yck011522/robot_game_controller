"""Arena light-column I/O process entry point.

Subscribes to ``state.full`` from the game controller and drives the 16 arena
LED strips across three RS485 COM ports. Each tick:

1. drain to the most recent ``state.full`` snapshot,
2. recompute desired strip colors for the current game stage (memory only),
3. transmit one paced frame per COM port (round-robin over that port's strips).

Run standalone for bring-up (normally the launcher spawns it):

    $env:PYTHONPATH = "src"
    & C:/Users/yck01/miniconda3/envs/game/python.exe -m apps.light_column \
        --profile config/profiles/dev_team_a_led_integration.yaml \
        --proc light_column
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
from core.device_connection import require_serial_float  # noqa: E402
from core.proc import Proc, banner, parse_proc_args  # noqa: E402
from subsystems.light_column.controller import (  # noqa: E402
    LedColumnController,
    LightColumnConfig,
)
from subsystems.light_column.layout import load_light_column_layout  # noqa: E402
from subsystems.light_column.transport import (  # noqa: E402
    SERIAL_SETTINGS_KEY,
    LedTransport,
)

STATE_TOPIC = "state.full"
DEFAULT_TARGET_HZ = 200.0


def main(argv: list[str] | None = None) -> int:
    """Run the light-column controller against the configured RS485 buses."""

    args, _ = parse_proc_args(argv, default_proc="light_column")
    profile = load_profile(args.profile_path)

    impl = profile.subsystem_impl("light_column")
    if impl is None:
        print(f"[{args.proc}] light_column not enabled in profile", file=sys.stderr, flush=True)
        return 2
    if impl != "real":
        print(
            f"[{args.proc}] unsupported light_column impl {impl!r} (only 'real')",
            file=sys.stderr,
            flush=True,
        )
        return 2

    target_hz = default_runtime_setting("light_column", "fps_target", DEFAULT_TARGET_HZ)
    proc = Proc(args, profile, target_hz=target_hz or DEFAULT_TARGET_HZ)

    layout = load_light_column_layout()
    config = LightColumnConfig.from_profile(profile)
    # The on-wire spacing is owned by the device file; profiles never set it.
    config.inter_command_delay_s = max(
        0.0, require_serial_float(SERIAL_SETTINGS_KEY, "inter_command_delay_s", min_value=0.0)
    )
    transport = LedTransport(layout)
    controller = LedColumnController(transport, layout, config)

    state_sub = bus.make_sub(proc.ctx, topics=[STATE_TOPIC])
    banner(
        proc.proc,
        f"impl=real target_hz={proc.target_hz:.1f} ports={list(layout.serial_ports)}",
    )

    def setup(_: Proc) -> None:
        """Open the configured RS485 buses (best-effort, logs failures)."""

        if not transport.open():
            banner(proc.proc, "WARNING: no LED buses opened; frames will be dropped")

    def tick(p: Proc) -> None:
        """Drain to the latest state, recompute colors, and send paced frames."""

        latest = _drain_latest_state(state_sub)
        if latest is not None:
            controller.set_state(latest)
        now_mono = time.perf_counter()
        now_wall = time.time()
        controller.update(now_mono, now_wall)
        controller.pump(now_mono)

    def teardown(_: Proc) -> None:
        """Release the serial ports and the state subscription."""

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
