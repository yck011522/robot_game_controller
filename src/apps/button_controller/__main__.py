"""Button controller process entry point.

Run directly:
    python -m apps.button_controller --profile config/profiles/two_teams.yaml
    python -m apps.button_controller --profile config/profiles/two_teams.yaml --proc button_controller

The process reads the HY-IO4400S-4NN admin-button unit on the configured
``admin_buttons`` serial port, publishes ``telem.buttons``, and drives the
green resume lamp from ``state.full.paused`` while the physical e-stop is clear.
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
from core.device_connection import (  # noqa: E402
    load_serial_settings,
    require_serial_baudrate,
    require_serial_float,
    require_serial_int,
    resolve_serial_ports,
)
from core.proc import Proc, banner, parse_proc_args  # noqa: E402
from subsystems.admin_buttons.common import (  # noqa: E402
    AdminButtonConfig,
    AdminButtonRuntime,
    snapshot_to_payload,
)
from subsystems.admin_buttons.real import RealAdminButtonUnit  # noqa: E402
from subsystems.admin_buttons.sim import SimAdminButtonUnit  # noqa: E402


DEFAULT_FPS_TARGET = 40.0
SERIAL_PORTS_KEY = "admin_buttons"
SERIAL_SETTINGS_KEY = "admin_buttons"
STATE_TOPIC = "state.full"
TELEM_TOPIC = "telem.buttons"


def main(argv: list[str] | None = None) -> int:
    """Run the configured physical/simulated button controller."""

    args, _ = parse_proc_args(argv, default_proc="button_controller")
    profile = load_profile(args.profile_path)
    target_hz = profile.subsystem_float(
        "button_controller",
        "fps_target",
        default_runtime_setting("button_controller", "fps_target", DEFAULT_FPS_TARGET),
    )
    proc = Proc(args, profile, target_hz=target_hz or DEFAULT_FPS_TARGET)
    impl_name = proc.profile.subsystem_impl("button_controller")
    if impl_name is None:
        print(f"[{proc.proc}] no button_controller impl configured", file=sys.stderr, flush=True)
        return 2

    runtime = _make_runtime(impl_name)
    runtime.driver.connect()

    pub = bus.make_pub(proc.ctx)
    state_sub = bus.make_sub(proc.ctx, topics=[STATE_TOPIC])
    proc.use_heartbeat_pub(pub)
    banner(proc.proc, f"impl={impl_name} target_hz={proc.target_hz:.1f}")

    seq = 0
    latest_paused = False  # Latest state.full pause flag; controls the green lamp.

    def tick(p: Proc) -> None:
        """Drain state, poll buttons once, update lamp, and publish telemetry."""

        nonlocal seq, latest_paused
        latest_paused = _drain_latest_pause(state_sub, latest_paused)
        snapshot = runtime.tick(paused=latest_paused, now_mono_s=time.perf_counter())
        env = bus.make_envelope(p.proc, with_wall=True, seq=seq)
        env.update(snapshot_to_payload(snapshot))
        bus.publish(pub, TELEM_TOPIC, env)
        seq += 1

    def teardown(_: Proc) -> None:
        """Turn off the lamp, close hardware, and close the state subscription."""

        runtime.close()
        state_sub.close(0)

    return proc.run(tick, teardown=teardown)


def _make_runtime(impl_name: str) -> AdminButtonRuntime:
    """Construct the real or simulated admin-button runtime."""

    config = _load_admin_button_config()
    if impl_name in ("sim", "sim_idle"):
        return AdminButtonRuntime(SimAdminButtonUnit(config), config)
    if impl_name == "real":
        port_resolution = resolve_serial_ports(SERIAL_PORTS_KEY)
        if not port_resolution.ports:
            raise ValueError("serial_ports.admin_buttons must provide one COM port for real button_controller")
        driver = RealAdminButtonUnit(
            port=port_resolution.ports[0],
            baudrate=require_serial_baudrate(SERIAL_SETTINGS_KEY),
            timeout_s=require_serial_float(
                SERIAL_SETTINGS_KEY,
                "read_timeout_s",
                min_value=0.0,
            ),
            config=config,
        )
        return AdminButtonRuntime(driver, config)
    raise NotImplementedError(f"button_controller impl {impl_name!r} is not available")


def _load_admin_button_config() -> AdminButtonConfig:
    """Load HY-IO4400S-4NN address, input mapping, coil, and debounce settings."""

    settings = load_serial_settings().get(SERIAL_SETTINGS_KEY, {})
    if not settings:
        raise ValueError("serial_settings.admin_buttons must be configured")
    return AdminButtonConfig(
        station_label=str(settings.get("station_label", "admin")),
        slave_address=require_serial_int(SERIAL_SETTINGS_KEY, "slave_address", min_value=1),
        input_start_address=require_serial_int(SERIAL_SETTINGS_KEY, "input_start_address", min_value=0),
        input_count=require_serial_int(SERIAL_SETTINGS_KEY, "input_count", min_value=3),
        resume_input_index=require_serial_int(SERIAL_SETTINGS_KEY, "resume_input_index", min_value=0),
        skip_input_index=require_serial_int(SERIAL_SETTINGS_KEY, "skip_input_index", min_value=0),
        estop_input_index=require_serial_int(SERIAL_SETTINGS_KEY, "estop_input_index", min_value=0),
        resume_lamp_coil_address=require_serial_int(
            SERIAL_SETTINGS_KEY,
            "resume_lamp_coil_address",
            min_value=0,
        ),
        skip_cooldown_s=require_serial_float(
            SERIAL_SETTINGS_KEY,
            "skip_cooldown_s",
            min_value=0.0,
        ),
    )


def _drain_latest_pause(sub: zmq.Socket, fallback: bool) -> bool:
    """Return the latest queued ``state.full.paused`` value, or the fallback."""

    paused = bool(fallback)
    while True:
        try:
            _, body = bus.recv(sub, flags=zmq.NOBLOCK)
        except zmq.Again:
            return paused
        paused = bool(body.get("paused", False))


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
