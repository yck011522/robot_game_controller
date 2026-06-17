"""Safety barrier controller process entry point."""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Any

_SRC = Path(__file__).resolve().parents[2]
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

from core import bus  # noqa: E402
from core.com_ports import load_serial_settings, resolve_serial_ports  # noqa: E402
from core.config import default_runtime_setting, load as load_profile  # noqa: E402
from core.proc import Proc, banner, parse_proc_args  # noqa: E402
from subsystems.safety_barrier.common import (  # noqa: E402
    SafetyBarrierConfig,
    SafetyBarrierSnapshot,
    resolve_safety_barrier_config,
)
from subsystems.safety_barrier.real import RealModbusSafetyBarrier, RealSafetyBarrierTransport  # noqa: E402
from subsystems.safety_barrier.sim import SimOpenSafetyBarrier  # noqa: E402


DEFAULT_FPS_TARGET = 18.0
SERIAL_SETTINGS_KEY = "safety_barrier"
TELEM_TOPIC = "telem.safety"


def main(argv: list[str] | None = None) -> int:
    """Run the configured safety barrier controller implementation."""

    args, _ = parse_proc_args(argv, default_proc="safety_barrier_controller")
    profile = load_profile(args.profile_path)
    target_hz = profile.subsystem_float(
        "safety_barrier_controller",
        "fps_target",
        default_runtime_setting("safety_barrier_controller", "fps_target", DEFAULT_FPS_TARGET),
    )
    proc = Proc(args, profile, target_hz=target_hz or DEFAULT_FPS_TARGET)

    impl_name = proc.profile.subsystem_impl("safety_barrier_controller")
    if impl_name is None:
        print(f"[{proc.proc}] no safety_barrier_controller impl configured", file=sys.stderr, flush=True)
        return 2

    controller = _make_controller(impl_name, proc.profile)
    pub = bus.make_pub(proc.ctx)
    proc.use_heartbeat_pub(pub)
    banner(proc.proc, f"impl={impl_name} target_hz={proc.target_hz:.1f}")

    seq = 0

    def tick(p: Proc) -> None:
        nonlocal seq
        snapshot = controller.read()
        env = _snapshot_to_envelope(p.proc, snapshot, seq)
        bus.publish(pub, TELEM_TOPIC, env)
        seq += 1

    def teardown(_: Proc) -> None:
        controller.close()

    return proc.run(tick, teardown=teardown)


def _make_controller(impl_name: str, profile: Any):
    """Construct the selected safety barrier implementation."""

    config = _load_channel_config(profile)
    if impl_name == "sim_open":
        return SimOpenSafetyBarrier(config)
    if impl_name == "real":
        transport = _load_transport(profile)
        return RealModbusSafetyBarrier(transport, config)
    if impl_name == "sim_random":
        raise NotImplementedError("safety_barrier_controller sim_random is not implemented yet")
    raise NotImplementedError(f"safety_barrier_controller impl {impl_name!r} is not available")


def _load_channel_config(profile: Any) -> SafetyBarrierConfig:
    """Load channel labels from com_ports.yaml and bypass policy from the profile."""

    settings = load_serial_settings().get(SERIAL_SETTINGS_KEY, {})
    channel_order = settings.get("channel_order")
    if not isinstance(channel_order, list):
        raise ValueError("serial_settings.safety_barrier.channel_order must be a list")
    safety_tuning = profile.tuning.get("safety_barrier") if isinstance(profile.tuning, dict) else {}
    bypass_channels = safety_tuning.get("bypass_channels") if isinstance(safety_tuning, dict) else {}
    if bypass_channels is not None and not isinstance(bypass_channels, dict):
        raise ValueError("tuning.safety_barrier.bypass_channels must be a mapping")
    return resolve_safety_barrier_config(
        channel_order=channel_order,
        bypass_channels=bypass_channels,
    )


def _load_transport(profile: Any) -> RealSafetyBarrierTransport:
    """Load Modbus serial settings for the real safety barrier hardware."""

    port_resolution = resolve_serial_ports(profile, "safety_barrier")
    if not port_resolution.ports:
        raise ValueError("serial_ports.safety_barrier must provide one COM port for real safety barrier")
    settings = load_serial_settings().get(SERIAL_SETTINGS_KEY, {})
    addresses = settings.get("slave_addresses")
    if not isinstance(addresses, list) or not addresses:
        raise ValueError("serial_settings.safety_barrier.slave_addresses must be a non-empty list")
    return RealSafetyBarrierTransport(
        port=port_resolution.ports[0],
        baudrate=int(settings.get("baudrate", 115200)),
        slave_addresses=tuple(int(address) for address in addresses),
        read_timeout_s=float(settings.get("read_timeout_s", 0.070)),
        inter_request_delay_s=float(settings.get("inter_request_delay_s", 0.006)),
    )


def _snapshot_to_envelope(producer: str, snapshot: SafetyBarrierSnapshot, seq: int) -> dict[str, Any]:
    """Convert one safety snapshot into the BUS.md telem.safety payload."""

    env = bus.make_envelope(producer, with_wall=True, seq=seq)
    env.update({
        "ok": snapshot.ok,
        "channels": snapshot.channels,
        "effective_channels": snapshot.effective_channels,
        "channel_labels": snapshot.labels,
        "bypass_channels": snapshot.bypass_channels,
        "errors": snapshot.errors,
    })
    return env


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
