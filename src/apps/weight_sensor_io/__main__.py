"""Weight sensor I/O process entry point."""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Any

_SRC = Path(__file__).resolve().parents[2]
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

import zmq  # noqa: E402

from core import bus  # noqa: E402
from core.config import default_runtime_setting, load as load_profile  # noqa: E402
from core.device_connection import (  # noqa: E402
    load_serial_settings,
    require_serial_baudrate,
    require_serial_int_list,
    resolve_serial_ports,
)
from core.proc import Proc, banner, parse_proc_args  # noqa: E402
from subsystems.weight_sensor.common import LOAD_CELL_IDS, WeightSensorConfig  # noqa: E402
from subsystems.weight_sensor.real import RealLoadCellBus  # noqa: E402
from subsystems.weight_sensor.runtime import (  # noqa: E402
    DEFAULT_CLIENT_TIMEOUT_S,
    DEFAULT_TARGET_HZ,
    DEFAULT_TARE_CYCLES,
    WeightSensorRuntime,
)
from subsystems.weight_sensor.sim import SimLoadCellBus  # noqa: E402


SERIAL_PORTS_KEY = "weight_sensor"
SERIAL_SETTINGS_KEY = "weight_sensor"
TELEM_TOPIC = "telem.weight"
TARE_TOPIC = "cmd.weight.tare"


def main(argv: list[str] | None = None) -> int:
    """Run the configured weight sensor implementation."""

    args, _ = parse_proc_args(argv, default_proc="weight_sensor_io")
    profile = load_profile(args.profile_path)
    target_hz = profile.subsystem_float(
        "weight_sensor_io",
        "fps_target",
        default_runtime_setting("weight_sensor_io", "fps_target", DEFAULT_TARGET_HZ),
    )
    proc = Proc(args, profile, target_hz=target_hz or DEFAULT_TARGET_HZ)
    impl_name = proc.profile.subsystem_impl("weight_sensor_io")
    if impl_name is None:
        print(f"[{proc.proc}] no weight_sensor_io impl configured", file=sys.stderr, flush=True)
        return 2

    runtime = _make_runtime(impl_name)
    runtime.driver.connect()

    pub = bus.make_pub(proc.ctx)
    tare_sub = bus.make_sub(proc.ctx, topics=[TARE_TOPIC])
    proc.use_heartbeat_pub(pub)
    banner(proc.proc, f"impl={impl_name} target_hz={proc.target_hz:.1f}")

    seq = 0

    def setup(_: Proc) -> None:
        """Tare all sensors once on startup before publishing readings."""

        runtime.tare(samples=DEFAULT_TARE_CYCLES, reason="startup")

    def tick(p: Proc) -> None:
        """Handle tare commands, poll all sensors once, and publish telemetry."""

        nonlocal seq
        _drain_tare_commands(tare_sub, runtime=runtime)
        runtime.sample_cycle()
        env = bus.make_envelope(p.proc, with_wall=True, seq=seq)
        env.update(runtime.snapshot())
        bus.publish(pub, TELEM_TOPIC, env)
        seq += 1

    def teardown(_: Proc) -> None:
        """Close the serial client and ZMQ command socket."""

        runtime.driver.close()
        tare_sub.close(0)

    return proc.run(tick, setup=setup, teardown=teardown)


def _make_runtime(impl_name: str) -> WeightSensorRuntime:
    """Construct the real or simulated weight sensor runtime."""

    config = _load_weight_config()
    if impl_name == "sim":
        return WeightSensorRuntime(driver=SimLoadCellBus(config), config=config)
    if impl_name == "real":
        port_resolution = resolve_serial_ports(SERIAL_PORTS_KEY)
        if not port_resolution.ports:
            raise ValueError("serial_ports.weight_sensor must provide one COM port for real weight_sensor_io")
        driver = RealLoadCellBus(
            port=port_resolution.ports[0],
            baudrate=require_serial_baudrate(SERIAL_SETTINGS_KEY),
            timeout_s=DEFAULT_CLIENT_TIMEOUT_S,
            config=config,
        )
        return WeightSensorRuntime(driver=driver, config=config)
    raise NotImplementedError(f"weight_sensor_io impl {impl_name!r} is not available")


def _load_weight_config() -> WeightSensorConfig:
    """Load fixed load-cell slave IDs and conversion constants."""

    settings = load_serial_settings().get(SERIAL_SETTINGS_KEY, {})
    if "slave_addresses" in settings:
        slave_addresses = require_serial_int_list(SERIAL_SETTINGS_KEY, "slave_addresses", min_length=1)
    else:
        slave_addresses = LOAD_CELL_IDS
    return WeightSensorConfig(
        slave_addresses=tuple(int(slave) for slave in slave_addresses),
        zero_count=0.0,
        grams_per_count=1.0,
    )


def _drain_tare_commands(sub: zmq.Socket, *, runtime: WeightSensorRuntime) -> None:
    """Process every queued tare command in order."""

    while True:
        try:
            _, body = bus.recv(sub, flags=zmq.NOBLOCK)
        except zmq.Again:
            return
        reason = body.get("reason") if isinstance(body, dict) else None
        runtime.tare(samples=DEFAULT_TARE_CYCLES, reason=str(reason or "command"))


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
