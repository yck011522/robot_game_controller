"""Bucket controller process entry point."""

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
    resolve_serial_ports,
)
from core.proc import Proc, banner, parse_proc_args  # noqa: E402
from subsystems.bucket.common import (  # noqa: E402
    BUCKET_LABELS,
    BucketMotorConfig,
)
from subsystems.bucket.controller import BucketControllerRuntime  # noqa: E402
from subsystems.bucket.controller import (  # noqa: E402
    DEFAULT_CLIENT_TIMEOUT_S,
    DEFAULT_CLOSE_DIRECTION,
    DEFAULT_COMMAND_TIMEOUT_S,
    DEFAULT_INTER_REQUEST_DELAY_S,
    DEFAULT_MOTOR_SPEED,
    DEFAULT_OPEN_DIRECTION,
    DEFAULT_STATUS_POLL_HZ,
)
from subsystems.bucket.real import RealBucketMotorBus  # noqa: E402
from subsystems.bucket.sim import SimBucketMotorBus  # noqa: E402


DEFAULT_FPS_TARGET = 20.0
SERIAL_PORTS_KEY = "bucket_motors"
SERIAL_SETTINGS_KEY = "bucket_motors"
CMD_TOPIC = "cmd.bucket"
TELEM_TOPIC = "telem.bucket"


def main(argv: list[str] | None = None) -> int:
    """Run the configured bucket controller implementation."""

    args, _ = parse_proc_args(argv, default_proc="bucket_controller")
    profile = load_profile(args.profile_path)
    target_hz = profile.subsystem_float(
        "bucket_controller",
        "fps_target",
        default_runtime_setting("bucket_controller", "fps_target", DEFAULT_FPS_TARGET),
    )
    proc = Proc(args, profile, target_hz=target_hz or DEFAULT_FPS_TARGET)
    impl_name = proc.profile.subsystem_impl("bucket_controller")
    if impl_name is None:
        print(f"[{proc.proc}] no bucket_controller impl configured", file=sys.stderr, flush=True)
        return 2

    runtime = _make_runtime(impl_name, profile=proc.profile)
    runtime.driver.connect()

    pub = bus.make_pub(proc.ctx)
    cmd_sub = bus.make_sub(proc.ctx, topics=[CMD_TOPIC])
    proc.use_heartbeat_pub(pub)
    banner(proc.proc, f"impl={impl_name} target_hz={proc.target_hz:.1f}")

    seq = 0

    def tick(p: Proc) -> None:
        """Process queued commands, run watchdog/status logic, and publish telemetry."""

        nonlocal seq
        _drain_commands(cmd_sub, runtime=runtime)
        runtime.tick()
        env = bus.make_envelope(p.proc, with_wall=True, seq=seq)
        env.update(runtime.snapshot())
        bus.publish(pub, TELEM_TOPIC, env)
        seq += 1

    def teardown(_: Proc) -> None:
        """Stop all motors before closing the serial client and sockets."""

        runtime.stop_all(message="process teardown stop_all")
        runtime.driver.close()
        cmd_sub.close(0)

    return proc.run(tick, teardown=teardown)


def _make_runtime(impl_name: str, *, profile: Any) -> BucketControllerRuntime:
    """Construct the real or simulated bucket runtime selected by profile."""

    config = _load_bucket_config(profile)
    if impl_name == "sim":
        return BucketControllerRuntime(driver=SimBucketMotorBus(config), config=config)
    if impl_name == "real":
        port_resolution = resolve_serial_ports(SERIAL_PORTS_KEY)
        if not port_resolution.ports:
            raise ValueError("serial_ports.bucket_motors must provide one COM port for real bucket_controller")
        driver = RealBucketMotorBus(
            port=port_resolution.ports[0],
            baudrate=require_serial_baudrate(SERIAL_SETTINGS_KEY),
            timeout_s=DEFAULT_CLIENT_TIMEOUT_S,
            config=config,
        )
        return BucketControllerRuntime(driver=driver, config=config)
    raise NotImplementedError(f"bucket_controller impl {impl_name!r} is not available")


def _load_bucket_config(profile: Any) -> BucketMotorConfig:
    """Load hardware addresses plus profile-tuned speed into runtime config."""

    settings = load_serial_settings().get(SERIAL_SETTINGS_KEY, {})
    addresses = _load_addresses(settings)
    motor_speed = _profile_motor_speed(profile)
    return BucketMotorConfig(
        addresses=addresses,
        open_direction=DEFAULT_OPEN_DIRECTION,
        close_direction=DEFAULT_CLOSE_DIRECTION,
        speed=motor_speed,
        command_timeout_s=DEFAULT_COMMAND_TIMEOUT_S,
        status_poll_interval_s=1.0 / DEFAULT_STATUS_POLL_HZ,
        inter_request_delay_s=DEFAULT_INTER_REQUEST_DELAY_S,
    )


def _load_addresses(settings: dict[str, Any]) -> dict[str, int]:
    """Load and validate the fixed logical-bucket to Modbus-address map."""

    raw = settings.get("addresses")
    if not isinstance(raw, dict):
        raise ValueError("serial_settings.bucket_motors.addresses must be a mapping")
    addresses: dict[str, int] = {}
    for label in BUCKET_LABELS:
        if label not in raw:
            raise ValueError(f"serial_settings.bucket_motors.addresses.{label} is required")
        try:
            addresses[label] = int(raw[label])
        except (TypeError, ValueError) as exc:
            raise ValueError(f"serial_settings.bucket_motors.addresses.{label} must be an integer") from exc
    return addresses


def _profile_motor_speed(profile: Any) -> int:
    """Load the operator-tunable bucket motor speed from the active profile."""

    tuning = profile.tuning.get("bucket_controller") if isinstance(profile.tuning, dict) else {}
    raw_speed = tuning.get("motor_speed") if isinstance(tuning, dict) else None
    try:
        speed = int(raw_speed if raw_speed is not None else DEFAULT_MOTOR_SPEED)
    except (TypeError, ValueError):
        speed = DEFAULT_MOTOR_SPEED
    return max(1, min(15, speed))


def _drain_commands(sub: zmq.Socket, *, runtime: BucketControllerRuntime) -> None:
    """Process every queued bucket command in order."""

    while True:
        try:
            _, body = bus.recv(sub, flags=zmq.NOBLOCK)
        except zmq.Again:
            return
        runtime.handle_command(body)


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
