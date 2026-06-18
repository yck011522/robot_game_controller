"""Focused tests for the P5 single-dial haptic backend.

Validates two things without requiring physical ESP32 boards:
1. The real backend discovers dial IDs 21-26, seeds position, and emits
   high-rate `R` / `C` / `S` serial commands.
2. Profile validation rejects malformed `hardware.serial_ports.haptic_*`
   values before the launcher ever spawns the process.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace

REPO_ROOT = Path(__file__).resolve().parent.parent
SRC = REPO_ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from core import com_ports  # noqa: E402
from core.config import ConfigError, load as load_profile  # noqa: E402
from led_serial import LEDSystem  # noqa: E402
from subsystems.haptic.real import (  # noqa: E402
    RealHaptic,
    _build_enable_lines,
    _robot_bounds_to_dial_bounds_rad,
)


class _FakeSerial:
    def __init__(
        self,
        scripts_by_port: dict[str, str],
        writes_by_port: dict[str, list[str]],
        baudrates_by_port: dict[str, int] | None = None,
    ):
        self._scripts_by_port = scripts_by_port
        self._writes_by_port = writes_by_port
        self._baudrates_by_port = baudrates_by_port
        self.port = ""
        self.baudrate = 0
        self.timeout = 0.0
        self.write_timeout = 0.0
        self.dtr = False
        self.rts = False
        self.is_open = False
        self._buffer = bytearray()

    @property
    def in_waiting(self) -> int:
        return len(self._buffer)

    def open(self) -> None:
        self.is_open = True
        if self._baudrates_by_port is not None:
            self._baudrates_by_port[self.port] = self.baudrate
        script = self._scripts_by_port.get(self.port, "")
        self._buffer.extend(script.encode("ascii"))

    def close(self) -> None:
        self.is_open = False

    def reset_input_buffer(self) -> None:
        pass

    def read(self, size: int = 1) -> bytes:
        if size <= 0:
            return b""
        chunk = bytes(self._buffer[:size])
        del self._buffer[:size]
        return chunk

    def write(self, payload: bytes) -> int:
        line = payload.decode("ascii").strip()
        self._writes_by_port.setdefault(self.port, []).append(line)
        return len(payload)


def _make_profile() -> SimpleNamespace:
    return SimpleNamespace(
        hardware={"serial_ports": {"haptic_b": []}},
        tuning={
            "haptic": {
                "bounds_deg_min": [-90, -90, -90, -90, -90, -90],
                "bounds_deg_max": [90, 90, 90, 90, 90, 90],
                "tracking_kp": 12.0,
                "tracking_kd": 0.6,
                "tracking_max_torque": 0.6,
                "bounds_kp": 60.0,
                "oob_kick": {
                    "enabled": True,
                    "amplitude": 0.35,
                    "pulse_interval_ms": 80,
                },
            }
        },
    )


def _write_com_ports_config(path: Path, serial_ports_yaml: str = "") -> None:
    """Write a test COM-port config with the required haptic serial baudrate."""

    path.write_text(
        "serial_ports:\n"
        f"{serial_ports_yaml}"
        "serial_settings:\n"
        "  p5_haptic_single_dial:\n"
        "    baudrate: 123456\n",
        encoding="utf-8",
    )


def test_real_backend_discovers_and_drives() -> None:
    with TemporaryDirectory() as tmpdir:
        original = com_ports.DEFAULT_COM_PORTS_PATH
        com_ports.clear_cache()
        com_ports.DEFAULT_COM_PORTS_PATH = Path(tmpdir) / "com_ports.yaml"
        _write_com_ports_config(com_ports.DEFAULT_COM_PORTS_PATH)
        scripts_by_port = {
            f"COM{21 + idx}": f"T,{21 + idx},0,{100 + idx},{10 + idx},0,{1000 + idx},0\nI,2,{21 + idx}\nV,1,0.3.0\n"
            for idx in range(6)
        }
        writes_by_port: dict[str, list[str]] = {}
        baudrates_by_port: dict[str, int] = {}

        def serial_factory():
            return _FakeSerial(scripts_by_port, writes_by_port, baudrates_by_port)

        ports = [
            SimpleNamespace(device=port, vid=0x1A86, pid=0x7523)
            for port in scripts_by_port
        ]

        rig = RealHaptic(
            team="b",
            profile=_make_profile(),
            serial_factory=serial_factory,
            list_ports_fn=lambda: ports,
        )
        try:
            rig.update_robot_actual([0.0, 0.05, 0.1, 0.15, 0.2, 0.25])
            rig.apply_command({
                "tracking_target_rad": [0.1, 0.2, 0.3, 0.4, 0.5, 0.6],
                "bounds_min_rad": [-0.5] * 6,
                "bounds_max_rad": [0.5] * 6,
            })
            sample = rig.sample()
            rig.request_reseat([0.3, 0.35, 0.4, 0.45, 0.5, 0.55])
            rig.sample()

            assert sample["board_connected"] == [True] * 6
            assert sample["board_loop_hz"] == [1000, 1001, 1002, 1003, 1004, 1005]
            assert abs(sample["dial_pos_rad"][0] - math.radians(10.0)) < 1e-6
            assert abs(sample["dial_vel_rad_s"][0] - math.radians(1.0)) < 1e-6

            for port, lines in writes_by_port.items():
                assert any(line.startswith("V,") for line in lines), port
                assert any(line.startswith("I,") for line in lines), port
                assert any(line.startswith("S,") for line in lines), port
                assert sum(1 for line in lines if line.startswith("R,")) >= 2, port
                assert any(line.startswith("C,") for line in lines), port
            assert set(baudrates_by_port.values()) == {123456}
            print("[test] real haptic backend discovery + command path: OK")
        finally:
            rig.close()
            com_ports.DEFAULT_COM_PORTS_PATH = original
            com_ports.clear_cache()


def test_oob_kick_enable_defaults_to_protocol_enabled() -> None:
    """OOB kick should stay enabled unless a profile explicitly disables it."""

    default_lines = _build_enable_lines({})
    explicit_on_lines = _build_enable_lines({"oob_kick": {"enabled": True}})
    explicit_off_lines = _build_enable_lines({"oob_kick": {"enabled": False}})

    assert ("enable_oob_kick", 1) in default_lines
    assert ("enable_oob_kick", 1) in explicit_on_lines
    assert ("enable_oob_kick", 0) in explicit_off_lines
    print("[test] oob kick default enable behavior: OK")


def test_real_backend_static_bounds_convert_to_dial_space() -> None:
    """RealHaptic startup fallback bounds should use the same gear convention."""

    bounds_min, bounds_max = _robot_bounds_to_dial_bounds_rad(
        [math.radians(-180.0)] * 6,
        [math.radians(180.0)] * 6,
        [0.1] * 6,
    )

    assert math.isclose(bounds_min[0], math.radians(-1800.0), abs_tol=1e-9)
    assert math.isclose(bounds_max[0], math.radians(1800.0), abs_tol=1e-9)
    print("[test] real haptic static bounds gear conversion: OK")


def test_config_rejects_bad_haptic_port_list() -> None:
    with TemporaryDirectory() as tmpdir:
        path = Path(tmpdir) / "bad_haptic_profile.yaml"
        path.write_text(
            "profile_name: bad_haptic\n"
            "active_teams: [b]\n"
            "subsystems:\n"
            "  haptic_io: {a: null, b: real}\n"
            "  robot_io: {a: null, b: sim_pybullet}\n"
            "  jogging_planner: {a: null, b: in_process}\n"
            "  collision_workers: {count: 0}\n"
            "  bus_broker: {impl: real}\n"
            "tuning:\n"
            "  robot:\n"
            "    q_limits_min_deg: [-180, -180, -180, -180, -180, -180]\n"
            "    q_limits_max_deg: [180, 180, 180, 180, 180, 180]\n"
            "hardware:\n"
            "  serial_ports:\n"
            "    haptic_b: COM7\n",
            encoding="utf-8",
        )
        try:
            load_profile(path)
        except ConfigError as exc:
            assert "hardware.serial_ports.haptic_b" in str(exc)
            print("[test] config rejects malformed haptic serial_ports: OK")
            return
        raise AssertionError("ConfigError not raised for malformed hardware.serial_ports.haptic_b")


def test_com_ports_yaml_empty_disables_haptic_scan() -> None:
    with TemporaryDirectory() as tmpdir:
        path = Path(tmpdir) / "com_ports.yaml"
        _write_com_ports_config(path, "  haptic_b: []\n")
        original = com_ports.DEFAULT_COM_PORTS_PATH
        com_ports.clear_cache()
        com_ports.DEFAULT_COM_PORTS_PATH = path
        try:
            rig = RealHaptic(
                team="b",
                profile=_make_profile(),
                serial_factory=lambda: _FakeSerial({}, {}),
                list_ports_fn=lambda: [
                    SimpleNamespace(device="COM99", vid=0x1A86, pid=0x7523)
                ],
            )
            try:
                sample = rig.sample()
                assert sample["board_connected"] == [False] * 6
                assert rig._candidate_ports() == []
                print("[test] com_ports.yaml empty haptic entry disables scan: OK")
            finally:
                rig.close()
        finally:
            com_ports.DEFAULT_COM_PORTS_PATH = original
            com_ports.clear_cache()


def test_com_ports_yaml_overrides_profile_haptic_ports() -> None:
    with TemporaryDirectory() as tmpdir:
        path = Path(tmpdir) / "com_ports.yaml"
        _write_com_ports_config(path, "  haptic_b: [COM31, COM32]\n")
        original = com_ports.DEFAULT_COM_PORTS_PATH
        com_ports.clear_cache()
        com_ports.DEFAULT_COM_PORTS_PATH = path
        try:
            rig = RealHaptic(
                team="b",
                profile=SimpleNamespace(
                    hardware={"serial_ports": {"haptic_b": ["COM21"]}},
                    tuning={"haptic": {}},
                ),
                serial_factory=lambda: _FakeSerial({}, {}),
                list_ports_fn=lambda: [],
            )
            try:
                assert rig._candidate_ports() == ["COM31", "COM32"]
                print("[test] com_ports.yaml overrides profile haptic ports: OK")
            finally:
                rig.close()
        finally:
            com_ports.DEFAULT_COM_PORTS_PATH = original
            com_ports.clear_cache()


def test_missing_haptic_baudrate_is_rejected() -> None:
    with TemporaryDirectory() as tmpdir:
        path = Path(tmpdir) / "com_ports.yaml"
        path.write_text("serial_ports:\n  haptic_b: [COM31]\n", encoding="utf-8")
        original = com_ports.DEFAULT_COM_PORTS_PATH
        com_ports.clear_cache()
        com_ports.DEFAULT_COM_PORTS_PATH = path
        try:
            try:
                RealHaptic(
                    team="b",
                    profile=_make_profile(),
                    serial_factory=lambda: _FakeSerial({}, {}),
                    list_ports_fn=lambda: [],
                )
            except ValueError as exc:
                assert "serial_settings.p5_haptic_single_dial.baudrate" in str(exc)
                print("[test] missing haptic baudrate rejected: OK")
                return
            raise AssertionError("ValueError not raised for missing haptic baudrate")
        finally:
            com_ports.DEFAULT_COM_PORTS_PATH = original
            com_ports.clear_cache()


def test_led_serial_settings_loaded_from_yaml() -> None:
    with TemporaryDirectory() as tmpdir:
        path = Path(tmpdir) / "com_ports.yaml"
        path.write_text(
            "serial_settings:\n"
            "  light_columns:\n"
            "    baudrate: 765432\n"
            "    inter_command_delay_s: 0.004\n",
            encoding="utf-8",
        )
        original = com_ports.DEFAULT_COM_PORTS_PATH
        com_ports.clear_cache()
        com_ports.DEFAULT_COM_PORTS_PATH = path
        try:
            system = LEDSystem(auto_discover=False)
            assert system._baudrate == 765432
            assert system._inter_command_delay_s == 0.004
            print("[test] LED serial settings load from YAML: OK")
        finally:
            com_ports.DEFAULT_COM_PORTS_PATH = original
            com_ports.clear_cache()


def main() -> int:
    test_real_backend_discovers_and_drives()
    test_oob_kick_enable_defaults_to_protocol_enabled()
    test_real_backend_static_bounds_convert_to_dial_space()
    test_config_rejects_bad_haptic_port_list()
    test_com_ports_yaml_empty_disables_haptic_scan()
    test_com_ports_yaml_overrides_profile_haptic_ports()
    test_missing_haptic_baudrate_is_rejected()
    test_led_serial_settings_loaded_from_yaml()
    print("\n[test] P5 HAPTIC REAL TESTS PASSED\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
