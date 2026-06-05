from __future__ import annotations

import argparse
import socket
import sys
import time
from typing import Any


PORT_DASHBOARD = 29999


def _call_if_present(obj: Any, name: str) -> tuple[bool, Any]:
    if not hasattr(obj, name):
        return False, None
    member = getattr(obj, name)
    if not callable(member):
        return False, None
    try:
        return True, member()
    except Exception as exc:  # noqa: BLE001
        return True, f"ERROR: {exc}"


def _query_rtde_receive(host: str) -> dict[str, Any]:
    result: dict[str, Any] = {"connected": False}
    try:
        import rtde_receive
    except Exception as exc:  # noqa: BLE001
        result["import_error"] = str(exc)
        return result

    try:
        rtde_r = rtde_receive.RTDEReceiveInterface(host)
    except Exception as exc:  # noqa: BLE001
        result["connect_error"] = str(exc)
        return result

    result["connected"] = True
    for name in [
        "isConnected",
        "isProtectiveStopped",
        "isEmergencyStopped",
        "isSafetyStopped",
        "isProgramRunning",
        "isSteady",
        "getRobotMode",
        "getRobotStatus",
        "getSafetyMode",
        "getSafetyStatusBits",
        "getActualQ",
        "getActualQd",
    ]:
        present, value = _call_if_present(rtde_r, name)
        if present:
            result[name] = value

    try:
        rtde_r.disconnect()
    except Exception:
        pass
    return result


class DashboardSocket:
    def __init__(self, host: str, timeout_s: float = 2.0) -> None:
        self._host = host
        self._timeout_s = timeout_s
        self._sock: socket.socket | None = None
        self.banner = ""

    def connect(self) -> None:
        self._sock = socket.create_connection((self._host, PORT_DASHBOARD), timeout=self._timeout_s)
        self._sock.settimeout(self._timeout_s)
        self.banner = self._recv_line()

    def close(self) -> None:
        if self._sock is None:
            return
        try:
            self._sock.close()
        finally:
            self._sock = None

    def command(self, cmd: str) -> str:
        if self._sock is None:
            raise RuntimeError("dashboard socket is not connected")
        self._sock.sendall((cmd.strip() + "\n").encode("utf-8"))
        return self._recv_line()

    def _recv_line(self) -> str:
        if self._sock is None:
            raise RuntimeError("dashboard socket is not connected")
        chunks: list[bytes] = []
        while True:
            chunk = self._sock.recv(1)
            if not chunk:
                break
            if chunk == b"\n":
                break
            chunks.append(chunk)
        return b"".join(chunks).decode("utf-8", errors="replace").strip()


def _query_dashboard(host: str) -> dict[str, Any]:
    result: dict[str, Any] = {"connected": False}
    dash = DashboardSocket(host)
    try:
        dash.connect()
    except Exception as exc:  # noqa: BLE001
        result["connect_error"] = str(exc)
        return result

    result["connected"] = True
    result["banner"] = dash.banner
    for cmd in [
        "robotmode",
        "safetymode",
        "safetystatus",
        "running",
        "programState",
    ]:
        try:
            result[cmd] = dash.command(cmd)
        except Exception as exc:  # noqa: BLE001
            result[cmd] = f"ERROR: {exc}"
    dash.close()
    return result


def _attempt_recovery(host: str, cooldown_s: float) -> dict[str, Any]:
    result: dict[str, Any] = {"attempted": True, "cooldown_s": cooldown_s}
    dash = DashboardSocket(host, timeout_s=max(2.0, cooldown_s + 1.0))
    try:
        dash.connect()
        result["banner"] = dash.banner
        result["pre.robotmode"] = dash.command("robotmode")
        result["pre.safetystatus"] = dash.command("safetystatus")
        result["waited"] = cooldown_s
        if cooldown_s > 0:
            time.sleep(cooldown_s)
        result["close_safety_popup"] = dash.command("close safety popup")
        result["unlock_protective_stop"] = dash.command("unlock protective stop")
        time.sleep(1.0)
        result["power_on"] = dash.command("power on")
        time.sleep(2.0)
        result["brake_release"] = dash.command("brake release")
        time.sleep(2.0)
        result["post.robotmode"] = dash.command("robotmode")
        result["post.safetystatus"] = dash.command("safetystatus")
    except Exception as exc:  # noqa: BLE001
        result["error"] = str(exc)
    finally:
        dash.close()
    return result


def _print_section(title: str, data: dict[str, Any]) -> None:
    print(f"[{title}]")
    for key, value in data.items():
        print(f"  {key}: {value}")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Probe UR protective-stop state via RTDE receive and Dashboard")
    parser.add_argument("--host", default="192.168.0.2", help="Robot controller IP")
    parser.add_argument(
        "--attempt-recover",
        action="store_true",
        help="Attempt Dashboard-based protective-stop recovery after status reads",
    )
    parser.add_argument(
        "--cooldown-s",
        type=float,
        default=5.0,
        help="Seconds to wait before unlock_protective_stop when recovery is requested",
    )
    args = parser.parse_args(argv)

    rtde_receive = _query_rtde_receive(args.host)
    dashboard = _query_dashboard(args.host)

    _print_section("rtde_receive", rtde_receive)
    _print_section("dashboard", dashboard)

    if args.attempt_recover:
        recovery = _attempt_recovery(args.host, max(0.0, float(args.cooldown_s)))
        _print_section("recovery", recovery)
        rtde_receive_after = _query_rtde_receive(args.host)
        dashboard_after = _query_dashboard(args.host)
        _print_section("rtde_receive_after", rtde_receive_after)
        _print_section("dashboard_after", dashboard_after)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())