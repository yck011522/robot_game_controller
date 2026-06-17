from __future__ import annotations

import argparse
import socket
import sys
import time
from typing import Any


PORT_DASHBOARD = 29999
PORT_RTDE = 30004
DEFAULT_HOST = "192.168.0.2"


def _now_s(start_t: float) -> str:
    return f"+{time.monotonic() - start_t:7.2f}s"


def _log(start_t: float, message: str) -> None:
    print(f"{_now_s(start_t)} {message}", flush=True)


def _tcp_open(host: str, port: int, timeout_s: float) -> tuple[bool, str | None]:
    try:
        with socket.create_connection((host, port), timeout=timeout_s):
            return True, None
    except OSError as exc:
        return False, str(exc)


class DashboardSocket:
    def __init__(self, host: str, timeout_s: float = 2.0) -> None:
        self._host = host
        self._timeout_s = max(0.1, float(timeout_s))
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


def _dashboard_commands(host: str, commands: list[str], timeout_s: float) -> dict[str, Any]:
    result: dict[str, Any] = {"connected": False, "responses": {}}
    dash = DashboardSocket(host, timeout_s=timeout_s)
    try:
        dash.connect()
        result["connected"] = True
        result["banner"] = dash.banner
        responses: dict[str, str] = {}
        for command in commands:
            responses[command] = dash.command(command)
        result["responses"] = responses
    finally:
        dash.close()
    return result


def _status_snapshot(host: str, timeout_s: float) -> dict[str, str]:
    info = _dashboard_commands(
        host,
        [
            "robotmode",
            "safetymode",
            "safetystatus",
            "running",
            "programState",
            "is in remote control",
        ],
        timeout_s,
    )
    banner = info.get("banner")
    if isinstance(banner, str) and banner:
        responses = dict(info.get("responses", {}))
        responses["banner"] = banner
        return responses
    return dict(info.get("responses", {}))


def _print_snapshot(start_t: float, title: str, snapshot: dict[str, str]) -> None:
    _log(start_t, title)
    for key, value in snapshot.items():
        print(f"  {key}: {value}", flush=True)


def _is_remote_control(snapshot: dict[str, str]) -> bool | None:
    value = snapshot.get("is in remote control")
    if not isinstance(value, str):
        return None
    normalized = value.strip().lower()
    if normalized == "true":
        return True
    if normalized == "false":
        return False
    return None


def _is_starting_up(snapshot: dict[str, str]) -> bool:
    robotmode = snapshot.get("robotmode", "").upper()
    return "BOOT" in robotmode or "START" in robotmode


def _wait_until_dashboard_ready(
    host: str,
    *,
    wait_timeout_s: float,
    poll_s: float,
    socket_timeout_s: float,
    start_t: float,
) -> dict[str, str]:
    deadline = time.monotonic() + wait_timeout_s
    last_error = ""
    while time.monotonic() <= deadline:
        dash_ok, dash_error = _tcp_open(host, PORT_DASHBOARD, socket_timeout_s)
        rtde_ok, rtde_error = _tcp_open(host, PORT_RTDE, socket_timeout_s)
        if dash_ok:
            try:
                snapshot = _status_snapshot(host, socket_timeout_s)
            except Exception as exc:  # noqa: BLE001
                last_error = f"dashboard status failed: {exc}"
            else:
                remote_control = _is_remote_control(snapshot)
                starting_up = _is_starting_up(snapshot)
                _print_snapshot(
                    start_t,
                    f"reachable dashboard={dash_ok} rtde={rtde_ok} remote={remote_control} starting_up={starting_up}",
                    snapshot,
                )
                if remote_control is not False and not starting_up:
                    return snapshot
                if remote_control is False:
                    last_error = "robot is reachable but still in local control"
                elif starting_up:
                    last_error = "robot is still starting up"
        else:
            last_error = f"dashboard closed: {dash_error}"
            if not rtde_ok:
                last_error += f"; rtde closed: {rtde_error}"
            _log(start_t, f"waiting for robot reachability: {last_error}")

        remaining = deadline - time.monotonic()
        if remaining <= 0:
            break
        time.sleep(min(poll_s, remaining))

    raise TimeoutError(f"robot did not become dashboard-ready within {wait_timeout_s:.1f}s: {last_error}")


def _poll_until(
    host: str,
    *,
    title: str,
    predicate,
    timeout_s: float,
    poll_s: float,
    socket_timeout_s: float,
    start_t: float,
) -> dict[str, str]:
    deadline = time.monotonic() + timeout_s
    last_snapshot: dict[str, str] = {}
    while time.monotonic() <= deadline:
        last_snapshot = _status_snapshot(host, socket_timeout_s)
        _print_snapshot(start_t, title, last_snapshot)
        if predicate(last_snapshot):
            return last_snapshot
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            break
        time.sleep(min(poll_s, remaining))
    raise TimeoutError(f"timed out waiting for {title}; last={last_snapshot}")


def _rtde_receive_probe(host: str, start_t: float) -> dict[str, Any]:
    result: dict[str, Any] = {"connected": False}
    _log(start_t, "opening RTDE receive")
    try:
        import rtde_receive
    except Exception as exc:  # noqa: BLE001
        result["import_error"] = str(exc)
        return result

    rtde_r = None
    try:
        rtde_r = rtde_receive.RTDEReceiveInterface(host)
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
            member = getattr(rtde_r, name, None)
            if callable(member):
                try:
                    result[name] = member()
                except Exception as exc:  # noqa: BLE001
                    result[name] = f"ERROR: {exc}"
    except Exception as exc:  # noqa: BLE001
        result["error"] = str(exc)
    finally:
        if rtde_r is not None:
            try:
                rtde_r.disconnect()
            except Exception:
                pass
    return result


def _rtde_control_probe(host: str, frequency_hz: float, start_t: float) -> dict[str, Any]:
    result: dict[str, Any] = {"connected": False}
    _log(start_t, f"opening RTDE control at {frequency_hz:.1f} Hz")
    try:
        import rtde_control
    except Exception as exc:  # noqa: BLE001
        result["import_error"] = str(exc)
        return result

    rtde_c = None
    try:
        flags = 0
        upload_flag = getattr(rtde_control.RTDEControlInterface, "FLAG_UPLOAD_SCRIPT", 0)
        if isinstance(upload_flag, int):
            flags |= upload_flag
        if flags:
            rtde_c = rtde_control.RTDEControlInterface(host, frequency_hz, flags)
        else:
            rtde_c = rtde_control.RTDEControlInterface(host, frequency_hz)
        result["connected"] = True
        member = getattr(rtde_c, "isConnected", None)
        if callable(member):
            result["isConnected"] = member()
    except Exception as exc:  # noqa: BLE001
        result["error"] = str(exc)
    finally:
        if rtde_c is not None:
            for name in ("servoStop", "stopScript", "disconnect"):
                member = getattr(rtde_c, name, None)
                if callable(member):
                    try:
                        member()
                        result[f"{name}_ok"] = True
                    except Exception as exc:  # noqa: BLE001
                        result[f"{name}_error"] = str(exc)
    return result


def _print_dict(start_t: float, title: str, data: dict[str, Any]) -> None:
    _log(start_t, title)
    for key, value in data.items():
        print(f"  {key}: {value}", flush=True)


def run_startup_sequence(args: argparse.Namespace) -> int:
    start_t = time.monotonic()
    _log(start_t, f"startup sequence probe host={args.host}")
    _log(
        start_t,
        "expected sequence: wait dashboard/rtde reachability -> check remote mode -> power on -> brake release -> RTDE receive -> RTDE control",
    )

    initial = _wait_until_dashboard_ready(
        args.host,
        wait_timeout_s=args.wait_timeout_s,
        poll_s=args.poll_s,
        socket_timeout_s=args.socket_timeout_s,
        start_t=start_t,
    )
    _print_snapshot(start_t, "initial ready snapshot", initial)

    _log(start_t, "sending dashboard command: power on")
    power = _dashboard_commands(args.host, ["power on"], args.socket_timeout_s)
    _print_dict(start_t, "power on response", power)

    _poll_until(
        args.host,
        title="waiting for IDLE/RUNNING robotmode after power on",
        predicate=lambda snapshot: (
            "IDLE" in snapshot.get("robotmode", "").upper()
            or "RUNNING" in snapshot.get("robotmode", "").upper()
        )
        and not _is_starting_up(snapshot),
        timeout_s=args.power_timeout_s,
        poll_s=args.poll_s,
        socket_timeout_s=args.socket_timeout_s,
        start_t=start_t,
    )

    _log(start_t, "sending dashboard command: brake release")
    brake = _dashboard_commands(args.host, ["brake release"], args.socket_timeout_s)
    _print_dict(start_t, "brake release response", brake)

    post_brake = _poll_until(
        args.host,
        title="waiting for RUNNING robotmode after brake release",
        predicate=lambda snapshot: "RUNNING" in snapshot.get("robotmode", "").upper(),
        timeout_s=args.brake_timeout_s,
        poll_s=args.poll_s,
        socket_timeout_s=args.socket_timeout_s,
        start_t=start_t,
    )
    _print_snapshot(start_t, "dashboard ready for RTDE", post_brake)

    rtde_receive = _rtde_receive_probe(args.host, start_t)
    _print_dict(start_t, "rtde_receive probe", rtde_receive)
    if not rtde_receive.get("connected"):
        return 3

    if args.skip_control:
        _log(start_t, "skipping RTDE control probe by request")
        return 0

    rtde_control = _rtde_control_probe(args.host, args.servo_hz, start_t)
    _print_dict(start_t, "rtde_control probe", rtde_control)
    if not rtde_control.get("connected"):
        return 4

    _log(start_t, "startup sequence probe succeeded")
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Probe UR startup power-on/brake-release sequence.")
    parser.add_argument("--host", default=DEFAULT_HOST, help="Robot controller IP")
    parser.add_argument("--wait-timeout-s", type=float, default=120.0)
    parser.add_argument("--power-timeout-s", type=float, default=30.0)
    parser.add_argument("--brake-timeout-s", type=float, default=30.0)
    parser.add_argument("--poll-s", type=float, default=1.0)
    parser.add_argument("--socket-timeout-s", type=float, default=2.0)
    parser.add_argument("--servo-hz", type=float, default=200.0)
    parser.add_argument("--skip-control", action="store_true")
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        return run_startup_sequence(args)
    except KeyboardInterrupt:
        print("Interrupted", flush=True)
        return 130
    except Exception as exc:  # noqa: BLE001
        print(f"ERROR: {exc}", flush=True)
        return 1


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
