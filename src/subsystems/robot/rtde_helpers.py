"""Thin `ur_rtde` helpers copied from `incoming_code/rtde_core.py`.

Kept under `src/` so the runtime no longer depends on importing from
the migration staging area.
"""

from __future__ import annotations

import socket
import time
from typing import Any

from rtde_control import RTDEControlInterface
from rtde_receive import RTDEReceiveInterface


PORT_DASHBOARD = 29999
PORT_RTDE = 30004
DASHBOARD_STATUS_COMMANDS = [
    "robotmode",
    "safetymode",
    "safetystatus",
    "running",
    "programState",
    "is in remote control",
]


class DashboardClient:
    """Minimal line-oriented client for the UR Dashboard server."""

    def __init__(self, host: str, timeout_s: float = 1.0) -> None:
        """Store connection settings for a future dashboard session."""
        self._host = host
        self._timeout_s = max(0.1, float(timeout_s))
        self._sock: socket.socket | None = None
        self.banner = ""

    def connect(self) -> None:
        """Open the dashboard socket and read its greeting banner."""
        self._sock = socket.create_connection((self._host, PORT_DASHBOARD), timeout=self._timeout_s)
        self._sock.settimeout(self._timeout_s)
        self.banner = self._recv_line()

    def close(self) -> None:
        """Close the dashboard socket if it is open."""
        if self._sock is None:
            return
        try:
            self._sock.close()
        finally:
            self._sock = None

    def command(self, cmd: str) -> str:
        """Send one dashboard command and return its single-line response."""
        if self._sock is None:
            raise RuntimeError("dashboard socket is not connected")
        self._sock.sendall((cmd.strip() + "\n").encode("utf-8"))
        return self._recv_line()

    def _recv_line(self) -> str:
        """Read one newline-terminated dashboard response."""
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


def run_dashboard_sequence(robot_ip: str, commands: list[str], timeout_s: float = 1.0) -> dict[str, Any]:
    """Run a short dashboard command sequence over one temporary connection."""
    result: dict[str, Any] = {
        "connected": False,
        "responses": {},
    }
    dash = DashboardClient(robot_ip, timeout_s=timeout_s)
    try:
        dash.connect()
        result["connected"] = True
        result["banner"] = dash.banner
        responses: dict[str, str] = {}
        for cmd in commands:
            responses[cmd] = dash.command(cmd)
        result["responses"] = responses
    finally:
        dash.close()
    return result


def dashboard_status_snapshot(robot_ip: str, timeout_s: float = 1.0) -> dict[str, str]:
    """Fetch the dashboard status lines used by startup and recovery."""
    info = run_dashboard_sequence(robot_ip, DASHBOARD_STATUS_COMMANDS, timeout_s=timeout_s)
    responses = info.get("responses", {}) if isinstance(info, dict) else {}
    return dict(responses) if isinstance(responses, dict) else {}


def dashboard_robotmode_running(response: str | None) -> bool:
    """Return whether a dashboard robotmode line reports RUNNING."""
    return isinstance(response, str) and "RUNNING" in response.upper()


def dashboard_robotmode_idle_or_running(response: str | None) -> bool:
    """Return whether the robot has finished the power-on transition."""
    if not isinstance(response, str):
        return False
    upper = response.upper()
    return "IDLE" in upper or "RUNNING" in upper


def dashboard_robotmode_starting(response: str | None) -> bool:
    """Return whether a dashboard robotmode line still looks transitional."""
    if not isinstance(response, str):
        return False
    upper = response.upper()
    return "BOOT" in upper or "START" in upper


def dashboard_remote_control(response: str | None) -> bool | None:
    """Parse the dashboard remote-control boolean when the controller exposes it."""
    if not isinstance(response, str):
        return None
    normalized = response.strip().lower()
    if normalized == "true":
        return True
    if normalized == "false":
        return False
    return None


def dashboard_safety_protective(response: str | None) -> bool:
    """Return whether a dashboard safetystatus line reports a protective stop."""
    return isinstance(response, str) and "PROTECTIVE_STOP" in response.upper()


def dashboard_running_true(response: str | None) -> bool:
    """Return whether a dashboard running line reports the program as running."""
    return isinstance(response, str) and "program running: true" in response.lower()


def wait_for_dashboard_ready(
    robot_ip: str,
    *,
    ready_timeout_s: float = 120.0,
    poll_s: float = 1.0,
    socket_timeout_s: float = 1.0,
    log=None,
) -> dict[str, str]:
    """Wait until Dashboard/RTDE are reachable, not local, and not booting."""
    deadline = time.monotonic() + max(0.1, float(ready_timeout_s))
    poll_s = max(0.05, float(poll_s))
    last_note = "not checked yet"
    while time.monotonic() <= deadline:
        dashboard_open = is_tcp_open(robot_ip, PORT_DASHBOARD, timeout_s=socket_timeout_s)
        rtde_open = is_tcp_open(robot_ip, PORT_RTDE, timeout_s=socket_timeout_s)
        if dashboard_open:
            try:
                snapshot = dashboard_status_snapshot(robot_ip, timeout_s=socket_timeout_s)
            except Exception as exc:  # noqa: BLE001
                last_note = f"dashboard status failed: {exc}"
            else:
                remote = dashboard_remote_control(snapshot.get("is in remote control"))
                starting = dashboard_robotmode_starting(snapshot.get("robotmode"))
                if log is not None:
                    log(
                        "startup_wait",
                        {
                            "dashboard_open": dashboard_open,
                            "rtde_open": rtde_open,
                            "remote_control": remote,
                            "starting": starting,
                            "status": snapshot,
                        },
                    )
                if rtde_open and remote is not False and not starting:
                    return snapshot
                if not rtde_open:
                    last_note = f"RTDE TCP {PORT_RTDE} is not reachable"
                elif remote is False:
                    last_note = "robot is in local control"
                elif starting:
                    last_note = f"robot is starting up: {snapshot.get('robotmode')}"
        else:
            last_note = f"Dashboard TCP {PORT_DASHBOARD} is not reachable"

        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            break
        time.sleep(min(poll_s, remaining))

    raise TimeoutError(
        f"UR dashboard/RTDE did not become ready within {ready_timeout_s:.1f}s: {last_note}"
    )


def _poll_dashboard_status(
    robot_ip: str,
    *,
    predicate,
    timeout_s: float,
    poll_s: float,
    socket_timeout_s: float,
    log=None,
    label: str,
) -> dict[str, str]:
    deadline = time.monotonic() + max(0.1, float(timeout_s))
    poll_s = max(0.05, float(poll_s))
    last_status: dict[str, str] = {}
    while time.monotonic() <= deadline:
        last_status = dashboard_status_snapshot(robot_ip, timeout_s=socket_timeout_s)
        if log is not None:
            log(label, last_status)
        if predicate(last_status):
            return last_status
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            break
        time.sleep(min(poll_s, remaining))
    raise TimeoutError(f"timed out waiting for {label}: {last_status}")


def power_on_and_release_brakes(
    robot_ip: str,
    *,
    power_timeout_s: float = 30.0,
    brake_timeout_s: float = 30.0,
    poll_s: float = 1.0,
    socket_timeout_s: float = 1.0,
    log=None,
) -> dict[str, Any]:
    """Run the Dashboard power-on/brake-release sequence and wait for RUNNING."""
    result: dict[str, Any] = {
        "power_on": run_dashboard_sequence(robot_ip, ["power on"], timeout_s=socket_timeout_s),
    }
    if log is not None:
        log("power_on", result["power_on"])

    result["post_power"] = _poll_dashboard_status(
        robot_ip,
        predicate=lambda status: dashboard_robotmode_idle_or_running(status.get("robotmode")),
        timeout_s=power_timeout_s,
        poll_s=poll_s,
        socket_timeout_s=socket_timeout_s,
        log=log,
        label="post_power",
    )

    result["brake_release"] = run_dashboard_sequence(
        robot_ip, ["brake release"], timeout_s=socket_timeout_s
    )
    if log is not None:
        log("brake_release", result["brake_release"])

    result["post_brake"] = _poll_dashboard_status(
        robot_ip,
        predicate=lambda status: dashboard_robotmode_running(status.get("robotmode")),
        timeout_s=brake_timeout_s,
        poll_s=poll_s,
        socket_timeout_s=socket_timeout_s,
        log=log,
        label="post_brake",
    )
    return result


def is_tcp_open(host: str, port: int, timeout_s: float = 1.0) -> bool:
    """Return whether a TCP endpoint is reachable within the timeout."""
    try:
        with socket.create_connection((host, port), timeout=timeout_s):
            return True
    except OSError:
        return False


def preflight_network_checks(robot_ip: str) -> None:
    """Fail fast when the robot RTDE endpoint is unreachable."""
    if is_tcp_open(robot_ip, PORT_RTDE):
        return

    dashboard_open = is_tcp_open(robot_ip, PORT_DASHBOARD)

    hints = [
        f"TCP {PORT_RTDE} is not reachable on {robot_ip}.",
        "Ping alone is not enough; RTDE needs TCP port 30004 open.",
    ]
    if not dashboard_open:
        hints.append("Dashboard port 29999 is also closed (robot services may be stopped or blocked).")

    hints.extend(
        [
            "Check robot teach pendant: Settings -> System -> Remote Control (enabled).",
            "Make sure PolyScope is running and robot is not in a blocked/safety-stop state.",
            "Verify no firewall/VLAN rule blocks TCP 29999/30001-30004 between PC and robot.",
            "Confirm this IP is the robot controller (not another device).",
        ]
    )

    raise RuntimeError("\n".join(hints))


def preflight_control_checks(robot_ip: str) -> None:
    """Fail fast when Dashboard control prerequisites are missing."""
    if is_tcp_open(robot_ip, PORT_DASHBOARD):
        return

    raise RuntimeError(
        "Control mode requires Dashboard server on TCP 29999, but it is not reachable.\n"
        "Receive mode can work on 30004 while control still fails.\n"
        "On the robot, enable Dashboard service and Remote Control mode, then retry."
    )


def connect_receive(robot_ip: str) -> RTDEReceiveInterface:
    """Connect the RTDE receive interface after basic network checks."""
    preflight_network_checks(robot_ip)
    return RTDEReceiveInterface(robot_ip)


def connect_control(robot_ip: str, frequency_hz: float = -1.0) -> RTDEControlInterface:
    """Connect the RTDE control interface with script-upload support when available."""
    preflight_control_checks(robot_ip)
    try:
        flags = 0
        upload_flag = getattr(RTDEControlInterface, "FLAG_UPLOAD_SCRIPT", 0)
        if isinstance(upload_flag, int):
            flags |= upload_flag

        if flags:
            return RTDEControlInterface(robot_ip, frequency_hz, flags)
        return RTDEControlInterface(robot_ip, frequency_hz)
    except RuntimeError as exc:
        raise RuntimeError(
            "Failed to start RTDE control channel. "
            "This usually means Dashboard server (29999) is disabled/unreachable "
            "or robot is not in Remote Control mode."
        ) from exc
