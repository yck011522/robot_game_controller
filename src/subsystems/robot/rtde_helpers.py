"""Thin `ur_rtde` helpers copied from `incoming_code/rtde_core.py`.

Kept under `src/` so the runtime no longer depends on importing from
the migration staging area.
"""

from __future__ import annotations

import socket

from rtde_control import RTDEControlInterface
from rtde_receive import RTDEReceiveInterface


PORT_DASHBOARD = 29999
PORT_RTDE = 30004


def is_tcp_open(host: str, port: int, timeout_s: float = 1.0) -> bool:
    try:
        with socket.create_connection((host, port), timeout=timeout_s):
            return True
    except OSError:
        return False


def preflight_network_checks(robot_ip: str) -> None:
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
    if is_tcp_open(robot_ip, PORT_DASHBOARD):
        return

    raise RuntimeError(
        "Control mode requires Dashboard server on TCP 29999, but it is not reachable.\n"
        "Receive mode can work on 30004 while control still fails.\n"
        "On the robot, enable Dashboard service and Remote Control mode, then retry."
    )


def connect_receive(robot_ip: str) -> RTDEReceiveInterface:
    preflight_network_checks(robot_ip)
    return RTDEReceiveInterface(robot_ip)


def connect_control(robot_ip: str, frequency_hz: float = -1.0) -> RTDEControlInterface:
    preflight_control_checks(robot_ip)
    try:
        return RTDEControlInterface(robot_ip, frequency_hz)
    except RuntimeError as exc:
        raise RuntimeError(
            "Failed to start RTDE control channel. "
            "This usually means Dashboard server (29999) is disabled/unreachable "
            "or robot is not in Remote Control mode."
        ) from exc