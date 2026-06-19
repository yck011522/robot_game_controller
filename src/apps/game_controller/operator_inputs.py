"""Operator-input helpers for the game controller.

This module owns the request/reply path for UI buttons and future physical
operator buttons. All mutable controller state is passed in explicitly.
"""

from __future__ import annotations

from typing import Any

import zmq

from core import bus


def _drain_operator_input_requests(rep: zmq.Socket, *, on_msg) -> None:
    """Drain pending operator-input requests and reply to each one.

    The game controller owns the bind-side REP socket, so every received
    request must produce exactly one reply before the next request can be read.
    """

    while True:
        try:
            body = bus.recv_json(rep, flags=zmq.NOBLOCK)
        except zmq.Again:
            break
        bus.send_json(rep, on_msg(body))


def _handle_operator_input_request(
    control_state: dict[str, Any],
    stage_state: dict[str, Any],
    teams: dict[str, dict],
    body: dict[str, Any],
    now_ns: int,
    *,
    producer: str,
    recovery_timeout_s: float,
) -> dict[str, Any]:
    """Apply one operator-input request with request-id dedupe and reply formatting."""

    request = body if isinstance(body, dict) else {}
    source = (
        request.get("source") if isinstance(request.get("source"), str) else "unknown"
    )
    request_id = (
        request.get("request_id")
        if isinstance(request.get("request_id"), (int, str))
        else None
    )

    last_request_id_by_source = control_state.setdefault(
        "last_request_id_by_source", {}
    )
    last_reply_by_source = control_state.setdefault("last_reply_by_source", {})
    if request_id is not None and last_request_id_by_source.get(source) == request_id:
        cached = last_reply_by_source.get(source)
        if isinstance(cached, dict):
            return dict(cached)

    ok, error, action = True, None, None
    record_last_action = False
    action = request.get("action") if isinstance(request, dict) else None
    if not isinstance(action, str):
        ok, error = False, "missing action"
        action = None
    elif action == "play_resume":
        if bool(control_state.get("safety_blocked", False)):
            control_state["soft_pause"] = True
            ok, error = False, "safety barrier is not clear"
        else:
            recovery_teams = [
                team
                for team, st in teams.items()
                if _is_robot_status_recovery_required(st.get("robot_status", {}))
            ]
            if recovery_teams:
                control_state["soft_pause"] = True
                control_state["safety_pause_latched"] = False
                control_state["recovery_active"] = True
                control_state["recovery_deadline_mono_ns"] = now_ns + int(
                    recovery_timeout_s * 1e9
                )
                control_state["recovery_pending_dispatch"] = True
                control_state["recovery_request_id"] = (
                    int(control_state.get("recovery_request_id", 0)) + 1
                )
                control_state["recovery_teams"] = list(recovery_teams)
            else:
                control_state["soft_pause"] = False
                control_state["safety_pause_latched"] = False
                control_state["recovery_active"] = False
                control_state["recovery_pending_dispatch"] = False
                control_state["recovery_teams"] = []
            record_last_action = True
    elif action == "soft_estop":
        control_state["soft_pause"] = True
        control_state["recovery_active"] = False
        control_state["recovery_pending_dispatch"] = False
        control_state["recovery_teams"] = []
        record_last_action = True
    elif action in ("skip", "end_game"):
        stage = stage_state.get("stage")
        if stage not in ("play", "tutorial"):
            ok, error, action = False, f"skip not allowed in stage '{stage}'", "skip"
        else:
            stage_state["skip_requested"] = True
            print(f"[game_controller] SKIP requested in stage '{stage}'", flush=True)
            action = "skip"
            record_last_action = True
    else:
        ok, error = False, f"unsupported action: {action}"

    if record_last_action:
        control_state["last_action"] = action
        control_state["last_action_ts_mono_ns"] = now_ns

    reply = bus.make_envelope(producer, with_wall=True)
    reply.update(
        {
            "ok": ok,
            "error": error,
            "request_id": request_id,
            "source": source,
            "result": {
                "action": action,
                "soft_estop": bool(control_state.get("soft_pause", False)),
                "active_stage": stage_state["stage"],
                "last_action": control_state.get("last_action"),
            },
        }
    )
    if request_id is not None:
        last_request_id_by_source[source] = request_id
        last_reply_by_source[source] = dict(reply)
    return reply


def _is_robot_status_recovery_required(status: dict[str, Any]) -> bool:
    """Return whether a robot status still needs an explicit recovery request."""

    if bool(status.get("fault_active", False)):
        return True
    if status.get("program_running") is False:
        return True
    return not bool(status.get("control_ok", True))


def _is_robot_status_recovered(status: dict[str, Any]) -> bool:
    """Return whether a robot status is good enough to clear recovery pause."""

    if bool(status.get("fault_active", False)):
        return False
    if not bool(status.get("control_ok", False)):
        return False
    return status.get("program_running") is not False


def _publish_pending_recovery_requests(
    pub: zmq.Socket,
    producer: str,
    control_state: dict[str, Any],
    *,
    recovery_timeout_s: float,
) -> None:
    """Publish queued recovery requests exactly once per resume attempt."""

    if not bool(control_state.get("recovery_active", False)):
        return
    if not bool(control_state.get("recovery_pending_dispatch", False)):
        return

    request_id = int(control_state.get("recovery_request_id", 0))
    for team in list(control_state.get("recovery_teams", [])):
        env = bus.make_envelope(producer)
        env.update(
            {
                "team": team,
                "request_id": request_id,
                "timeout_s": recovery_timeout_s,
            }
        )
        bus.publish(pub, f"cmd.robot.recover.{team}", env)
    control_state["recovery_pending_dispatch"] = False