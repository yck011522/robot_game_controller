"""Real UR10e RobotIO backend using `ur_rtde`.

This is the P3 bridge from `cmd.robot.target.<team>` to the actual
robot. The startup and `servoJ` behavior is intentionally copied from
the archived keyboard explorer because that is the last validated
real-robot control path in this repository.
"""

from __future__ import annotations

import time
from typing import Any


DEFAULT_SERVO_HZ = 200.0
DEFAULT_LOOKAHEAD_TIME = 0.05
DEFAULT_SERVO_GAIN = 500
SERVOJ_SPEED = 0.5
SERVOJ_ACCELERATION = 0.5
CONTROL_RETRY_BACKOFF_S = 0.5
PROTECTIVE_STOP_UNLOCK_COOLDOWN_S = 0.0
DASHBOARD_TIMEOUT_S = 2.0
DEFAULT_STARTUP_READY_TIMEOUT_S = 120.0
DEFAULT_RECOVERY_TIMEOUT_S = 5.0
SAFETY_CLEAR_POLL_S = 0.3
SAFETY_CLEAR_MAX_POLLS = 6
POWER_ON_TIMEOUT_S = 30.0
BRAKE_RELEASE_TIMEOUT_S = 30.0
DASHBOARD_POWER_POLL_S = 1.0
# Bounded retry while reopening the control channel inside the recovery window.
CONTROL_CONNECT_RETRY_S = 0.4


class RealRtdeRobot:
    """Drive a real UR robot over RTDE with protective-stop-aware recovery."""

    def __init__(
        self,
        *,
        host: str,
        port: int | None = None,
        servo_hz: float = DEFAULT_SERVO_HZ,
        lookahead_time: float = DEFAULT_LOOKAHEAD_TIME,
        gain: int = DEFAULT_SERVO_GAIN,
        startup_ready_timeout_s: float = DEFAULT_STARTUP_READY_TIMEOUT_S,
        startup_timeout_s: float = 5.0,
        startup_poll_s: float = 0.05,
        _rtde_helpers: Any | None = None,
    ) -> None:
        """Connect RTDE interfaces and seed commanded joints from measured state."""
        del port  # `ur_rtde` selects the standard RTDE ports internally.
        if _rtde_helpers is None:
            from subsystems.robot import rtde_helpers as _local_rtde_helpers

            _rtde_helpers = _local_rtde_helpers

        self._rtde_helpers = _rtde_helpers
        self._host = host
        self._servo_hz = float(servo_hz)
        self._servo_dt = 1.0 / self._servo_hz if self._servo_hz > 0 else 0.005
        self._lookahead_time = float(lookahead_time)
        self._gain = int(gain)
        self._startup_poll_s = max(0.0, float(startup_poll_s))
        self._closed = False
        self._receive_ok = False
        self._control_ok = False
        self._last_send_err: str | None = None
        self._protective_stopped = False
        self._emergency_stopped = False
        self._safety_stopped = False
        self._program_running: bool | None = None
        self._robot_mode: int | None = None
        self._robot_status: int | None = None
        self._safety_mode: int | None = None
        self._safety_status_bits: int | None = None
        self._protective_stop_since_t: float | None = None
        self._rtde_program_running_supported: bool | None = None
        self._recovery_requested = False
        self._recovery_deadline_t = 0.0
        self._recovery_attempted = False
        self._last_recovery_note: str | None = None

        self._wait_for_dashboard_startup_ready(float(startup_ready_timeout_s))
        self._power_on_and_release_brakes(context="startup")
        self._rtde_r = self._rtde_helpers.connect_receive(host)
        self._actual_q = self._wait_for_initial_actual_q(float(startup_timeout_s))
        self._actual_qd = [0.0] * len(self._actual_q)
        self._target_q = list(self._actual_q)
        self._last_send_t = time.perf_counter()
        self._next_control_attempt_t = self._last_send_t
        self._rtde_c = self._rtde_helpers.connect_control(host, frequency_hz=self._servo_hz)
        self._receive_ok = True
        self._control_ok = True
        self._refresh_receive_status()
        print(
            f"[robot_real_rtde] startup: connected host={self._host} servo_hz={self._servo_hz:.1f}",
            flush=True,
        )

    @property
    def rtde_ok(self) -> bool:
        """Return whether both RTDE channels are currently usable."""
        return self._receive_ok and self._control_ok and not self._closed

    def _call_optional_bool(self, name: str) -> bool | None:
        """Call an optional RTDE receive boolean API when this build exposes it."""
        if not hasattr(self._rtde_r, name):
            return None
        value = getattr(self._rtde_r, name)()
        return bool(value)

    def _call_optional_int(self, name: str) -> int | None:
        """Call an optional RTDE receive integer API when this build exposes it."""
        if not hasattr(self._rtde_r, name):
            return None
        value = getattr(self._rtde_r, name)()
        return int(value)

    def _refresh_program_running_from_receive(self) -> None:
        """Refresh program-running from RTDE when the local ur_rtde build supports it."""
        if self._rtde_program_running_supported is False:
            return

        program_running = self._call_optional_bool("isProgramRunning")
        if program_running is None:
            if self._rtde_program_running_supported is None:
                self._rtde_program_running_supported = False
                print(
                    "[robot_real_rtde] RTDE receive has no isProgramRunning(); "
                    "using dashboard/latching fallback",
                    flush=True,
                )
            return

        self._rtde_program_running_supported = True
        self._program_running = program_running

    def _refresh_receive_status(self) -> None:
        """Refresh safety and execution status from the RTDE receive channel."""
        was_protective_stopped = self._protective_stopped
        self._protective_stopped = bool(self._call_optional_bool("isProtectiveStopped") or False)
        self._emergency_stopped = bool(self._call_optional_bool("isEmergencyStopped") or False)
        self._safety_stopped = bool(self._call_optional_bool("isSafetyStopped") or False)
        self._refresh_program_running_from_receive()
        self._robot_mode = self._call_optional_int("getRobotMode")
        self._robot_status = self._call_optional_int("getRobotStatus")
        self._safety_mode = self._call_optional_int("getSafetyMode")
        self._safety_status_bits = self._call_optional_int("getSafetyStatusBits")
        if self._protective_stopped:
            # Keep servoJ muted until an explicit recovery path re-inits control.
            self._control_ok = False
        if self._protective_stopped:
            if not was_protective_stopped or self._protective_stop_since_t is None:
                self._protective_stop_since_t = time.perf_counter()
                print(
                    "[robot_real_rtde] detected protective_stop=True; entering fault handling",
                    flush=True,
                )
        else:
            if was_protective_stopped:
                print("[robot_real_rtde] protective_stop cleared", flush=True)
            self._protective_stop_since_t = None

    def status_snapshot(self) -> dict[str, Any]:
        """Return the compact robot status published onto telemetry."""
        reason = None
        if self._emergency_stopped:
            reason = "emergency_stop"
        elif self._protective_stopped:
            reason = "protective_stop"
        elif self._safety_stopped:
            reason = "safety_stop"
        elif self._program_running is False:
            reason = "program_not_running"
        elif not self._control_ok:
            reason = "rtde_control_error"
        elif not self._receive_ok:
            reason = "rtde_receive_error"

        return {
            "rtde_ok": self.rtde_ok,
            "receive_ok": self._receive_ok and not self._closed,
            "control_ok": self._control_ok and not self._closed,
            "fault_active": reason is not None,
            "fault_reason": reason,
            "protective_stopped": self._protective_stopped,
            "emergency_stopped": self._emergency_stopped,
            "safety_stopped": self._safety_stopped,
            "program_running": self._program_running,
            "robot_mode": self._robot_mode,
            "robot_status": self._robot_status,
            "safety_mode": self._safety_mode,
            "safety_status_bits": self._safety_status_bits,
            "last_send_error": self._last_send_err,
            "last_recovery_note": self._last_recovery_note,
            "recovery_requested": self._recovery_requested,
        }

    def _close_rtde_interfaces(self, *, control: Any, receive: Any, log_errors: bool) -> None:
        """Best-effort shutdown of one RTDE control/receive pair."""
        for name in ("servoStop", "stopScript", "disconnect"):
            try:
                fn = getattr(control, name, None)
                if callable(fn):
                    fn()
            except Exception as exc:  # noqa: BLE001
                if log_errors:
                    print(f"[robot_real_rtde] control.{name}() cleanup raised: {exc}", flush=True)
        try:
            receive.disconnect()
        except Exception as exc:  # noqa: BLE001
            if log_errors:
                print(f"[robot_real_rtde] receive.disconnect() cleanup raised: {exc}", flush=True)

    def _dashboard_log(self, label: str, payload: Any) -> None:
        """Print compact dashboard sequence debug lines."""
        print(f"[robot_real_rtde] dashboard {label}: {payload}", flush=True)

    def _wait_for_dashboard_startup_ready(self, timeout_s: float) -> None:
        """Wait for the robot controller to be reachable and remotely controllable."""
        print(
            f"[robot_real_rtde] startup_step: wait dashboard ready timeout_s={timeout_s:.1f}",
            flush=True,
        )
        self._rtde_helpers.wait_for_dashboard_ready(
            self._host,
            ready_timeout_s=timeout_s,
            poll_s=DASHBOARD_POWER_POLL_S,
            socket_timeout_s=DASHBOARD_TIMEOUT_S,
            log=self._dashboard_log,
        )

    def _power_on_and_release_brakes(
        self,
        *,
        context: str,
        power_timeout_s: float = POWER_ON_TIMEOUT_S,
        brake_timeout_s: float = BRAKE_RELEASE_TIMEOUT_S,
    ) -> None:
        """Run the shared Dashboard power/brake sequence."""
        print(
            f"[robot_real_rtde] {context}_step: power on + brake release",
            flush=True,
        )
        self._rtde_helpers.power_on_and_release_brakes(
            self._host,
            power_timeout_s=power_timeout_s,
            brake_timeout_s=brake_timeout_s,
            poll_s=DASHBOARD_POWER_POLL_S,
            socket_timeout_s=DASHBOARD_TIMEOUT_S,
            log=self._dashboard_log,
        )

    def _reinitialize_interfaces(self) -> None:
        """Reconnect RTDE receive/control after the robot clears a protective stop."""
        # Release the single control slot BEFORE reconnecting.
        self._close_rtde_interfaces(control=self._rtde_c, receive=self._rtde_r, log_errors=True)

        self._rtde_r = self._rtde_helpers.connect_receive(self._host)
        self._actual_q = self._wait_for_initial_actual_q(2.0)
        self._actual_qd = [0.0] * len(self._actual_q)
        self._target_q = list(self._actual_q)

        # Reopen control with bounded retries inside the recovery window.
        # Always make at least one attempt even if the deadline already
        # elapsed, so we never leave a disconnected control interface behind.
        attempt = 0
        connected = False
        last_exc: Exception | None = None
        while True:
            attempt += 1
            try:
                self._rtde_c = self._rtde_helpers.connect_control(
                    self._host, frequency_hz=self._servo_hz
                )
                connected = True
                last_exc = None
                break
            except Exception as exc:  # noqa: BLE001
                last_exc = exc
                print(
                    f"[robot_real_rtde] reinit: connect_control attempt {attempt} failed: {exc}",
                    flush=True,
                )
                if self._recovery_deadline_expired():
                    break
                time.sleep(CONTROL_CONNECT_RETRY_S)
        if not connected:
            raise last_exc if last_exc is not None else RuntimeError(
                "connect_control failed before recovery deadline"
            )

        self._refresh_receive_status()

    @staticmethod
    def _dashboard_safety_protective(response: str | None) -> bool:
        """Return whether a dashboard safetystatus line still reports a protective stop."""
        if not isinstance(response, str):
            return False
        return "PROTECTIVE_STOP" in response.upper()

    def _dashboard_status_snapshot(self, *, label: str) -> dict[str, str]:
        """Fetch a small dashboard status snapshot used by recovery decisions."""
        responses = self._rtde_helpers.dashboard_status_snapshot(
            self._host,
            timeout_s=DASHBOARD_TIMEOUT_S,
        )
        if isinstance(responses, dict):
            self._dashboard_log(label, responses)
            return dict(responses)
        del label
        return {}

    def _set_program_running(self, value: bool | None, *, source: str) -> None:
        """Update the cached program-running state from a trusted source."""
        del source
        self._program_running = value

    def _recovery_deadline_expired(self) -> bool:
        """Return whether the current recovery window has elapsed."""
        if not self._recovery_requested:
            return True
        if time.perf_counter() <= self._recovery_deadline_t:
            return False
        self._finish_recovery("recovery_timeout")
        return True

    @staticmethod
    def _dashboard_running_true(response: str | None) -> bool:
        """Return whether a dashboard running line reports the program as running."""
        if not isinstance(response, str):
            return False
        return "program running: true" in response.lower()

    def request_recovery(self, timeout_s: float = DEFAULT_RECOVERY_TIMEOUT_S) -> None:
        """Arm one user-triggered recovery attempt for the next control tick."""
        timeout_s = max(0.1, float(timeout_s))
        now = time.perf_counter()
        self._recovery_requested = True
        self._recovery_deadline_t = now + timeout_s
        self._recovery_attempted = False
        self._last_recovery_note = "recovery_requested"
        print(
            f"[robot_real_rtde] request_recovery: timeout_s={timeout_s:.2f}",
            flush=True,
        )

    def _finish_recovery(self, note: str) -> None:
        """Close out the active recovery request and remember the terminal note."""
        self._recovery_requested = False
        self._recovery_attempted = False
        self._recovery_deadline_t = 0.0
        self._last_recovery_note = note
        print(f"[robot_real_rtde] recovery_done: {note}", flush=True)

    def _maybe_attempt_dashboard_recovery(self, now: float) -> None:
        """Run the dashboard-assisted protective-stop recovery flow once per request."""
        if not self._recovery_requested:
            return
        if now > self._recovery_deadline_t:
            self._finish_recovery("recovery_timeout")
            return
        if self._recovery_attempted:
            return
        if self._emergency_stopped:
            self._finish_recovery("recovery_blocked_emergency_stop")
            return

        if self._protective_stopped and self._protective_stop_since_t is not None:
            elapsed = now - self._protective_stop_since_t
            if elapsed < PROTECTIVE_STOP_UNLOCK_COOLDOWN_S:
                self._finish_recovery("recovery_wait_protective_cooldown")
                return

        self._recovery_attempted = True
        try:
            self._dashboard_status_snapshot(label="pre-status")
            if self._recovery_deadline_expired():
                return

            print("[robot_real_rtde] recovery_step: clear protective stop", flush=True)
            self._rtde_helpers.run_dashboard_sequence(
                self._host,
                [
                    "close safety popup",
                    "unlock protective stop",
                ],
                timeout_s=DASHBOARD_TIMEOUT_S,
            )

            # Wait briefly for safetystatus to leave PROTECTIVE_STOP.
            safety_ok = False
            last_status: dict[str, str] = {}
            for _ in range(SAFETY_CLEAR_MAX_POLLS):
                if self._recovery_deadline_expired():
                    return
                time.sleep(SAFETY_CLEAR_POLL_S)
                last_status = self._dashboard_status_snapshot(label="post-unlock")
                if not self._dashboard_safety_protective(last_status.get("safetystatus")):
                    safety_ok = True
                    break
            if not safety_ok:
                self._finish_recovery(
                    f"recovery_safety_not_ready={last_status.get('safetystatus')}"
                )
                return

            if self._recovery_deadline_expired():
                return
            remaining_s = max(0.1, self._recovery_deadline_t - time.perf_counter())
            self._power_on_and_release_brakes(
                context="recovery",
                power_timeout_s=min(POWER_ON_TIMEOUT_S, remaining_s),
                brake_timeout_s=min(BRAKE_RELEASE_TIMEOUT_S, remaining_s),
            )

            if self._recovery_deadline_expired():
                return
            print("[robot_real_rtde] recovery_step: reconnect RTDE control", flush=True)
            self._reinitialize_interfaces()
            post_responses = self._dashboard_status_snapshot(label="post-reinit")

            running_response = post_responses.get("running") if isinstance(post_responses, dict) else None
            running_true = self._dashboard_running_true(running_response)
            self._set_program_running(running_true, source=f"dashboard recovery ({running_response})")
            if not running_true:
                # Keep paused for operator retry; do not resume servoJ yet.
                self._control_ok = False
                self._finish_recovery(f"recovery_running_false={running_response}")
                return

            self._control_ok = True
            self._last_send_err = None
            self._finish_recovery("recovery_ok")
        except Exception as exc:  # noqa: BLE001
            self._finish_recovery(f"recovery_error: {exc}")

    def _wait_for_initial_actual_q(self, timeout_s: float) -> list[float]:
        """Wait for the first valid measured joint sample after receive reconnect."""
        deadline = time.monotonic() + max(0.0, timeout_s)
        last_exc: Exception | None = None
        while time.monotonic() <= deadline:
            try:
                actual_q = self._rtde_r.getActualQ()
                if isinstance(actual_q, (list, tuple)) and len(actual_q) >= 6:
                    return [float(v) for v in actual_q[:6]]
            except Exception as exc:  # noqa: BLE001
                last_exc = exc
            if self._startup_poll_s > 0.0:
                time.sleep(self._startup_poll_s)

        msg = f"RTDE receive connected to {self._host} but no valid actual_q arrived within {timeout_s:.1f}s"
        if last_exc is not None:
            raise RuntimeError(msg) from last_exc
        raise RuntimeError(msg)

    def set_target(self, q: list[float]) -> None:
        """Store the latest joint target that should be sent on the next servo tick."""
        if len(q) < 6:
            return
        self._target_q = [float(v) for v in q[:6]]

    def maybe_step(self) -> None:
        """Run one control tick, or the recovery flow if the robot is not ready."""
        if self._closed:
            return
        now = time.perf_counter()
        if now < self._next_control_attempt_t:
            return

        robot_not_ready = (
            self._protective_stopped
            or self._emergency_stopped
            or self._safety_stopped
            or self._program_running is False
            or not self._control_ok
        )
        if self._recovery_requested and not robot_not_ready:
            self._finish_recovery("recovery_not_needed")

        if robot_not_ready:
            self._control_ok = False
            self._maybe_attempt_dashboard_recovery(now)
            self._next_control_attempt_t = now + CONTROL_RETRY_BACKOFF_S
            return

        dt = now - self._last_send_t
        self._last_send_t = now
        servo_time = max(self._servo_dt, dt)
        try:
            self._rtde_c.servoJ(
                list(self._target_q),
                SERVOJ_SPEED,
                SERVOJ_ACCELERATION,
                servo_time,
                self._lookahead_time,
                self._gain,
            )
            self._last_send_err = None
            self._control_ok = True
            self._next_control_attempt_t = now
        except Exception as exc:  # noqa: BLE001
            self._last_send_err = str(exc)
            self._control_ok = False
            print(
                f"[robot_real_rtde] servoJ failed: {self._last_send_err}",
                flush=True,
            )
            self._next_control_attempt_t = now + CONTROL_RETRY_BACKOFF_S

    def read_state(self) -> tuple[list[float], list[float]]:
        """Return measured joint position/velocity and refresh cached status."""
        if self._closed:
            return list(self._actual_q), list(self._actual_qd)
        try:
            actual_q = self._rtde_r.getActualQ()
            if isinstance(actual_q, (list, tuple)) and len(actual_q) >= 6:
                self._actual_q = [float(v) for v in actual_q[:6]]
            if hasattr(self._rtde_r, "getActualQd"):
                actual_qd = self._rtde_r.getActualQd()
                if isinstance(actual_qd, (list, tuple)) and len(actual_qd) >= 6:
                    self._actual_qd = [float(v) for v in actual_qd[:6]]
            self._receive_ok = True
            self._refresh_receive_status()
        except Exception:  # noqa: BLE001
            self._receive_ok = False
        return list(self._actual_q), list(self._actual_qd)

    def close(self) -> None:
        """Stop motion and close the RTDE interfaces exactly once."""
        if self._closed:
            return
        self._closed = True
        self._close_rtde_interfaces(control=self._rtde_c, receive=self._rtde_r, log_errors=False)
