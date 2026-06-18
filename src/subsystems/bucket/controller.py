"""Bucket command handling, status polling, and per-motor watchdogs."""

from __future__ import annotations

import time
from collections import deque
from typing import Any, Callable

from subsystems.bucket.common import (
    BUCKET_LABELS,
    ActiveBucketCommand,
    BucketCommandResult,
    BucketMotorConfig,
    Direction,
    MotorStatus,
    bucket_label,
    normalize_bucket_label,
)


DEFAULT_CLIENT_TIMEOUT_S = 0.2  # Per-Modbus request timeout; keeps stop/watchdog actions responsive.
DEFAULT_OPEN_DIRECTION: Direction = "negative"  # Tested physical direction that opens bucket doors.
DEFAULT_CLOSE_DIRECTION: Direction = "positive"  # Tested physical direction that closes bucket doors.
DEFAULT_MOTOR_SPEED = 8  # Profile fallback motor speed, valid 1-15; profiles may tune this during bring-up.
DEFAULT_COMMAND_TIMEOUT_S = 10.0  # Watchdog timeout before a moving bucket receives an explicit stop.
DEFAULT_STATUS_POLL_HZ = 2.0  # Full six-bucket scans per second; revise here after RS-485 saturation tests.
DEFAULT_INTER_REQUEST_DELAY_S = 0.002  # Quiet gap between Modbus requests for this controller bus.


class BucketControllerRuntime:
    """Stateful runtime for six bucket motors independent of ZMQ transport."""

    def __init__(
        self,
        *,
        driver: Any,
        config: BucketMotorConfig,
        now_fn: Callable[[], float] = time.perf_counter,
    ) -> None:
        self.driver = driver  # Concrete real or simulated motor driver.
        self.config = config  # Direction, speed, address, poll, and timeout settings.
        self._now = now_fn  # Monotonic clock injection used by tests.
        self._active: dict[str, ActiveBucketCommand] = {}  # In-flight commands watched by timeout.
        self._last_status: dict[str, MotorStatus | None] = {label: None for label in BUCKET_LABELS}
        self._last_result: dict[str, BucketCommandResult | None] = {label: None for label in BUCKET_LABELS}
        self._last_error: dict[str, str | None] = {label: None for label in BUCKET_LABELS}
        self._next_status_scan_s = 0.0  # Next monotonic time when a full six-motor scan is allowed.
        self._scan_started_s = 0.0  # Start time of the most recent status scan.
        self._last_scan_duration_ms = 0.0  # Duration of the latest full status scan for saturation tuning.
        self._scan_times: deque[float] = deque(maxlen=30)  # Recent scan timestamps for observed scan rate.

    def handle_command(self, body: dict[str, Any]) -> list[BucketCommandResult]:
        """Validate and apply one ``cmd.bucket`` payload."""

        action = str(body.get("action") or "").strip().lower()
        request_id = body.get("request_id")
        try:
            labels = self._labels_for_command(body, action)
        except ValueError as exc:
            return [BucketCommandResult(False, "", action or "unknown", request_id, str(exc))]

        results: list[BucketCommandResult] = []
        for label in labels:
            if action in ("open", "close"):
                results.append(self._start_motion(label, action, request_id))
            elif action == "stop":
                results.append(self._stop(label, action, request_id, message="stop requested"))
            elif action in ("open_all", "close_all"):
                single_action = "open" if action == "open_all" else "close"
                results.append(self._start_motion(label, single_action, request_id))
            elif action == "stop_all":
                results.append(self._stop(label, action, request_id, message="stop_all requested"))
            else:
                results.append(BucketCommandResult(False, label, action, request_id, f"unsupported action {action!r}"))
        return results

    def tick(self) -> None:
        """Run watchdog checks and status polling for the current process tick."""

        self._stop_timed_out_commands()
        now = self._now()
        if now < self._next_status_scan_s:
            return
        self._scan_started_s = now
        for label in BUCKET_LABELS:
            status = self.driver.read_status(label)
            self._last_status[label] = status
            if status is None:
                self._last_error[label] = "no status response"
                continue
            self._last_error[label] = None
            active = self._active.get(label)
            if active is not None and (status.state == "stopped" or status.at_limit):
                self._last_result[label] = BucketCommandResult(
                    True,
                    label,
                    active.action,
                    active.request_id,
                    f"{active.action} completed: {status.description}",
                )
                self._active.pop(label, None)
        finished = self._now()
        self._last_scan_duration_ms = max(0.0, (finished - self._scan_started_s) * 1000.0)
        self._scan_times.append(finished)
        self._next_status_scan_s = finished + self.config.status_poll_interval_s

    def stop_all(self, *, request_id: str | int | None = None, message: str = "shutdown stop_all") -> None:
        """Stop every bucket motor and clear all active watchdog timers."""

        for label in BUCKET_LABELS:
            self._stop(label, "stop_all", request_id, message=message)

    def snapshot(self) -> dict[str, Any]:
        """Return a JSON-serializable telemetry snapshot."""

        return {
            "connected": bool(getattr(self.driver, "connected", False)),
            "active_count": len(self._active),
            "status_poll_interval_s": self.config.status_poll_interval_s,
            "last_scan_duration_ms": self._last_scan_duration_ms,
            "observed_status_scan_hz": self.observed_status_scan_hz(),
            "buckets": {
                label: self._bucket_snapshot(label)
                for label in BUCKET_LABELS
            },
        }

    def observed_status_scan_hz(self) -> float:
        """Return the observed full-bus status scan frequency."""

        if len(self._scan_times) < 2:
            return 0.0
        span_s = self._scan_times[-1] - self._scan_times[0]
        if span_s <= 0.0:
            return 0.0
        return (len(self._scan_times) - 1) / span_s

    def _labels_for_command(self, body: dict[str, Any], action: str) -> list[str]:
        """Resolve a command payload to one or more logical bucket labels."""

        if action in ("open_all", "close_all", "stop_all"):
            return list(BUCKET_LABELS)
        if body.get("bucket_label") is not None:
            return [normalize_bucket_label(body.get("bucket_label"))]
        team = body.get("team")
        bucket_number = body.get("bucket_number")
        if team is None or bucket_number is None:
            raise ValueError("single-bucket commands require bucket_label or team + bucket_number")
        return [bucket_label(str(team), int(bucket_number))]

    def _start_motion(self, label: str, action: str, request_id: str | int | None) -> BucketCommandResult:
        """Send an open/close motor command and arm its watchdog timer."""

        direction = self.config.open_direction if action == "open" else self.config.close_direction
        ok = bool(self.driver.move(label, direction, self.config.speed))
        message = f"{action} sent {direction}" if ok else f"{action} failed {direction}"
        result = BucketCommandResult(ok, label, action, request_id, message)
        self._last_result[label] = result
        if ok:
            self._active[label] = ActiveBucketCommand(
                action=action,
                direction=direction,
                request_id=request_id,
                started_mono_s=self._now(),
                timeout_s=self.config.command_timeout_s,
            )
        return result

    def _stop(self, label: str, action: str, request_id: str | int | None, *, message: str) -> BucketCommandResult:
        """Send an immediate stop command and clear this label's watchdog."""

        ok = bool(self.driver.stop(label))
        result = BucketCommandResult(ok, label, action, request_id, message if ok else f"{message} failed")
        self._active.pop(label, None)
        self._last_result[label] = result
        return result

    def _stop_timed_out_commands(self) -> None:
        """Stop any motor that has exceeded its per-command watchdog timeout."""

        now = self._now()
        for label, active in list(self._active.items()):
            if now - active.started_mono_s < active.timeout_s:
                continue
            stopped = bool(self.driver.stop(label))
            message = (
                f"watchdog stopped {active.action} after {active.timeout_s:.1f}s"
                if stopped
                else f"watchdog stop failed after {active.timeout_s:.1f}s"
            )
            self._last_result[label] = BucketCommandResult(stopped, label, active.action, active.request_id, message)
            self._last_error[label] = message
            self._active.pop(label, None)
            # TODO(bucket-controller): surface repeated watchdog trips as an
            # operator-visible warning once the admin alert path exists.

    def _bucket_snapshot(self, label: str) -> dict[str, Any]:
        """Return telemetry fields for one logical bucket label."""

        status = self._last_status.get(label)
        active = self._active.get(label)
        result = self._last_result.get(label)
        return {
            "address": self.config.addresses.get(label),
            "status": _status_to_dict(status),
            "active_command": _active_to_dict(active),
            "last_result": _result_to_dict(result),
            "last_error": self._last_error.get(label),
        }


def _status_to_dict(status: MotorStatus | None) -> dict[str, Any] | None:
    """Convert a motor status dataclass into a JSON-ready mapping."""

    if status is None:
        return None
    return {
        "raw": status.raw,
        "state": status.state,
        "direction": status.direction,
        "speed": status.speed,
        "is_moving": status.is_moving,
        "at_limit": status.at_limit,
        "description": status.description,
    }


def _active_to_dict(active: ActiveBucketCommand | None) -> dict[str, Any] | None:
    """Convert active watchdog state into a JSON-ready mapping."""

    if active is None:
        return None
    return {
        "action": active.action,
        "direction": active.direction,
        "request_id": active.request_id,
        "started_mono_s": active.started_mono_s,
        "timeout_s": active.timeout_s,
    }


def _result_to_dict(result: BucketCommandResult | None) -> dict[str, Any] | None:
    """Convert a command result into a JSON-ready mapping."""

    if result is None:
        return None
    return {
        "ok": result.ok,
        "label": result.label,
        "action": result.action,
        "request_id": result.request_id,
        "message": result.message,
    }
