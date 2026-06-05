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


class RealRtdeRobot:
    def __init__(
        self,
        *,
        host: str,
        port: int | None = None,
        servo_hz: float = DEFAULT_SERVO_HZ,
        lookahead_time: float = DEFAULT_LOOKAHEAD_TIME,
        gain: int = DEFAULT_SERVO_GAIN,
        startup_timeout_s: float = 5.0,
        startup_poll_s: float = 0.05,
        _rtde_helpers: Any | None = None,
    ) -> None:
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
        self._rtde_ok = False
        self._last_send_err: str | None = None

        self._rtde_r = self._rtde_helpers.connect_receive(host)
        self._actual_q = self._wait_for_initial_actual_q(float(startup_timeout_s))
        self._actual_qd = [0.0] * len(self._actual_q)
        self._target_q = list(self._actual_q)
        self._last_send_t = time.perf_counter()
        self._rtde_c = self._rtde_helpers.connect_control(host, frequency_hz=self._servo_hz)
        self._rtde_ok = True

    @property
    def rtde_ok(self) -> bool:
        return self._rtde_ok and not self._closed

    def _wait_for_initial_actual_q(self, timeout_s: float) -> list[float]:
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
        if len(q) < 6:
            return
        self._target_q = [float(v) for v in q[:6]]

    def maybe_step(self) -> None:
        if self._closed:
            return
        now = time.perf_counter()
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
            self._rtde_ok = True
        except Exception as exc:  # noqa: BLE001
            self._last_send_err = str(exc)
            self._rtde_ok = False

    def read_state(self) -> tuple[list[float], list[float]]:
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
            self._rtde_ok = True
        except Exception:  # noqa: BLE001
            self._rtde_ok = False
        return list(self._actual_q), list(self._actual_qd)

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        try:
            self._rtde_c.servoStop()
        except Exception:  # noqa: BLE001
            pass
        try:
            self._rtde_c.stopScript()
        except Exception:  # noqa: BLE001
            pass
        try:
            self._rtde_r.disconnect()
        except Exception:  # noqa: BLE001
            pass