"""Focused P3 test for the real RTDE RobotIO backend.

Uses a stub `rtde_helpers` module so the startup-sync and `servoJ`
path can be validated without a real robot or the external `ur_rtde`
dependency.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
SRC = REPO_ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from subsystems.robot.robot_real_rtde import (  # noqa: E402
    DEFAULT_LOOKAHEAD_TIME,
    DEFAULT_SERVO_GAIN,
    RealRtdeRobot,
)


class _FakeReceive:
    def __init__(self) -> None:
        self.q = [0.1, -1.2, 1.3, 0.4, -0.5, 0.6]
        self.qd = [0.0] * 6
        self.disconnected = False

    def getActualQ(self) -> list[float]:
        return list(self.q)

    def getActualQd(self) -> list[float]:
        return list(self.qd)

    def disconnect(self) -> None:
        self.disconnected = True


class _FakeControl:
    def __init__(self) -> None:
        self.servo_calls: list[tuple] = []
        self.stopped = False
        self.script_stopped = False

    def servoJ(self, q, speed, acceleration, dt, lookahead_time, gain) -> None:
        self.servo_calls.append((list(q), speed, acceleration, dt, lookahead_time, gain))

    def servoStop(self) -> None:
        self.stopped = True

    def stopScript(self) -> None:
        self.script_stopped = True


class _FakeRtdeCore:
    def __init__(self) -> None:
        self.receive = _FakeReceive()
        self.control = _FakeControl()
        self.connect_receive_calls: list[str] = []
        self.connect_control_calls: list[tuple[str, float]] = []

    def connect_receive(self, host: str):
        self.connect_receive_calls.append(host)
        return self.receive

    def connect_control(self, host: str, frequency_hz: float = -1.0):
        self.connect_control_calls.append((host, frequency_hz))
        return self.control


def main() -> int:
    fake = _FakeRtdeCore()
    robot = RealRtdeRobot(
        host="192.168.0.2",
        servo_hz=200.0,
        startup_timeout_s=0.1,
        startup_poll_s=0.0,
        _rtde_helpers=fake,
    )

    q0, qd0 = robot.read_state()
    assert q0 == fake.receive.q, f"initial actual_q mismatch: {q0}"
    assert qd0 == fake.receive.qd, f"initial actual_qd mismatch: {qd0}"
    assert fake.connect_receive_calls == ["192.168.0.2"]
    assert fake.connect_control_calls == [("192.168.0.2", 200.0)]
    assert robot.rtde_ok is True

    target = [v + 0.05 for v in q0]
    robot.set_target(target)
    robot.maybe_step()

    assert len(fake.control.servo_calls) == 1, "servoJ was not called"
    sent_q, speed, accel, dt, lookahead, gain = fake.control.servo_calls[-1]
    assert sent_q == target, f"servoJ sent wrong target: {sent_q}"
    assert math.isclose(speed, 0.5)
    assert math.isclose(accel, 0.5)
    assert dt >= 1.0 / 200.0, f"servo dt too small: {dt}"
    assert math.isclose(lookahead, DEFAULT_LOOKAHEAD_TIME)
    assert gain == DEFAULT_SERVO_GAIN

    fake.receive.q = [v + 0.01 for v in target]
    fake.receive.qd = [0.2] * 6
    q1, qd1 = robot.read_state()
    assert q1 == fake.receive.q, f"updated actual_q mismatch: {q1}"
    assert qd1 == fake.receive.qd, f"updated actual_qd mismatch: {qd1}"

    robot.close()
    assert fake.control.stopped is True
    assert fake.control.script_stopped is True
    assert fake.receive.disconnected is True

    print("P3 RTDE backend smoke test passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())