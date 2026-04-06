"""Phase 6.3 — servoJ control loop test.

Connects to a UR robot (or simulator) via RTDE and moves the base joint
by 1 radian over 10 seconds (toward zero).  Prints current vs target
position throughout, then waits for the robot to settle at the final
target before exiting.

Prerequisites:
  - UR simulator running and robot powered ON + initialized (ON → START)
  - VirtualBox host-only adapter or bridged networking configured

Examples:
  python tests/test_servo_j.py --host 192.168.56.101
  python tests/test_servo_j.py --host 127.0.0.1
"""

from __future__ import annotations

import argparse
import math
import time

import rtde_control
import rtde_receive

SETTLE_TOLERANCE = 0.015   # rad — matches ~0.012 rad steady-state servo lag
SETTLE_TIMEOUT   = 5.0     # seconds max to wait after servo loop ends


def run_servo_j_test(host: str) -> None:
    print(f"Connecting to {host} ...")
    flags = rtde_control.RTDEControlInterface.FLAG_VERBOSE | rtde_control.RTDEControlInterface.FLAG_UPLOAD_SCRIPT
    rtde_c = rtde_control.RTDEControlInterface(host, 500.0, flags)
    rtde_r = rtde_receive.RTDEReceiveInterface(host)
    print("  Control + Receive interfaces connected.")

    # servoJ parameters
    velocity = 0.5
    acceleration = 0.5
    dt = 1.0 / 500        # 2 ms cycle time
    lookahead_time = 0.1   # [0.03 – 0.2] smoothing
    gain = 300             # [100 – 2000] tracking stiffness

    # Read current joint positions as the starting target
    joint_q = list(rtde_r.getActualQ())
    start_q0 = joint_q[0]

    # Move 1 rad toward zero (positive → subtract, negative → add)
    direction = -1.0 if start_q0 >= 0 else 1.0
    target_q0 = start_q0 + direction * 1.0

    print(f"  Starting base joint: {start_q0:+.4f} rad")
    print(f"  Target  base joint:  {target_q0:+.4f} rad  (delta {direction:+.1f} rad)")

    duration_s = 10.0
    iterations = int(duration_s / dt)  # 5000 iterations at 500 Hz
    step = (target_q0 - start_q0) / iterations
    print_every = 500  # print once per second at 500 Hz

    print(f"  Running {iterations}-iteration servoJ loop ({duration_s:.0f}s) ...\n")
    print(f"  {'Time':>6s}   {'Target':>9s}   {'Actual':>9s}   {'Error':>8s}")
    print(f"  {'─' * 6}   {'─' * 9}   {'─' * 9}   {'─' * 8}")

    t_wall_start = time.perf_counter()
    for i in range(iterations):
        t_start = rtde_c.initPeriod()
        rtde_c.servoJ(joint_q, velocity, acceleration, dt, lookahead_time, gain)
        joint_q[0] += step

        if i % print_every == 0 or i == iterations - 1:
            actual_q = rtde_r.getActualQ()
            elapsed = time.perf_counter() - t_wall_start
            error = actual_q[0] - joint_q[0]
            print(f"  {elapsed:6.2f}s   {joint_q[0]:+9.4f}   {actual_q[0]:+9.4f}   {error:+8.4f}")

        rtde_c.waitPeriod(t_start)

    t_wall_end = time.perf_counter()
    print(f"\n  Servo loop done — wall time: {t_wall_end - t_wall_start:.3f}s")

    # Stop servo mode, then wait for robot to settle at final target
    rtde_c.servoStop()
    print(f"  Waiting for robot to reach {target_q0:+.4f} rad (±{SETTLE_TOLERANCE} rad) ...")

    t_settle_start = time.perf_counter()
    while True:
        actual_q = rtde_r.getActualQ()
        error = actual_q[0] - target_q0
        elapsed_settle = time.perf_counter() - t_settle_start
        print(f"\r  Actual: {actual_q[0]:+.4f}   Error: {error:+.4f}   ({elapsed_settle:.1f}s)", end="")

        if abs(error) <= SETTLE_TOLERANCE:
            print(f"\n  Robot settled at {actual_q[0]:+.4f} rad ✓")
            break
        if elapsed_settle > SETTLE_TIMEOUT:
            print(f"\n  Settle timeout after {SETTLE_TIMEOUT:.0f}s — final error: {error:+.4f} rad")
            break
        time.sleep(0.05)

    # Clean shutdown
    rtde_c.stopScript()
    rtde_r.disconnect()
    print("servoJ test complete ✓")


def main() -> None:
    parser = argparse.ArgumentParser(description="Phase 6.3 — servoJ control loop test")
    parser.add_argument("--host", default="192.168.56.101",
                        help="Robot/simulator IP (default: 192.168.56.101)")
    args = parser.parse_args()
    run_servo_j_test(args.host)


if __name__ == "__main__":
    main()
