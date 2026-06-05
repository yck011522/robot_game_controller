"""Pybullet viewer that mirrors telem.robot.actual.<team>.

Use this alongside `dev_one_robot.yaml` so the real RTDE-backed
RobotIO stays authoritative and the viewer remains a passive bus
consumer.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
SRC = REPO_ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

import zmq  # noqa: E402

from core import bus  # noqa: E402
from core.config import load as load_profile  # noqa: E402
from subsystems.robot.robot_sim_pybullet import SimPybulletRobot  # noqa: E402


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Passive pybullet viewer for telem.robot.actual.<team>")
    ap.add_argument("--profile", default=str(REPO_ROOT / "config" / "profiles" / "dev_one_robot.yaml"))
    ap.add_argument("--team", default="b", choices=["a", "b"])
    ap.add_argument("--headless", action="store_true",
                    help="Run without the pybullet GUI window")
    args = ap.parse_args(argv)

    profile = load_profile(args.profile)
    initial_pose_deg = profile.tuning.get("robot", {}).get(
        "initial_pose_deg", [0.0, -90.0, 90.0, 0.0, 0.0, 0.0]
    )
    import math as _math

    initial_pose_rad = [_math.radians(float(v)) for v in initial_pose_deg]
    viewer = SimPybulletRobot(headless=bool(args.headless),
                              initial_pose_rad=initial_pose_rad)

    ctx = zmq.Context.instance()
    sub = bus.make_sub(ctx, topics=[f"telem.robot.actual.{args.team}"])
    poller = zmq.Poller()
    poller.register(sub, zmq.POLLIN)
    last_print = 0.0

    print(f"[robot_viewer] subscribed to telem.robot.actual.{args.team}", flush=True)
    try:
        while True:
            events = dict(poller.poll(200))
            if sub in events:
                while True:
                    try:
                        _, body = bus.recv(sub, flags=zmq.NOBLOCK)
                    except zmq.Again:
                        break
                    q = body.get("q_rad")
                    if isinstance(q, list) and len(q) >= 6:
                        viewer.set_target([float(v) for v in q[:6]])

            now = time.perf_counter()
            if now - last_print >= 2.0:
                q, _ = viewer.read_state()
                deg = ", ".join(f"{_math.degrees(v):+.1f}" for v in q)
                print(f"[robot_viewer] pose_deg={deg}", flush=True)
                last_print = now
    except KeyboardInterrupt:
        return 0
    finally:
        sub.close(0)
        ctx.destroy(linger=0)
        viewer.close()


if __name__ == "__main__":
    raise SystemExit(main())