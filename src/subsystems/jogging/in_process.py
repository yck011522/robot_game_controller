"""In-process jogging planner.

Takes the latest haptic sample for a team, converts dial positions into
a joint-space target, optionally runs a collision check against the
worker pool, and returns the target the robot should track this tick.

The transform is intentionally trivial for P2:

    q_target[i] = clamp(gear_ratio[i] * dial_pos[i] + q_home[i],
                        q_min[i], q_max[i])

with `gear_ratio` from `tuning.haptic.gear_ratio`, `q_home` from
`tuning.robot.initial_pose_deg` (so `dial=0` parks the robot at the
collision-free home pose instead of the all-zero scene pose), and
joint limits from `tuning.robot.q_limits_{min,max}_rad` (defaults to
±180° per joint). Velocity / acceleration limits land in a later phase
(`tuning.robot.max_velocity_*` already exists in the profile schema
but the integrator code that needs them does not).

Collision check is best-effort: if the planner has a REQ socket
configured and the worker pool responds within the bundle timeout, a
hit is reported back to the caller; if anything goes wrong (timeout,
worker pool empty, error reply) we let the target through. The
gameplay-blocking behavior described in CONFIG.md §5.4 (refuse motion
on persistent collision-check failure) lives in GameController on top
of this hint.
"""

from __future__ import annotations

import json
import math
import time
from typing import Optional

import zmq

from core import bus


_DEFAULT_LIMIT_RAD = 3.14159  # ±180°
_DEFAULT_HOME_DEG = [0.0, -90.0, 90.0, 0.0, 0.0, 0.0]


class InProcessPlanner:
    def __init__(self, *, ctx: zmq.Context, profile, team: str,
                 collision_enabled: bool):
        self._team = team
        haptic = profile.tuning.get("haptic", {})
        self._gear = list(haptic.get("gear_ratio", [1.0] * 6))
        robot_tune = profile.tuning.get("robot", {})
        q_min = robot_tune.get("q_limits_min_rad", [-_DEFAULT_LIMIT_RAD] * 6)
        q_max = robot_tune.get("q_limits_max_rad", [_DEFAULT_LIMIT_RAD] * 6)
        self._q_min = list(q_min)
        self._q_max = list(q_max)
        home_deg = robot_tune.get("initial_pose_deg", _DEFAULT_HOME_DEG)
        self._q_home = [math.radians(float(v)) for v in home_deg][:6]
        while len(self._q_home) < 6:
            self._q_home.append(0.0)

        self._collision_enabled = collision_enabled
        self._req: Optional[zmq.Socket] = None
        self._req_seq = 0
        self._timeout_ms = int(profile.tuning.get("collision", {}).get("timeout_ms", 80))
        self._check_self = bool(profile.tuning.get("collision", {}).get("check_self", True))
        self._check_world = bool(profile.tuning.get("collision", {}).get("check_world", True))
        if self._collision_enabled:
            self._req = ctx.socket(zmq.REQ)
            self._req.setsockopt(zmq.LINGER, 0)
            # Tell ZMQ to allow recovery from a missed REP without the
            # REQ socket becoming wedged.
            self._req.setsockopt(zmq.REQ_RELAXED, 1)
            self._req.setsockopt(zmq.REQ_CORRELATE, 1)
            self._req.connect(bus.COLLISION_ROUTER_ENDPOINT)

    def plan(self, dial_pos_rad: list[float]) -> tuple[list[float], dict]:
        """Return `(q_target_rad, info)`. `info` carries diagnostic
        fields the caller can fold into state.full.
        """
        q = [
            _clamp(self._gear[i] * dial_pos_rad[i] + self._q_home[i],
                   self._q_min[i], self._q_max[i])
            for i in range(min(6, len(dial_pos_rad)))
        ]
        # Pad if dial sample was short.
        while len(q) < 6:
            q.append(self._q_home[len(q)] if len(q) < len(self._q_home) else 0.0)

        info: dict = {"collision_checked": False, "collision": False,
                      "collision_first_hit": None, "collision_compute_ms": 0.0}
        if self._req is not None:
            ok, body = self._collision_request([q])
            info["collision_checked"] = True
            if ok and body.get("ok"):
                results = body.get("results") or [{}]
                first = results[0]
                info["collision"] = bool(first.get("collision"))
                info["collision_first_hit"] = first.get("first_hit")
                info["compute_ms"] = float(body.get("compute_ms", 0.0))
        return q, info

    def close(self) -> None:
        if self._req is not None:
            self._req.close(0)
            self._req = None

    # ---- internals ------------------------------------------------------
    def _collision_request(self, configs: list[list[float]]) -> tuple[bool, dict]:
        assert self._req is not None
        self._req_seq += 1
        env = bus.make_envelope(f"jogging_planner.{self._team}")
        env.update({
            "request_id": self._req_seq,
            "configs_rad": configs,
            "check_self": self._check_self,
            "check_world": self._check_world,
        })
        try:
            bus.publish(self._req, "req.collision_check", env)
        except zmq.ZMQError:
            return False, {}
        # Poll with timeout — REQ_RELAXED lets us recover next round
        # without resetting the socket if the reply doesn't come.
        if self._req.poll(self._timeout_ms) == 0:
            return False, {}
        try:
            _, body = bus.recv(self._req, flags=zmq.NOBLOCK)
            return True, body
        except zmq.Again:
            return False, {}


def _clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x
