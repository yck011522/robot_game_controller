"""In-process jogging planner with explorer-style safety filter.

Ported from `archive/bullet_collision_keyboard_explorer.py`. Every
50 Hz GC tick this runs the same pipeline as the explorer's control
thread, but with the collision pool reached over ZMQ ROUTER/DEALER
instead of an in-process `ProcessPoolExecutor`.

Pipeline (per tick)
-------------------
1. **Desired velocity** from haptic intent. The planner derives
   velocity *internally* from per-tick position deltas:
       v_des[i] = gear[i] * (dial_pos[i] - prev_dial_pos[i]) / dt
   This keeps the contract identical for keyboard (which integrates
   key state into a virtual dial position) and real haptic dials
   (which only publish absolute encoder positions). The first tick
   after init or seed-reset uses v_des=0 because there is no prior
   sample yet.
2. **Acceleration clamp**: per axis, slew v_cur toward v_des by at
   most `max_accel * dt`.
3. **Velocity clamp**: per axis, saturate to +/- max_vel.
4. **Forward safety gate (SYNC)**: walk N_FORWARD_STEPS points along
   the unit direction of v_cmd, each one FORWARD_STEP_DEG away in
   joint space. Ask the collision pool whether any of them collides.
   Distance to the first hit drives `path_scalar` in [0..1]; an
   obstacle within PATH_CUTOFF_DEG forces it to 0. This is the gate:
   if the bundle reply doesn't arrive within `timeout_ms` we fall
   back to the previous tick's result (last-known-good) so a stalled
   worker pool doesn't cause hiccups during normal jogging.
5. **Proximity slowdown (ASYNC, round-robin)**: dispatch ONE axis per
   tick (20 probes of +/- PROBE_HALF_DEG around the current pose). The
   six axes refresh on a six-tick rotation (~8 Hz at 50 Hz). Combine
   the cached per-axis probes into a single global `prox_scalar`
   between PROX_FLOOR and 1.0 -- never zero (this is the soft
   slow-down, not a hard gate).
6. **Combine and integrate**:
       final = min(path_scalar, prox_scalar)
       v_out = v_cmd * final
       q_target = clamp(q_cur + v_out*dt, q_min, q_max)
       q_cur = q_target

The planner is stateful per team: it owns `q_cur` (the integrated
target) and `v_cur` (current velocity). On first plan() call,
`seed(q_actual)` should be called once with the robot's measured pose
so the integrator starts from the real state and there is no startup
snap.

Returned info dict
------------------
The `info` returned from `plan()` carries the two scalars and a few
diagnostic fields. GC forwards `path_scalar`, `prox_scalar`,
`final_scalar` in the cmd.robot.target envelope so robot_io / the sim
viewer can overlay them on the pybullet GUI (per user request: see
the clamps live while jogging the keyboard).
"""

from __future__ import annotations

import math
import time
import json
from typing import Optional

import zmq

from core import bus


# Joint limits / kinematics defaults
_DEFAULT_LIMIT_RAD = math.pi
_DEFAULT_HOME_DEG = [0.0, -90.0, 90.0, 0.0, 0.0, 0.0]
_DEFAULT_MAX_VEL_DPS = [20.0, 20.0, 20.0, 30.0, 30.0, 30.0]
_DEFAULT_MAX_ACCEL_DPS2 = [50.0, 50.0, 50.0, 80.0, 80.0, 80.0]

# Forward path check
N_FORWARD_STEPS = 12
FORWARD_STEP_DEG = 1.0
FORWARD_HORIZON_DEG = N_FORWARD_STEPS * FORWARD_STEP_DEG  # 12 deg total
PATH_CUTOFF_DEG = 3.0  # path_scalar = 0 if a hit lands within this distance

# Proximity probes (per axis, +/- PROBE_HALF_DEG, integer-degree steps)
PROBE_HALF_DEG = 10
PROBE_OFFSETS_DEG = [d for d in range(-PROBE_HALF_DEG, 0)] + [d for d in range(1, PROBE_HALF_DEG + 1)]
PROBE_OFFSETS_RAD = [math.radians(d) for d in PROBE_OFFSETS_DEG]
PROX_FLOOR = 0.5  # soft slowdown never drops below this

# Forward request timeout. Workers measure ~1 ms/config so a 12-config
# bundle returns in ~12 ms; allow a generous margin before falling back
# to last-known-good.
DEFAULT_FORWARD_TIMEOUT_MS = 18


class InProcessPlanner:
    def __init__(self, *, ctx: zmq.Context, profile, team: str,
                 collision_enabled: bool):
        self._team = team
        haptic = profile.tuning.get("haptic", {})
        self._gear = list(haptic.get("gear_ratio", [1.0] * 6))
        while len(self._gear) < 6:
            self._gear.append(1.0)

        robot_tune = profile.tuning.get("robot", {})
        q_min = robot_tune.get("q_limits_min_rad", [-_DEFAULT_LIMIT_RAD] * 6)
        q_max = robot_tune.get("q_limits_max_rad", [_DEFAULT_LIMIT_RAD] * 6)
        self._q_min = list(q_min)[:6]
        self._q_max = list(q_max)[:6]
        home_deg = robot_tune.get("initial_pose_deg", _DEFAULT_HOME_DEG)
        self._q_home = [math.radians(float(v)) for v in home_deg][:6]
        while len(self._q_home) < 6:
            self._q_home.append(0.0)
        max_vel_dps = robot_tune.get("max_velocity_deg_s", _DEFAULT_MAX_VEL_DPS)
        max_accel_dps2 = robot_tune.get("max_acceleration_deg_s2", _DEFAULT_MAX_ACCEL_DPS2)
        self._max_vel = [math.radians(float(v)) for v in max_vel_dps][:6]
        self._max_accel = [math.radians(float(a)) for a in max_accel_dps2][:6]
        while len(self._max_vel) < 6:
            self._max_vel.append(math.radians(20.0))
        while len(self._max_accel) < 6:
            self._max_accel.append(math.radians(50.0))

        # Integrator state.
        self._q_cur = list(self._q_home)
        self._v_cur = [0.0] * 6
        self._seeded = False
        # Previous dial position for internal velocity derivation. None
        # means "no prior sample" -> first tick uses v_des=0 to avoid
        # spiking off whatever the initial dial reading happens to be.
        self._prev_dial_pos: Optional[list[float]] = None

        # Cached collision results (rolled forward between ticks).
        self._fwd_hits: list[bool] = [False] * N_FORWARD_STEPS
        self._fwd_age_ticks = 9999
        self._prox_hits: list[list[bool]] = [
            [False] * len(PROBE_OFFSETS_DEG) for _ in range(6)
        ]
        self._prox_age_ticks: list[int] = [9999] * 6
        self._prox_rr = 0  # round-robin axis to dispatch next

        # In-flight DEALER request tracking (request_id -> kind).
        # kind = ("fwd", None) or ("prox", axis_idx).
        self._pending: dict[int, tuple[str, Optional[int]]] = {}
        self._req_seq = 0

        # We use DEALER (not REQ) so we can have multiple requests in
        # flight without the strict REQ/REP send-then-recv lockstep --
        # the forward gate and the per-axis proximity checks all share
        # this socket. Replies are matched back by request_id.
        coll_tune = profile.tuning.get("collision", {})
        self._timeout_s = float(coll_tune.get("timeout_ms", DEFAULT_FORWARD_TIMEOUT_MS)) / 1000.0
        self._collision_enabled = collision_enabled
        self._dealer: Optional[zmq.Socket] = None
        if collision_enabled:
            self._dealer = ctx.socket(zmq.DEALER)
            self._dealer.setsockopt(zmq.LINGER, 0)
            self._dealer.setsockopt_string(zmq.IDENTITY, f"jogging_planner.{team}")
            self._dealer.connect(bus.COLLISION_ROUTER_ENDPOINT)

    # ---- public API -----------------------------------------------------

    @property
    def q_cur(self) -> list[float]:
        """Current integrator pose (read-only snapshot)."""
        return list(self._q_cur)

    def seed(self, q_actual: list[float] | None) -> None:
        """One-time seed of the integrator from the robot's measured pose."""
        if self._seeded or q_actual is None or len(q_actual) < 6:
            return
        self._q_cur = [float(v) for v in q_actual[:6]]
        self._v_cur = [0.0] * 6
        self._seeded = True

    def plan(self, *, dial_pos_rad: list[float],
             dt: float) -> tuple[list[float], dict]:
        """Run one safety-filtered jog step."""
        if not self._seeded:
            self._seeded = True
        if dt <= 0.0:
            dt = 1.0 / 50.0

        # 1. Derive desired joint velocity from haptic position delta.
        #    Real haptic dials publish absolute positions only; the sim
        #    keyboard publishes a virtual integrated position that
        #    follows the same contract.
        v_des = [0.0] * 6
        cur_pos = [float(v) for v in (list(dial_pos_rad)[:6] + [0.0] * 6)][:6]
        if self._prev_dial_pos is not None:
            for i in range(6):
                d = (cur_pos[i] - self._prev_dial_pos[i]) / dt
                v_des[i] = self._gear[i] * d
        self._prev_dial_pos = cur_pos

        # 2. Per-axis acceleration clamp.
        v_cmd = [0.0] * 6
        for i in range(6):
            dv_max = self._max_accel[i] * dt
            delta = v_des[i] - self._v_cur[i]
            if delta > dv_max:
                delta = dv_max
            elif delta < -dv_max:
                delta = -dv_max
            v_cmd[i] = self._v_cur[i] + delta

        # 3. Per-axis velocity clamp.
        for i in range(6):
            if v_cmd[i] > self._max_vel[i]:
                v_cmd[i] = self._max_vel[i]
            elif v_cmd[i] < -self._max_vel[i]:
                v_cmd[i] = -self._max_vel[i]

        # 4-5. Collision pool: SYNC forward gate + ASYNC round-robin proximity.
        if self._dealer is not None:
            self._dispatch_forward(v_cmd)
            self._dispatch_one_proximity()
            self._wait_for_forward()
            self._harvest_replies_nonblocking()
            self._fwd_age_ticks += 1
            for i in range(6):
                self._prox_age_ticks[i] += 1

        path_scalar = self._compute_path_scalar()
        prox_scalar = self._compute_prox_scalar()
        final_scalar = min(path_scalar, prox_scalar)

        # 6. Integrate.
        v_out = [v_cmd[i] * final_scalar for i in range(6)]
        self._v_cur = list(v_out)
        q_target = [
            _clamp(self._q_cur[i] + v_out[i] * dt, self._q_min[i], self._q_max[i])
            for i in range(6)
        ]
        self._q_cur = list(q_target)

        in_collision = bool(self._fwd_hits[0]) if self._fwd_age_ticks <= 2 else False
        first_hit_step = None
        for k, h in enumerate(self._fwd_hits, start=1):
            if h:
                first_hit_step = k
                break

        info = {
            "path_scalar": path_scalar,
            "prox_scalar": prox_scalar,
            "final_scalar": final_scalar,
            "v_cmd_rad_s": v_cmd,
            "v_out_rad_s": v_out,
            "collision": in_collision,
            "collision_first_hit": (
                {"distance_deg": first_hit_step * FORWARD_STEP_DEG}
                if first_hit_step is not None else None
            ),
        }
        return q_target, info

    def close(self) -> None:
        if self._dealer is not None:
            self._dealer.close(0)
            self._dealer = None

    # ---- collision dispatch ---------------------------------------------

    def _dispatch_forward(self, v_cmd: list[float]) -> None:
        v_norm = math.sqrt(sum(v * v for v in v_cmd))
        if v_norm < 1e-6:
            # Stationary -> no forward path to check; mark clear.
            self._fwd_hits = [False] * N_FORWARD_STEPS
            self._fwd_age_ticks = 0
            return
        step_rad = math.radians(FORWARD_STEP_DEG)
        unit = [v / v_norm for v in v_cmd]
        step_vec = [u * step_rad for u in unit]
        configs = [
            [self._q_cur[i] + step_vec[i] * k for i in range(6)]
            for k in range(1, N_FORWARD_STEPS + 1)
        ]
        rid = self._send_request(configs)
        self._pending[rid] = ("fwd", None)

    def _dispatch_one_proximity(self) -> None:
        axis = self._prox_rr
        self._prox_rr = (self._prox_rr + 1) % 6
        configs = []
        for off in PROBE_OFFSETS_RAD:
            q = list(self._q_cur)
            q[axis] = q[axis] + off
            configs.append(q)
        rid = self._send_request(configs)
        self._pending[rid] = ("prox", axis)

    def _send_request(self, configs: list[list[float]]) -> int:
        assert self._dealer is not None
        self._req_seq += 1
        rid = self._req_seq
        env = bus.make_envelope(f"jogging_planner.{self._team}")
        env.update({
            "request_id": rid,
            "configs_rad": configs,
            "check_self": True,
            "check_world": True,
        })
        # REP behind a ROUTER->DEALER proxy expects the empty
        # delimiter that REQ would add automatically. DEALER doesn't,
        # so we prepend it by hand -- otherwise the worker silently
        # never replies and every safety scalar stays at 1.0.
        try:
            self._dealer.send_multipart([
                b"",
                b"req.collision_check",
                json.dumps(env, separators=(",", ":")).encode("utf-8"),
            ])
        except zmq.ZMQError:
            self._pending.pop(rid, None)
        return rid

    def _wait_for_forward(self) -> None:
        if self._dealer is None:
            return
        target_rid = max(
            (rid for rid, (kind, _) in self._pending.items() if kind == "fwd"),
            default=None,
        )
        if target_rid is None:
            return
        deadline = time.perf_counter() + self._timeout_s
        while time.perf_counter() < deadline:
            remaining_ms = max(0, int((deadline - time.perf_counter()) * 1000))
            if self._dealer.poll(remaining_ms) == 0:
                break
            body = self._recv_reply_nonblocking()
            if body is None:
                continue
            self._apply_reply(body)
            if target_rid not in self._pending:
                return

    def _harvest_replies_nonblocking(self) -> None:
        if self._dealer is None:
            return
        while True:
            body = self._recv_reply_nonblocking()
            if body is None:
                return
            self._apply_reply(body)

    def _recv_reply_nonblocking(self) -> Optional[dict]:
        """Receive one DEALER reply, stripping the empty delimiter.

        Wire shape coming back through ROUTER->DEALER from REP is
        [empty, topic, body] (REP re-prepends what it stripped). On a
        non-empty leading frame we tolerate the two-frame [topic,body]
        layout too -- some libzmq builds don't echo the delim.
        """
        assert self._dealer is not None
        try:
            frames = self._dealer.recv_multipart(flags=zmq.NOBLOCK)
        except zmq.Again:
            return None
        if not frames:
            return None
        if frames[0] == b"" and len(frames) >= 3:
            payload = frames[2]
        elif len(frames) >= 2:
            payload = frames[1]
        else:
            payload = frames[0]
        try:
            return json.loads(payload.decode("utf-8"))
        except Exception:
            return None

    def _apply_reply(self, body: dict) -> bool:
        rid = body.get("request_id")
        if not isinstance(rid, int):
            return False
        kind = self._pending.pop(rid, None)
        if kind is None:
            return False
        if not body.get("ok"):
            return True
        results = body.get("results") or []
        hits = [bool(r.get("collision")) for r in results]
        if kind[0] == "fwd":
            if len(hits) == N_FORWARD_STEPS:
                self._fwd_hits = hits
                self._fwd_age_ticks = 0
        elif kind[0] == "prox":
            axis = kind[1]
            if axis is not None and len(hits) == len(PROBE_OFFSETS_DEG):
                self._prox_hits[axis] = hits
                self._prox_age_ticks[axis] = 0
        return True

    # ---- scalar derivation ----------------------------------------------

    def _compute_path_scalar(self) -> float:
        if not self._collision_enabled:
            return 1.0
        first_hit_k: Optional[int] = None
        for k, h in enumerate(self._fwd_hits, start=1):
            if h:
                first_hit_k = k
                break
        if first_hit_k is None:
            return 1.0
        distance_deg = first_hit_k * FORWARD_STEP_DEG
        if distance_deg <= PATH_CUTOFF_DEG:
            return 0.0
        span = FORWARD_HORIZON_DEG - PATH_CUTOFF_DEG
        if span <= 0:
            return 1.0
        return max(0.0, min(1.0, (distance_deg - PATH_CUTOFF_DEG) / span))

    def _compute_prox_scalar(self) -> float:
        if not self._collision_enabled:
            return 1.0
        worst = 1.0
        for axis in range(6):
            results = self._prox_hits[axis]
            nearest_deg: Optional[int] = None
            for j, off in enumerate(PROBE_OFFSETS_DEG):
                if results[j]:
                    d = abs(off)
                    if nearest_deg is None or d < nearest_deg:
                        nearest_deg = d
            if nearest_deg is None:
                scalar = 1.0
            elif nearest_deg <= 1:
                scalar = PROX_FLOOR
            else:
                # Linear ramp: at PROBE_HALF_DEG -> 1.0, at 0 -> PROX_FLOOR.
                scalar = PROX_FLOOR + (1.0 - PROX_FLOOR) * (nearest_deg / PROBE_HALF_DEG)
            if scalar < worst:
                worst = scalar
        return worst


def _clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x
