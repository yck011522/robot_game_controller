"""In-process jogging planner with explorer-style safety filter.

Ported from `archive/bullet_collision_keyboard_explorer.py`. Every
game_controller tick this runs the same pipeline as the explorer's
control thread, but with the collision pool reached over ZMQ
ROUTER/DEALER instead of an in-process `ProcessPoolExecutor`.

Pipeline (per tick)
-------------------
1. **Desired velocity** from haptic intent. The planner derives
   velocity *internally* from per-tick dial-position deltas:
       v_des[i] = gear[i] * (dial_pos[i] - prev_dial_pos[i]) / dt
   This keeps the contract identical for keyboard (which integrates
   key state into a virtual dial position) and real haptic dials
   (which only publish absolute encoder positions). The first tick
   after init/seed uses v_des=0 because there is no prior sample.
2. **Acceleration clamp**: per axis, slew v_cur toward v_des by at
   most `max_accel * dt`.
3. **Velocity clamp**: per axis, saturate to +/- max_vel.
4. **Forward safety gate (HARD, blocking)**: walk N forward steps
   along the unit direction of v_cmd, each FORWARD_STEP_DEG away in
   joint space, split into `forward_bundle_size`-config chunks and
   dispatched in parallel to the worker pool. We wait up to
   `forward_timeout_ms` for ALL chunks to reply. If any chunk is
   missing, path_scalar = 0 (motion stopped). There is NO
   last-known-good fallback -- every commanded step must be
   certified-clear by a reply that arrived this tick.
5. **Proximity slowdown (SOFT, async round-robin)**: one axis per
   tick (20 probes of +/- PROBE_HALF_DEG around the current pose).
   Six axes refresh on a six-tick rotation. The combined prox_scalar
   is clamped to [PROX_FLOOR, 1.0] -- never zero. A stale axis is
   treated as clear (soft-by-default) because this is a comfort
   slowdown, not a safety gate.
6. **Combine and integrate**:
       final = min(path_scalar, prox_scalar)
       v_out = v_cmd * final
       q_target = clamp(q_cur + v_out*dt, q_min, q_max)
       q_cur = q_target

Speed-clamp units
-----------------
`path_scalar`, `prox_scalar`, `final_scalar` are dimensionless
attenuation factors in [0.0, 1.0]. They multiply the already
accel-and-velocity-clamped `v_cmd` (in rad/s). They are NOT
"percent of max velocity" -- v_cmd is already the post-clamp value
that respects `tuning.robot.max_velocity_deg_s`. Examples:

    final=1.0  -> robot moves at v_cmd (the user's full requested rate)
    final=0.5  -> robot moves at half v_cmd (soft prox slowdown)
    final=0.0  -> robot frozen (forward path blocked or not certified)

The full chain is:
    v_des  (haptic intent, gear-scaled)
      -> v_accel_clamp  (accel limit per axis)
      -> v_cmd  (vel limit per axis)
      -> v_out = v_cmd * final_scalar  (safety attenuation)
      -> q_target = q_cur + v_out * dt
"""

from __future__ import annotations

import math
import time
import json
from typing import Optional

import zmq

from core import bus
from subsystems.robot.joint_limits import resolve_joint_limits_rad


# --- Defaults (overridden by profile.tuning.jogging / .robot / .collision) ---

# Robot kinematics defaults (used only when tuning.robot omits them).
_DEFAULT_LIMIT_RAD = math.pi
_DEFAULT_MAX_VEL_DPS = [20.0, 20.0, 20.0, 30.0, 30.0, 30.0]
_DEFAULT_MAX_ACCEL_DPS2 = [50.0, 50.0, 50.0, 80.0, 80.0, 80.0]

# Jogging-planner defaults. Profile keys (under `tuning.jogging:`):
#   n_forward_steps        : how many points along v_cmd direction
#   forward_step_deg       : spacing between forward-path points
#   path_cutoff_deg        : path_scalar=0 if first hit within this
#   forward_bundle_size    : configs per worker request (parallelism)
#   probe_half_deg         : prox probes span +/- this many degrees
#   prox_floor             : prox_scalar never drops below this
#   forward_timeout_ms     : hard deadline; missing -> path_scalar=0
_DEFAULTS_JOG = {
    "n_forward_steps": 12,
    "forward_step_deg": 1.0,
    "path_cutoff_deg": 3.0,
    "forward_bundle_size": 3,
    "probe_half_deg": 10,
    "prox_floor": 0.5,
    "forward_timeout_ms": 25,
}


def _chunks(seq: list, size: int) -> list[list]:
    if size <= 0:
        return [list(seq)]
    return [seq[i:i + size] for i in range(0, len(seq), size)]


class InProcessPlanner:
    def __init__(self, *, ctx: zmq.Context, profile, team: str,
                 collision_enabled: bool):
        self._team = team

        # ---- haptic / kinematics from profile -----------------------
        haptic = profile.tuning.get("haptic", {})
        self._gear = list(haptic.get("gear_ratio", [1.0] * 6))
        while len(self._gear) < 6:
            self._gear.append(1.0)

        robot_tune = profile.tuning.get("robot", {})
        self._q_min, self._q_max = resolve_joint_limits_rad(robot_tune, axes=6)
        max_vel_dps = robot_tune.get("max_velocity_deg_s", _DEFAULT_MAX_VEL_DPS)
        max_accel_dps2 = robot_tune.get("max_acceleration_deg_s2", _DEFAULT_MAX_ACCEL_DPS2)
        self._max_vel = [math.radians(float(v)) for v in max_vel_dps][:6]
        self._max_accel = [math.radians(float(a)) for a in max_accel_dps2][:6]
        while len(self._max_vel) < 6:
            self._max_vel.append(math.radians(20.0))
        while len(self._max_accel) < 6:
            self._max_accel.append(math.radians(50.0))

        # ---- jogging-planner tunables from profile ------------------
        jog = dict(_DEFAULTS_JOG)
        jog.update(profile.tuning.get("jogging", {}) or {})
        self._n_fwd = int(jog["n_forward_steps"])
        self._fwd_step_deg = float(jog["forward_step_deg"])
        self._fwd_horizon_deg = self._n_fwd * self._fwd_step_deg
        self._path_cutoff_deg = float(jog["path_cutoff_deg"])
        self._fwd_bundle_size = max(1, int(jog["forward_bundle_size"]))
        self._probe_half_deg = int(jog["probe_half_deg"])
        self._prox_floor = float(jog["prox_floor"])
        self._probe_offsets_deg = (
            list(range(-self._probe_half_deg, 0)) +
            list(range(1, self._probe_half_deg + 1))
        )
        self._probe_offsets_rad = [math.radians(d) for d in self._probe_offsets_deg]
        self._n_probes = len(self._probe_offsets_deg)

        # Collision timeout: legacy `tuning.collision.timeout_ms` still
        # wins if present (back-compat with existing profiles).
        coll_tune = profile.tuning.get("collision", {}) or {}
        timeout_ms = coll_tune.get("timeout_ms", jog["forward_timeout_ms"])
        self._timeout_s = float(timeout_ms) / 1000.0

        # ---- integrator + cache state -------------------------------
        self._q_cur = [0.0] * 6
        self._v_cur = [0.0] * 6
        self._seeded = False
        self._prev_dial_pos: Optional[list[float]] = None

        # Forward results are computed FRESH every tick from this
        # tick's replies; no last-known-good cache. Proximity results
        # are cached per axis (stale-clear semantics, soft slowdown).
        self._prox_hits: list[list[bool]] = [
            [False] * self._n_probes for _ in range(6)
        ]
        self._prox_age_ticks: list[int] = [9999] * 6
        self._prox_rr = 0

        # Per-tick forward group bookkeeping. group_id increments each
        # tick; chunks for older groups are dropped on arrival.
        self._fwd_group_id = 0
        self._fwd_group_hits: list[Optional[bool]] = [None] * self._n_fwd
        self._fwd_group_pending: set[int] = set()

        # In-flight request tracking.
        # pending[rid] = ("fwd", group_id, chunk_start, chunk_len)
        #             or ("prox", axis_idx, 0, 0)
        self._pending: dict[int, tuple[str, int, int, int]] = {}
        self._req_seq = 0

        # DEALER -- multiple in-flight requests, replies matched by rid.
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
        return list(self._q_cur)

    def seed(self, q_actual: list[float] | None) -> None:
        if self._seeded or q_actual is None or len(q_actual) < 6:
            return
        self._q_cur = [float(v) for v in q_actual[:6]]
        self._v_cur = [0.0] * 6
        self._seeded = True

    def plan(self, *, dial_pos_rad: list[float],
             dt: float) -> tuple[list[float], dict]:
        if dt <= 0.0:
            dt = 1.0 / 50.0

        # 1. Derive desired joint velocity from haptic position delta.
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

        # 4-5. Collision pool: HARD forward gate + SOFT round-robin prox.
        forward_certified = False
        if self._dealer is not None and self._collision_enabled:
            self._dispatch_forward(v_cmd)
            self._dispatch_one_proximity()
            forward_certified = self._wait_for_forward()
            self._harvest_replies_nonblocking()
            for i in range(6):
                self._prox_age_ticks[i] += 1
        elif not self._collision_enabled:
            forward_certified = True

        # Stationary -> nothing to certify.
        if math.sqrt(sum(v * v for v in v_cmd)) < 1e-6:
            forward_certified = True

        path_scalar = self._compute_path_scalar(forward_certified)
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

        in_collision = (
            forward_certified
            and len(self._fwd_group_hits) > 0
            and bool(self._fwd_group_hits[0])
        )
        first_hit_step = None
        for k, h in enumerate(self._fwd_group_hits, start=1):
            if h:
                first_hit_step = k
                break

        info = {
            "path_scalar": path_scalar,
            "prox_scalar": prox_scalar,
            "final_scalar": final_scalar,
            "v_cmd_rad_s": v_cmd,
            "v_out_rad_s": v_out,
            "forward_certified": forward_certified,
            "collision": bool(in_collision),
            "collision_first_hit": (
                {"distance_deg": first_hit_step * self._fwd_step_deg}
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
        # New group; reset assembly buffers.
        self._fwd_group_id += 1
        gid = self._fwd_group_id
        self._fwd_group_hits = [None] * self._n_fwd
        self._fwd_group_pending = set()

        v_norm = math.sqrt(sum(v * v for v in v_cmd))
        if v_norm < 1e-6:
            return

        step_rad = math.radians(self._fwd_step_deg)
        unit = [v / v_norm for v in v_cmd]
        step_vec = [u * step_rad for u in unit]
        all_configs = [
            [self._q_cur[i] + step_vec[i] * k for i in range(6)]
            for k in range(1, self._n_fwd + 1)
        ]
        chunks = _chunks(all_configs, self._fwd_bundle_size)
        offset = 0
        for chunk in chunks:
            rid = self._send_request(chunk)
            self._pending[rid] = ("fwd", gid, offset, len(chunk))
            self._fwd_group_pending.add(rid)
            offset += len(chunk)

    def _dispatch_one_proximity(self) -> None:
        axis = self._prox_rr
        self._prox_rr = (self._prox_rr + 1) % 6
        configs = []
        for off in self._probe_offsets_rad:
            q = list(self._q_cur)
            q[axis] = q[axis] + off
            configs.append(q)
        rid = self._send_request(configs)
        self._pending[rid] = ("prox", axis, 0, 0)

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
        # REP behind ROUTER->DEALER expects the empty delimiter REQ
        # would add automatically; DEALER doesn't, so we prepend it.
        try:
            self._dealer.send_multipart([
                b"",
                b"req.collision_check",
                json.dumps(env, separators=(",", ":")).encode("utf-8"),
            ])
        except zmq.ZMQError:
            self._pending.pop(rid, None)
        return rid

    def _wait_for_forward(self) -> bool:
        """Block until every chunk for the current group has replied,
        or the deadline expires. Returns True iff every chunk arrived
        in time AND the assembled hits buffer has no None entries.
        """
        if self._dealer is None:
            return True
        if not self._fwd_group_pending:
            # Stationary tick -- vacuously certified.
            return True
        deadline = time.perf_counter() + self._timeout_s
        while self._fwd_group_pending and time.perf_counter() < deadline:
            remaining_ms = max(0, int((deadline - time.perf_counter()) * 1000))
            if self._dealer.poll(remaining_ms) == 0:
                break
            body = self._recv_reply_nonblocking()
            if body is None:
                continue
            self._apply_reply(body)
        if self._fwd_group_pending:
            return False
        return all(h is not None for h in self._fwd_group_hits)

    def _harvest_replies_nonblocking(self) -> None:
        if self._dealer is None:
            return
        while True:
            body = self._recv_reply_nonblocking()
            if body is None:
                return
            self._apply_reply(body)

    def _recv_reply_nonblocking(self) -> Optional[dict]:
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
        tag, a, b, c = kind
        if tag == "fwd":
            group_id, offset, length = a, b, c
            if group_id != self._fwd_group_id:
                return True
            if len(hits) != length:
                return True
            for i, h in enumerate(hits):
                self._fwd_group_hits[offset + i] = h
            self._fwd_group_pending.discard(rid)
        elif tag == "prox":
            axis = a
            if len(hits) == self._n_probes:
                self._prox_hits[axis] = hits
                self._prox_age_ticks[axis] = 0
        return True

    # ---- scalar derivation ----------------------------------------------

    def _compute_path_scalar(self, forward_certified: bool) -> float:
        if not self._collision_enabled:
            return 1.0
        if not forward_certified:
            # SAFETY: no certified-clear reply this tick -> stop.
            return 0.0
        first_hit_k: Optional[int] = None
        for k, h in enumerate(self._fwd_group_hits, start=1):
            if h:
                first_hit_k = k
                break
        if first_hit_k is None:
            return 1.0
        distance_deg = first_hit_k * self._fwd_step_deg
        if distance_deg <= self._path_cutoff_deg:
            return 0.0
        span = self._fwd_horizon_deg - self._path_cutoff_deg
        if span <= 0:
            return 1.0
        return max(0.0, min(1.0, (distance_deg - self._path_cutoff_deg) / span))

    def _compute_prox_scalar(self) -> float:
        if not self._collision_enabled:
            return 1.0
        worst = 1.0
        for axis in range(6):
            results = self._prox_hits[axis]
            nearest_deg: Optional[int] = None
            for j, off in enumerate(self._probe_offsets_deg):
                if results[j]:
                    d = abs(off)
                    if nearest_deg is None or d < nearest_deg:
                        nearest_deg = d
            if nearest_deg is None:
                scalar = 1.0
            elif nearest_deg <= 1:
                scalar = self._prox_floor
            else:
                # Linear ramp: at probe_half_deg -> 1.0, near 0 -> prox_floor.
                scalar = (self._prox_floor +
                          (1.0 - self._prox_floor) * (nearest_deg / self._probe_half_deg))
            if scalar < worst:
                worst = scalar
        return worst


def _clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x
