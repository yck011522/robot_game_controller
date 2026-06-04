"""Collision-check worker (P2, compas_fab).

Loads the curated `robot_cell_and_state.json` scene into a **headless**
compas_fab `PyBulletClient(direct)` via
[scene.make_planner](../robot/scene.py), opens a REP socket connected
to the collision broker's DEALER endpoint (`tcp://127.0.0.1:5561`), and
answers `req.collision_check` bundles with `rep.collision_result` per
[BUS.md 8](../../../docs/architecture/BUS.md#8-collision-reqrep-router--dealer-at-55605561).

Why compas_fab instead of raw pybullet
--------------------------------------
The curated scene (UR10e + ground + walls + pedestal + bucket tool +
the per-game player rigid bodies) is authored as a compas_fab
RobotCell + RobotCellState. The matching `bullet_collision_pair_discovery.json`
file ships per-body / per-tool `touch_links_candidates` +
`touch_bodies_candidates` whitelists that the explorer
([bullet_collision_keyboard_explorer.py](../../../archive/bullet_collision_keyboard_explorer.py))
uses to suppress contacts that the curator deems "expected" (e.g. tool
touching its mount). Going through `PyBulletPlanner.check_collision`
applies those whitelists for us; raw `pb.getContactPoints()` would not,
so we would either flag harmless self-touches as collisions or have to
re-implement the filter logic by hand.

Behavior is "single static config = single collision call":

1. For each `q` in the request bundle, write `q` into a copy of the
   RobotCellState's robot_configuration.
2. Call `planner.check_collision(state, options={'verbose': False})`.
   - returns normally          -> not in collision
   - raises CollisionCheckError -> in collision; the exception message
     names the colliding pair(s) so we pass it through as
     `first_hit.detail`.
3. We deliberately do NOT honour `check_self` / `check_world` flags at
   the bus level any more -- compas_fab applies both at once, and
   splitting them would require re-running the check on a stripped-
   down state. Both flags are still accepted in the request body (for
   wire compatibility with BUS.md 6.7) but logged + ignored.

REP socket lifecycle
--------------------
A single REP socket follows the strict request/reply cadence: every
`recv()` must be matched by exactly one `send()`. We catch exceptions
*inside* the per-bundle handler and always send a reply (with
`ok=false` on failure) so the planner's REQ socket never deadlocks.

Pooled worker bookkeeping
-------------------------
The supervisor spawns N copies of this module with `--instance 0..N-1`.
The canonical process name in `producer` / `heartbeat.<proc>` is
`collision_worker_{instance:02d}` so log lines and per-process Hz
boxes can tell them apart.
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

_SRC = Path(__file__).resolve().parents[2]
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

import zmq  # noqa: E402

from core import bus  # noqa: E402
from core.proc import Proc, banner, parse_proc_args  # noqa: E402
from core.config import load as load_profile  # noqa: E402


TICK_HZ = 1000.0


def main(argv: list[str] | None = None) -> int:
    args, _ = parse_proc_args(argv, default_proc="collision_worker")
    profile = load_profile(args.profile_path)
    if args.instance is not None:
        proc_name = f"collision_worker_{args.instance:02d}"
    else:
        proc_name = args.proc
    args.proc = proc_name

    # Late imports keep `--help` cheap and avoid pulling in pybullet at
    # module import time (collision worker is the only consumer here).
    from compas_fab.backends.exceptions import CollisionCheckError
    from subsystems.robot.scene import make_planner, UR10E_JOINT_NAMES

    proc = Proc(args, profile, target_hz=TICK_HZ)

    rep: zmq.Socket | None = None
    client = None
    planner = None
    rcs = None
    cfg = None  # working copy of robot_configuration; mutated per request

    def setup(p: Proc) -> None:
        nonlocal rep, client, planner, rcs, cfg
        client, planner, _rc, rcs, stats = make_planner(connection_type="direct")
        # Pre-copy the joint-value carrier so per-request work is one
        # list assignment + one attribute set, not a fresh Configuration.
        cfg = rcs.robot_configuration.copy()
        rep = p.ctx.socket(zmq.REP)
        rep.connect(bus.COLLISION_DEALER_ENDPOINT)
        banner(
            p.proc,
            f"ready: compas_fab PyBullet(direct), touch lists "
            f"{stats.get('n_bodies_patched', 0)}b+"
            f"{stats.get('n_tools_patched', 0)}t, "
            f"connected to {bus.COLLISION_DEALER_ENDPOINT}",
        )

    def teardown(_: Proc) -> None:
        if rep is not None:
            rep.close(0)
        if client is not None:
            try:
                client.__exit__(None, None, None)
            except Exception:
                pass

    poller = zmq.Poller()

    def tick(p: Proc) -> None:
        nonlocal rep
        assert rep is not None
        if rep not in dict(poller.sockets):
            poller.register(rep, zmq.POLLIN)
        events = dict(poller.poll(1))
        if rep not in events:
            return

        try:
            topic, body = bus.recv(rep, flags=zmq.NOBLOCK)
        except zmq.Again:
            return

        reply_topic = "rep.collision_result"
        try:
            results, compute_ms = _check_bundle(
                planner, rcs, cfg, UR10E_JOINT_NAMES, CollisionCheckError, body
            )
            reply = bus.make_envelope(p.proc, seq=int(body.get("request_id", 0)))
            reply.update({
                "request_id": body.get("request_id"),
                "ok": True,
                "error": None,
                "results": results,
                "compute_ms": compute_ms,
            })
        except Exception as e:
            reply = bus.make_envelope(p.proc, seq=int(body.get("request_id", 0)))
            reply.update({
                "request_id": body.get("request_id"),
                "ok": False,
                "error": f"{type(e).__name__}: {e}",
                "results": [],
                "compute_ms": 0.0,
            })
        bus.publish(rep, reply_topic, reply)

    return proc.run(tick, setup=setup, teardown=teardown)


def _check_bundle(planner, rcs, cfg, joint_names, CollisionCheckError, body
                  ) -> tuple[list[dict], float]:
    """Run check_collision once per config in the bundle."""
    configs = body.get("configs_rad") or []
    n_joints = len(joint_names)
    t0 = time.perf_counter_ns()
    results: list[dict] = []
    for q in configs:
        if len(q) != n_joints:
            raise ValueError(f"expected {n_joints} joints, got {len(q)}")
        cfg.joint_values = [float(v) for v in q]
        rcs.robot_configuration = cfg
        try:
            planner.check_collision(rcs, options={"verbose": False})
            results.append({"collision": False, "first_hit": None})
        except CollisionCheckError as exc:
            # Exception message names the colliding pair(s); surface it
            # verbatim so callers can log without re-parsing.
            results.append({
                "collision": True,
                "first_hit": {"detail": str(exc)},
            })
    compute_ms = (time.perf_counter_ns() - t0) / 1e6
    return results, compute_ms


if __name__ == "__main__":
    sys.exit(main())
