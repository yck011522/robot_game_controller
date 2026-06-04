"""End-to-end sanity check for the collision broker + worker pool.

Spawns just `bus_broker`, `collision_broker`, and 2 `collision_worker`s
(no haptic / robot_io / game_controller), then opens a DEALER on the
ROUTER endpoint and sends two hand-built `req.collision_check`
bundles:

    config A = [0, -90, 90, 0, 0, 0] deg   -> expected NOT in collision
    config B = [  0,   0,  0, 0, 0, 0] deg -> expected IN collision (zero
                                              pose sits inside pedestal)

If the worker's reply for B is `collision=False`, the planner pipeline
will never trigger the safety clamps -- this isolates "is the worker
reporting correctly" from "is the planner asking the right questions".

Run with:
    $env:PYTHONPATH = "src"
    & <env-python> tests\test_collision_pipeline.py
"""

from __future__ import annotations

import math
import os
import subprocess
import sys
import time
from pathlib import Path

_REPO = Path(__file__).resolve().parents[1]
_SRC = _REPO / "src"
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

import zmq  # noqa: E402

from core import bus  # noqa: E402


PROFILE = _REPO / "config" / "profiles" / "dev_keyboard_headless.yaml"


def _spawn(module: str, *extra: str) -> subprocess.Popen:
    env = os.environ.copy()
    env["PYTHONPATH"] = str(_SRC)
    cmd = [sys.executable, "-m", module, "--profile", str(PROFILE), *extra]
    return subprocess.Popen(
        cmd,
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )


def _drain(p: subprocess.Popen, prefix: str) -> None:
    try:
        while True:
            line = p.stdout.readline()
            if not line:
                return
            print(f"[{prefix}] {line.rstrip()}")
    except Exception:
        pass


def main() -> int:
    if not PROFILE.exists():
        print(f"[test] missing profile: {PROFILE}", file=sys.stderr)
        return 2

    procs: list[tuple[str, subprocess.Popen]] = []
    procs.append(("bus_broker", _spawn("apps.bus_broker", "--proc", "bus_broker")))
    time.sleep(0.5)
    procs.append(("collision_broker",
                  _spawn("apps.collision_broker", "--proc", "collision_broker")))
    time.sleep(0.5)
    procs.append(("worker_00",
                  _spawn("subsystems.collision_worker",
                         "--proc", "collision_worker", "--instance", "0")))
    procs.append(("worker_01",
                  _spawn("subsystems.collision_worker",
                         "--proc", "collision_worker", "--instance", "1")))

    # Wait for workers to print their `ready:` banners.
    deadline = time.perf_counter() + 30.0
    workers_ready = 0
    while time.perf_counter() < deadline and workers_ready < 2:
        for name, p in procs:
            if not name.startswith("worker"):
                continue
            line = p.stdout.readline()
            if not line:
                continue
            print(f"[{name}] {line.rstrip()}")
            if "ready:" in line and "compas_fab" in line:
                workers_ready += 1
    if workers_ready < 2:
        print("[test] FAIL: workers did not become ready in time", file=sys.stderr)
        _shutdown(procs)
        return 1
    print(f"[test] both workers ready ({workers_ready}/2)")

    ctx = zmq.Context.instance()
    # First validate with REQ (auto-prepends empty delim that REP
    # behind ROUTER requires), then with DEALER + manual empty delim
    # -- the same framing the real planner uses.
    exit_code = 0
    exit_code |= _run_case_set(ctx, "REQ", _send_req, _recv_req)
    exit_code |= _run_case_set(ctx, "DEALER", _send_dealer, _recv_dealer)

    _shutdown(procs)
    print()
    print("[test] PASSED" if exit_code == 0 else "[test] FAILED")
    return exit_code


_CASES = [
    ("ready_pose", [0.0, -90.0, 90.0, 0.0, 0.0, 0.0], False),
    ("zero_pose",  [0.0,   0.0,  0.0, 0.0, 0.0, 0.0], True),
]


def _run_case_set(ctx, label_socket, send_fn, recv_fn) -> int:
    if label_socket == "REQ":
        sock = ctx.socket(zmq.REQ)
    else:
        sock = ctx.socket(zmq.DEALER)
    sock.setsockopt(zmq.LINGER, 0)
    sock.connect(bus.COLLISION_ROUTER_ENDPOINT)
    time.sleep(0.3)
    fail = 0
    for rid, (label, deg, expect_collision) in enumerate(_CASES, start=1):
        rad = [math.radians(v) for v in deg]
        env = bus.make_envelope("test_collision_client")
        env.update({
            "request_id": rid,
            "configs_rad": [rad],
            "check_self": True,
            "check_world": True,
        })
        send_fn(sock, env)
        if sock.poll(5000) == 0:
            print(f"[test] FAIL[{label_socket}] {label}: no reply within 5s")
            fail = 1
            continue
        body = recv_fn(sock)
        results = body.get("results") or []
        if not body.get("ok") or not results:
            print(f"[test] FAIL[{label_socket}] {label}: body={body}")
            fail = 1
            continue
        got = bool(results[0].get("collision"))
        verdict = "OK" if got == expect_collision else "FAIL"
        compute_ms = body.get("compute_ms", 0.0)
        print(f"[test] {verdict}[{label_socket}] {label}: "
              f"expected={expect_collision} got={got} "
              f"compute_ms={compute_ms:.2f} reply_from={body.get('producer')}")
        if got != expect_collision:
            fail = 1
    sock.close(0)
    return fail


def _send_req(sock, env: dict) -> None:
    import json as _json
    sock.send_multipart([
        b"req.collision_check",
        _json.dumps(env, separators=(",", ":")).encode("utf-8"),
    ])


def _recv_req(sock) -> dict:
    import json as _json
    frames = sock.recv_multipart()
    return _json.loads(frames[1].decode("utf-8"))


def _send_dealer(sock, env: dict) -> None:
    import json as _json
    sock.send_multipart([
        b"",
        b"req.collision_check",
        _json.dumps(env, separators=(",", ":")).encode("utf-8"),
    ])


def _recv_dealer(sock) -> dict:
    import json as _json
    frames = sock.recv_multipart()
    payload = frames[2] if frames and frames[0] == b"" and len(frames) >= 3 \
        else frames[-1]
    return _json.loads(payload.decode("utf-8"))


def _shutdown(procs: list[tuple[str, subprocess.Popen]]) -> None:
    for _, p in procs:
        try:
            p.terminate()
        except Exception:
            pass
    deadline = time.perf_counter() + 3.0
    for _, p in procs:
        remaining = max(0.0, deadline - time.perf_counter())
        try:
            p.wait(timeout=remaining if remaining > 0 else 0.1)
        except Exception:
            try:
                p.kill()
            except Exception:
                pass


if __name__ == "__main__":
    sys.exit(main())
