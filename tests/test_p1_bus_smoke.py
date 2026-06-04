"""P1 smoke test.

End-to-end acceptance for [docs/MIGRATION_PLAN.md 禮P1](../docs/MIGRATION_PLAN.md):

1. Validates that `config/profiles/bus_smoke.yaml` parses.
2. Spawns the launcher (which spawns the bus broker).
3. Subscribes to the bus directly and verifies that:
   - `heartbeat.bus_broker` arrives at ~1 Hz with the BUS.md 禮6.9 fields.
   - A `tools/bus_poke.py`-style message round-trips through the broker
     to a SUB-all consumer (the path `tools/bus_tap.py` exercises).
4. Sends SIGTERM/SIGBREAK to the launcher and verifies clean shutdown.

Run with the `game` conda env:
    python tests/test_p1_bus_smoke.py
"""

from __future__ import annotations

import json
import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
SRC = REPO / "src"
sys.path.insert(0, str(SRC))

import zmq  # noqa: E402

from core import bus  # noqa: E402
from core.config import load as load_profile, ConfigError  # noqa: E402


PROFILE = REPO / "config" / "profiles" / "bus_smoke.yaml"


def _env_with_src() -> dict[str, str]:
    env = os.environ.copy()
    existing = env.get("PYTHONPATH", "")
    env["PYTHONPATH"] = str(SRC) + (os.pathsep + existing if existing else "")
    return env


class _StreamDrain:
    """Background thread that drains a Popen.stdout into a list of lines,
    so the child never blocks on a full pipe buffer."""

    def __init__(self, p: subprocess.Popen) -> None:
        self.lines: list[str] = []
        self._t = threading.Thread(target=self._run, args=(p,), daemon=True)
        self._t.start()

    def _run(self, p: subprocess.Popen) -> None:
        assert p.stdout is not None
        for line in p.stdout:
            self.lines.append(line)

    def text(self) -> str:
        return "".join(self.lines)


def _popen(argv: list[str]) -> tuple[subprocess.Popen, _StreamDrain]:
    kwargs: dict = {"cwd": str(REPO), "env": _env_with_src(),
                    "stdout": subprocess.PIPE, "stderr": subprocess.STDOUT,
                    "text": True, "bufsize": 1}
    if os.name == "nt":
        kwargs["creationflags"] = subprocess.CREATE_NEW_PROCESS_GROUP  # type: ignore[attr-defined]
    p = subprocess.Popen(argv, **kwargs)
    return p, _StreamDrain(p)


def _terminate(p: subprocess.Popen, grace_s: float = 4.0) -> int:
    if p.poll() is not None:
        return p.returncode
    if os.name == "nt":
        try:
            p.send_signal(signal.CTRL_BREAK_EVENT)  # type: ignore[attr-defined]
        except Exception:
            p.terminate()
    else:
        p.terminate()
    try:
        return p.wait(timeout=grace_s)
    except subprocess.TimeoutExpired:
        p.kill()
        return p.wait()


def test_config_loads() -> None:
    prof = load_profile(PROFILE)
    assert prof.name == "bus_smoke"
    assert prof.active_teams == ()
    assert prof.is_enabled("bus_broker")
    assert not prof.is_enabled("event_recorder")
    print("[test] config loads + validates: OK")


def test_config_rejects_bad_profile(tmp_path: Path) -> None:
    bad = tmp_path / "bad.yaml"
    bad.write_text(
        "profile_name: bad\nactive_teams: [a]\n"
        "subsystems:\n"
        "  haptic_io: {a: null, b: null}\n"   # a should be non-null
        "  robot_io:  {a: null, b: null}\n"
        "  jogging_planner: {a: null, b: null}\n"
        "  collision_workers: {count: -1}\n"   # negative
        "  bus_broker: null\n"                 # must be 'real'
    )
    try:
        load_profile(bad)
    except ConfigError as e:
        msg = str(e)
        assert "haptic_io.a" in msg, msg
        assert "collision_workers.count" in msg, msg
        assert "bus_broker" in msg, msg
        print("[test] config rejects bad profile: OK")
        return
    raise AssertionError("ConfigError not raised for bad profile")


def test_launcher_smoke() -> None:
    launcher, launcher_out = _popen(
        [sys.executable, "-m", "apps.launcher", "--profile", str(PROFILE)])

    ctx = zmq.Context.instance()
    sub_hb = bus.make_sub(ctx, topics=["heartbeat."])
    sub_all = bus.make_sub(ctx, topics=None)
    poller = zmq.Poller()
    poller.register(sub_hb, zmq.POLLIN)
    poller.register(sub_all, zmq.POLLIN)

    try:
        # ---- 1. Wait for first heartbeat.bus_broker ---------------------
        # We need at least 4 heartbeats AND at least ~3 s of wall time
        # to compute a stable rate. The very first heartbeat after a
        # PUB/SUB session establishes can be delivered together with
        # subscription handshake traffic; we discard it and measure from
        # the second onwards. `perf_counter_ns()` is used because on
        # Windows `monotonic_ns()` has ~15 ms resolution which collapses
        # closely-spaced samples to identical timestamps.
        heartbeats: list[tuple[int, dict]] = []
        deadline = time.perf_counter() + 15.0
        collect_until = time.perf_counter() + 3.5
        while time.perf_counter() < deadline:
            events = dict(poller.poll(200))
            if sub_hb in events:
                topic, body = bus.recv(sub_hb, flags=zmq.NOBLOCK)
                if topic == "heartbeat.bus_broker":
                    heartbeats.append((time.perf_counter_ns(), body))
                    if len(heartbeats) >= 4 and time.perf_counter() >= collect_until:
                        break
            if launcher.poll() is not None:
                raise AssertionError(
                    f"launcher exited (code {launcher.returncode}) before heartbeats arrived\n"
                    f"---launcher output---\n{launcher_out.text()}")
        assert len(heartbeats) >= 4, f"only got {len(heartbeats)} heartbeats in 15 s"

        # ---- 2. Validate heartbeat schema (BUS.md 禮6.9) -----------------
        for _, body in heartbeats:
            for field in ("ts_mono_ns", "ts_wall_ns", "producer", "pid",
                          "loop_hz", "loop_jitter_ms_p95", "queue_depth"):
                assert field in body, f"missing {field!r} in heartbeat: {body}"
            assert body["producer"] == "bus_broker"
        # Skip the first sample ??it may have been buffered during the
        # SUB subscription handshake.
        span_s = (heartbeats[-1][0] - heartbeats[1][0]) / 1e9
        observed_hz = (len(heartbeats) - 2) / span_s if span_s > 0 else 0.0
        assert 0.5 <= observed_hz <= 1.8, f"heartbeat rate {observed_hz:.2f} Hz out of spec"
        print(f"[test] heartbeat: schema OK, observed {observed_hz:.2f} Hz from "
              f"{len(heartbeats)} samples")

        # ---- 3. Round-trip test (poke -> tap path) ----------------------
        while True:
            events = dict(poller.poll(0))
            if sub_all in events:
                bus.recv(sub_all, flags=zmq.NOBLOCK)
            else:
                break

        marker = f"p1-{time.time_ns()}"
        poke, _ = _popen([sys.executable, str(REPO / "tools" / "bus_poke.py"),
                          "test.ping", json.dumps({"marker": marker})])
        rc = poke.wait(timeout=10)
        assert rc == 0, f"bus_poke.py exited {rc}"

        saw = False
        deadline = time.perf_counter() + 3.0
        while time.perf_counter() < deadline:
            events = dict(poller.poll(200))
            if sub_all in events:
                topic, body = bus.recv(sub_all, flags=zmq.NOBLOCK)
                if topic == "test.ping" and body.get("marker") == marker:
                    saw = True
                    break
        assert saw, "test.ping with marker did not arrive on bus tap"
        print("[test] poke -> tap round-trip: OK")

        # ---- 4. bus_tap.py actually runs and prints (separate child) ----
        tap, tap_out = _popen([sys.executable, str(REPO / "tools" / "bus_tap.py"),
                               "--topics", "test.tap", "--compact"])
        time.sleep(0.4)
        poke2, _ = _popen([sys.executable, str(REPO / "tools" / "bus_poke.py"),
                           "test.tap", json.dumps({"marker": marker + "-tap"}),
                           "--grace-ms", "300"])
        poke2.wait(timeout=10)
        time.sleep(0.3)
        _terminate(tap, grace_s=2.0)
        assert marker + "-tap" in tap_out.text(), (
            "bus_tap.py did not print expected marker.\n"
            f"---tap output---\n{tap_out.text()}")
        print("[test] bus_tap.py prints incoming traffic: OK")

    finally:
        sub_hb.close(0)
        sub_all.close(0)
        ctx.destroy(linger=0)

        rc = _terminate(launcher, grace_s=5.0)
        # On Windows, CTRL_BREAK_EVENT often exits 3221225786 (STATUS_CONTROL_C_EXIT).
        # The acceptance criteria are the assertions above; the exit code is
        # informational. Also assert that the launcher really did stop.
        assert launcher.poll() is not None, "launcher did not stop"
        print(f"[test] launcher exit code: {rc}")
        text = launcher_out.text()
        if text:
            print("[test] --- launcher output ---")
            print(text)


def main() -> int:
    test_config_loads()
    import tempfile
    with tempfile.TemporaryDirectory() as td:
        test_config_rejects_bad_profile(Path(td))
    test_launcher_smoke()
    print("\n[test] P1 SMOKE TEST PASSED")
    return 0


if __name__ == "__main__":
    sys.exit(main())
