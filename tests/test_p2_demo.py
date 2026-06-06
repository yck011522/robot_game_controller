"""P2 integration smoke test.

Spins up the full headless dev profile and asserts the dataflow:

    haptic_io.a (scripted) ??game_controller ??cmd.robot.target.a
                                            ??state.full
    robot_io.a (pybullet DIRECT) ??telem.robot.actual.a

Acceptance:
    1. Every expected heartbeat arrives within startup window.
    2. cmd.robot.target.a is published and changes over time (the
       scripted haptic is a slow sine, so q_target should drift).
    3. telem.robot.actual.a tracks: |q_actual - q_target| shrinks
       below 0.05 rad on at least one joint within ~5 s.
    4. state.full carries the expected fields and the team `a` block.
    5. Clean shutdown via CTRL_BREAK_EVENT (exit code informational).
"""

from __future__ import annotations

import json
import os
import signal
import subprocess
import sys
import threading
import time
from collections import defaultdict
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
SRC = REPO_ROOT / "src"
PROFILE = REPO_ROOT / "config" / "profiles" / "dev_keyboard_headless.yaml"

if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

import zmq  # noqa: E402

from core import bus  # noqa: E402


class _StreamDrain(threading.Thread):
    """Drain a child's stdout in the background and tee to our stdout."""

    def __init__(self, stream, prefix: str):
        super().__init__(daemon=True)
        self._stream = stream
        self._prefix = prefix
        self.lines: list[str] = []

    def run(self) -> None:
        try:
            for raw in iter(self._stream.readline, b""):
                line = raw.decode("utf-8", errors="replace").rstrip()
                if not line:
                    continue
                self.lines.append(line)
                print(f"{self._prefix}{line}", flush=True)
        except Exception:
            pass


def _popen_launcher() -> tuple[subprocess.Popen, _StreamDrain]:
    env = os.environ.copy()
    env["PYTHONPATH"] = str(SRC) + (os.pathsep + env.get("PYTHONPATH", "")
                                    if env.get("PYTHONPATH") else "")
    argv = [sys.executable, "-m", "apps.launcher", "--profile", str(PROFILE)]
    kwargs: dict = {"cwd": str(REPO_ROOT), "env": env,
                    "stdout": subprocess.PIPE, "stderr": subprocess.STDOUT}
    if os.name == "nt":
        kwargs["creationflags"] = subprocess.CREATE_NEW_PROCESS_GROUP  # type: ignore[attr-defined]
    p = subprocess.Popen(argv, **kwargs)
    drain = _StreamDrain(p.stdout, prefix="[launcher.out] ")
    drain.start()
    return p, drain


def _terminate(p: subprocess.Popen, grace_s: float = 8.0) -> None:
    if p.poll() is not None:
        return
    try:
        if os.name == "nt":
            p.send_signal(signal.CTRL_BREAK_EVENT)  # type: ignore[attr-defined]
        else:
            p.terminate()
    except Exception:
        pass
    try:
        p.wait(timeout=grace_s)
    except subprocess.TimeoutExpired:
        p.kill()


def main() -> int:
    expected_heartbeats = {
        "bus_broker",
        "collision_broker",
        "collision_worker_00", "collision_worker_01",
        "game_controller",
        "robot_io.a", "haptic_io.a",
    }

    ctx = zmq.Context.instance()
    sub = bus.make_sub(ctx, topics=["heartbeat.", "cmd.robot.target.a",
                                     "telem.robot.actual.a", "state.full"])
    poller = zmq.Poller()
    poller.register(sub, zmq.POLLIN)

    launcher, drain = _popen_launcher()
    seen_hb: set[str] = set()
    cmd_samples: list[list[float]] = []
    actual_samples: list[list[float]] = []
    state_samples: list[dict] = []

    deadline = time.perf_counter() + 45.0  # startup + collection budget
    last_status = time.perf_counter()

    try:
        while time.perf_counter() < deadline:
            if launcher.poll() is not None:
                print(f"[test] launcher exited prematurely rc={launcher.returncode}",
                      file=sys.stderr, flush=True)
                return 1

            events = dict(poller.poll(200))
            if sub in events:
                while True:
                    try:
                        topic, body = bus.recv(sub, flags=zmq.NOBLOCK)
                    except zmq.Again:
                        break
                    if topic.startswith("heartbeat."):
                        seen_hb.add(topic[len("heartbeat."):])
                    elif topic == "cmd.robot.target.a":
                        q = body.get("q_target_rad")
                        if isinstance(q, list):
                            cmd_samples.append(q)
                    elif topic == "telem.robot.actual.a":
                        q = body.get("q_rad")
                        if isinstance(q, list):
                            actual_samples.append(q)
                    elif topic == "state.full":
                        state_samples.append(body)

            if time.perf_counter() - last_status > 3.0:
                print(f"[test] hb={len(seen_hb)}/{len(expected_heartbeats)}  "
                      f"cmd={len(cmd_samples)}  actual={len(actual_samples)}  "
                      f"state={len(state_samples)}", flush=True)
                last_status = time.perf_counter()

            # Bail out early once we have plenty of data.
            if (expected_heartbeats <= seen_hb
                    and len(cmd_samples) >= 50
                    and len(actual_samples) >= 50
                    and len(state_samples) >= 25):
                break
        # ---- assertions -----------------------------------------------
        missing = expected_heartbeats - seen_hb
        assert not missing, f"missing heartbeats: {missing}  (seen={sorted(seen_hb)})"
        print(f"[test] OK: all {len(expected_heartbeats)} heartbeats received")

        assert len(cmd_samples) >= 50, f"only {len(cmd_samples)} cmd.robot.target.a samples"
        assert len(actual_samples) >= 50, f"only {len(actual_samples)} telem.robot.actual.a samples"
        print(f"[test] OK: cmd={len(cmd_samples)} actual={len(actual_samples)} state={len(state_samples)}")

        # cmd_samples should not be all-zero (the scripted haptic sine
        # must produce motion; gear ratio of 10 ??簣0.5 rad).
        max_abs = max(abs(v) for q in cmd_samples for v in q)
        assert max_abs > 0.05, f"cmd.robot.target.a never moved (max |q|={max_abs})"
        print(f"[test] OK: cmd q range max |q| = {max_abs:.3f} rad")

        # Pybullet teleport tracking. Last cmd vs last actual won't be
        # perfectly aligned (cmd at 50 Hz, actual at 100 Hz, with one
        # tick of pipeline lag), so look for the best alignment in the
        # tail rather than just the final sample.
        tail_targets = cmd_samples[-10:]
        tail_actuals = actual_samples[-10:]
        best = min(
            max(abs(t - a) for t, a in zip(tg, ac))
            for tg in tail_targets for ac in tail_actuals
        )
        print(f"[test] best tail max-joint error: {best:.3f} rad")
        assert best < 0.1, f"sim robot did not track target ({best=:.3f})"

        # Also confirm the sim actually moved (not just held zero).
        actual_motion = max(abs(v) for q in actual_samples for v in q)
        assert actual_motion > 0.05, f"sim robot never moved (max |q|={actual_motion:.3f})"
        print(f"[test] OK: sim robot moved, max |q|={actual_motion:.3f} rad")

        # state.full schema.
        last_state = state_samples[-1]
        assert last_state.get("stage") == "play"
        assert "teams" in last_state and "a" in last_state["teams"]
        team_a = last_state["teams"]["a"]
        assert "robot" in team_a and "q_target_rad" in team_a["robot"]
        assert "haptic" in team_a and "dial_pos_rad" in team_a["haptic"]
        assert "connected" in team_a["haptic"]
        assert "board_loop_hz" in team_a["haptic"]
        print("[test] OK: state.full schema looks right")

        print("\n[test] P2 SMOKE TEST PASSED\n")
        return 0

    finally:
        _terminate(launcher)
        sub.close(0)
        ctx.destroy(linger=0)


if __name__ == "__main__":
    sys.exit(main())
