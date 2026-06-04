"""Launcher entry point. `python -m apps.launcher --profile <path>`."""

from __future__ import annotations

import argparse
import os
import signal
import subprocess
import sys
import time
from collections import deque
from pathlib import Path

_SRC = Path(__file__).resolve().parents[2]
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

import zmq  # noqa: E402

from core import bus  # noqa: E402
from core.config import Profile, load as load_profile  # noqa: E402


REPO_ROOT = _SRC.parent

# Per-process startup timeout for "first heartbeat seen" (SUPERVISOR.md §2).
STARTUP_HEARTBEAT_TIMEOUT_S = 10.0


def _module_for(proc: str) -> str:
    """Map canonical process name → Python module path. Per-team and pooled
    processes still resolve to a single module here (their parameters are
    passed via --proc / --instance).
    """
    # bus_broker -> apps.bus_broker
    base = proc.split(".")[0]
    return f"apps.{base}"


def _default_profile_from_launcher_yaml() -> Path | None:
    """Read config/launcher.yaml's default_profile if it exists."""
    yml = REPO_ROOT / "config" / "launcher.yaml"
    if not yml.exists():
        return None
    import yaml as _yaml
    data = _yaml.safe_load(yml.read_text(encoding="utf-8")) or {}
    default = data.get("default_profile")
    if not default:
        return None
    return REPO_ROOT / "config" / "profiles" / f"{default}.yaml"


def _spawn(proc_name: str, profile_path: Path, instance: int | None = None) -> subprocess.Popen:
    argv = [sys.executable, "-m", _module_for(proc_name),
            "--profile", str(profile_path),
            "--proc", proc_name]
    if instance is not None:
        argv += ["--instance", str(instance)]

    env = os.environ.copy()
    # Children import `core`, `apps`, etc. as top-level packages.
    existing = env.get("PYTHONPATH", "")
    env["PYTHONPATH"] = str(_SRC) + (os.pathsep + existing if existing else "")

    popen_kwargs: dict[str, object] = {"cwd": str(REPO_ROOT), "env": env}
    if os.name == "nt":
        popen_kwargs["creationflags"] = subprocess.CREATE_NEW_PROCESS_GROUP  # type: ignore[attr-defined]

    print(f"[launcher] spawn {proc_name}: {' '.join(argv)}", flush=True)
    return subprocess.Popen(argv, **popen_kwargs)  # type: ignore[arg-type]


def _terminate(child: subprocess.Popen, name: str, grace_s: float = 2.0) -> None:
    if child.poll() is not None:
        return
    try:
        if os.name == "nt":
            child.send_signal(signal.CTRL_BREAK_EVENT)  # type: ignore[attr-defined]
        else:
            child.terminate()
    except Exception as e:
        print(f"[launcher] terminate({name}) raised: {e}", flush=True)
    try:
        child.wait(timeout=grace_s)
    except subprocess.TimeoutExpired:
        print(f"[launcher] {name} did not stop in {grace_s}s; killing", flush=True)
        child.kill()


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Game-controller launcher / supervisor (P1 slice)")
    ap.add_argument("--profile", default=None,
                    help="path to profile YAML, or its bare name under config/profiles/. "
                         "Defaults to config/launcher.yaml:default_profile.")
    args = ap.parse_args(argv)

    profile_path = _resolve_profile_path(args.profile)
    if profile_path is None:
        print("[launcher] no profile given and config/launcher.yaml has no default_profile",
              file=sys.stderr, flush=True)
        return 2

    profile: Profile = load_profile(profile_path)
    print(f"[launcher] profile: {profile.name}  ({profile_path})", flush=True)
    print(f"[launcher]   active_teams: {list(profile.active_teams)}", flush=True)

    children: dict[str, subprocess.Popen] = {}
    stop = {"flag": False}

    def _on_signal(*_: object) -> None:
        stop["flag"] = True
    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    ctx = zmq.Context.instance()
    sub = bus.make_sub(ctx, topics=["heartbeat."])
    poller = zmq.Poller()
    poller.register(sub, zmq.POLLIN)

    # Per-child bookkeeping. Track the most recent heartbeat ts_mono_ns
    # *on the launcher's monotonic clock* (i.e. time we received it), plus
    # the producer-reported loop_hz, plus a tiny rolling window for an
    # observed-from-the-bus heartbeat rate.
    seen_first: dict[str, bool] = {}
    last_recv_mono_ns: dict[str, int] = {}
    last_loop_hz: dict[str, float] = {}
    recv_window: dict[str, deque[int]] = {}

    exit_code = 0
    try:
        # ---- startup: spawn enabled processes ---------------------------
        # P1: only bus_broker. Future phases add more here following
        # SUPERVISOR.md §2 startup order.
        if profile.is_enabled("bus_broker"):
            children["bus_broker"] = _spawn("bus_broker", profile_path)
            if not _wait_for_first_heartbeat(sub, poller, "bus_broker",
                                              STARTUP_HEARTBEAT_TIMEOUT_S,
                                              children, seen_first,
                                              last_recv_mono_ns, last_loop_hz, recv_window):
                print("[launcher] bus_broker failed to produce a heartbeat; aborting",
                      file=sys.stderr, flush=True)
                exit_code = 1
                return exit_code

        print(f"[launcher] all P1 children up. heartbeats:", flush=True)
        # ---- main loop: print heartbeats, watch for death ---------------
        next_status = time.monotonic() + 5.0
        while not stop["flag"]:
            events = dict(poller.poll(timeout=200))
            if sub in events:
                while True:
                    try:
                        topic, body = bus.recv(sub, flags=zmq.NOBLOCK)
                    except zmq.Again:
                        break
                    _record_heartbeat(topic, body, last_recv_mono_ns, last_loop_hz,
                                       recv_window, seen_first)

            # Detect crashed children.
            for name, child in list(children.items()):
                rc = child.poll()
                if rc is not None:
                    print(f"[launcher] {name} exited with code {rc}; shutting down",
                          file=sys.stderr, flush=True)
                    exit_code = rc if rc != 0 else 1
                    stop["flag"] = True
                    break

            if time.monotonic() >= next_status:
                _print_status(children, last_recv_mono_ns, last_loop_hz, recv_window)
                next_status = time.monotonic() + 5.0

    finally:
        print("[launcher] shutting down children...", flush=True)
        # Reverse startup order; for P1 there's only one.
        for name, child in reversed(list(children.items())):
            _terminate(child, name)
        sub.close(0)
        # destroy() also covers the case where a poller registration kept
        # a socket alive past the explicit close above.
        ctx.destroy(linger=0)

    return exit_code


def _resolve_profile_path(arg: str | None) -> Path | None:
    if arg is None:
        return _default_profile_from_launcher_yaml()
    p = Path(arg)
    if p.exists():
        return p.resolve()
    # bare name → config/profiles/<name>.yaml
    candidate = REPO_ROOT / "config" / "profiles" / f"{arg}.yaml"
    if candidate.exists():
        return candidate.resolve()
    return p.resolve()  # let load() raise a clear error


def _record_heartbeat(topic: str, body: dict, last_recv_mono_ns: dict,
                      last_loop_hz: dict, recv_window: dict, seen_first: dict) -> None:
    if not topic.startswith("heartbeat."):
        return
    proc = topic[len("heartbeat."):]
    now_ns = time.monotonic_ns()
    last_recv_mono_ns[proc] = now_ns
    last_loop_hz[proc] = float(body.get("loop_hz", 0.0))
    seen_first[proc] = True
    w = recv_window.setdefault(proc, deque(maxlen=10))
    w.append(now_ns)


def _print_status(children: dict, last_recv_mono_ns: dict,
                  last_loop_hz: dict, recv_window: dict) -> None:
    now_ns = time.monotonic_ns()
    print("[launcher] --- status ---", flush=True)
    for name in children:
        last = last_recv_mono_ns.get(name)
        if last is None:
            print(f"  {name:24s}  no heartbeat", flush=True)
            continue
        age_ms = (now_ns - last) / 1e6
        observed_hz = _observed_hz(recv_window.get(name))
        print(f"  {name:24s}  age {age_ms:6.1f} ms  "
              f"reported_loop_hz {last_loop_hz.get(name, 0.0):6.2f}  "
              f"observed_hb_hz {observed_hz:5.2f}", flush=True)


def _observed_hz(window) -> float:
    if window is None or len(window) < 2:
        return 0.0
    span_ns = window[-1] - window[0]
    if span_ns <= 0:
        return 0.0
    return (len(window) - 1) * 1e9 / span_ns


def _wait_for_first_heartbeat(sub, poller, name: str, timeout_s: float,
                              children: dict, seen_first: dict,
                              last_recv_mono_ns: dict, last_loop_hz: dict,
                              recv_window: dict) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        events = dict(poller.poll(timeout=200))
        if sub in events:
            while True:
                try:
                    topic, body = bus.recv(sub, flags=zmq.NOBLOCK)
                except zmq.Again:
                    break
                _record_heartbeat(topic, body, last_recv_mono_ns, last_loop_hz,
                                   recv_window, seen_first)
        if seen_first.get(name):
            print(f"[launcher] {name} heartbeat received", flush=True)
            return True
        child = children.get(name)
        if child is not None and child.poll() is not None:
            print(f"[launcher] {name} exited (code {child.returncode}) before first heartbeat",
                  file=sys.stderr, flush=True)
            return False
    return False


if __name__ == "__main__":
    sys.exit(main())
