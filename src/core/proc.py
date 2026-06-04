"""Process helpers ??argv parsing and the standard run-loop scaffold.

Every long-lived process in the system follows the same pattern:

    1. Parse `--profile <yaml> --proc <name> [--instance <n>]` (the
       SUPERVISOR.md 禮3 spawn contract).
    2. Load + validate the profile.
    3. Open a ZMQ context, install signal handlers that flip a `stop`
       event.
    4. Run a tight loop at a target Hz, publishing a 1 Hz
       `heartbeat.<proc>` with the BUS.md 禮6.9 schema.
    5. On shutdown, `ctx.destroy(linger=0)` to unblock any blocking
       socket call.

Re-implementing that boilerplate per process is both error-prone and
makes the actual subsystem logic harder to spot. `Proc` encapsulates
the scaffold; subsystems supply a `tick()` callback (and optionally a
`setup()` / `teardown()`).
"""

from __future__ import annotations

import argparse
import os
import signal
import sys
import time
from collections import deque
from dataclasses import dataclass
from typing import Callable

import zmq

from core import bus
from core.config import Profile, load as load_profile


# Cap on the sliding window used to compute the rolling-average loop_hz
# reported in heartbeats. 200 samples at 100 Hz = ~2 s window ??short
# enough to react to a stall, long enough to ignore single-tick jitter.
_LOOP_WINDOW = 200


@dataclass
class ProcArgs:
    """Parsed CLI args for any subsystem process."""
    profile_path: str
    proc: str
    instance: int | None


def parse_proc_args(argv: list[str] | None = None,
                    *, default_proc: str | None = None,
                    extra: Callable[[argparse.ArgumentParser], None] | None = None,
                    ) -> tuple[ProcArgs, argparse.Namespace]:
    """Parse the standard SUPERVISOR.md 禮3 args.

    Subsystems that need additional flags (e.g. `--headless` for the
    keyboard UI) pass an `extra` callback that adds them to the parser
    before parsing. The full namespace is returned alongside `ProcArgs`
    so callers can read those extras.
    """
    ap = argparse.ArgumentParser()
    ap.add_argument("--profile", required=True)
    ap.add_argument("--proc", default=default_proc, required=default_proc is None)
    ap.add_argument("--instance", type=int, default=None)
    if extra is not None:
        extra(ap)
    ns = ap.parse_args(argv)
    return ProcArgs(profile_path=ns.profile, proc=ns.proc, instance=ns.instance), ns


def install_signal_handlers(stop_callback: Callable[[], None]) -> None:
    """Install SIGINT / SIGTERM / SIGBREAK handlers that call back into
    the caller's "set stop event" function. Doing this in a single place
    keeps Windows (SIGBREAK from CTRL_BREAK_EVENT) and POSIX (SIGTERM)
    behavior consistent across every subsystem.
    """
    def _handler(*_: object) -> None:
        stop_callback()
    signal.signal(signal.SIGINT, _handler)
    signal.signal(signal.SIGTERM, _handler)
    if hasattr(signal, "SIGBREAK"):
        signal.signal(signal.SIGBREAK, _handler)  # type: ignore[attr-defined]


class Proc:
    """Run-loop scaffold for a long-lived subsystem process.

    Usage:

        def tick(p: Proc) -> None:
            ... # one iteration of work; do NOT loop here

        p = Proc.from_argv(target_hz=50.0)
        p.run(tick)

    The loop ticks at `target_hz` and emits `heartbeat.<proc>` at 1 Hz
    on whatever PUB socket the subsystem stored at `p.heartbeat_pub`.
    If the subsystem doesn't supply one (set via `p.use_heartbeat_pub`),
    a new PUB is created on the standard bus.
    """

    def __init__(self, args: ProcArgs, profile: Profile, *, target_hz: float):
        self.args = args
        self.profile = profile
        self.proc: str = args.proc
        self.target_hz = float(target_hz)
        self._period_s = 1.0 / self.target_hz

        self.ctx = zmq.Context.instance()

        # Internal liveness state. `stop` flips to True on signal or on
        # uncaught exception in `tick`.
        self._stop = False

        # `loop_window` tracks the gap between successive tick-starts so
        # we can publish a real loop_hz instead of repeating `target_hz`.
        self._loop_window: deque[int] = deque(maxlen=_LOOP_WINDOW)
        self._last_tick_mono_ns: int | None = None

        # Heartbeat cadence is decoupled from the tick rate: the loop
        # may tick at 100 Hz but we still only emit one heartbeat per
        # second.
        self._heartbeat_period_s = 1.0
        self._heartbeat_next = 0.0
        self._heartbeat_seq = 0
        self._heartbeat_pub: zmq.Socket | None = None

        install_signal_handlers(self.stop)

    @classmethod
    def from_argv(cls, *, target_hz: float, default_proc: str | None = None,
                  extra: Callable[[argparse.ArgumentParser], None] | None = None,
                  ) -> tuple["Proc", argparse.Namespace]:
        """Convenience: parse argv, load the profile, return a ready Proc."""
        args, ns = parse_proc_args(default_proc=default_proc, extra=extra)
        profile = load_profile(args.profile_path)
        return cls(args, profile, target_hz=target_hz), ns

    # ---- public API -----------------------------------------------------
    def stop(self) -> None:
        self._stop = True

    @property
    def stopped(self) -> bool:
        return self._stop

    def use_heartbeat_pub(self, pub: zmq.Socket) -> None:
        """Reuse an existing PUB for heartbeats (avoids two PUB sockets
        on a process that already publishes telemetry)."""
        self._heartbeat_pub = pub

    # ---- main loop ------------------------------------------------------
    def run(self, tick: Callable[["Proc"], None],
            setup: Callable[["Proc"], None] | None = None,
            teardown: Callable[["Proc"], None] | None = None) -> int:
        """Drive the standard loop.

        Returns the process exit code (0 on clean shutdown).
        """
        if self._heartbeat_pub is None:
            self._heartbeat_pub = bus.make_pub(self.ctx)
        # Slow-joiner mitigation: give the broker a tick to propagate
        # any subscription updates to our newly-connected PUB so the
        # first sample isn't silently dropped. This applies whether we
        # made the heartbeat PUB ourselves or the subsystem handed us
        # one ??either way it was created moments ago.
        time.sleep(0.15)

        if setup is not None:
            setup(self)

        next_tick = time.perf_counter()
        self._heartbeat_next = next_tick
        exit_code = 0
        try:
            while not self._stop:
                tick_start_mono_ns = time.perf_counter_ns()
                if self._last_tick_mono_ns is not None:
                    self._loop_window.append(tick_start_mono_ns - self._last_tick_mono_ns)
                self._last_tick_mono_ns = tick_start_mono_ns

                try:
                    tick(self)
                except Exception:
                    import traceback
                    traceback.print_exc()
                    exit_code = 1
                    break

                self._maybe_publish_heartbeat()

                next_tick += self._period_s
                sleep_for = next_tick - time.perf_counter()
                if sleep_for > 0:
                    # Use a polling sleep so a signal can wake us
                    # within ~10 ms even at low tick rates.
                    end = time.perf_counter() + sleep_for
                    while not self._stop and time.perf_counter() < end:
                        # `end - now` can go slightly negative between
                        # the loop check and the sleep call on slow
                        # ticks -- clamp to 0 so time.sleep doesn't
                        # raise ValueError.
                        time.sleep(max(0.0, min(0.01, end - time.perf_counter())))
                else:
                    # Behind schedule; resync cadence rather than burn
                    # cycles in a tight catch-up loop.
                    next_tick = time.perf_counter()
        finally:
            if teardown is not None:
                try:
                    teardown(self)
                except Exception:
                    import traceback
                    traceback.print_exc()
            if self._heartbeat_pub is not None:
                self._heartbeat_pub.close(0)
            self.ctx.destroy(linger=0)

        return exit_code

    # ---- internals ------------------------------------------------------
    def _maybe_publish_heartbeat(self) -> None:
        now = time.perf_counter()
        if now < self._heartbeat_next:
            return
        # Reschedule on the original cadence (1 Hz) ??we do NOT drift to
        # `now + period`, so a tick that ran long still gets caught up.
        self._heartbeat_next += self._heartbeat_period_s
        if self._heartbeat_next < now:
            self._heartbeat_next = now + self._heartbeat_period_s

        loop_hz = self._observed_loop_hz()
        jitter_ms_p95 = self._loop_jitter_ms_p95()
        env = bus.make_envelope(self.proc, with_wall=True, seq=self._heartbeat_seq)
        env.update({
            "pid": os.getpid(),
            "loop_hz": loop_hz,
            "loop_jitter_ms_p95": jitter_ms_p95,
            "queue_depth": 0,
        })
        assert self._heartbeat_pub is not None
        bus.publish(self._heartbeat_pub, f"heartbeat.{self.proc}", env)
        self._heartbeat_seq += 1

    def _observed_loop_hz(self) -> float:
        if not self._loop_window:
            return 0.0
        avg_gap_ns = sum(self._loop_window) / len(self._loop_window)
        return 1e9 / avg_gap_ns if avg_gap_ns > 0 else 0.0

    def _loop_jitter_ms_p95(self) -> float:
        if len(self._loop_window) < 5:
            return 0.0
        # Approximate p95 via sorted index; this list is small (??00)
        # so the sort cost is irrelevant compared to the heartbeat
        # cadence.
        gaps_ms = sorted(g / 1e6 for g in self._loop_window)
        idx = int(0.95 * (len(gaps_ms) - 1))
        target_ms = (1.0 / self.target_hz) * 1000.0
        return abs(gaps_ms[idx] - target_ms)


def banner(proc: str, msg: str) -> None:
    """Standardized one-line startup banner, easy to grep for in
    multi-process stdout. Encodes ASCII-safe so it survives Windows
    cp950 / cp1252 console codepages."""
    safe = msg.encode("ascii", errors="replace").decode("ascii")
    print(f"[{proc}] {safe}", flush=True)
