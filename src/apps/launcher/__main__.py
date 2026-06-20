"""Launcher / supervisor.

Spawns every enabled subsystem of a profile, waits for each tier's
heartbeat, then runs a watchdog loop until shutdown (Ctrl-C, SIGTERM,
or any child crash).

Mechanism (6 steps)
-------------------
1. Resolve the profile path (CLI `--profile`, falling back to
   `config/launcher.yaml:default_profile`).
2. Load + validate the profile.
3. Open a SUB on `heartbeat.*` so we can both wait for startup and
   monitor health.
4. Spawn the enabled processes in **tiers** (bus broker first, then
   collision broker + workers, then game_controller, then
   robot_io / haptic_io / UI). Each tier must produce its first
   heartbeat before the next tier starts ??that ordering comes from
   SUPERVISOR.md 禮2.
5. Run the main loop at 5 Hz: print a status table every 5 s,
   detect any child exit, propagate the shutdown signal.
6. On shutdown, send `CTRL_BREAK_EVENT` (Windows) / `SIGTERM`
   (POSIX) to every child in reverse startup order, wait for them,
   then `ctx.destroy(linger=0)`.

Not in this slice yet (lands later)
-----------------------------------
- Crash ??respawn with backoff (P12).
- Circuit breaker that demotes the profile to a degraded variant
  after N restarts (P12).
- Hot-reload via REQ/REP (P4+).
"""

from __future__ import annotations

import argparse
import os
import signal
import subprocess
import sys
import time
from collections import deque
from pathlib import Path

if os.name == "nt":
    import ctypes
    from ctypes import wintypes

_SRC = Path(__file__).resolve().parents[2]
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

import zmq  # noqa: E402

from core import bus  # noqa: E402
from core.config import Profile, load as load_profile  # noqa: E402


REPO_ROOT = _SRC.parent

# How long we'll wait for any one process's first heartbeat before
# giving up and shutting the whole thing down. Pybullet startup can be
# slow on first run (loading meshes), hence the generous cap.
STARTUP_HEARTBEAT_TIMEOUT_S = 20.0


# ---------------------------------------------------------------- orphan prevention (Windows-only)

if os.name == "nt":
    _kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
    _kernel32.CreateJobObjectW.argtypes = [ctypes.c_void_p, wintypes.LPCWSTR]
    _kernel32.CreateJobObjectW.restype = wintypes.HANDLE
    _kernel32.SetInformationJobObject.argtypes = [
        wintypes.HANDLE,
        ctypes.c_int,
        ctypes.c_void_p,
        wintypes.DWORD,
    ]
    _kernel32.SetInformationJobObject.restype = wintypes.BOOL
    _kernel32.AssignProcessToJobObject.argtypes = [wintypes.HANDLE, wintypes.HANDLE]
    _kernel32.AssignProcessToJobObject.restype = wintypes.BOOL
    _kernel32.CloseHandle.argtypes = [wintypes.HANDLE]
    _kernel32.CloseHandle.restype = wintypes.BOOL

    _JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE = 0x00002000
    _JOB_OBJECT_EXTENDED_LIMIT_INFORMATION = 9


    class _IO_COUNTERS(ctypes.Structure):
        _fields_ = [
            ("ReadOperationCount", ctypes.c_ulonglong),
            ("WriteOperationCount", ctypes.c_ulonglong),
            ("OtherOperationCount", ctypes.c_ulonglong),
            ("ReadTransferCount", ctypes.c_ulonglong),
            ("WriteTransferCount", ctypes.c_ulonglong),
            ("OtherTransferCount", ctypes.c_ulonglong),
        ]


    class _JOBOBJECT_BASIC_LIMIT_INFORMATION(ctypes.Structure):
        _fields_ = [
            ("PerProcessUserTimeLimit", ctypes.c_longlong),
            ("PerJobUserTimeLimit", ctypes.c_longlong),
            ("LimitFlags", wintypes.DWORD),
            ("MinimumWorkingSetSize", ctypes.c_size_t),
            ("MaximumWorkingSetSize", ctypes.c_size_t),
            ("ActiveProcessLimit", wintypes.DWORD),
            ("Affinity", ctypes.c_size_t),
            ("PriorityClass", wintypes.DWORD),
            ("SchedulingClass", wintypes.DWORD),
        ]


    class _JOBOBJECT_EXTENDED_LIMIT_INFORMATION(ctypes.Structure):
        _fields_ = [
            ("BasicLimitInformation", _JOBOBJECT_BASIC_LIMIT_INFORMATION),
            ("IoInfo", _IO_COUNTERS),
            ("ProcessMemoryLimit", ctypes.c_size_t),
            ("JobMemoryLimit", ctypes.c_size_t),
            ("PeakProcessMemoryUsed", ctypes.c_size_t),
            ("PeakJobMemoryUsed", ctypes.c_size_t),
        ]


    class _WindowsProcessJob:
        """Windows-only orphan prevention for launcher children.

        This repository is intended to run on Windows. We keep each child
        in a Job Object with KILL_ON_JOB_CLOSE so a forced launcher exit
        still tears down the supervised processes.
        """

        def __init__(self) -> None:
            self._handle = _kernel32.CreateJobObjectW(None, None)
            if not self._handle:
                raise ctypes.WinError(ctypes.get_last_error())

            info = _JOBOBJECT_EXTENDED_LIMIT_INFORMATION()
            # If this launcher process dies without running our normal
            # shutdown path, Windows closes the job handle and terminates
            # every assigned child for us.
            info.BasicLimitInformation.LimitFlags = _JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE
            if not _kernel32.SetInformationJobObject(
                self._handle,
                _JOB_OBJECT_EXTENDED_LIMIT_INFORMATION,
                ctypes.byref(info),
                ctypes.sizeof(info),
            ):
                err = ctypes.get_last_error()
                _kernel32.CloseHandle(self._handle)
                self._handle = None
                raise ctypes.WinError(err)

        def assign(self, child: subprocess.Popen) -> None:
            if self._handle is None:
                return
            # Reuse the child process handle that Python already opened
            # for the Popen object instead of opening a second handle.
            process_handle = wintypes.HANDLE(child._handle)  # type: ignore[attr-defined]
            if not _kernel32.AssignProcessToJobObject(self._handle, process_handle):
                raise ctypes.WinError(ctypes.get_last_error())

        def close(self) -> None:
            if self._handle is None:
                return
            # On clean shutdown the children should already be gone; this
            # close mainly releases the Windows handle and resets state.
            _kernel32.CloseHandle(self._handle)
            self._handle = None


    _PROCESS_JOB: _WindowsProcessJob | None = None
else:
    _PROCESS_JOB = None


def _ensure_process_job() -> _WindowsProcessJob | None:
    global _PROCESS_JOB
    if os.name != "nt":
        return None
    if _PROCESS_JOB is not None:
        return _PROCESS_JOB
    try:
        # Create the shared job lazily so every spawned child can be
        # attached to the same ownership boundary.
        _PROCESS_JOB = _WindowsProcessJob()
    except OSError as e:
        print(f"[launcher] warning: could not create Windows job object: {e}",
              file=sys.stderr, flush=True)
        _PROCESS_JOB = None
    return _PROCESS_JOB


# ---------------------------------------------------------------- helpers


def _module_for(proc: str, registry: dict[str, str]) -> str:
    """Map a canonical process name to its Python module.

    Per-team processes look like `haptic_io.a`; the `.a` part is just
    a label that becomes the `--proc` arg -- the module is still
    `apps.haptic_io`. Pooled workers are spawned as `collision_worker`
    (no team suffix) and disambiguated via `--instance`.

    `registry` comes from `config/launcher.yaml:process_modules` and
    overrides the default `apps.<base>` mapping. This lets a process
    whose implementation lives under `subsystems/` (e.g. a pooled
    worker) be spawned directly via `python -m subsystems.<name>`
    instead of needing an empty `apps/<name>/__main__.py` wrapper.
    """
    base = proc.split(".")[0]
    if base in registry:
        return registry[base]
    return f"apps.{base}"


def _load_launcher_yaml() -> dict:
    yml = REPO_ROOT / "config" / "launcher.yaml"
    if not yml.exists():
        return {}
    import yaml as _yaml
    return _yaml.safe_load(yml.read_text(encoding="utf-8")) or {}


def _default_profile_from_launcher_yaml() -> Path | None:
    data = _load_launcher_yaml()
    default = data.get("default_profile")
    if not default:
        return None
    return REPO_ROOT / "config" / "profiles" / f"{default}.yaml"


def _process_modules_from_launcher_yaml() -> dict[str, str]:
    data = _load_launcher_yaml()
    raw = data.get("process_modules") or {}
    return {str(k): str(v) for k, v in raw.items()}


def _spawn(proc_name: str, profile_path: Path, *,
           module_registry: dict[str, str],
           instance: int | None = None,
           ) -> subprocess.Popen:
    """Spawn a child python process per SUPERVISOR.md §3.

    Two Windows-specific things matter:

    - `PYTHONPATH=src` so the child can `import core`, `import apps`,
      etc. without us repacking the source as a wheel. We *prepend*
      so a developer's existing PYTHONPATH still takes precedence for
      shadowed packages.
    - `CREATE_NEW_PROCESS_GROUP` lets us later send the child a
      `CTRL_BREAK_EVENT` without also breaking ourselves. The default
      process group on Windows would treat Ctrl-Break as a
      console-wide signal.
    """
    module = _module_for(proc_name, module_registry)
    argv = [sys.executable, "-m", module,
            "--profile", str(profile_path),
            "--proc", proc_name]
    if instance is not None:
        argv += ["--instance", str(instance)]

    env = os.environ.copy()
    existing = env.get("PYTHONPATH", "")
    env["PYTHONPATH"] = str(_SRC) + (os.pathsep + existing if existing else "")

    popen_kwargs: dict[str, object] = {"cwd": str(REPO_ROOT), "env": env}
    if os.name == "nt":
        popen_kwargs["creationflags"] = subprocess.CREATE_NEW_PROCESS_GROUP  # type: ignore[attr-defined]

    print(f"[launcher] spawn {proc_name}: {' '.join(argv)}", flush=True)
    child = subprocess.Popen(argv, **popen_kwargs)  # type: ignore[arg-type]
    process_job = _ensure_process_job()
    if process_job is not None:
        try:
            # Direct child tracking handles graceful shutdown; the job is
            # the backstop for unexpected launcher death.
            process_job.assign(child)
        except OSError as e:
            print(f"[launcher] warning: could not add {proc_name} to Windows job object: {e}",
                  file=sys.stderr, flush=True)
    return child


def _spawn_bus_trace_recorder(profile: Profile) -> subprocess.Popen:
    """Spawn the optional passive diagnostic recorder before runtime tiers.

    The recorder connects before the broker exists and ZMQ reconnects it when
    the broker binds. Because it is inserted first in ``children``, reverse
    shutdown order terminates it last, after every message producer.
    """

    diagnostics = profile.raw.get("diagnostics")
    config = diagnostics if isinstance(diagnostics, dict) else {}
    output = str(
        config.get("bus_trace_output", "logs/trace/bus_trace_latest.jsonl")
    )
    argv = [
        sys.executable,
        str(REPO_ROOT / "tools" / "bus_trace_recorder.py"),
        "--output",
        output,
    ]
    for team in profile.active_teams:
        argv.extend(["--team", team])

    env = os.environ.copy()
    existing = env.get("PYTHONPATH", "")
    env["PYTHONPATH"] = str(_SRC) + (os.pathsep + existing if existing else "")
    popen_kwargs: dict[str, object] = {"cwd": str(REPO_ROOT), "env": env}
    if os.name == "nt":
        popen_kwargs["creationflags"] = subprocess.CREATE_NEW_PROCESS_GROUP  # type: ignore[attr-defined]

    print(f"[launcher] spawn bus_trace_recorder: {' '.join(argv)}", flush=True)
    child = subprocess.Popen(argv, **popen_kwargs)  # type: ignore[arg-type]
    process_job = _ensure_process_job()
    if process_job is not None:
        try:
            process_job.assign(child)
        except OSError as exc:
            print(
                f"[launcher] warning: could not add bus_trace_recorder to "
                f"Windows job object: {exc}",
                file=sys.stderr,
                flush=True,
            )
    return child


def _terminate(child: subprocess.Popen, name: str, grace_s: float = 3.0) -> None:
    """Ask a child to stop, then kill if it doesn't.

    On Windows we send `CTRL_BREAK_EVENT` ??the only Python-installable
    signal a child created with `CREATE_NEW_PROCESS_GROUP` will see as
    a graceful-shutdown request (it arrives as SIGBREAK). On POSIX we
    SIGTERM. After the grace period we SIGKILL either way.
    """
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


# ------------------------------------------------------------------- main


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Game-controller launcher / supervisor")
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
    module_registry = _process_modules_from_launcher_yaml()
    print(f"[launcher] profile: {profile.name}  ({profile_path})", flush=True)
    print(f"[launcher]   active_teams: {list(profile.active_teams)}", flush=True)
    if module_registry:
        print(f"[launcher]   process_modules overrides: {module_registry}", flush=True)

    children: dict[str, subprocess.Popen] = {}
    stop = {"flag": False}

    def _on_signal(*_: object) -> None:
        stop["flag"] = True
    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)
    if hasattr(signal, "SIGBREAK"):
        signal.signal(signal.SIGBREAK, _on_signal)  # type: ignore[attr-defined]

    ctx = zmq.Context.instance()
    sub = bus.make_sub(ctx, topics=["heartbeat.", "cmd.launcher.shutdown"])
    actual_sub = bus.make_sub(ctx, topics=["telem.robot.actual."])
    poller = zmq.Poller()
    poller.register(sub, zmq.POLLIN)
    actual_poller = zmq.Poller()
    actual_poller.register(actual_sub, zmq.POLLIN)

    seen_first: dict[str, bool] = {}
    last_recv_mono_ns: dict[str, int] = {}
    last_loop_hz: dict[str, float] = {}
    last_heartbeat_body: dict[str, dict] = {}
    recv_window: dict[str, deque[int]] = {}

    exit_code = 0
    try:
        # ---- diagnostics: starts before every runtime process ------------
        diagnostics = profile.raw.get("diagnostics")
        diagnostics = diagnostics if isinstance(diagnostics, dict) else {}
        if bool(diagnostics.get("bus_trace_enabled", False)):
            children["bus_trace_recorder"] = _spawn_bus_trace_recorder(profile)

        # ---- tier 1: bus broker ----------------------------------------
        if profile.is_enabled("bus_broker"):
            children["bus_broker"] = _spawn("bus_broker", profile_path,
                                             module_registry=module_registry)
            if not _wait_for_first_heartbeat(sub, poller, "bus_broker",
                                              STARTUP_HEARTBEAT_TIMEOUT_S,
                                              children, seen_first,
                                              last_recv_mono_ns, last_loop_hz, recv_window):
                exit_code = 1
                return exit_code

        # ---- tier 2: collision broker + pool ---------------------------
        cw_count = 0
        cw_node = profile.subsystems.get("collision_workers")
        if isinstance(cw_node, dict):
            cw_count = int(cw_node.get("count", 0))

        if cw_count > 0:
            children["collision_broker"] = _spawn("collision_broker", profile_path,
                                                   module_registry=module_registry)
            if not _wait_for_first_heartbeat(sub, poller, "collision_broker",
                                              STARTUP_HEARTBEAT_TIMEOUT_S,
                                              children, seen_first,
                                              last_recv_mono_ns, last_loop_hz, recv_window):
                exit_code = 1
                return exit_code
            # Pool ??spawn each worker with --instance N. Wait for the
            # *last* one's heartbeat; ZMQ DEALER buffers requests behind
            # any worker still booting up.
            for i in range(cw_count):
                pname = f"collision_worker_{i:02d}"
                children[pname] = _spawn("collision_worker", profile_path,
                                          module_registry=module_registry,
                                          instance=i)
            last_pname = f"collision_worker_{cw_count - 1:02d}"
            if not _wait_for_first_heartbeat(sub, poller, last_pname,
                                              STARTUP_HEARTBEAT_TIMEOUT_S,
                                              children, seen_first,
                                              last_recv_mono_ns, last_loop_hz, recv_window):
                exit_code = 1
                return exit_code

        preseed_teams = _preseed_robot_teams(profile)

        # ---- tier 3: RobotIO pre-seed ----------------------------------
        for team in preseed_teams:
            pname = f"robot_io.{team}"
            children[pname] = _spawn(pname, profile_path,
                                      module_registry=module_registry)

        for team in preseed_teams:
            pname = f"robot_io.{team}"
            if not _wait_for_first_heartbeat(sub, poller, pname,
                                              STARTUP_HEARTBEAT_TIMEOUT_S,
                                              children, seen_first,
                                              last_recv_mono_ns, last_loop_hz, recv_window):
                exit_code = 1
                return exit_code
            if not _wait_for_first_robot_actual(actual_sub, actual_poller, team,
                                                STARTUP_HEARTBEAT_TIMEOUT_S,
                                                children):
                exit_code = 1
                return exit_code

        # ---- tier 4: game controller (only if any team is active) ------
        if profile.active_teams:
            children["game_controller"] = _spawn("game_controller", profile_path,
                                                  module_registry=module_registry)
            if not _wait_for_first_heartbeat(sub, poller, "game_controller",
                                              STARTUP_HEARTBEAT_TIMEOUT_S,
                                              children, seen_first,
                                              last_recv_mono_ns, last_loop_hz, recv_window):
                exit_code = 1
                return exit_code

        # ---- tier 5: global IO -----------------------------------------
        for pname in ("safety_barrier_controller", "weight_sensor_io", "bucket_controller", "light_column"):
            if profile.is_enabled(pname):
                children[pname] = _spawn(pname, profile_path,
                                          module_registry=module_registry)
                if not _wait_for_first_heartbeat(sub, poller, pname,
                                                  STARTUP_HEARTBEAT_TIMEOUT_S,
                                                  children, seen_first,
                                                  last_recv_mono_ns, last_loop_hz, recv_window):
                    exit_code = 1
                    return exit_code

        # ---- tier 6: remaining per-team IO -----------------------------
        for team in profile.active_teams:
            if profile.is_enabled("robot_io", team=team):
                pname = f"robot_io.{team}"
                if pname not in children:
                    children[pname] = _spawn(pname, profile_path,
                                              module_registry=module_registry)
            if profile.is_enabled("haptic_io", team=team):
                pname = f"haptic_io.{team}"
                children[pname] = _spawn(pname, profile_path,
                                          module_registry=module_registry)

        for team in profile.active_teams:
            for sub_name in ("robot_io", "haptic_io"):
                if profile.is_enabled(sub_name, team=team):
                    pname = f"{sub_name}.{team}"
                    if not _wait_for_first_heartbeat(sub, poller, pname,
                                                      STARTUP_HEARTBEAT_TIMEOUT_S,
                                                      children, seen_first,
                                                      last_recv_mono_ns, last_loop_hz, recv_window):
                        exit_code = 1
                        return exit_code

        # ---- tier 7: spectator dashboard -------------------------------
        # Launch by default for every profile so the observer UI is always
        # present when the runtime comes up, even if the profile's legacy
        # `subsystems.gamemaster_ui` entry is null.
        children["gamemaster_ui"] = _spawn("gamemaster_ui", profile_path,
                                             module_registry=module_registry)
        if not _wait_for_first_heartbeat(sub, poller, "gamemaster_ui",
                                          STARTUP_HEARTBEAT_TIMEOUT_S,
                                          children, seen_first,
                                          last_recv_mono_ns, last_loop_hz, recv_window):
            exit_code = 1
            return exit_code

        # ---- tier 8: state broadcaster ---------------------------------
        # UDP fan-out of state.full to the player-display Raspberry Pis.
        # Launched by default (like the dashboard) so every runtime exposes
        # the display feed; it has no hardware dependencies.
        children["state_broadcaster"] = _spawn("state_broadcaster", profile_path,
                                                module_registry=module_registry)
        if not _wait_for_first_heartbeat(sub, poller, "state_broadcaster",
                                          STARTUP_HEARTBEAT_TIMEOUT_S,
                                          children, seen_first,
                                          last_recv_mono_ns, last_loop_hz, recv_window):
            exit_code = 1
            return exit_code

        print(f"[launcher] all children up: {list(children.keys())}", flush=True)

        # ---- main watchdog loop ----------------------------------------
        next_status = time.perf_counter() + 5.0
        while not stop["flag"]:
            events = dict(poller.poll(timeout=200))
            if sub in events:
                while True:
                    try:
                        topic, body = bus.recv(sub, flags=zmq.NOBLOCK)
                    except zmq.Again:
                        break
                    if topic.startswith("heartbeat."):
                        last_heartbeat_body[topic[len("heartbeat."):]] = body
                    elif _is_graceful_shutdown_request(topic, body):
                        print(
                            "[launcher] graceful shutdown requested by "
                            f"game_controller: {body.get('reason')}",
                            flush=True,
                        )
                        stop["flag"] = True
                    _record_heartbeat(topic, body, last_recv_mono_ns, last_loop_hz,
                                       recv_window, seen_first)

            for name, child in list(children.items()):
                rc = child.poll()
                if rc is not None:
                    print(f"[launcher] {name} exited with code {rc}; shutting down",
                          file=sys.stderr, flush=True)
                    exit_code = rc if rc != 0 else 1
                    stop["flag"] = True
                    break

            # Periodic status table temporarily disabled to keep the console
            # clear for game_controller state-transition diagnostics. Re-enable
            # by uncommenting the block below.
            if time.perf_counter() >= next_status:
                # _print_status(children, last_recv_mono_ns, last_loop_hz,
                #               last_heartbeat_body, recv_window)
                next_status = time.perf_counter() + 5.0

    finally:
        print("[launcher] shutting down children...", flush=True)
        for name, child in reversed(list(children.items())):
            _terminate(child, name)
        process_job = _ensure_process_job()
        if process_job is not None:
            # Closing the job after explicit child termination keeps the
            # graceful path unchanged while still handling crash-only exits.
            process_job.close()
        sub.close(0)
        actual_sub.close(0)
        ctx.destroy(linger=0)

    return exit_code


def _is_graceful_shutdown_request(topic: str, body: dict) -> bool:
    """Accept shutdown only from the authoritative game-controller producer."""

    return (
        topic == "cmd.launcher.shutdown"
        and body.get("producer") == "game_controller"
        and body.get("reason") == "batch_validation_complete"
    )


def _resolve_profile_path(arg: str | None) -> Path | None:
    if arg is None:
        return _default_profile_from_launcher_yaml()
    p = Path(arg)
    if p.exists():
        return p.resolve()
    candidate = REPO_ROOT / "config" / "profiles" / f"{arg}.yaml"
    if candidate.exists():
        return candidate.resolve()
    return p.resolve()


def _record_heartbeat(topic: str, body: dict, last_recv_mono_ns: dict,
                      last_loop_hz: dict, recv_window: dict, seen_first: dict) -> None:
    if not topic.startswith("heartbeat."):
        return
    proc = topic[len("heartbeat."):]
    now_ns = time.perf_counter_ns()
    last_recv_mono_ns[proc] = now_ns
    last_loop_hz[proc] = float(body.get("loop_hz", 0.0))
    seen_first[proc] = True
    w = recv_window.setdefault(proc, deque(maxlen=10))
    w.append(now_ns)


def _print_status(children: dict, last_recv_mono_ns: dict,
                  last_loop_hz: dict, last_heartbeat_body: dict,
                  recv_window: dict) -> None:
    now_ns = time.perf_counter_ns()
    print("[launcher] --- status ---", flush=True)
    for name in children:
        last = last_recv_mono_ns.get(name)
        if last is None:
            print(f"  {name:28s}  no heartbeat", flush=True)
            continue
        age_ms = (now_ns - last) / 1e6
        observed_hz = _observed_hz(recv_window.get(name))
        line = (f"  {name:28s}  age {age_ms:6.1f} ms  "
            f"reported_loop_hz {last_loop_hz.get(name, 0.0):7.2f}  "
            f"observed_hb_hz {observed_hz:5.2f}")
        checks_per_sec = last_heartbeat_body.get(name, {}).get("checks_per_sec")
        if checks_per_sec is not None:
            line += f"  checks_per_sec {float(checks_per_sec):7.2f}"
        print(line, flush=True)


def _observed_hz(window) -> float:
    if window is None or len(window) < 2:
        return 0.0
    span_ns = window[-1] - window[0]
    if span_ns <= 0:
        return 0.0
    return (len(window) - 1) * 1e9 / span_ns


def _preseed_robot_teams(profile: Profile) -> list[str]:
    robot_io = profile.subsystems.get("robot_io", {}) or {}
    return [
        team for team in profile.active_teams
        if robot_io.get(team) is not None
    ]


def _wait_for_first_heartbeat(sub, poller, name: str, timeout_s: float,
                              children: dict, seen_first: dict,
                              last_recv_mono_ns: dict, last_loop_hz: dict,
                              recv_window: dict) -> bool:
    """Block (with 200 ms slices) until `heartbeat.<name>` arrives or we time out.

    While we wait we also drain *other* heartbeats so the status table
    is correct as soon as the loop starts, and we detect any sibling
    child that crashes during startup (cascading failure).
    """
    deadline = time.perf_counter() + timeout_s
    print(f"[launcher] waiting for {name} first heartbeat...", flush=True)
    while time.perf_counter() < deadline:
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
        for cn, child in children.items():
            if child.poll() is not None:
                print(f"[launcher] {cn} died (rc={child.returncode}) while waiting for {name}",
                      file=sys.stderr, flush=True)
                return False
    print(f"[launcher] timeout waiting for {name} heartbeat", file=sys.stderr, flush=True)
    return False


def _wait_for_first_robot_actual(sub, poller, team: str, timeout_s: float,
                                 children: dict) -> bool:
    deadline = time.perf_counter() + timeout_s
    topic_name = f"telem.robot.actual.{team}"
    print(f"[launcher] waiting for {topic_name} first sample...", flush=True)
    while time.perf_counter() < deadline:
        events = dict(poller.poll(timeout=200))
        if sub in events:
            while True:
                try:
                    topic, body = bus.recv(sub, flags=zmq.NOBLOCK)
                except zmq.Again:
                    break
                if topic != topic_name:
                    continue
                q = body.get("q_rad")
                if isinstance(q, list) and len(q) >= 6:
                    print(f"[launcher] {topic_name} sample received", flush=True)
                    return True
        for cn, child in children.items():
            if child.poll() is not None:
                print(f"[launcher] {cn} died (rc={child.returncode}) while waiting for {topic_name}",
                      file=sys.stderr, flush=True)
                return False
    print(f"[launcher] timeout waiting for {topic_name}", file=sys.stderr, flush=True)
    return False


if __name__ == "__main__":
    sys.exit(main())
