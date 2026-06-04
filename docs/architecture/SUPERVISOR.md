# Supervisor

The launcher is also the supervisor: one process that reads the active
profile, spawns every other process, watches their heartbeats, and
respawns the ones that crash. There is no separate orchestrator.

Status: **DRAFT — subject to more detailed review. **
Last reviewed: 2026-06-04.

See [SYSTEM_MAP.md §7](SYSTEM_MAP.md#7-launcher--supervisor) for the
process catalog this works against, [BUS.md §6.9](BUS.md#69-heartbeatproc)
for the heartbeat schema, and [CONFIG.md §2](CONFIG.md#2-top-level-schema)
for the profile fields it reads.

---

## 1. Responsibilities

In order of importance:

1. **Read the active profile** (CLI `--profile <name>` or
   `config/launcher.yaml`'s `default_profile`) and validate it
   ([CONFIG.md §5](CONFIG.md#5-validation)).
2. **Spawn each enabled process** with `subprocess.Popen`, passing
   the CLI args described in §3.
3. **Subscribe to `heartbeat.<proc>`** on the bus and track per-child
   liveness.
4. **Respawn crashed processes** per the policy in §5.
5. **Shut everything down cleanly** on Ctrl-C or a `shutdown` REQ.

The supervisor itself does **not** publish on the realtime bus except
its own `heartbeat.launcher` at 1 Hz.

---

## 2. Startup sequence

Strictly ordered. Each step waits for the previous step's heartbeat
before continuing — this guarantees the bus is up before any PUB
connects.

1. **Bus broker** (`bus_broker: real`) — XSUB/XPUB proxy. Wait for
   `heartbeat.bus_broker`.
2. **Collision broker** (only if `collision_workers.count > 0`) —
   ROUTER/DEALER proxy. Wait for `heartbeat.collision_broker`.
3. **EventRecorder** (`event_recorder: real`) — must come up before
   the rest so it sees their first heartbeats. Wait for
   `heartbeat.event_recorder`. *(Deferred per
   [LOGGING.md](LOGGING.md) — until then the recorder is a stub that
   only emits its heartbeat.)*
4. **GameController** — wait for `heartbeat.game_controller`.
5. **Everything else, in any order** — per-team I/O, shared I/O,
   global I/O, collision workers, UI. The supervisor fires off all
   `Popen` calls in a single pass and then watches heartbeats arrive.

Per-step timeout is 10 s. Missing heartbeat at this stage aborts the
run with a non-zero exit code; supervisor does not retry during
startup. (Crash respawn behavior in §5 only applies after a process
has produced its **first** heartbeat.)

---

## 3. Spawn contract

No env vars. Every child takes the same tiny CLI:

```
python -m <module> --profile <path-to-yaml> --proc <name> [--instance <n>]
```

| Arg | Purpose |
|-----|---------|
| `--profile` | Absolute path to the active profile YAML. The child reads it directly; the supervisor does not pass anything else. |
| `--proc` | Canonical process name (e.g. `haptic_io.a`, `weight_sensor_io`). Used as the `producer` field on every published message and as the `<proc>` in `heartbeat.<proc>`. The child looks itself up in the profile under this name. |
| `--instance` | Integer index for pooled processes (currently only `collision_worker`). Combined with `--proc` to form the unique name (e.g. `--proc collision_worker --instance 7` → `collision_worker_07`). |

Everything else — bus endpoints, hardware addresses, tuning, team
assignment for per-team subsystems, recorder root — lives in the
profile and is read by `core/config.py`. The bus endpoint constants
from [BUS.md §2](BUS.md#2-endpoint-table) are also baked into
`core/bus.py` as defaults, so a child started in isolation against the
default broker needs only `--profile` and `--proc`.

Working directory is the repo root. stdout / stderr are inherited
(visible in the launcher's terminal). Crash files (LOGGING.md §6) are
the long-term home for tracebacks; while logging is deferred, the
inherited streams are the only debug surface.

### 3.1 Launching a single process for development

Because the spawn contract is just CLI args, any one process can be
started by hand against an already-running bus broker. Typical dev
workflow:

```
# Terminal 1: bring up just the broker (and nothing else)
python -m apps.launcher --profile config/profiles/bus_smoke.yaml

# Terminal 2: launch the one process you're iterating on
python -m apps.haptic_io --profile config/profiles/dev_keyboard.yaml --proc haptic_io.a
```

No special "dev mode" flag, no env-var setup, no wrapper script. The
supervisor's own `Popen` invocation is exactly the line above; copy it
out of the supervisor log if in doubt. Killing the manually-launched
process and re-running the same command is the entire edit-test loop.

For pooled processes, supply `--instance`:

```
python -m apps.collision_worker --profile config/profiles/show.yaml \
    --proc collision_worker --instance 0
```

The supervisor will not fight a hand-launched process: it only spawns
the processes its profile enables, and only respawns the ones it
itself spawned (it tracks PIDs, not names). Two `haptic_io.a`
processes publishing on the same topic is a self-inflicted wound, not
a protected configuration — watch for it.

---

## 4. Heartbeat protocol

Every long-lived process publishes `heartbeat.<PROC_NAME>` at **1 Hz**.
Body schema is fixed in [BUS.md §6.9](BUS.md#69-heartbeatproc):
`{ts_wall_ns, ts_mono_ns, pid, loop_hz, loop_jitter_ms_p95,
queue_depth}`.

Supervisor-side bookkeeping per child:

- `last_seen_wall_ns` — wall clock at most recent heartbeat receipt.
- `consecutive_misses` — count of 1 s windows with no heartbeat.
- `restarts_total` — lifetime restart counter.
- `restarts_in_last_60s` — sliding window for circuit breaker (§6).

A child is considered **dead** when `consecutive_misses >= 5`
(5 s silence) **or** the OS reports `Popen.poll()` returned a non-None
exit code, whichever comes first.

---

## 5. Respawn policy

Per-process policy is fixed in code (not in YAML) — operator hot-edits
of restart behavior have no good use case.

| Process | Policy |
|---------|--------|
| `bus_broker` | `always` |
| `collision_broker` | `always` |
| `collision_worker_*` (×N) | `always` |
| `haptic_io.<team>` | `always` |
| `robot_io.<team>` | `always` |
| `jogging_planner.<team>` (when `standalone`) | `always` |
| `weight_sensor_io` | `always` |
| `light_column_*` | `always` |
| `display_broadcaster` | `always` |
| `scoreboard_broadcaster` | `always` |
| `bucket_controller` | `always` |
| `button_controller` | `always` |
| `safety_barrier_controller` | `always` |
| `event_recorder` | `always` |
| `game_controller` | `never` (a GC crash ends the run; supervisor exits with non-zero) |
| `gamemaster_ui` | `never` (pygame on the main thread; the gamemaster will see and relaunch) |

`always` means: on detected death, log the event, kill the process if
`Popen.poll()` is still `None` (sigterm → 2 s grace → sigkill), then
re-`Popen` with the same argv. The name (and `--instance` for pooled
processes) is preserved across respawns so subscribers see continuity
on `heartbeat.<proc>`.

`never` means: log the event and tear down the rest of the system in
the same shutdown order as Ctrl-C (§7).

The optional `at_game_start` policy mentioned in NEXT_STEPS §3 is **not
implemented yet**; the slot exists in code so collision workers can be
opted in later for a "fresh slate every game" pattern.

---

## 6. Restart circuit breaker

A process that crashes immediately on startup will otherwise restart
forever in a tight loop. When that happens, something is seriously
wrong (config mismatch, missing hardware, code regression) and the
right answer is to stop the whole system loudly, not to keep limping
along with a subsystem permanently absent.

- If `restarts_in_last_60s >= 5` for **any single process**, the
  supervisor logs the failing process name and exit codes, then
  initiates the shutdown sequence from §7 with a non-zero exit code.
- No "disabled" state, no UI override, no degraded-mode operation.
- The operator restarts the launcher after fixing the underlying
  cause. The circuit breaker is intentionally easy to trip so failure
  is visible immediately.

---

## 7. Shutdown sequence

Triggered by Ctrl-C on the launcher terminal, by `verb: "shutdown"`
sent to GC's UI socket (which then notifies the supervisor over a
local pipe), or by a fatal child crash (§5 `never` policy).

1. Supervisor stops respawning anything new.
2. `SIGTERM` to every child in **reverse startup order** (UI first,
   GC last on its tier, brokers last overall).
3. Wait up to 5 s per tier for `Popen.wait()`.
4. `SIGKILL` to anyone still alive.
5. Supervisor exits with the worst child exit code it observed.

External Vision / Audio PCs are not signaled — they are independent
machines and notice the bus going quiet on their own.

---

## 8. Implementation notes

- `subprocess.Popen` on Windows: use `creationflags=CREATE_NEW_PROCESS_GROUP`
  so Ctrl-C in the launcher terminal doesn't blast every child
  simultaneously and bypass the orderly shutdown above. Send
  `signal.CTRL_BREAK_EVENT` when we want them to stop.
- Heartbeat SUB uses `setsockopt(SUBSCRIBE, b"heartbeat.")` (prefix
  match) — one socket sees every child.
- Supervisor's own loop runs at 10 Hz: cheap, plenty for 1 Hz
  heartbeats and child-poll responsiveness.
- No threading inside the supervisor. One asyncio loop or a plain
  `while True: poll() ; sleep(0.1)` is enough; pick whichever the
  rest of `core/` settles on.

---

## 9. Open items

- `at_game_start` respawn policy for collision workers (§5). Defer
  until P10.
- Whether the 5-in-60s circuit-breaker threshold is right. Numbers
  are a guess; revisit after the first week of unattended running.
- Whether the UI gets a "process panel" with manual respawn buttons,
  or whether respawn stays CLI-only. Defer to gamemaster UI design.
- Whether to expose supervisor state on the bus as a regular
  `state.supervisor` topic (vs only via `process_health` inside
  `state.full`). Defer until the dashboard is built and we know what
  it needs.
