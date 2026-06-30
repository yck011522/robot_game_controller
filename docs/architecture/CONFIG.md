# Config

YAML profiles select **which processes to start, which teams are active,
which subsystems use Real vs Sim impls, and all tuning parameters**.
Installation-local hardware wiring lives separately in
`config/device_ports_and_addr.yaml`, so profiles stay focused on
run-to-run behavior instead of PC-specific COM ports and IP addresses.

Status: **CONFIRMED for the current P1-P4 runtime slice; revise again when the deferred hardware broadcasters/controllers land.**
Last reviewed: 2026-06-06.

---

## 1. Layout

```
config/
  launcher.yaml          # tiny file: which profile to use when --profile is not given
  device_ports_and_addr.yaml  # installation-local COM ports, serial settings, robot IPs
  profiles/
    bus_smoke.yaml       # absolute minimum: bus broker + a tap. Nothing else.
    dev_keyboard.yaml    # P2 manual smoke: all sim, keyboard input, pybullet GUI
    dev_keyboard_headless.yaml  # P2 regression: scripted haptic + headless pybullet
    dev_one_robot_keyboard.yaml   # next P3 profile: real UR10e on team B, everything else sim
    show.yaml            # full hardware, both teams (deployment)
```

The active profile is selected **only** via the CLI flag:

```
python -m apps.launcher --profile bus_smoke
```

If `--profile` is omitted, the launcher reads `config/launcher.yaml`:

```yaml
# config/launcher.yaml
default_profile: bus_smoke   # name of a file under config/profiles/ (no .yaml suffix)
```

There is intentionally **no env-var fallback**: it makes the choice
harder to audit from `ps`/recorder metadata and easier to forget after
an SSH session. The default lives in a checked-in YAML so a fresh
clone of the repo runs the right thing without extra setup.

The launcher reads exactly one profile per run. Live edits during a run
require sending the `reload_config` REQ to the GameController (see
[BUS.md §7](BUS.md#7-ui--gc-commands-reqrep-at-5570)); not every field
is reloadable (see §6).

---

## 2. Top-level schema

Annotated reference. Real profiles in §4 omit comments where the value
is self-explanatory; copy from here when adding new ones.

```yaml
# ============================================================
# Identification (informational; appears in recordings/index.jsonl)
# ============================================================
profile_name: dev_keyboard
description: "P2 smoke test — keyboard → sim robot → pygame dashboard"

# ============================================================
# Active teams
# Subset of [a, b]. Drives how many per-team processes the launcher
# spawns. Per-team subsystems below MUST be `null` for teams not
# listed here, and non-`null` for teams listed here (fail loud, §5).
# ============================================================
active_teams: [a]

# ============================================================
# Subsystem selection
# Each entry resolves to one of:
#   null                 — subsystem not spawned at all (its bus topics
#                          simply will not exist; consumers must handle this)
#   "<impl name>"        — direct impl selection for simple/global entries
#   {impl: <name>, ...}  — global subsystem with impl-specific settings
#   {count: N, ...}      — spawn a pool of N processes (collision_workers)
# Per-team subsystems take a mapping {a: ..., b: ...}.
#
# Shared runtime settings currently used by the live dashboard/runtime
# live in config/runtime.yaml:
#   fps_target         — actual loop target used by that process (or broker heartbeat cadence)
#   fps_min            — dashboard warning threshold for loop_hz
#   heartbeat_age_max  — dashboard warning threshold for last-seen heartbeat age (ms)
# ============================================================
subsystems:

  # 6 ESP32-driven haptic dials per team, over USB serial.
  # sim_keyboard — keyboard producer (P2 milestone, no hardware)
  # sim_replay   — replays a recorded telem.haptic.<team> stream
  # real         — actual ESP32 boards
  haptic_io:
    a: sim_keyboard
    b: null

  # UR10e arm per team.
  # sim_pybullet — pybullet-backed simulator with the URDF from
  #                incoming_code/ur10e_robot/. No real robot needed.
  # real_rtde    — actual UR10e over RTDE TCP (uses config/device_ports_and_addr.yaml robot.<team>).
  robot_io:
    a: sim_pybullet
    b: null

  # Joint planner: gear ratio, clamp, rate-limit, collision check.
  # Collision check is always on (even in simulation); the only knobs
  # are `tuning.collision.check_self` / `check_world` below. There is
  # no way to disable collision check entirely — use those flags to
  # narrow what is checked.
  # in_process — runs as a Python module inside GameController (no IPC).
  # standalone — separate process per team, publishes cmd.robot.target.<team>
  #              over the bus. Use when planning latency would stall GC's
  #              50 Hz tick.
  jogging_planner:
    a: in_process
    b: null

  # 6 RS-485 load cells (3 buckets per team). Single shared process.
  weight_sensor_io: sim           # sim | real

  # 3 light columns per RS-485 bus, split across 3 USB adapters
  # (cols 1-3, 4-5, 7-9). Each group runs as its own process to keep
  # SYSTEM_MAP rule #7 (one RS-485 adapter per process) intact. Set
  # any single group to null to disable just that group.
  light_column_1_3: null          # sim | real | null
  light_column_4_5: null          # sim | real | null
  light_column_6_8: null          # sim | real | null

  # Bridges state.full to UDP for the RPi display nodes. No sim variant
  # — the UDP protocol is the same whether you're simulating or not, so
  # "real" just means "spawn this process". Use null to disable.
  display_broadcaster: null       # real | null

  # RS-485 sender driving the LED scoreboard panels.
  scoreboard_broadcaster: null    # sim | real | null

  # RS-485 driver for the 6 motorized buckets (3 per team).
  bucket_controller: null         # sim | real | null

  # RS-485 reader for two physical button stations (play / stop / e-stop
  # at left and right corners). Global.
  # sim_keyboard — keyboard maps to button presses for desk testing.
  button_controller: sim_keyboard # sim_keyboard | real

  # RS-485 reader for the 8-channel safety light barrier. Global.
  # sim_open   — always reports all channels unbroken
  # sim_random — occasionally trips a channel for fault-injection testing
  safety_barrier_controller: sim_open  # sim_open | sim_random | real

  # Pool of pybullet collision-check workers, shared by both teams.
  # count: 0 disables collision checking entirely (planner sends targets
  # without checking).
  collision_workers:
    count: 4
    fps_target: 1000.0
    fps_min: 800.0
    heartbeat_age_max: 1100

  # ROUTER/DEALER broker for the shared collision worker pool.
  collision_broker:
    fps_target: 1.0
    fps_min: 0.8
    heartbeat_age_max: 1100

  # Per-game folder writer. null disables recording (useful for some tests).
  event_recorder: real            # real | null

  # Reserved dashboard config slot. Current launcher policy auto-starts
  # the dashboard for all runtime profiles; this field is kept so the
  # eventual explicit enable/disable semantics still have a home.
  gamemaster_ui: real             # real | null

  # The XSUB/XPUB proxy from BUS.md §1. Required at runtime by any other
  # process. null is only valid in unit tests that mock the bus.
  bus_broker:
    impl: real
    fps_target: 1.0
    fps_min: 0.8
    heartbeat_age_max: 1100

# ============================================================
# Static tuning
# Values that used to live in the gamemaster UI (now read-only at runtime,
# see NEXT_STEPS §2.B/D). Hot-reloadable via `reload_config` REQ (§6).
# ============================================================
tuning:

  # Haptic dial behavior. Per-dial arrays have 6 entries (one per UR10e joint).
  # All angular values are in DEGREES (and deg/s, deg/s²) — they tune
  # by hand more intuitively than radians. The runtime conversion to
  # radians happens inside the consuming process.
  haptic:
    gear_ratio:          [10, 10, 10, 5, 5, 5]    # dial → joint multiplier (unitless)
    # Power-on / reset defaults for the soft bounds the dial firmware
    # enforces. NOTE: while a game is running, GameController
    # continuously overrides the matching `cmd.haptic.<team>`
    # `bounds_min_rad` / `bounds_max_rad` fields, after converting the
    # YAML degrees here to runtime radians, so the dial's hand-feel
    # matches the robot's reachable envelope. These YAML values only
    # apply at power-on / Reset stage before any collision data has
    # arrived.
    bounds_deg_min:      [-180, -180, -180, -180, -180, -180]
    bounds_deg_max:      [ 180,  180,  180,  180,  180,  180]
    bounds_kp:           60.0       # stiffness (Nm/rad) of the wall the dial hits at a bound
    tracking_kp:         12.0       # PD: position gain for following the GC tracking target
    tracking_kd:         0.6        # PD: velocity gain
    tracking_max_torque: 0.6        # ceiling on the PD output (Nm)
    oob_kick:                       # out-of-bound nudge: pulse the dial inward when held past a bound
      enabled:           true
      amplitude:         0.35       # torque per pulse (Nm)
      pulse_interval_ms: 80         # gap between pulses

  # Tutorial-only haptic/gameplay tuning. The GameController requests
  # tracking_kp via sparse cmd.haptic.param.<team> messages on tutorial/play
  # stage edges; HapticIO owns the firmware S write/readback/retry loop.
  tutorial:
    duration_s: 90
    tracking_kp: 2.0
    tutorial_scroll_dial_start_end: [0, -10000]   # dial-space deci-degrees
    tutorial_scroll_dial_bound: [-10050, 50]      # dial-space deci-degrees
    tutorial_detents_pct: [0, 25, 50, 75, 100]

  # Robot motion envelope. Forwarded to the planner and to RTDE.
  # Degrees for human readability; converted to radians at the boundary.
  robot:
    max_velocity_deg_s:      [180, 180, 180, 180, 180, 180]
    max_acceleration_deg_s2: [690, 690, 690, 690, 690, 690]

  # Collision check policy (consumed by JoggingPlanner).
  collision:
    check_self:  true     # self-collision (link vs link)
    check_world: true     # world-collision (link vs table / buckets / fixtures)
    timeout_ms:  80       # REQ timeout per bundle before retry
    retries:     2        # bundle retries before giving up (and refusing motion)
    bundle_size: 8        # configs per req.collision_check bundle (BUS.md §8); tuned by P10-bench

  # Game flow tuning consumed by the new runtime GameController. The
  # controller currently boots straight into Play for bring-up, but the
  # long-term intent is still Idle -> Tutorial -> Play -> Conclusion.
  game:
    duration_s: 240
    sum_score_rate_unit_per_s: 100
    # sim_bucket_values: {a: [320, 240, 160]}  # optional dev-only seed
    # force_stage: play   # dev escape hatch; current profiles still pin Play

# ============================================================
# Recorder
# Per-game folders are written under <root>/games/<game_id>/.
# See LOGGING.md for the on-disk layout (to be written).
# ============================================================
recorder:
  root:            "C:/recordings"
  enabled:         true       # master switch. false → process runs but writes nothing
                              # (useful for desk testing without filling disk). If you
                              # want the recorder process gone entirely, set
                              # subsystems.event_recorder: null instead.
  # Pull raw media files from the external Vision / Audio PCs at game end,
  # in addition to the processed *.jsonl streams. These are big and slow
  # to transfer; default off. NOT YET IMPLEMENTED — reserved fields so
  # the schema is stable when LOGGING.md / the recorder ship.
  keep_raw_audio:  false      # raw microphone WAV from each Audio PC mic (TODO)
  keep_raw_video:  false      # raw camera MP4 from each Vision PC camera (TODO)
```

---

## 3. Subsystem selection rules

For each entry under `subsystems:` the launcher resolves a value to one
of four outcomes:

1. **`null`** — the subsystem is not spawned at all. The bus topics it
   would have produced simply do not exist. Consumers must treat
   missing topics as "no data" (e.g. UI greys out the team).
2. **String value matching a known impl** — the subsystem is enabled
  with runtime thresholds loaded from `config/runtime.yaml`.
3. **Object with `impl: ...`** — same impl selection as above, but with
  explicit impl/config payload when the subsystem needs it.
4. **Object with `count: N`** — pool of N processes (only
  `collision_workers` for now).

### 3.1 Per-team subsystems

`haptic_io`, `robot_io`, `jogging_planner` take a mapping
`{a: ..., b: ...}`. If `active_teams: [a]` and `b` is not `null`,
the launcher refuses to start (mismatch is a config error, not a
silent skip — fail loud at startup).

### 3.2 Available impls

| Subsystem | Impl strings |
|-----------|--------------|
| `haptic_io.<team>` | `sim_keyboard`, `sim_replay`, `real` |
| `robot_io.<team>` | `sim_pybullet`, `real_rtde` |
| `jogging_planner.<team>` | `in_process`, `standalone` |
| `weight_sensor_io` | `sim`, `real` |
| `light_column_1_3` | `sim`, `real` |
| `light_column_4_5` | `sim`, `real` |
| `light_column_6_8` | `sim`, `real` |
| `display_broadcaster` | `real` (UDP out; no sim variant needed) |
| `scoreboard_broadcaster` | `sim`, `real` |
| `bucket_controller` | `sim`, `real` |
| `button_controller` | `sim_keyboard`, `real` |
| `safety_barrier_controller` | `sim_open`, `sim_random`, `real` |
| `collision_workers` | `{count: N}` plus optional runtime settings |
| `collision_broker` | `real` with optional runtime settings |
| `event_recorder` | `real`, `null` |
| `gamemaster_ui` | `real`, `null` |
| `bus_broker` | `real` or `{impl: real, ...}` (always required at runtime) |

New impls should be listed here when their runtime process lands.

---

## 4. Example profiles

### 4.1 `bus_smoke.yaml` — first 0MQ backbone test

Absolute minimum: only the bus broker. Everything else is `null`. Used
to verify the broker comes up, publishes/subscribes work end-to-end via
`tools/bus_tap.py`, and the supervisor's spawn/heartbeat plumbing fires
 correctly. No GameController, no UI, no hardware, no recorder —
any of those should be added one at a time in later smoke tests.

```yaml
profile_name: bus_smoke
description: "Bus broker only. First-light test for the 0MQ backbone."

active_teams: []                  # no teams → no per-team processes

subsystems:
  # Per-team subsystems must be null when active_teams is empty.
  haptic_io:       {a: null, b: null}
  robot_io:        {a: null, b: null}
  jogging_planner: {a: null, b: null}

  # All shared and global I/O off.
  weight_sensor_io:          null
  light_column_1_3:          null
  light_column_4_5:          null
  light_column_6_8:          null
  display_broadcaster:       null
  scoreboard_broadcaster:    null
  bucket_controller:         null
  button_controller:         null
  safety_barrier_controller: null

  # No collision pool, no GC, no UI, no recorder yet.
  collision_workers: {count: 0}
  event_recorder:    null
  gamemaster_ui:     null

  # The only thing that runs: the XSUB/XPUB proxy.
  bus_broker: real

tuning: {}                        # nothing reads tuning in this profile
recorder: {root: "./recordings", enabled: false, keep_raw_audio: false, keep_raw_video: false}
```

Recommended first-test workflow with this profile:

1. `python -m apps.launcher --profile bus_smoke` — supervisor spawns
   only the broker; heartbeat from the broker appears.
2. In another shell, `python tools/bus_tap.py` connects as a SUB and
   prints any traffic.
3. In a third shell, a one-line `tools/bus_poke.py` PUBs a test
   message on topic `test.ping`. The tap should print it.
4. Kill the broker manually; supervisor logs the missed heartbeat and
   respawns it (when P13 is reached; before that, it just exits).

### 4.2 `dev_keyboard.yaml` — P2 milestone

```yaml
profile_name: dev_keyboard
description: "P2 milestone: keyboard → sim robot, single team A"

active_teams: [a]

subsystems:
  haptic_io: {a: sim_keyboard, b: null}
  robot_io:  {a: sim_pybullet, b: null}
  jogging_planner: {a: in_process, b: null}
  weight_sensor_io: null
  light_column_1_3:       null
  light_column_4_5:       null
  light_column_6_8:       null
  display_broadcaster:    null
  scoreboard_broadcaster: null
  bucket_controller:      null
  button_controller:      null
  safety_barrier_controller: null
  collision_workers: {count: 14}
  collision_broker: null
  event_recorder: null
  gamemaster_ui:  null
  bus_broker:     {impl: real}

tuning:
  haptic:
    gear_ratio: [10.0, 10.0, 10.0, 5.0, 5.0, 5.0]
  robot:
    max_velocity_deg_s:      [30, 30, 30, 60, 60, 60]
    max_acceleration_deg_s2: [50, 50, 50, 100, 200, 200]
    headless: false
    initial_pose_deg: [0.0, -90.0, 90.0, 0.0, 0.0, 0.0]
  collision: { check_self: true, check_world: true, timeout_ms: 40, retries: 2, bundle_size: 1 }
  jogging:
    n_forward_steps: 12
    forward_step_deg: 1.0
    path_cutoff_deg: 3.0
    forward_bundle_size: 1
    probe_half_deg: 10
    prox_floor: 0.6
    forward_timeout_ms: 40
  game:
    duration_s: 10
    sum_score_rate_unit_per_s: 100
    sim_bucket_values: {a: [320, 240, 160]}
    force_stage: play
recorder: { root: "./recordings", enabled: false, keep_raw_audio: false, keep_raw_video: false }
```

`dev_keyboard_headless.yaml` is the CI / regression sibling profile: it
switches `haptic_io.a` to `sim_scripted`, sets `tuning.robot.headless:
true`, and reduces the collision-worker count for repeatable automated
tests.

### 4.3 `dev_one_robot_keyboard.yaml`

```yaml
profile_name: dev_one_robot_keyboard
description: "Real UR10e on team B, everything else sim."

active_teams: [b]

subsystems:
  haptic_io: {a: null, b: sim_keyboard}
  robot_io:  {a: null, b: real_rtde}
  jogging_planner: {a: null, b: in_process}
  weight_sensor_io: null
  light_column_1_3:       null
  light_column_4_5:       null
  light_column_6_8:       null
  display_broadcaster:    null
  scoreboard_broadcaster: null
  bucket_controller:      null
  button_controller:      null
  safety_barrier_controller: null
  collision_workers: {count: 6}
  collision_broker: null
  event_recorder: null
  gamemaster_ui:  null
  bus_broker:     {impl: real}

tuning:
  # Reduced limits for first real-robot bring-up.
  haptic:
    gear_ratio: [10.0, 10.0, 10.0, 5.0, 5.0, 5.0]
  robot:
    q_limits_min_deg: [-180, -180, -180, -180, -180, -180]
    q_limits_max_deg: [ 180,  180,  180,  180,  180,  180]
    max_velocity_deg_s:      [30, 30, 30, 30, 30, 30]
    max_acceleration_deg_s2: [115, 115, 115, 115, 115, 115]
    headless: false
    initial_pose_deg: [0.0, -90.0, 90.0, 0.0, 0.0, 0.0]
  collision: { check_self: true, check_world: true, timeout_ms: 40, retries: 2, bundle_size: 1 }
  jogging:
    n_forward_steps: 12
    forward_step_deg: 1.0
    path_cutoff_deg: 3.0
    forward_bundle_size: 1
    probe_half_deg: 10
    prox_floor: 0.6
    forward_timeout_ms: 40
  game: { duration_s: 240, sum_score_rate_unit_per_s: 100, force_stage: play }
```

### 4.4 `show.yaml`

```yaml
profile_name: show
description: "Full hardware, both teams. Deployment profile."

active_teams: [a, b]

subsystems:
  haptic_io: {a: real, b: real}
  robot_io:  {a: real_rtde, b: real_rtde}
  jogging_planner: {a: standalone, b: standalone}
  weight_sensor_io: real
  light_column_1_3:       real
  light_column_4_5:       real
  light_column_6_8:       real
  display_broadcaster:    real
  scoreboard_broadcaster: real
  bucket_controller:      real
  button_controller:      real
  safety_barrier_controller: real
  collision_workers: {count: 16}
  event_recorder: real
  gamemaster_ui:  real
  bus_broker:     real

tuning:
  haptic:    { ... full defaults ... }
  robot:     { ... full defaults ... }
  collision: { check_self: true, check_world: true, timeout_ms: 80, retries: 2, bundle_size: 8 }
  game:
    duration_s: 240
    sum_score_rate_unit_per_s: 100

The installation-wide conclusion pose placeholders live outside the
profiles in [config/robot_show_poses.yaml](c:/Users/yck01/GitHub/robot_game_controller/config/robot_show_poses.yaml). This keeps show choreography separate from per-profile runtime wiring.

recorder: { root: "D:/recordings", enabled: true, keep_raw_audio: false, keep_raw_video: false }
```

---

## 5. Validation

The launcher validates a loaded profile before spawning anything. On
failure it prints all errors and exits non-zero. Checks:

1. `active_teams` is a non-empty subset of `[a, b]`.
2. For every per-team subsystem, teams not in `active_teams` are `null`,
   and teams in `active_teams` are non-`null`.
3. Every impl string is registered in `core/subsystem_registry.py`.
4. `collision_workers.count` is `>= 0`.
5. Profiles must not contain `hardware`; ports and robot addresses belong
   in `config/device_ports_and_addr.yaml`.
6. `recorder.root` parent directory is writable.

A schema file (`config/schema.json`) is generated from the dataclass
definitions in `src/core/config.py` and used by the validator. Editors
with JSON-schema YAML support get autocompletion for free.

---

## 6. Reload behavior

`reload_config` REQ on the UI socket re-reads the active profile.
Fields fall into three categories:

| Category | Examples | Behavior on reload |
|----------|----------|--------------------|
| **Hot** | `tuning.haptic.*`, `tuning.collision.*`, `tuning.game.*` (except `force_stage`) | Applied immediately. Logged as a `state.stage`-adjacent event. |
| **Warm** | `recorder.*` | Applied on next game start. Reload returns `ok: true` with a `pending: true` flag. |
| **Cold** | `active_teams`, every `subsystems.*` value | Reload returns `ok: false, error: "requires_restart"`. Launcher restart needed. |

The launcher does not auto-restart on cold edits — the gamemaster must
acknowledge and trigger it. This is deliberate (no surprise process
churn during a show).

---

## 7. Open items

- COM port numbers, serial connection settings, and robot RTDE endpoints
  live in `config/device_ports_and_addr.yaml` so the launcher does not
  probe serial hardware during normal startup. Discovery tools should
  update that file out of band. Missing required hardware settings fail
  explicitly; an empty haptic port list is the deliberate way to run that
  team disconnected without scanning unrelated COM ports.
- Whether profiles should compose (e.g. `extends: dev_keyboard`) once
  there are more than ~5 of them. Defer.
- Whether `force_stage` in `tuning.game` is the right place for dev
  overrides, or whether it should be a top-level `dev.*` block.
  Revisit after P3 (full state machine) when force_stage becomes less
  useful.
