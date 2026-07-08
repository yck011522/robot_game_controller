# Gameplay Recorder — Implementation Plan

**Purpose:** Temporary working file to track the design decisions and staged
implementation of the new per-game Parquet/CSV gameplay recorder, so work can
resume across multiple sessions/turns without redoing the design discussion.

**Status:** Design finalized. Stage 1 (bus plumbing) is done. Stages 2-5 remain.

---

## 1. Background / why this exists

Replaces nothing that currently runs in production — it's a **new, separate**
recording mechanism, additive to the existing `display_broadcast_recording`
(state_broadcaster → state_replayer "daydream" tool), which stays as-is for now
(possible future migration to consume this recorder's output is a later idea,
not part of this plan).

Goals (from design discussion):
1. Give the rewind-shortcut tuning work real recorded gameplay to validate
   against (instead of only synthetic `RandomTrajectoryHaptic` batch runs).
2. Capture exactly the fields needed for per-team gameplay analysis
   (joint tracking error, haptic force feedback, collision behavior, scoring),
   nothing more — not a raw full-bus tap.

Recording window: **Tutorial entry → end of Play** (excludes Conclusion,
Reset, and rewind motion entirely).

## 2. Finalized schema

### `recordings/games_index.csv` — one row per completed game, permanent ledger
(survives even if a game's Parquet folder is later deleted)

| Column | Notes |
|---|---|
| `date` | `YYYY-MM-DD`, local (HK) time — matches date folder name |
| `time` | `HH-MM-SS`, local (HK) time — matches time folder name |
| `profile_name` | look up tuning/gear/rewind params from the named profile file |
| `tutorial_entered_at` | ISO-8601 local w/ UTC offset |
| `play_entered_at` | ISO-8601 local w/ UTC offset |
| `play_ended_at` | ISO-8601 local w/ UTC offset (= recording stop) |
| `total_game_time_s` | `play_ended_at - tutorial_entered_at`, wall-clock, pauses included |
| `score_a`, `score_b` | final score per team (blank if team inactive) |
| `a_joint1_distance_rad` … `a_joint6_distance_rad` | Σ\|Δq\| per joint from `robot_actual.parquet` |
| `b_joint1_distance_rad` … `b_joint6_distance_rad` | same, team B |

No `game_id`, no `active_teams`, no tuning/gear/rewind params (all derivable
from `profile_name`).

### Folder layout

```
recordings/
  games_index.csv
  games/
    2026-07-07/                 <- date folder, HK local time
      14-32-05/                 <- time folder, HK local time
        state_global.parquet    <- shared, not per-team
        a/
          game_controller.parquet
          haptic.parquet
          robot_actual.parquet
          weight.parquet
        b/
          game_controller.parquet
          haptic.parquet
          robot_actual.parquet
          weight.parquet
        skeleton/                <- reserved, future (vision PC pull)
        audio/                   <- reserved, future (audio PC pull)
```

Single-team dev runs simply omit the `b/` folder and leave `b`'s ledger
columns blank. All four per-team files have **identical column names/shapes**
between `a/` and `b/` — nothing team-specific baked into column names.

### `state_global.parquet` (shared)

| Field | Source | Notes |
|---|---|---|
| `ts_wall_ns` | `state.full` | alignment clock |
| `stage` | `state.full.active_stage` | use `active_stage` semantics (not masked to `"paused"`) |
| `paused` | `state.full.paused` | separate bool |
| `countdown_s` | `state.full.countdown_s` | |

(`seq`, `pause_reason` dropped — not reconfirmed as needed.)

### `<team>/game_controller.parquet`

| Field | Source | Notes |
|---|---|---|
| `ts_wall_ns` | `state.full` | |
| `in_collision` | `teams.<t>.collision.in_collision` | |
| `first_hit_detail` | `teams.<t>.collision.first_hit.detail` | nullable string (real shape is `None` or `{"detail": "<msg>"}`) |
| `prox_zones` | `teams.<t>.collision.prox_zones` | nested `list<struct{valid,free_min_deg,free_max_deg,blocked_above_till_deg,blocked_below_till_deg}>`, 6 entries |
| `q_target_rad` (6) | `teams.<t>.robot.q_target_rad` | |
| `v_cmd_rad_s` (6) | `teams.<t>.planner.v_cmd_rad_s` | |
| `v_out_rad_s` (6) | `teams.<t>.planner.v_out_rad_s` | |
| `clamp_path`, `clamp_prox`, `clamp_final` | `teams.<t>.collision.path_scalar/prox_scalar/final_scalar` | |
| `practice_player` | `teams.<t>.practice.active_player` | int; `0` = not in practice, `1`-`6` = active player |

(`forward_certified`, score/buckets dropped — not needed here, derivable
elsewhere.)

### `<team>/haptic.parquet`

| Field | Source | Notes |
|---|---|---|
| `ts_wall_ns` | `telem.haptic.<team>` | **needs `with_wall=True` fix — DONE (stage 1)** |
| `dial_pos_rad` (6) | `telem.haptic.<team>` | raw |
| `dial_vel_rad_s` (6) | `telem.haptic.<team>` | raw |
| `torque_ma` (6) | `telem.haptic.<team>` | **new field — DONE (stage 1)** |
| `dial_robot_deg` (6) | computed by recorder from `dial_pos_rad * gear_ratio` (profile config) | stored per-sample, not derived at analysis time |

(`board_connected`, `board_loop_hz` dropped.)

### `<team>/robot_actual.parquet`

| Field | Source | Notes |
|---|---|---|
| `ts_wall_ns` | `telem.robot.actual.<team>` | **needs `with_wall=True` fix — DONE (stage 1)** |
| `q_rad` (6) | `telem.robot.actual.<team>` | |
| `qd_rad_s` (6) | `telem.robot.actual.<team>` | |
| `fault_active` | `telem.robot.actual.<team>.robot_status.fault_active` | protective-stop flag |
| `fault_reason` | `telem.robot.actual.<team>.robot_status.fault_reason` | nullable string |

(`rtde_ok`, command-queue diagnostics dropped.)

### `<team>/weight.parquet`

| Field | Source | Notes |
|---|---|---|
| `ts_wall_ns` | `telem.weight` | |
| `bucket_1_g`, `bucket_2_g`, `bucket_3_g` | `telem.weight.cells_g`, split per team via the static `TEAM_BUCKET_IDS`-style cell-id convention (`a`→11/12/13, `b`→21/22/23) | generic names since file is already per-team |

### Storage/dependency decisions

- Parquet via **pyarrow** (built directly with `pa.table(...)`, not through
  pandas, so nested `prox_zones` gets a clean `list<struct>` type).
- **pandas** + **plotly** were also pre-approved for later analysis/viz work,
  but are **not added to `requirements.txt` yet** — only add a dependency when
  something actually imports it (pyarrow now; pandas/plotly when the analysis
  tool is built later).
- Buffer each stream fully in memory for the game's duration (a few minutes),
  write Parquet + append the CSV ledger row once at Play-end. No periodic
  flush/crash-safety chunking (explicitly declined — crashes are rare and a
  lost in-progress game is acceptable).
- Recorder is its own standalone launcher-spawned process (`gameplay_recorder`),
  not owned by `GameController`. Subscribes directly to `state.full`,
  `telem.haptic.<team>`, `telem.robot.actual.<team>`, `telem.weight` at each
  topic's native rate — no changes needed to `state.full`'s schema beyond what
  it already publishes.
- On by default for every profile; opt out per-profile via a new
  `gameplay_recording: {enabled: false}` block (mirrors the existing
  `display_broadcast_recording` block pattern, but defaults to **enabled**
  when the block is absent, unlike that one). Matches the existing
  `gamemaster_ui`/`state_broadcaster` "always spawn the process, let it no-op
  internally when disabled" launcher pattern.
- Recorder must drain **every** queued bus message per tick (not
  latest-wins/CONFLATE) — every frame is a real analysis data point, unlike
  most bus consumers.
- New game starts on the `active_stage` edge into `"tutorial"`; any prior
  unfinished recording (never reached Play) is silently discarded, not
  written. Finalize (write files + ledger row) on the edge **out of**
  `"play"`.

## 3. Implementation stages

### Stage 1 — Bus plumbing (DONE)
- [x] Add `torque_ma` to `telem.haptic.<team>` in `subsystems/haptic/real.py`
      (from firmware `telemetry.torque_ma`) and as a `[0.0]*6` placeholder in
      `sim_scripted.py`, `sim_keyboard.py`, `random_trajectory.py`.
- [x] Add `with_wall=True` to the `telem.haptic.<team>` envelope in
      `apps/haptic_io/__main__.py` (was missing `ts_wall_ns` entirely).
- [x] Add `with_wall=True` to the `telem.robot.actual.<team>` envelope in
      `apps/robot_io/__main__.py` (same gap).
- [ ] Sanity-check the 6 edited files still import/parse cleanly (quick
      `get_errors` pass) before moving on.

### Stage 2 — Core recording engine
- [x] Create `src/core/gameplay_recording.py`:
  - `GameRecording` class: buffers rows per stream/team in memory; one
    instance per in-progress game.
  - `record_state_global`, `record_game_controller`, `record_haptic`,
    `record_robot_actual`, `record_weight` — one call per received bus
    message, called by the app layer (stage 3).
  - `finalize(play_ended_wall_ns, final_score)` — computes per-joint
    distances, writes all Parquet files under the HK-local date/time folder,
    appends the ledger CSV row.
  - Helpers: local date/time folder naming, ISO-local timestamp formatting,
    nested `prox_zones` struct-array construction, CSV ledger append
    (header-on-first-write). Nullable string columns (`first_hit_detail`,
    `fault_reason`) get an explicitly pinned `pa.string()` dtype so an
    all-`None` game doesn't produce a schema-incompatible `null`-typed column.
  - Verified via an ad-hoc smoke test (buffer sample rows → finalize →
    inspect folder tree, ledger CSV, and nested `prox_zones` round-trip)
    rather than a checked-in unit test file — revisit if a regression shows
    up later.
- [ ] Unit test (`tests/test_gameplay_recording.py`) exercising `GameRecording`
      directly (no bus/ZMQ needed): feed a few synthetic samples, finalize,
      assert the Parquet files and ledger row look correct.

### Stage 3 — Recorder process
- [x] Create `src/apps/gameplay_recorder/__init__.py` + `__main__.py`:
  - Standard `Proc` scaffold (profile load, heartbeat).
  - Subscribes to `state.full`, `telem.haptic.<team>` /
    `telem.robot.actual.<team>` for each active team, `telem.weight`.
  - Drains **every** queued message per tick (not latest-wins).
  - Tracks `active_stage` edges to open/finalize a `GameRecording`.
  - Splits `telem.weight.cells_g` into per-team bucket columns.
  - Computes `dial_robot_deg` from the profile's `tuning.haptic.gear_ratio`.
  - Resolves `gameplay_recording.enabled`/`dir` from the profile (new helper
    in `core/gameplay_recording.py`), defaulting to enabled.
  - Built in 6 small increments (imports/constants -> main()/subscriptions ->
    small helpers -> state.full lifecycle handler -> game_controller row
    extraction + coercion helpers -> haptic/robot_actual/weight handlers).
  - Verified via an ad-hoc smoke test: imported the module and drove
    `_handle_state_full`/`_handle_haptic`/`_handle_robot_actual`/
    `_handle_weight` directly with synthetic bodies through a full
    idle->tutorial->play->conclusion cycle (no live ZMQ bus needed) --
    recording start/finalize edges, `dial_robot_deg` gearing, per-team
    weight split, and the ledger CSV row all came out correct.
- [ ] CLI run-line + docstring at the top of `__main__.py` per usual
      convention. (Docstring + run-line already included when the file was
      created; nothing further needed here.)

### Stage 4 — Wiring
- [x] Register `gameplay_recorder` as an always-spawned tier in
      `apps/launcher/__main__.py` (after `state_broadcaster`), same pattern as
      `gamemaster_ui`/`state_broadcaster` (spawn unconditionally, process
      no-ops internally if disabled).
- [x] Add `pyarrow` to `requirements.txt`; installed into the `game` conda env
      (`C:\Users\leungp\anaconda3\envs\game`) along with the rest of
      `requirements.txt`, which was also previously missing from that env on
      this machine.
- [x] Add a `gameplay_recorder` entry to `config/runtime.yaml`
      (`fps_target: 120.0`, matching `state_broadcaster`'s pattern; sizing
      rationale -- ~460 msg/s aggregate worst case with both teams active,
      well under ZMQ's default 1000-message SUB HWM -- discussed and
      confirmed sufficient without raising the HWM).

### Stage 5 — Validation
- [x] Ran the full existing test suite (`pytest tests/ -v`, after installing
      `pytest` into the `game` env — most files use plain pytest-style
      functions, not `unittest.TestCase`, so `unittest discover` alone only
      found 44/175 tests): **172 passed, 3 failed**. All 3 failures are
      pre-existing and unrelated to this work (confirmed via `git status`
      showing none of the implicated files were touched):
  - `test_free_motion_interrupt.py` — `tools/validate_free_motion_planner.py`
    has a pre-existing corrupted first line (`SyntaxError`).
  - `test_p1_bus_smoke.py::test_launcher_smoke` — `gamemaster_ui` (spawned
    unconditionally at tier 7, before our new tier 9) crashes on this
    machine because `interface_design/Font_Roboto/static/Roboto-Bold.ttf`
    is missing — a pre-existing missing-asset gap, unrelated to
    `gameplay_recorder`.
  - `test_safety_barrier.py::test_profiles_carry_default_bypass_map` —
    `config/profiles/dev_random_trajectory_rewind_batch.yaml` is missing a
    `safety_barrier.bypass_channels` block; that profile was never touched
    by this work.
- [x] Live end-to-end smoke test (real ZMQ processes, not just direct
      function calls): spawned `apps.bus_broker` + `apps.gameplay_recorder`
      as real subprocesses against a temp profile (based on `bus_smoke.yaml`
      + a `gameplay_recording.dir` override to a temp folder), published a
      synthetic idle->tutorial->play->conclusion `state.full` sequence over
      the real bus, and confirmed `state_global.parquet` + the correct
      `games_index.csv` row were written with the right stage-edge timing.
      Scratch script deleted after use (`tools/_tmp_smoke_gameplay_recorder.py`
      was temporary, not committed).
- [ ] Optional follow-up (not required for this plan): smoke-test with at
      least one active team + real/sim haptic+robot_io to also exercise
      `haptic.parquet`/`robot_actual.parquet`/`weight.parquet` and the
      per-team folder split over a live bus (already covered at the
      direct-function level in Stage 3; only the multi-process wiring for
      an active team remains unexercised live).

## 4. Open follow-ups (not blocking, revisit later)

- Whether synthetic batch-validation profiles
  (`dev_random_trajectory_rewind_batch.yaml` etc.) should record too, or set
  `gameplay_recording.enabled: false` to avoid clutter — not decided yet.
- Migrating the daydream `display_broadcast_recording`/`state_replayer` tool
  to consume this recorder's output — explicitly deferred, not part of this
  plan.
- The offline "replay recorded dial trajectories through the planner for
  rewind-shortcut tuning" tool — explicitly deferred, separate later task.
