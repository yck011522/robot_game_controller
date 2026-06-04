# Collision-Aware Jog Explorer — Design Specification

This document describes the **behaviour and architecture** of
[pybullet/bullet_collision_keyboard_explorer.py](pybullet/bullet_collision_keyboard_explorer.py)
as it currently stands, so it can be **rewritten from scratch** by another
implementer. Implementation details (Tk widgets, attribute names) are
deliberately omitted; performance-critical contracts and correctness-critical
contracts are spelled out.

The next iteration is expected to:

- Replace Tkinter with **Pygame** for the UI.
- Replace held-key jogging with **arbitrary input sources** (gamepad, haptic
  dials, network command, replay).
- Eventually drive a **real UR12e** instead of (or in addition to) the PyBullet
  simulator.

The collision-check worker pipeline is performant enough today and should be
kept structurally identical; only the dispatching glue should change.

---

## 1. Top-level loops and threading model

The program runs **three independent loops** that communicate through
**snapshots**, never through shared mutable state behind a lock:

| Loop | Owner thread/process | Rate target | Responsibility |
|------|----------------------|-------------|----------------|
| **Input loop** | UI thread (Tk today, Pygame tomorrow) | event-driven | Read user input, build an immutable `IntentSnapshot`, atomically rebind it. |
| **Control loop** | One dedicated background thread | configurable (default 60 Hz; achieved 25–35 Hz on Windows + Tk due to GIL) | Do all motion math, dispatch collision workers, integrate position, write the per-tick log. **No UI calls. No GUI-PyBullet calls.** |
| **View loop** | UI thread, scheduled via the toolkit's timer | 10–50 Hz (default 25 Hz) | Read the latest control-thread state, repaint widgets, push the current pose to the pyBullet 3D visualisation backend. |
| **Forward scheduler queue** | Scheduler + shared worker processes | one synchronous batch per control tick | Forward-path collision sweep (safety-gate, high priority). |
| **Proximity scheduler queue** | Scheduler + shared worker processes | asynchronous, one batch at a time | Per-axis proximity probes (soft slowdown, lower priority). |

### Why three loops, not one

The collision checks and the UI updates cannot share a thread without one
starving the other. The UI must remain responsive (handle resize, focus,
redraw) while the control loop runs at a stable rate. The split also makes the
control loop **independently testable** in headless mode (no UI thread at
all).

### The Intent Snapshot contract

The only thing the input loop hands to the control loop is an **immutable
snapshot** containing:

- Set of held keys representing user input / active velocity commands (an opaque set; semantics live in the
  control loop's mapping function). Those keys may represent Slow speed (deg/s) or Fast speed (deg/s). But in actual game, this is likely a scalar value per axis, not a set of keys.

### Controller Configuration (tunable parameters)
- Per-axis max velocity (deg/s, length-6 tuple).
- Per-axis max acceleration (deg/s², length-6 tuple).
- Proximity speed reduction floor as a percentage of full speed(0..100).
- Forward-collision speed clamp cutoff distance (deg).
- Forward-collision speed clamp shape (`"linear"` or `"exponential"`) and an exponential
  steepness `k`.
- Target control FPS.

**Rebuilding rule:** the input loop rebuilds the entire snapshot and
reassigns it to a single shared attribute every time *any* input changes.
The control loop reads that attribute once at the top of each tick and uses
the result for the whole tick. No fields are mutated. This is safe lock-free
under the CPython GIL (single attribute write is atomic); for a non-Python
rewrite, use any atomic-pointer-swap primitive (`std::atomic<shared_ptr>` /
`AtomicReference`).

### Shutdown contract

- A single `stop` flag (event/atomic bool) signals the control thread to exit.
- The control thread's pacing sleep MUST be a sleep that wakes early on the
  stop flag (e.g. `Event.wait(timeout)`), so shutdown is prompt.
- The close handler MUST be **idempotent** (it is called both from the
  window-close event and from a `finally:` around the UI main loop).
- Close order: set stop → join control thread (≤ 2 s timeout) → dump metrics
  → close log file → shut down collision worker processes/scheduler → close
  visualisation client → destroy window.

---

## 2. Control loop: per-tick pipeline

Every control tick (target dt = 1 / target_fps), in order:

1. **Snapshot intent.** Read the shared intent pointer once.
2. **Harvest proximity.** If a previous async proximity batch has completed,
   collect its results and update the cached `prox_results`. Record the
   wall-clock pipeline time and the **age** of the data we are about to use.
3. **Desired velocity.** Map the held-key set or velocity input to a per-axis desired velocity
   vector `v_des` (deg/s, then converted to rad/s). The mapping is
   **algebraic per axis**: each key contributes ± slow or ± fast deg/s; held
   keys are summed; opposite keys cancel.
4. **Acceleration clamp.** For each axis, step `v_cur` toward `v_des` by no
   more than `max_accel[i] * dt`.
5. **Per-axis velocity clamp.** Clamp each `v_cmd[i]` to ± `max_vel[i]`.
6. **Synchronous forward check.** With `v_cmd` as the direction, dispatch
  the forward sweep on the forward scheduler queue, **block until all
  chunks return**,
   reassemble the ordered bool list. (See §3.) Do not skip even if `|v_cmd|` is small.
7. **Asynchronous proximity dispatch.** Submit a new proximity batch to
  the proximity scheduler queue for the current pose. If a batch is still in flight, **skip
   this dispatch** — do not queue. (See §3.)
8. **Forward-collision speed clamp scalar** (see §4).
9. **Proximity clamp scalar**  (see §4).
10. **Apply the smaller scalar to the whole vector**: `v_out = v_cmd *
  min(forward_collision_scalar, prox_scalar)`. The scalar is **global**, applied
    uniformly to all six axes, so the direction the workers checked is
    exactly the direction the robot moves.
1.  **Integrate**: `pos += v_out * dt`, then clamp `pos` to joint limits and
    zero the velocity if it would push past a limit.
2.  **Record metrics** (control samples, intent log) and **write one log
    row** (see §6).
3.  **Pace.** Sleep `target_dt − elapsed`, waking early on stop.

### Pacing notes (Windows-specific)

The default Windows timer quantum (≈ 15.6 ms) bounds achievable tick rates
to ≈ 30 Hz unless `winmm.timeBeginPeriod(1)` is called. The current code
**does not** do this; rewriters who need ≥ 50 Hz on Windows should.

---

## 3. Collision worker pipeline

This is the part to **preserve structurally**. Performance was measured
acceptable on a 12-thread dev laptop with 5 worker processes per robot.
Actual hardware (Intel Core Ultra 5 225, 10 cores) will change the optimal
process counts; the architecture should not change.

### 3.1 Hybrid scheduler model (unified worker primitive + dual queues)

The recommended rewrite model is:

- One **common collision worker primitive**:
  `evaluate_batch(configs_6d) -> collision_flags`.
- Two **separate scheduler queues**:
  - **Forward collision check queue** (safety-gate, high priority).
  - **Proximity queue** (soft slowdown, lower priority).
- **Strict dequeue policy** at workers/scheduler:
  always pull from the forward collision check queue first; only pull proximity work when the forward collision check queue is empty.

This keeps the worker API clean and uniform while preserving the safety
semantics that matter for control correctness and jitter control.

For sizing, a production starting point remains effectively equivalent to the
previous split (about 3 forward-equivalent + 2 proximity-equivalent workers
per robot), but the exact assignment should be tuned on target hardware.

However, note that in final production, there would be more services that need to be run simultaneously, such as reading multiple inputs and also producing lighting effects, etc.

The assignment of processes to cores and how they line up with physical vs logical cores will need to be tested on the actual hardware together with other services, and may require tweaks to the default process counts or the use of process affinity settings.

### 3.2 Collision Worker process initialisation

Each collision-checking worker, at start:

1. Loads the compas_fab robot cell + state from the same JSON the main process uses.
2. Patches in the precomputed touch-lists (`per_rigid_body`, `per_tool`)
   from `bullet_collision_pair_discovery.json`. This is to reduce unnecessary collision checks. We can still run without importing this file, but it's important to show a warning message to the user.
3. Opens a **`direct`-mode** PyBullet connection (no GUI).
4. Constructs a `PyBulletPlanner`, calls `set_robot_cell` and
   `set_robot_cell_state` once.
5. Runs one throwaway `check_collision` to warm the narrow-phase caches.

The worker keeps the planner, the cell state, and a reusable configuration
object as module-level globals for the lifetime of the process. **No object
is recreated per call.**

The main process also opens **one `gui`-mode** PyBullet connection used only
for visualisation. That client is touched only by the UI thread.

### 3.3 Chunking and ordering

Both schedulers partition their work **once at startup** into contiguous chunks, one chunk per worker invocation.
Chunk sizes differ by at most 1.

- Forward chunks partition the integer set `{1, 2, …, N_FORWARD_STEPS}` (N
  defaults to 12) across `n_forward_workers`.
- Proximity chunks partition `{0, 1, 2, 3, 4, 5}` (the six joint axes)
  across `n_prox_workers`.

A worker invocation does **exactly** `len(chunk)` collision checks. This makes per-tick cost predictable and removes scheduler jitter.

Ordering guarantees to preserve:

- Forward batch order is deterministic (step 1..N reconstruction is stable).
- Proximity batch order is deterministic (axis and offset order is stable).
- Returned collision flags must preserve the same order as submitted configs.

Even with queue priority, this deterministic ordering contract must not
change.

### 3.4 Forward (synchronous, safety gate)

Inputs:
- `base_rad`: current 6D joint position.
- `step_vec_rad`: 6D vector equal to the unit direction of `v_cmd` scaled to
  `FORWARD_STEP_DEG` (default **1°**) of 6D joint-space L2 distance.
- `chunk`: 1-based step indices this worker owns.

For each `k` in chunk, the worker computes
`q_k = base + step_vec * k`, sets it on the planner, calls `check_collision`,
and emits a bool (True = collision). The caller collects all chunks, blocks
on every future, reassembles into a length-N ordered list.

The spacing is **fixed in joint space**, not time. This means the
distance-to-collision in the resulting bool list is independent of current
speed; clamp behaviour is then a function of geometry, not of dt or v_cmd
magnitude.

Total horizon = `N * FORWARD_STEP_DEG` (default 12°).

### 3.5 Proximity (asynchronous, soft slowdown)

Inputs:
- `base_rad`: current 6D joint position.
- `axes`: subset of `{0..5}` this worker owns.
- `offsets_rad`: the probe offsets (the 20-element list
  `−PROBE_HALF, …, −1, +1, …, +PROBE_HALF` in degrees converted to radians;
  `PROBE_HALF_DEG` defaults to 10).

For each axis in `axes`, for each offset, the worker sets
`q' = base` with one coordinate shifted by the offset, calls
`check_collision`, and emits a bool. Returns a `dict[axis_index, list[bool]]`.

**Async discipline:**

- The main control thread keeps **at most one** proximity batch in flight.
- If a batch is still pending at dispatch time, **skip** the new dispatch —
  do not queue, do not let the pool back up.
- Each tick reuses the most recently harvested batch. The freshness is
  exposed as `prox_age_s` and logged. Typical age on the dev laptop:
  60–100 ms (i.e. 1–2 ticks stale).

This is safe because proximity is a **soft slowdown**, never a hard gate;
the forward sweep is the gate.

### 3.6 Failure handling

- A worker raising on `check_collision` is treated as **collision = True**
  by the call site (favours stopping over moving).
- A whole batch failure (worker died) keeps the prior `prox_results` (again,
  favours slowing down).

---

## 4. Clamp scalars

Both clamps return a scalar in `[0, 1]`, applied **as the minimum** of the
two to the entire velocity vector.

### 4.1 Forward-collision speed clamp (from the forward bool list)

Find the smallest `k` (1-based) such that step `k` is in collision. Let
`d = k * FORWARD_STEP_DEG` (degrees of joint-space distance to the nearest
hit). Let `D = N * FORWARD_STEP_DEG` (the horizon) and `c` = `path_cutoff_deg`
from intent.

- If no step collides → return **1.0**.
- If `d ≤ c` (hard cutoff) → return **0.0**. Motion stops.
- Otherwise let `norm = clamp01((d − c) / (D − c))`. Then:
  - `linear` shape: return `norm`.
  - `exponential` shape: return `clamp01(1 − exp(−k_steep * norm))` with
    `k_steep = max(0.1, exp_k)`.

### 4.2 Proximity clamp (from the per-axis bool lists)

Find the nearest collision across **all 6 axes and both directions**: the
smallest positive integer `d ∈ {1..PROBE_HALF}` such that any axis has a
collision at offset `±d`. (The lists are indexed so that index
`PROBE_HALF − 1 − j` is the `−(j+1)°` probe and `PROBE_HALF + j` is the
`+(j+1)°` probe.)

Let `floor = clamp01(prox_floor_pct / 100)`.

- If no collision found → return **1.0**.
- Otherwise let `frac = clamp01((d − 1) / (PROBE_HALF − 1))` and return
  `floor + (1 − floor) * frac`.

The proximity scalar never goes below `floor`; even when an obstacle is at
the closest probe, the robot can still creep at `floor × max_vel`. This is
intentional — proximity is for *feel*, not safety.

---

## 5. Functional UI layout

Toolkit-agnostic description of what the user must see. Pygame implementers
should reproduce the panels, the colour codes, and the **information
density** — exact widget choices are free.

### 5.1 Top status bar (always visible)

- **Current pose status**: one of `FREE` (green), `COLLISION` (red),
  `(checking…)` (grey). Updated at a throttled rate (default 10 Hz), NOT
  every control tick — running the collision check on the GUI client every
  tick costs ~20 ms and would cap the live rate.
- **FPS readout**: `ctrl <actual>/<target>  gui <actual>`. Two numbers: the
  control-loop EMA and the view-loop EMA. Colour the text box background red
  when it falls below 90 % of target. If there are other input or output services in the future, add more readouts here.
- **Clamp readout**: `forward_collision_speed=<value>  prox=<value>  final=<min>`, three
  scalars in `[0, 1]`.
- **Touch-list banner** (informational): patched body / tool counts and
  total skip-pair counts, plus a keyboard cheat-sheet. No longer needed after integrating haptic dials.

### 5.2 Per-axis row (one per joint, six rows)

Each row shows, for joint `i`:

- The joint name and index.
- Current position in degrees, signed, two decimals (e.g. `+43.30`).
- **Proximity bar** — a horizontal strip spanning `[−180°, +180°]`:
  - Tick marks every 30°.
  - One cell per probe offset (`±1° .. ±PROBE_HALF°`), coloured red if that
    probe is in collision, green if free.
  - A larger cell at the **current** position coloured by the current-pose
    status.
  - An **arrow** from the current position to `pos + v_out * 1 s`
    (the projected one-second clamped move on this axis).
  - A **vertical orange mark** at `pos + v_des * 1 s` (the desired pre-clamp
    move) so the user can see how much the clamp is biting.
  - A **triangle marker** above the strip at the current position, coloured
    by the current-pose status.
- **Velocity bar** — a horizontal strip spanning `[−max_vel[i], +max_vel[i]]`:
  - A filled band from 0 to `v_out` (final clamped axis velocity).
  - Vertical tick marks at `v_des` (blue), `v_cmd` (grey), `v_after_path`
    (orange, currently named from legacy terminology), `v_out`
    (black) — four distinct markers showing the stages of
    the clamp pipeline.
  - Compact text labels for `d=` (desired) at the left edge and `o=`
    (output) at the right edge in deg/s.

### 5.3 Single forward-trajectory bar (one, global)

- `N_FORWARD_STEPS` equal-width cells. Leftmost cell = step 1 (1° away),
  rightmost = step N (12° away).
- Each cell coloured red if that step collides, green if free, grey if the
  robot is idle (`|v_cmd|` below threshold).
- Distance ticks every 5 steps labelled in degrees.
- A single **vertical black line** at the first colliding step, so the user
  can see at a glance the nearest forward hit.

### 5.4 Clamp-diagnostics panel

Three horizontal progress bars in the same row, labelled:
- **Proximity collision clamp** — value of `prox_scalar`.
- **Forward collision clamp** — value of  `forward_collision_scalar` (0..1).
- **Overall Speed Reduction (% of max)** — due to the two clamps.

Plus one detail line showing: nearest proximity distance, nearest forward
collision distance, `|v_cmd|`, `|v_out|`, current forward-collision speed
clamp shape, proximity age (ms),
proximity pipeline duration (ms).

### 5.5 Controls panel

Sliders / numeric entries for: Target FPS, Slow key dps, Fast key dps,
Proximity floor %, Path cutoff °. A radio pair for the forward-collision
speed clamp shape
(`linear` vs `exponential`), and a separate slider for the exponential
steepness `k`. Two buttons: **Reset pose** (to the home position), **Stop
(zero vel)** (clears the current velocity but keeps held keys).

### 5.6 Per-axis-limits panel

Six numeric spin-boxes for **max velocity** per joint, six more for **max
acceleration** per joint. Changes take effect on the next control tick (they
flow through `IntentSnapshot`).

### 5.7 UI thread responsibilities and forbiddens

The UI thread MAY:
- Push the current pose to the visualisation backend on each refresh.
- Run **one** throttled collision check on the visualisation client to
  update the FREE / COLLISION label (default 10 Hz).
- Read any control-thread-owned state for drawing **without locks**. The
  read is allowed to be torn / stale by up to one tick; the human eye will
  not notice.

The UI thread MUST NOT:
- Touch worker pools.
- Run a per-frame collision check on the GUI client.
- Block on anything except the toolkit's own event wait.

### 5.8 Input handling rules

- Keyboard / button events go through a focus filter: if a text-entry
  widget has focus, do **not** consume jog keys.
- Releasing a key always clears it from the held set (even if focus moved
  away between press and release).
- Every input change rebuilds the entire `IntentSnapshot` and rebinds it.
- Future integration with haptic dials may require a richer input representation than a set of held keys; design the snapshot accordingly (e.g. a dict of axis name → value).

---

## 6. Headless mode, automation, and logging

These features exist so the program can be tested **without a human** and
benchmarked reproducibly.

### 6.1 Scripted-input format

A test scenario is a JSON array of events. Each event has a scheduled
offset `t` (seconds from start) and an `action`. Supported actions:

| Action       | Required fields                | Effect |
|--------------|--------------------------------|--------|
| `press`      | `key` (string)                 | Synthesises a key-press event on the UI thread; records the dispatch time. |
| `release`    | `key` (string)                 | Synthesises a key-release event; records the dispatch time. |
| `resize`     | `w`, `h` (ints, pixels)        | Sets the window geometry; records the dispatch time. |
| `set_fps`    | `value` (float, Hz)            | Changes the target control FPS by writing the corresponding Tk variable. |
| `screenshot` | `path` (string)                | Saves a PNG of the window's screen rectangle (current implementation uses `PIL.ImageGrab.grab(bbox=...)`). |
| `quit`       | —                              | Triggers the idempotent close handler. |

Example: [pybullet/scripts/jog_and_resize.json](pybullet/scripts/jog_and_resize.json).

### 6.2 CLI flags for automation

- `--script PATH` — load and replay a script file.
- `--duration SECONDS` — auto-quit after this many seconds (useful with or
  without a script).
- `--metrics PATH` — on exit, dump a metrics JSON to `PATH`.
- `--gui-hz HZ` — view-loop repaint rate (default 15; see §1).
- `--forward-workers N`, `--prox-workers M` — process counts (defaults 6/6;
  production target 3/2 per robot).

### 6.3 Per-tick session log (JSONL)

Written by the control thread, line-buffered, one file per session under
`pybullet/explorer_logs/session_<timestamp>.jsonl`.

- **Line 1 — header.** Records: schema version, start timestamp, joint
  names, joint limits in degrees, all forward/proximity geometry constants,
  worker counts and chunk assignments, keyboard layout, and the defaults
  for every tunable.
- **Lines 2..N — per-tick rows.** Each row contains:
  - `n` — monotonically-increasing tick index.
  - `t` — seconds since session start.
  - `dt` — measured tick duration.
  - `v_des`, `v_cmd`, `v_out`, `pos`, `vel` — six-element float arrays in
    degrees / deg/s.
  - `in_coll` — current-pose collision status (may be null between
    throttled refreshes).
  - `ps`, `qs`, `fs` — path scalar, proximity scalar, final scalar.
  - `p_near`, `q_near` — nearest distances in deg (path and proximity).
  - `fwd` — the `N_FORWARD_STEPS` bools packed into a single integer
    (bit `i` = step `i+1`). Helper `unpack_bits(value, length)` is provided
    for replay tools.
  - `prox` — list of six packed ints, one per axis.
  - `prox_age_ms`, `prox_pipe_ms` — staleness and pipeline timing.
  - `cfg` — snapshot of all tunables (per-axis limits, floor, cutoff, shape, exp_k, target_fps).

### 6.4 Metrics JSON (`--metrics`)

Produced on exit. Fields:

- `duration_s`, `ctrl_ticks`, `ctrl_fps`, `gui_frames`, `gui_fps` —
  overall counts and rates.
- For each of `ctrl_dt_ms`, `ctrl_late_ms`, `ctrl_fwd_ms`,
  `ctrl_prox_pipeline_ms`, `ctrl_prox_age_ms`, `gui_dt_ms`,
  `input_latency_ms`, `resize_stall_ms`: a stats dict
  `{n, p50, p95, p99, max, mean}`.
- `inputs_dispatched`, `inputs_resolved`, `resizes` — event counts.

**Input latency** is measured by walking the per-tick log of held keys
forward from each scripted dispatch time until the held-key set first
matches the expected post-event state, and recording the time delta. This
measures **end-to-end UI → control responsiveness**, not just the UI event
delay.

### 6.5 Testing matrix

Correctness and performance should be re-verified after any rewrite by
running, at minimum:

| Mode | Command shape | What it proves |
|------|---------------|----------------|
| **Headless soak** | `--script <jog_scenario> --duration N --metrics out.json` with the UI window hidden / off-screen | Control loop runs at target rate; no worker leaks; log file is well-formed JSONL; metrics dump is sane. |
| **Visual confirmation** | Run interactively; jog every axis with `1..6 / q..y / a..h / z..b..n` in turn | All six axes respond; proximity bars update; forward bar shows red when driving into a wall; FREE/COLLISION label flips correctly. |
| **GUI-hz sweep** | Run the same script at `--gui-hz` ∈ {5, 10, 15, 30}; compare metrics files | Quantifies UI ↔ control contention. Used to pick the default `--gui-hz`. |
| **Worker-count sweep** | Run the same script at several `--forward-workers/--prox-workers` combinations | Establishes the production sweet spot for the target CPU. **Re-run on the actual hardware** (the dev-laptop optimum will not transfer to the Core Ultra 5 225 production box.) |
| **Touch-list regression** | Run with the home pose and confirm `FREE`; deliberately omit the touch-list patch and confirm `COLLISION` | Self-collision suppression is wired correctly. |

A helper script [pybullet/_compare_metrics.py](pybullet/_compare_metrics.py)
formats the metrics from a sweep into a comparison table.

### 6.6 Replay

The per-tick log + the header is sufficient to **deterministically replay** a
session offline — all geometry constants, worker chunking, defaults, and
per-tick intent are recorded. A replay tool only needs to re-evaluate the
clamp math from the logged bool arrays; it does not need to re-run any
collision checks.

---

## 7. Where the bodies are buried (gotchas for rewriters)

- **`compas_fab` `set_robot_cell_state` deepcopy.** The upstream version
  deep-copies the state on every call, which dominates the per-check cost.
  The local environment has a patched version; the rewrite must either
  reapply the patch or avoid the round-trip.
- **PyBullet `gui` connections are thread-affine.** The single GUI client
  may only be touched from one thread. Workers must use `direct`.
- **Tk's tcl bindings do not release the GIL well.** Pushing the view-loop
  rate above ~15 Hz on Windows starves the control thread; the symptoms
  look like a broken worker pool but are actually GIL contention. A Pygame
  rewrite should not have this problem; verify by running the GUI-hz sweep.
- **Windows timer quantum (~15.6 ms) caps sleep precision.** Without
  `winmm.timeBeginPeriod(1)`, the control loop tops out around 30 Hz even
  when there is no contention. Call it (and `timeEndPeriod` at exit) if you
  need ≥ 50 Hz.
- **Joint-space step is fixed at 1°, but the URDF allows wrist joints
  ±360°.** The current code overrides limits to a uniform `±180°` to keep
  the visualisation bars sane; preserve this in the UI or expose a toggle.
- **Forward chunks are 1-based, proximity chunks are 0-based.** The forward
  list represents *future* step numbers (step 1 is the first one in front);
  the proximity list is indexed by joint axis number. Keep the convention or
  rename both — but don't half-rename.
- **Proximity result list layout.** Index `PROBE_HALF − 1 − j` is the
  `−(j+1)°` probe, index `PROBE_HALF + j` is the `+(j+1)°` probe. There is
  **no entry for offset 0**.
- **The proximity clamp is global, not per-axis.** A single obstacle on any
  axis slows down the whole vector. This is deliberate (preserves direction)
  but counter-intuitive; do not "fix" it without re-deriving the safety
  argument.

---

## 8. Open questions for the rewrite

These were left undecided in the current implementation and should be
revisited when picking up the next iteration:

- **Process vs thread for the control loop.** Threading caps at ~30 Hz on
  Windows + Tk; multiprocessing would bypass the GIL but adds an IPC step
  on the snapshot. Pygame may eliminate the Tk-specific contention entirely.
- **Per-frame collision check on the GUI client.** Currently throttled to
  10 Hz to protect the tick rate. A Pygame rewrite using a non-PyBullet
  visualiser (or no live visualiser) would not need this throttle.
- **`winmm.timeBeginPeriod(1)`.** Not currently called. Decide based on the
  target tick rate.
- **Multiple input sources.** Today the `pressed` field is `frozenset[str]`
  of key names. For a gamepad / haptic-dial / network rewrite, replace it
  with a richer command object (e.g. per-axis desired velocity, already
  in deg/s) and move the keyboard mapping into the UI layer. The control
  loop should not know the difference.
- **Real-robot output.** The integrator currently writes back into a local
  `pos_rad` array and pushes to the visualiser. The same `v_out` vector can
  be fed to the robot driver instead; the safety gate guarantees it is the
  vector actually checked. The 200 Hz physics-reference rate from
  [AGENTS.md](AGENTS.md) does **not** need to match the control rate, but
  the rate-limiter that today lives in the main game repo will need to be
  reintroduced after this filter.
