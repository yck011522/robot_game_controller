# Logging

The Game Controller PC does **not** keep 24/7 per-process INFO logs. The
system runs in arcade mode for weeks at a time; the noise from streaming
logs at 50 Hz would bury anything useful. Instead there is exactly one
recording system:

- **`EventRecorder`** taps the ZMQ bus and writes one folder per game.
- A running ledger `recordings/index.jsonl` lists every completed game
  with metadata + final scores.
- Per-process crash files are written **only** when a process dies of an
  uncaught exception. They are not used for routine debugging.

Everything else (live tap, replay, dashboards) reads from those files
or from the live bus, not from log files.

Status: **DRAFT — subject to more detailed review. Do not implement at this stage**
Last reviewed: 2026-06-04.

See [BUS.md](BUS.md) for the topic catalog this recorder subscribes to,
and [CONFIG.md §2](CONFIG.md#2-top-level-schema) for the `recorder.*`
configuration block.

---

## 1. Process model

`EventRecorder` is a single process spawned by the launcher. It opens
**one SUB socket** against the bus broker's XPUB endpoint with
`setsockopt(SUBSCRIBE, b"")` so it receives **every** topic. There is
no filter list; future topics are captured automatically.

It is purely passive on the realtime bus:

- No request/reply work — collision REQ/REP traffic at `:5560`/`:5561`
  is **not** recorded (it is workload, not state).
- It does not publish anything on the realtime bus except its own
  `heartbeat.event_recorder` at 1 Hz.
- It serves an HTTP client of its own only outbound, when pulling
  per-game files from the Vision / Audio PCs (§5).

The recorder is **always allowed to lag** the bus. CONFLATE is **not**
set on its SUB; we want every message. The launcher sizes the SUB high
water mark generously (default `RCVHWM = 100_000`); a brief filesystem
stall must not drop bus traffic.

---

## 2. On-disk layout

`recorder.root` (from CONFIG.md) is the directory the recorder writes
under. Default: `C:/recordings`. Structure:

```
<root>/
  index.jsonl                       # one line per completed game (§3)
  games/
    <game_id>/                      # one folder per game (§2.1)
      meta.json                     # static metadata + final state
      bus.jsonl                     # every ZMQ main-bus message
      haptic_raw/                   # optional raw serial dumps
        a.jsonl
        b.jsonl
      robot_rtde/                   # raw RTDE samples per team (real impl only)
        a.jsonl
        b.jsonl
      skeleton/                     # pulled from Vision PC after game end
        red.jsonl
        blue.jsonl
      audio/                        # pulled from Audio PC after game end
        red_1.jsonl … red_6.jsonl
        blue_1.jsonl … blue_6.jsonl
  crashes/                          # per-process crash dumps (§6)
    <proc>_<wall_ts>.txt
  archive/                          # optional: zipped old games (operator-managed)
```

The recorder creates `<root>/games/<game_id>/` lazily when GC first
publishes a non-null `game_id` on `state.full` (i.e. on the Idle →
Tutorial edge — see [BUS.md §4.3](BUS.md#43-game-relative-time)).
Before that moment, bus messages are buffered in memory; if no
Tutorial entry happens (e.g. the launcher is killed in Idle), nothing
is written.

### 2.1 `game_id`

Set by GameController. Format:
`YYYY-MM-DD_HH-MM-SS_run-NNN`, local time of the Controller PC.
`NNN` is a per-day counter (resets at midnight, persisted across
restarts via the highest `NNN` already on disk that day).

Same string is reused by the external PCs in their own per-game
capture folders so the recorder can match them up at game end.

### 2.2 `meta.json`

Written once at the moment the game enters **Reset** stage (i.e. the
game is over and the final scores are stable). Single JSON object:

```jsonc
{
  "game_id": "2026-06-04_19-12-03_run-042",
  "profile_name": "show",                    // CONFIG.md profile_name
  "git_sha": "abc1234",                      // controller PC repo head at launcher start
  "controller_pid": 9876,
  "active_teams": ["a", "b"],

  // Stage timeline (wall ns UTC, from state.full snapshots)
  "stages": {
    "tutorial_entered_wall_ns": 1717400000000000000,
    "play_entered_wall_ns":     1717400060000000000,
    "conclusion_entered_wall_ns": 1717400240000000000,
    "reset_entered_wall_ns":    1717400270000000000
  },
  "duration_s": {
    "tutorial":   60.0,
    "play":      180.0,
    "conclusion": 30.0
  },

  // Final scores (lifted from the last state.full before Reset).
  "final_score": {"a": 142, "b":  98},

  // Final bucket weights, grams.
  "final_buckets": {
    "11": 32.1, "12": 50.7, "13": 60.0,
    "21":  0.0, "22":  0.0, "23":  0.0
  },

  // Process liveness summary (see §4 for the per-process counters).
  "crash_flag": false,                       // true if any monitored process died mid-game
  "crashed_processes": [],                   // list of {name, ts_wall_ns, reason}

  // External PC pull status; null fields are filled in asynchronously
  // (see §5) and the index.jsonl row is appended only once both are
  // populated (or marked permanently_failed).
  "external_pull": {
    "vision": {"status": "ok",      "files": 2,  "bytes": 1820341},
    "audio":  {"status": "pending", "files": null, "bytes": null}
  }
}
```

### 2.3 `bus.jsonl`

One JSON object per line. Append-only, in receive order. Format:

```jsonc
{
  "ts_recv_wall_ns": 1717400000123456789,   // recorder's wall-clock at recv()
  "ts_recv_mono_ns": 9876543210,            // recorder's monotonic at recv()
  "topic":  "state.full",
  "body":   { ... full JSON body from BUS.md §6 ... }
}
```

The recorder's `ts_recv_wall_ns` is added so a reader can compute
broker-to-recorder transit time as
`ts_recv_wall_ns - body.ts_wall_ns` (when the producer carries
`ts_wall_ns`, which is required on `state.full` and `heartbeat.*`).

`bus.jsonl` is **not** sorted by the producer's timestamp; messages
land in whatever order the SUB delivers them. The replay tool (§7)
re-sorts as needed.

Size budget: with all 14ish PUBs running flat-out, a 5-minute game is
roughly 200–400 MB uncompressed. The recorder closes the file at game
end and optionally `gzip`s it (toggle in CONFIG.md `recorder.gzip_at_end`,
default `true`).

### 2.4 `haptic_raw/<team>.jsonl` and `robot_rtde/<team>.jsonl`

Optional raw streams that bypass the main bus. Written **by the
producing I/O process itself** (HapticIO writes its serial frames,
RobotIO writes RTDE samples) into the per-game folder, using the
recorder's filesystem path discovered via a `req.recorder_path` call.
The recorder is not in the hot path for these.

Disabled by default; enable per-subsystem in CONFIG.md
(`recorder.keep_haptic_raw`, `recorder.keep_robot_rtde`, both not yet
wired — reserved fields). Useful only when chasing hardware-side
glitches.

### 2.5 `skeleton/` and `audio/`

Pulled from the external Vision / Audio PCs after the game ends. See §5.

---

## 3. `recordings/index.jsonl`

One JSON object per **completed** game, appended atomically. The
gamemaster UI, post-hoc analytics, and the "high score" display
([NEXT_STEPS](../../NEXT_STEPS.md) §2.E.33/34) all read this file
instead of scanning `games/`.

```jsonc
{
  "game_id":            "2026-06-04_19-12-03_run-042",
  "tutorial_entered_wall_ns": 1717400000000000000,
  "reset_entered_wall_ns":    1717400270000000000,
  "duration_s":         270.0,
  "profile_name":       "show",
  "active_teams":       ["a", "b"],
  "final_score":        {"a": 142, "b": 98},
  "winner":             "a",                       // "a" | "b" | "tie"
  "crash_flag":         false,
  "folder":             "games/2026-06-04_19-12-03_run-042"
}
```

Appended **after** `meta.json` has been written and (if external PCs
are configured) their pulls have either succeeded or been marked
`permanently_failed`. The recorder writes a temp line + `fsync` +
rename so a power-cut never leaves a half-line.

A line is **never** rewritten in place. If late-arriving external
files change the picture, a follow-up line is appended with the same
`game_id` and a `"supersedes_prior": true` field. Readers should take
the last line per `game_id`.

---

## 4. Process health summary

The recorder also subscribes to every `heartbeat.<proc>` topic ([BUS.md
§6.9](BUS.md#69-heartbeatproc)) and tracks per-process last-seen
timestamps. At `meta.json` write time it scans the per-process
counters; any process that was expected (per the profile) but missed
heartbeats for > 5 s during the run is added to
`meta.crashed_processes` and `crash_flag` is set `true`.

This is the only "uptime audit" we keep. It does not replace the
Supervisor's own respawn logic (see SUPERVISOR.md, to be written).

---

## 5. External Vision / Audio file pull

Mechanism is fully specified in [BUS.md §11](BUS.md#11-external-pcs--visionaudio--out-of-band-file-transfer).
Recorder-side responsibilities:

1. On `state.full.stage` edge `conclusion → reset`, GameController
   sends `POST /game_ended` to each configured external PC. Recorder
   sees the same edge and arms its pull state machine.
2. Recorder issues `GET http://<vision_pc>:8080/captures/<game_id>/`
   (directory listing JSON) then `GET` each listed file into
   `<root>/games/<game_id>/skeleton/`. Same for `<audio_pc>` into
   `audio/`.
3. On HTTP failure (connection refused, 5xx, timeout), retry every
   60 s for up to 1 h. After that, mark `external_pull.<pc>.status =
   "permanently_failed"` and append the index row regardless.
4. If a retry eventually succeeds after the index row was already
   appended, write a fresh row with `supersedes_prior: true` (§3).
5. No auth, no TLS — closed LAN assumption (BUS.md §11).

External PCs are **not** required at launch; their absence shows up as
silent `heartbeat.vision_pc` / `heartbeat.audio_pc` on the dashboard
but does not block recording.

---

## 6. Crash files

One file per uncaught exception, written by the dying process itself
just before it exits:

```
<root>/crashes/<proc>_<wall_ts>.txt
```

Contents: full Python traceback, `sys.argv`, `os.environ` filtered to
relevant variables, last 200 lines of any per-process scratch log
(processes that maintain one in-memory), and the most recent
`state.full` the process had seen if it was a SUB. Plain text, not
JSON — these files are for humans, not the replay tool.

Implementation is deferred to a small `core/crashfile.py` helper
installed as `sys.excepthook` on process start. Not part of the first
integration milestones (P2..P6); slot exists so the path / format is
stable when it lands.

Crash files are never rotated automatically. Operator wipes them when
disk pressure demands it.

---

## 7. Replay tool

`tools/replay.py` reads a per-game folder and re-publishes the bus
traffic onto a fresh ZMQ broker, honoring the original inter-message
gaps. Used for:

- Debugging: re-run a recorded incident against a modified
  GameController.
- Integration tests: drive the system from a known-good `bus.jsonl`
  and assert on state.

CLI (proposed; finalized when the tool ships):

```
python tools/replay.py <game_folder>
    [--speed 1.0]              # 0.5 = half speed, 0 = as fast as possible
    [--topics state.full,...]  # filter; default = everything
    [--start-stage play]       # skip ahead to this stage's entry
    [--stop-at conclusion]
    [--bus tcp://127.0.0.1:5550]
```

Replay alignment with skeleton / audio:

- The replay tool reads `meta.json` to get `tutorial_entered_wall_ns`,
  then projects each `skeleton/*.jsonl` and `audio/*.jsonl` row by
  matching its `ts_wall_ns` to the corresponding `bus.jsonl` row.
- Output for analysis is a separate joined CSV per request; not part
  of the realtime replay.

The replay tool publishes onto the **same** XSUB endpoint a live bus
uses, so any subscriber (UI, LED, recorder) can run unchanged against
replayed traffic. The original recorder, if connected, will happily
write a new game folder out of the replay — that is a feature, not a
bug; it makes regression baselines trivial. Operators should point
replay at a non-production broker if they don't want that.

---

## 8. What is **not** recorded

By design:

- **No video.** Vision PC delivers skeleton joints only. Raw camera
  feeds never leave the Vision PC. (Privacy; the installation is
  unattended in public.)
- **No raw audio by default.** Audio PC delivers prosody features
  only. Raw microphone WAV is opt-in via `recorder.keep_raw_audio`
  (reserved; not yet implemented).
- **No INFO/DEBUG log streams** from individual processes. If a
  process wants to expose internal state, it publishes a normal
  `telem.*` topic.
- **No collision REQ/REP traffic.** It is high-volume workload that
  the bus replay can already reconstruct (JoggingPlanner's inputs and
  outputs are both on the main bus).

---

## 9. Disk hygiene

The recorder does not delete anything. Operator-side cleanup:

- `tools/recordings_gc.py` (to be written) — given a date cutoff,
  moves old `games/<game_id>/` folders into `archive/<YYYY-MM>/<...>.tar.gz`
  and rewrites their `index.jsonl` rows with a `archived: true` flag.
- Recommended cadence: monthly, after a successful offsite copy.

Master switch: `recorder.enabled: false` in CONFIG.md keeps the
recorder process alive (for heartbeats) but writes nothing. Use this
for desk testing. Setting `subsystems.event_recorder: null` removes
the process entirely.

---

## 10. Open items

- Whether to switch `bus.jsonl` to a binary container (e.g. CBOR
  framed, or directly the ZMQ frame stream) once profiling shows the
  per-line JSON encode cost matters. Defer past P4.
- Whether to add a small SQLite mirror of `index.jsonl` for fast UI
  queries once the row count climbs into the thousands. Defer.
- Crash file `sys.excepthook` helper placement (`core/crashfile.py`
  vs `core/supervisor_client.py`). Defer to SUPERVISOR.md.
