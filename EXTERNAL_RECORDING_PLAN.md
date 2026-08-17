# External Media Recording (Skeleton + Audio/Prosody) — Implementation Plan

**Purpose:** Working plan for adding per-game external media capture to the
two-team production profiles: skeleton tracking from the Jetson Nano (2 USB
cameras, one per team) and audio/prosody from 6 Raspberry Pis (2 microphones
each, 12 players total). Recording happens **on the external devices**; this
PC only orchestrates start/stop and pulls the finished files back after each
game into the existing `recordings/` folder structure.

**Status:** Protocols confirmed and verified live against the hardware
(2026-08-17). Jetson and Pi firmware fixes (conditional duration cap, URL
advertise host, async save ACK) **implemented locally** in the two cloned
repos and tested — awaiting manual push + redeploy to the devices (§8).
Our-side coordinator code not started yet.

Protocol sources (external repos, pulled locally):

- Jetson: `C:/Users/yck01/GitHub/jetson_camera_experiments/skeleton-tracker/jetson/tracker_api.md`
- Audio Pis: `C:/Users/yck01/GitHub/voiceAnonymizer_PI/docs/AUDIO_INTEGRATION_FOR_GAME_CONTROLLER.md`
  (plus reusable patterns in `save_and_pull_logs.py` / `speech_control.py`
  and the fleet list in `start_recording_session.yaml` in that repo)

Precedents this plan builds on:

- [GAMEPLAY_RECORDER_PLAN.md](GAMEPLAY_RECORDER_PLAN.md) — the Parquet/CSV
  gameplay recorder whose folder layout already reserves `skeleton/` and
  `audio/` subfolders per game.
- [docs/architecture/BUS.md §11](docs/architecture/BUS.md) — original
  "external PCs" file-transfer design. Written for one Vision PC + one Audio
  PC; this plan adapts it to the real 1-Jetson + 6-Pi topology.
- [docs/architecture/LOGGING.md §5](docs/architecture/LOGGING.md) — pull
  status / retry expectations.

---

## 1. Decisions locked in (2026-08-17)

| Decision | Choice |
|---|---|
| Recording window | **Tutorial entry → Play end** — identical stage edges to `gameplay_recorder`, so all per-game artifacts cover the same time span. Conclusion/Reset/rewind are excluded. |
| Service shape | **New launcher-spawned process** `external_media_coordinator`. `gameplay_recorder` stays a pure bus→Parquet writer; network orchestration and post-game pulls live in the new process (separate failure domain). |
| Offline device at game start | **Warn and play anyway.** Dashboard/log warning; the game proceeds and that device's media is simply missing (or partially pulled later) for that game. |
| `two_teams_record_session.yaml` | **Leave as-is.** It records the display-broadcast feed for the daydream replayer — a different feature despite the similar name. Not touched by this work. |
| Game identity distribution | **Push, not subscribe.** The coordinator sends the game identity string in the start command; devices do not need ZMQ or bus access. |
| Ledger | **`recordings/games_index.csv` stays byte-for-byte backwards compatible — untouched.** External-media pull status goes in a **separate** ledger, `recordings/external_media_index.csv`, joined by the same `date` + `time` key columns. |
| Raw media toggles | `keep_raw_video` / `keep_raw_audio` are profile config, sent **in the start command** (both protocols support this natively). Default profile `two_teams.yaml`: both **off** (skeleton/prosody only). `two_teams_longer_duration.yaml`: both **on** (MP4 + FLAC pulled too). |

## 2. Hardware topology (as described by user)

| Device | Count | Role | Produces |
|---|---|---|---|
| Jetson Nano (`recorder_node.py`) | 1 | Skeleton tracking via 2 USB cameras: **cam 0 → team a, cam 2 → team b** (fixed mapping in tracker_api.md) | Per team: `skeleton.parquet` + `frames.parquet`, optional `video.mp4` (mp4v, 30 fps, raw frames) |
| Raspberry Pi (rpi5-11 … rpi5-16) | 6 | Audio + prosody, **one process per mic** (ctrl ports 9001/9002), 12 mic processes total | Per mic: `opensmile_lld.csv`, `vad.csv`, optional `emotion.csv`, optional `audio.flac` + `audio.json` sidecar. **CSV, not Parquet** — a later parquet-conversion step is possible but out of scope. |

All devices are on the same LAN as this PC and **verified reachable by ping
(2026-08-17)**. Pi IPs are established: **192.168.0.11–16**. Jetson is
**192.168.0.101** (WS :9000 and HTTP :9100 both confirmed open).

Mic → player mapping (confirmed by user, mirrors display broadcast order but
kept as an independent copy in config): rpi5-11 MIC1→a1 MIC2→a2,
rpi5-12→a3/a4, rpi5-13→a5/a6, rpi5-14(0.14)→b1/b2, rpi5-15(0.15)→b3/b4,
rpi5-16(0.16)→b5/b6. Same convention as `display_broadcast.hosts` in
`config/device_ports_and_addr.yaml`, duplicated so it can diverge later.

## 3. On-disk layout (destination on this PC)

Extends the existing gameplay-recorder layout — no new root, no new ledger:

```
recordings/
  games_index.csv                 # existing ledger (unchanged schema)
  games/
    <date>/                       # e.g. 2026-08-17  (HK local, existing)
      <time>/                     # e.g. 14-32-05    (HK local, existing)
        state_global.parquet      # existing
        a/  b/                    # existing per-team Parquet
        skeleton/                 # NEW - pulled from Jetson (HTTP :9100)
          a/skeleton.parquet      #   mirrors Jetson on-device layout
          a/frames.parquet
          a/video.mp4             #   only when keep_raw_video: true
          b/ …
        audio/                    # NEW - pulled from Pis (SCP over SSH)
          <pi_id>/MIC1/opensmile_lld.csv, vad.csv, emotion.csv,
          <pi_id>/MIC1/audio.flac, audio.json   # only when keep_raw_audio: true
          <pi_id>/MIC2/ …         #   6 Pis x 2 mics = 12 folders
        external_media_manifest.json   # NEW - per-device pull status (§4.4)
```

The `audio/<pi_id>/<MIC>/` nesting preserves the Pi-side path structure
verbatim (`<pi_id>` = `rpi5-11`…`rpi5-16`) — the mic→player mapping stays in
config instead of being baked into folder names, so a mapping change never
invalidates old recordings. Player-level symlinks/copies can be added by
analysis tooling later if wanted.

### 3.1 `recordings/external_media_index.csv` (NEW ledger)

`games_index.csv` is **not modified**. New separate ledger, one row per game,
joinable on `date` + `time`:

| Column | Notes |
|---|---|
| `date`, `time` | same HK-local key as the game folder / `games_index.csv` |
| `skeleton_status`, `audio_status` | `ok | partial | failed | disabled` (rolled up from the manifest) |
| `skeleton_files`, `skeleton_bytes` | totals across both teams |
| `audio_files`, `audio_bytes` | totals across all 12 mics |
| `raw_video`, `raw_audio` | `0`/`1`, echoes the toggles the game was started with |
| `devices_missing` | e.g. `"pi_audio:rpi5-13:MIC2; jetson"`, empty when all ok |

Row is appended when every device reaches a terminal state (ok/failed) —
which can be up to the retry window after the game ends. Same atomic-append
approach as `games_index.csv`.

The coordinator never invents folder names itself: it consumes the active
game folder path published by `gameplay_recorder` (§4.2) so both processes
always agree on `<date>/<time>`.

## 4. `external_media_coordinator` service

New app at `src/apps/external_media_coordinator/`, spawned by the launcher
next to `gameplay_recorder` (same tier), enabled per profile via
`subsystems.external_media_coordinator: real|null`.

### 4.1 Responsibilities

1. Watch `state.full` for the same stage edges `gameplay_recorder` uses:
   **Idle → Tutorial** = game start, **edge out of Play** = game end.
2. At game start: send **start** to the Jetson and every configured Pi,
   including the game identity (`<date>/<time>`) so each device names its
   local capture folder identically. Devices that don't ack are logged and
   surfaced on the dashboard; the game is never blocked (locked decision).
3. At game end: send **stop** to every device that acked start.
4. After stop: pull each device's capture folder back into the game's
   `skeleton/` / `audio/` subfolders. Pulls are asynchronous — a slow or
   large transfer (video) must never delay the next game.
5. Retry queue: devices offline at game end are retried on a backoff
   schedule (per BUS.md §11: every minute for an hour, then marked
   `permanently_failed` in the manifest).
6. Publish `heartbeat.external_media` (1 Hz) plus a status topic
   (`telem.external_media`, low rate) with per-device
   online/recording/pulling/done/failed state for the dashboard.

### 4.2 Game folder identity

`gameplay_recorder` gains a tiny PUB topic, e.g. `recorder.game`, emitted on
the same stage edges:

```jsonc
// on Tutorial entry
{"event": "game_started", "folder": "recordings/games/2026-08-17/14-32-05",
 "date": "2026-08-17", "time": "14-32-05", "ts_wall_ns": 1755000000000000000}
// on Play end
{"event": "game_ended", "folder": "recordings/games/2026-08-17/14-32-05",
 "ts_wall_ns": 1755000180000000000}
```

The coordinator keys all device folders and destination paths off this. If
`gameplay_recording.enabled` is false in a profile, the coordinator falls
back to deriving the folder name from its own `state.full` timestamps using
the same HK-local convention (documented helper in `core.gameplay_recording`
so the naming logic exists exactly once).

### 4.3 Wire protocols (CONFIRMED)

**Jetson (skeleton/video)** — one WebSocket JSON command channel, HTTP pull:

- WS server on `ws://<jetson>:9000`; commands carry `type: "command"` +
  `request_id`; every command gets exactly one `ack` echoing the id.
- Start: `{command: "start_recording", args: {date, time, record_skeleton,
  record_video}}` — records **both** teams in one command; `record_video`
  is where `keep_raw_video` plugs in. Date/time are literal folder names
  (filesystem-sanitized only) — we pass the same `<date>/<time>` strings as
  the game folder.
- Stop: `{command: "stop_recording"}` — ack includes per-team file lists
  with ready-made download URLs and byte counts: `http://<jetson>:9100/
  recordings/<date>/<time>/<team>/{skeleton,frames}.parquet[,video.mp4]`.
- Also available: `get_status` (fps/people_count/inference_ms per team —
  useful for the dashboard status topic), `ping` (liveness), server-pushed
  events (`camera_lost`, auto-stop at the **20-minute max duration** with
  `reason: "max_duration"`).
- Session rules: WS disconnect keeps recording; a second `start_recording`
  discards the in-progress session. Coordinator must therefore never
  re-issue `start` with a new identity mid-game (same trap as the Pi's).

**Verified live 2026-08-17** (`tools/probe_jetson_recorder.py`): ping, status,
6 s skeleton-only session, stop, HTTP pull all work. Two discrepancies vs
tracker_api.md — both small Jetson-side bugs, not blockers (§8):
  - The stop ack's file URLs say `localhost:9100` and **omit the
    `/recordings/` prefix**; the HTTP server's root is the `recordings/`
    dir itself, so real URLs are `http://<jetson>:9100/<date>/<time>/
    <team>/<file>`. Our pull rewrites host→Jetson IP and uses the ack path
    as-is (doc's `/recordings/` prefix 404s).
  - Auto-stop at the 20-min cap calls the same `manager.stop()` as an
    explicit stop, so **a capped recording still saves and is pull-able** —
    the "grab the shortened file" behavior is already correct.

**Pis (audio/prosody)** — OSC over UDP per mic process, SCP pull:

- 12 endpoints: `<pi_ip>:9001` (MIC1) and `<pi_ip>:9002` (MIC2) for
  192.168.0.11–16. Commands to `/ctrl/<command>`, positional string args,
  every command ACKed (default timeout 0.75 s). Use `speech_control.py`'s
  `send_ctrl` / `parse_target_endpoint` from voiceAnonymizer_PI — do not
  hand-roll OSC. The integration doc explicitly prescribes the bridge-process
  shape we chose (their §7).
- Start: `log_start LOG_DAY LOG_TIME max_minutes record_audio` —
  `record_audio` is where `keep_raw_audio` plugs in. Same `LOG_DAY`/`LOG_TIME`
  for all 12 so the fleet shares one session identity.
- Stop/save: `log_save_stop` (writes files on the Pi). `log_discard_stop`
  available for aborted games.
- Also available: `log_pause` / `log_resume` — natural hook for the game's
  pause feature (candidate enhancement, not in the initial scope).
- Idempotency: duplicate `log_start` with the **same** identity is a no-op;
  with a **different** identity it discards the open session. Retry only
  with the original identity. Requires the latest `strip_monitor.py` on the
  Pis (pre-idempotency builds discard on duplicate start).
- Pull: files land at `/home/pi/SPEECH_RECORD_ANALYSIS/SESSION_LOGS/<day>/
  <time>/<MIC>/` on each Pi; harvest over SSH (paramiko, password auth —
  `config/secrets.yaml`). Paths are deterministic (no manifest over the wire
  yet — planned Pi-side, not built). Blocking `send_ctrl` and SSH calls must
  run on worker threads, never on the bus tick.

**Verified live 2026-08-17** (`tools/probe_pi_audio.py`, rpi5-11 MIC1):
OSC start + SSH file listing both work. Two operational findings, both now
addressed by firmware changes (§8):
  - `log_save_stop` used to block the ctrl handler through the whole write
    before ACKing (needs a long timeout and stalls other commands). **Fixed:**
    now ACKs immediately with `"saving"` and saves on a thread (§1 Save ACK
    pattern). The per-file `/saved` notices it already streams become the
    completion signal.
  - A duplicate `log_start` with the same identity **discarded the session**
    on this Pi → the fleet runs a **pre-idempotency `strip_monitor.py`**
    build (their doc §8). Redeploy the updated firmware (§8.3) before relying
    on retry logic. Until then: never re-issue `log_start`.

### 4.4 `external_media_manifest.json`

Written into each game folder; updated asynchronously as pulls complete:

```jsonc
{
  "folder": "recordings/games/2026-08-17/14-32-05",
  "devices": {
    "jetson":              {"status": "ok", "files": 4, "bytes": 183600497},
    "pi:rpi5-11:MIC1":     {"status": "ok", "files": 2, "bytes": 410233},
    "pi:rpi5-11:MIC2":     {"status": "pending", "files": null, "bytes": null}
  }
}
```

Status enum: `pending | pulling | ok | partial | unreachable_at_start |
permanently_failed`. The roll-up of this manifest is what lands in
`external_media_index.csv` (§3.1) once every device is terminal.

## 5. Configuration

New top-level profile block:

```yaml
external_recording:
  enabled: true
  skeleton:
    host: "192.168.0.101"       # Jetson Nano (verified reachable 2026-08-17)
    ws_port: 9000               # command channel (tracker_api.md)
    http_port: 9100             # file pull
    keep_raw_video: false       # true -> record + pull video.mp4 per team
  audio:
    keep_raw_audio: false       # true -> log_start record_audio=1 + pull audio.flac
    max_minutes: 60             # Pi in-RAM cap arg to log_start; must exceed profile game duration
    ack_timeout_s: 0.75         # per-command OSC ack timeout (their default)
    emotion: false              # enable emotion.csv alongside prosody/VAD
    ack_preflight: true         # verify all 12 mics ack log_start; unreachable ones are
                                # logged + flagged in manifest (game still proceeds, per locked decision)
    # 12 mic endpoints; (team, player) mapping is an INDEPENDENT COPY of the
    # display-node mapping in docs/DISPLAY_BROADCAST_PROTOCOL.md §4 (user
    # confirmed same order, kept separate so it can diverge later).
    devices:
      - {pi_id: rpi5-11, host: "192.168.0.11", mics: [{mic: 1, team: a, player: 1}, {mic: 2, team: a, player: 2}]}
      - {pi_id: rpi5-12, host: "192.168.0.12", mics: [{mic: 1, team: a, player: 3}, {mic: 2, team: a, player: 4}]}
      - {pi_id: rpi5-13, host: "192.168.0.13", mics: [{mic: 1, team: a, player: 5}, {mic: 2, team: a, player: 6}]}
      - {pi_id: rpi5-14, host: "192.168.0.14", mics: [{mic: 1, team: b, player: 1}, {mic: 2, team: b, player: 2}]}
      - {pi_id: rpi5-15, host: "192.168.0.15", mics: [{mic: 1, team: b, player: 3}, {mic: 2, team: b, player: 4}]}
      - {pi_id: rpi5-16, host: "192.168.0.16", mics: [{mic: 1, team: b, player: 5}, {mic: 2, team: b, player: 6}]}
  pull:
    timeout_s: 60               # per-file transfer timeout (video.mp4 can be ~300 MB / 12 min)
    retry_interval_s: 60        # offline-device retry cadence
    retry_window_s: 3600        # give up after this and mark permanently_failed
  # Pi SSH credentials for the pull live in config/secrets.yaml (gitignored),
  # read by the coordinator — not duplicated here.

Two toggle presets (locked decision): `two_teams.yaml` gets
`keep_raw_video: false, keep_raw_audio: false`;
`two_teams_longer_duration.yaml` gets both `true`. Note the Jetson's
20-minute recording cap: the longer-duration profile's Play duration must
stay under it (verify when wiring the profile).

Device addressing may instead live in `config/device_ports_and_addr.yaml`
with the profile referencing names — decide at implementation time.

Profile/launcher wiring:

- `subsystems.external_media_coordinator: real` in
  [config/profiles/two_teams.yaml](config/profiles/two_teams.yaml) and
  [config/profiles/two_teams_longer_duration.yaml](config/profiles/two_teams_longer_duration.yaml);
  `null` everywhere else until hardware is available.
- New rate entry `subsystems.external_media_coordinator` in
  [config/runtime.yaml](config/runtime.yaml) (poll/wake only; the process is
  event-driven like `gameplay_recorder`).
- Launcher: spawn in the same tier as `gameplay_recorder`, gated on the
  subsystem entry being non-null.

## 6. Implementation stages

1. **Stage 1 — bus plumbing:** `recorder.game` topic in
   `gameplay_recorder` + shared HK-local folder-naming helper in
   `core.gameplay_recording`. Testable without any external hardware.
2. **Stage 2 — coordinator scaffold:** new app
   `src/apps/external_media_coordinator/` with stage-edge detection, config
   parsing, heartbeat + status topics, manifest + `external_media_index.csv`
   writing, and device clients behind a small interface (`start(date, time,
   toggles)` / `stop()` / `pull(dest)` / `ping()`) with a **sim
   implementation** (loopback fake devices writing dummy files into a temp
   dir) so the whole lifecycle is testable on this PC.
3. **Stage 3 — real device clients:**
   - *Jetson client*: `websockets` for the :9000 command channel, HTTP GET
     for :9100 pulls (stdlib `urllib` is enough). **Installed** `websockets`
     during probing.
   - *Pi client*: vendor or wrap `speech_control.py` from voiceAnonymizer_PI
     for OSC `send_ctrl`; SSH pull via `paramiko` (pattern from
     `save_and_pull_logs.py`). **Installed** `python-osc`; `paramiko` was
     already present. Still to add to `requirements.txt`: `websockets`,
     `python-osc` (ask before committing).
   - Order: one Pi first, then the Jetson, then the full fleet.
4. **Stage 4 — profile rollout:** enable in `two_teams.yaml` (raw off) and
   `two_teams_longer_duration.yaml` (raw on); on-rig validation; dashboard
   badges (optional, separate task).

## 7. Testing strategy

- Unit: stage-edge detection, folder-naming helper, manifest state machine,
  retry/backoff logic (simulated clock).
- Integration: launcher profile with sim device clients; drive a synthetic
  game (pattern after `tests/test_p2_demo.py` /
  `dev_two_teams_random_trajectory_rewind_batch.yaml`) and assert files land
  in `recordings/games/<date>/<time>/{skeleton,audio}/` with a complete
  manifest.
- Hardware: one-Pi bring-up profile before enabling all 6 + Jetson.

## 8. External-repo changes (implemented locally; push + redeploy pending)

Both repos are cloned on this machine. The fixes below are **edited and
tested locally**; the user will push to GitHub and redeploy to the devices.
None block our Stage 1–2 sim work.

**Jetson** — `C:/Users/yck01/GitHub/jetson_camera_experiments/skeleton-tracker/jetson/`:
1. **Conditional max-duration** — `recorder_node.py`: new
   `max_duration_skeleton_sec` (default = `max_duration_sec`); `expired()`
   picks the cap from the session's `record_video` flag (stored at `start`).
   `config.yaml`: `max_duration_sec: 1200` (video) + new
   `max_duration_skeleton_sec: 3600` (skeleton-only). Compiles; YAML verified.
2. **Advertise-host URL fix** — `config.yaml` new `jetson.advertise_host:
   "192.168.0.101"` so `stop_recording` acks return LAN-reachable URLs
   (was defaulting to `localhost` when bound to 0.0.0.0). Code comment added
   at the `advertise_host` resolution. (`tracker_api.md`'s `/recordings/`
   URL prefix is stale — server root is already the recordings dir; real
   path is `http://<jetson>:9100/<date>/<time>/<team>/<file>`.)
   **Needs redeploy + restart of `recorder_node.py` on the Jetson.**

**Pis** — `C:/Users/yck01/GitHub/voiceAnonymizer_PI/strip_monitor.py`:
3. **Async save ACK** — `_handle_ctrl_command` now special-cases
   `log_save_stop` / `raw_save_stop`: sends an immediate `"saving"` ACK and
   runs the blocking save on a daemon thread (`_run_save_command`), which
   streams per-file `/saved` notices and sends the final completion ack.
   Also un-blocks the single-threaded ctrl listener during saves. Compiles;
   dispatch logic unit-verified (immediate ack + 2 saved notices + final
   ack). **Needs redeploy of the latest `strip_monitor.py` (which also has
   the `log_start` idempotency fix) to all 6 Pis + restart.**
4. **Pi SSH** — resolved: password auth works (`pi` / `pi1234`), stored in
   gitignored `config/secrets.yaml`. No key setup needed.

## 9. Hardware verification log (2026-08-17)

| Check | Result |
|---|---|
| Ping Jetson 192.168.0.101 | ✅ 1 ms |
| Ping all 6 Pis (.11–.16) | ✅ all reply (first batch needed ARP warm-up) |
| Jetson WS :9000 + HTTP :9100 open | ✅ |
| Jetson ping / get_status / start / stop | ✅ acks correct; status shows both cameras 15–16 fps |
| Jetson skeleton-only 6 s session + HTTP pull | ✅ `skeleton.parquet` + `frames.parquet` per team, pulled over HTTP |
| Jetson auto-stop saves partial file | ✅ confirmed in code (`expired()`→`stop()`→`_close_locked(discard=False)`) |
| Pi OSC log_start (rpi5-11 MIC1) | ✅ ack ~5 ms |
| Pi duplicate log_start idempotency | ❌ discarded session (pre-idempotency firmware) → fixed in new build, redeploy pending (§8.3) |
| Pi log_save_stop | ✅ worked but blocked the listener ~1.7 s → now async: immediate "saving" ack + per-file saved notices + final ack (§8.3) |
| Pi SSH password auth + file listing | ✅ `pi`/`pi1234`, listed session dir |

Probe scripts kept as `tools/probe_jetson_recorder.py` and
`tools/probe_pi_audio.py` (reusable for Stage 3 hardware bring-up). A probe
session folder `2099-01-01/` remains on the Jetson (read-only HTTP server,
needs on-device `rm` to clear) — harmless, tiny.

