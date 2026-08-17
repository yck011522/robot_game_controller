# External Media Recording (Skeleton + Audio/Prosody) — Implementation Plan

**Purpose:** Working plan for adding per-game external media capture to the
two-team production profiles: skeleton tracking from the Jetson Nano (2 USB
cameras, one per team) and audio/prosody from 6 Raspberry Pis (2 microphones
each, 12 players total). Recording happens **on the external devices**; this
PC only orchestrates start/stop and pulls the finished files back after each
game into the existing `recordings/` folder structure.

**Status:** Protocols confirmed + verified live (2026-08-17); device firmware
fixes deployed and re-verified on hardware (§8/§9/§10). **Our-side
implementation: Stage 1 + Stage 2 done** — `recorder.game` bus topic +
`external_media_coordinator` app (sim device clients, manifest, ledger) with
5/5 tests passing and a sim launcher bring-up confirmed. Stage 3 (real
Jetson/Pi clients in `devices.py`) is written but not yet exercised against
live hardware from the coordinator. Folder layout + compression finalized
(§3).

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
| Concurrent start/stop | **All 12 mic commands are fanned out concurrently (one thread per mic), never in a sequential loop.** A save holds the Pi's single-threaded OSC listener while it flushes the FLAC buffer to SD; sequential stops stack those delays so later mics over-record and the earliest time out. Verified: concurrent stop → all mics 120.7–122.0 s on a 120 s target; sequential stop → 160–250 s staircase + 15 s ACK timeouts (§10). Same applies to `log_start`. Matches the audio doc §7 bridge-process guidance. |
| File formats / compression | **Prosody/VAD CSVs → Parquet, converted on this PC during the pull step** (Pis keep writing CSV; no Pi redeploy). Measured on real 6-min data: openSMILE CSV → Parquet(zstd) is ~34–44% of original (~3× smaller), typed, and matches the rest of `recordings/` (all Parquet/pyarrow). **FLAC and MP4 left as-is** — both already compressed (FLAC→gzip only ~57% on near-silent test audio, ~100% on real audio; MP4→zip ~100%). Size levers stay the `keep_raw_*` toggles, not post-hoc compression. `skeleton.parquet` is already zstd. |

## 2. Hardware topology (as described by user)

| Device | Count | Role | Produces |
|---|---|---|---|
| Jetson Nano (`recorder_node.py`) | 1 | Skeleton tracking via 2 USB cameras: **cam 0 → team a, cam 2 → team b** (fixed mapping in tracker_api.md) | Per team: `skeleton.parquet` + `frames.parquet`, optional `video.mp4` (mp4v, 30 fps, raw frames) |
| Raspberry Pi (rpi5-11 … rpi5-16) | 6 | Audio + prosody, **one process per mic** (ctrl ports 9001/9002), 12 mic processes total | Per mic: `opensmile_lld.csv`, `vad.csv`, optional `emotion.csv`, optional `audio.flac` + `audio.json` sidecar. **CSV, not Parquet** — a later parquet-conversion step is possible but out of scope. |

All devices are on the same LAN as this PC and **verified reachable by ping
(2026-08-17)**. Pi IPs are established: **192.168.0.11–16**. Jetson is
**192.168.0.101** (WS :9000 and HTTP :9100 both confirmed open).

Mic → player mapping (confirmed by user, mirrors display broadcast order but
kept as an independent block so it can diverge later): rpi5-11 MIC1→a1
MIC2→a2, rpi5-12→a3/a4, rpi5-13→a5/a6, rpi5-14(0.14)→b1/b2,
rpi5-15(0.15)→b3/b4, rpi5-16(0.16)→b5/b6. **This mapping is hardware
addressing and lives ONCE in `config/device_ports_and_addr.yaml` under the
`audio_capture:` block** (loaded via `core.device_connection.load_audio_capture`),
NOT duplicated per profile. Profiles only carry the Jetson address, the
raw-media toggles, and pull/retry tuning.

## 3. On-disk layout (destination on this PC) — FINALIZED 2026-08-17

Extends the existing gameplay-recorder layout — no new root, and
`games_index.csv` is untouched (backwards compatible). External media slots
into the **same** `<date>/<time>/` game folder, as two sibling subfolders
next to `a/`/`b/`, plus a manifest. Preview below uses real 6-min pull sizes
with `keep_raw_video` + `keep_raw_audio` ON (the `two_teams_longer_duration`
shape); with both OFF, `video.mp4` and `audio.flac`/`audio.json` are absent.

```
recordings/
  games_index.csv                 # existing ledger (UNCHANGED schema)
  external_media_index.csv        # NEW ledger, joined by date+time (§3.1)
  games/
    <date>/                       # e.g. 2026-08-17  (HK local, existing)
      <time>/                     # e.g. 15-58-45    (HK local, existing) - one game
        state_global.parquet      # existing: shared game state
        a/  b/                    # existing: per-team gameplay Parquet
        external_media_manifest.json   # NEW: per-device pull status (§4.4)

        skeleton/                 # from Jetson (HTTP :9100 pull) - PER TEAM
          a/
            skeleton.parquet      #   pose rows (already zstd)
            frames.parquet        #   per-frame companion table
            video.mp4             #   ONLY when keep_raw_video (mp4v, ~205-247 MB / 6 min)
          b/  …                   #   cam 0 -> a, cam 2 -> b (fixed on Jetson)

        audio/                    # from 6 Pis (SFTP pull, then CSV->Parquet) - PER PLAYER
          a1/                     #   player folder = team+player, from the mic map (§2)
            opensmile_lld.parquet #   CONVERTED from opensmile_lld.csv (~3x smaller, zstd)
            vad.parquet           #   CONVERTED from vad.csv
            audio.flac            #   ONLY when keep_raw_audio (~1.3 MB / 6 min / mic)
            audio.json            #   alignment sidecar; kept with audio.flac
          a2/  a3/  a4/  a5/  a6/ #   rpi5-11..13
          b1/  b2/  b3/  b4/  b5/  b6/   # rpi5-14..16
```

Locked layout decisions (2026-08-17):

- **`skeleton/<team>/`** — team-level (Jetson produces one skeleton/frames/video
  per *team*, not per player). Mirrors the Jetson's own on-device layout.
- **`audio/<player>/`** (a1…b6) — **per-player**, mapping applied at pull time
  (each mic is one player). Analysis-ready; if the mic→player mapping ever
  changes, that game's mapping is recorded in `external_media_manifest.json`
  so old folders stay interpretable.
- **CSV → Parquet on pull, then DELETE the source `.csv`** after a successful
  conversion (Parquet is the only copy). `audio.json` stays JSON.
- Raw media (`video.mp4`, `audio.flac`) is **not** post-compressed — already
  compressed at the source.

Size per 6-min game: ~22 MB with raw OFF (parquet only), ~510 MB with raw ON
(dominated by the two `video.mp4`).

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
  "folder": "recordings/games/2026-08-17/15-58-45",
  "raw_video": true,               // toggles this game was started with
  "raw_audio": true,
  // The mic->player mapping used to lay out audio/<player>/ for THIS game,
  // recorded so the folder names stay interpretable if the map ever changes.
  "mic_player_map": {
    "rpi5-11": {"MIC1": "a1", "MIC2": "a2"},
    "rpi5-12": {"MIC1": "a3", "MIC2": "a4"}
    // … rpi5-16
  },
  "devices": {
    "jetson":              {"status": "ok", "files": 6, "bytes": 452100000},
    "pi:rpi5-11:MIC1":     {"status": "ok", "files": 2, "bytes": 1490000,
                            "player": "a1"},
    "pi:rpi5-11:MIC2":     {"status": "pending", "files": null, "bytes": null,
                            "player": "a2"}
  }
}
```

Status enum: `pending | pulling | ok | partial | unreachable_at_start |
permanently_failed`. `files`/`bytes` count the *stored* artifacts (Parquet
after CSV→Parquet conversion, not the transient CSV). The roll-up of this
manifest is what lands in `external_media_index.csv` (§3.1) once every device
is terminal.

## 5. Configuration

New top-level profile block:

```yaml
# In the PROFILE (behavior for a run):
external_recording:
  enabled: true
  skeleton:
    host: "192.168.0.101"       # Jetson Nano (verified reachable 2026-08-17)
    ws_port: 9000               # command channel (tracker_api.md)
    http_port: 9100             # file pull
    keep_raw_video: false       # true -> record + pull video.mp4 per team
  audio:
    keep_raw_audio: false       # true -> log_start record_audio=1 + pull audio.flac
  pull:
    timeout_s: 60               # per-file transfer timeout (video.mp4 can be ~300 MB / 12 min)
    retry_interval_s: 60        # offline-device retry cadence
    retry_window_s: 3600        # give up after this and mark permanently_failed
  # Pi SSH credentials for the pull live in config/secrets.yaml (tracked).

# In config/device_ports_and_addr.yaml (hardware addressing, defined ONCE):
audio_capture:
  max_minutes: 15        # Pi in-RAM cap arg to log_start; must exceed game duration
  ack_timeout_s: 5.0     # per-command OSC ack timeout (async save acks fast)
  emotion: false         # record emotion.csv too? off = prosody + VAD only
  hosts:                 # Pi hostname -> ip + per-mic player (mic->player map)
    rpi5-11: {ip: "192.168.0.11", mic_players: {mic1: "a1", mic2: "a2"}}
    rpi5-12: {ip: "192.168.0.12", mic_players: {mic1: "a3", mic2: "a4"}}
    rpi5-13: {ip: "192.168.0.13", mic_players: {mic1: "a5", mic2: "a6"}}
    rpi5-14: {ip: "192.168.0.14", mic_players: {mic1: "b1", mic2: "b2"}}
    rpi5-15: {ip: "192.168.0.15", mic_players: {mic1: "b3", mic2: "b4"}}
    rpi5-16: {ip: "192.168.0.16", mic_players: {mic1: "b5", mic2: "b6"}}
```

The mic fleet (mic→player mapping, OSC timing, processing toggles) is hardware
addressing and lives ONCE in `device_ports_and_addr.yaml` under `audio_capture:`,
loaded via `core.device_connection.load_audio_capture`. The profile keeps only
the per-run behavior: Jetson address, `keep_raw_video` / `keep_raw_audio`
toggles, and pull/retry tuning. (Decided 2026-08-17.)

Two toggle presets (locked decision): `two_teams.yaml` gets
`keep_raw_video: false, keep_raw_audio: false`;
`two_teams_longer_duration.yaml` gets both `true`. The Jetson's duration cap is
now conditional (video→20 min, skeleton-only→60 min, §8.1), so the 10-min
longer-duration profile fits comfortably.

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

## 8. External-repo changes (implemented; Jetson deployed+verified, Pi pending)

Both repos are cloned on this machine. Fixes are edited + tested locally;
the user pushes to GitHub and redeploys to the devices.

**Jetson** — `C:/Users/yck01/GitHub/jetson_camera_experiments/skeleton-tracker/jetson/`
— **✅ DEPLOYED + verified live 2026-08-17 (see §9):**
1. **Conditional max-duration** — `recorder_node.py`: new
   `max_duration_skeleton_sec` (default = `max_duration_sec`); `expired()`
   picks the cap from the session's `record_video` flag (stored at `start`).
   `config.yaml`: `max_duration_sec: 1200` (video) + new
   `max_duration_skeleton_sec: 3600` (skeleton-only).
2. **Advertise-host URL fix** — `config.yaml` `jetson.advertise_host:
   "192.168.0.101"` → `stop_recording` acks now return LAN-reachable URLs.
   (`tracker_api.md`'s `/recordings/` URL prefix is stale — server root is
   already the recordings dir; real path is
   `http://<jetson>:9100/<date>/<time>/<team>/<file>`.)

**Pis** — `C:/Users/yck01/GitHub/voiceAnonymizer_PI/strip_monitor.py`
— **✅ DEPLOYED + verified live 2026-08-17 (see §9):**
3. **Async save ACK** — `_handle_ctrl_command` special-cases `log_save_stop`
   / `raw_save_stop`: immediate `"saving"` ACK, blocking save on a daemon
   thread (`_run_save_command`), per-file `/saved` notices + final completion
   ack. Verified live: immediate ack ~7 ms, files land right after, full
   SFTP pull works.
4. **Idempotent `log_start`** — verified live: duplicate start with the same
   identity acks as a no-op (session preserved).
5. **Pi SSH** — resolved: password auth works (`pi` / `pi1234`), stored in
   gitignored `config/secrets.yaml`. SFTP pull verified (<10 ms/file).

## 9. Hardware verification log (2026-08-17)

| Check | Result |
|---|---|
| Ping Jetson 192.168.0.101 | ✅ 1 ms |
| Ping all 6 Pis (.11–.16) | ✅ all reply (first batch needed ARP warm-up) |
| Jetson WS :9000 + HTTP :9100 open | ✅ |
| Jetson ping / get_status / start / stop | ✅ acks correct; status shows both cameras 13–15 fps |
| Jetson skeleton-only 6 s session + HTTP pull | ✅ `skeleton.parquet` + `frames.parquet` per team, pulled over HTTP |
| Jetson auto-stop saves partial file | ✅ confirmed in code (`expired()`→`stop()`→`_close_locked(discard=False)`) |
| **Jetson advertise_host fix (post-redeploy)** | ✅ stop acks return `http://192.168.0.101:9100/...`; all 4 files downloaded via returned URLs verbatim, byte counts match |
| **Jetson conditional duration cap (post-redeploy)** | ✅ code path live (skeleton-only session runs new `expired()` branch); cap value not exposed over protocol, config key verified loaded (`max_duration_skeleton_sec=3600`) |
| Pi OSC log_start (rpi5-11 MIC1) | ✅ ack ~5 ms |
| **Pi idempotent log_start (post-redeploy)** | ✅ duplicate start acks no-op ~3 ms, session preserved |
| **Pi async log_save_stop (post-redeploy)** | ✅ immediate "saving" ack ~7 ms; `opensmile_lld.csv` + `vad.csv` land right after |
| **Pi raw audio path (post-redeploy)** | ✅ `record_audio=1` → `audio.flac` + `audio.json` sidecar alongside CSVs |
| **Pi SFTP pull (post-redeploy)** | ✅ all 4 files pulled <10 ms each; CSV headers + audio.json valid |
| Pi SSH password auth | ✅ `pi`/`pi1234` |

Probe scripts kept as `tools/probe_jetson_recorder.py` and
`tools/probe_pi_audio.py` (reusable for Stage 3 hardware bring-up). Probe
session folders `2099-01-01/` and `2099-01-02/` remain on the Jetson
(read-only HTTP server; clear with on-device `rm -rf recordings/2099-01-0*`) —
harmless, a few KB each.

## 10. Full-fidelity stress probe (2026-08-17) — `tools/probe_full_recording.py`

End-to-end run mirroring the coordinator's game-end flow: all 12 Pi mics +
Jetson, **raw audio + raw video ON**, then pull everything back and validate.

**2-minute pass — ✅ PASS** (session `2026-08-17/15-58-45`):

| Check | Result |
|---|---|
| Jetson 6/6 files (skeleton+frames+video ×2 teams) | ✅ 150 MB total; duration exactly 120.0 s; videos ~69/83 MB pull in ~6–7 s over HTTP |
| Pi 48/48 files (12 mics × opensmile/vad/flac/json) | ✅ 20.4 MB total |
| Per-mic `audio.json` duration | ✅ all 120.7–122.0 s (target 120) — no drift once stop is concurrent |
| Prosody/VAD CSV last `time_ms` | ✅ ~117–120 s across all 12 mics |
| Concurrent `log_save_stop` | ✅ all 12 ack "saving" in 2–41 ms |

**First attempt (sequential stop) — failed by probe design, not hardware:**
`log_save_stop` was sent one mic at a time; each save holds the Pi's OSC
listener while flushing the 2-min FLAC to SD, so later mics kept recording
(160–250 s staircase) and the first 8 timed out at 15 s. Fixed by fanning
stop out concurrently (§1 "Concurrent start/stop"). Lesson baked into the
coordinator design.

**6-minute pass — ✅ PASS** (session `2026-08-17/16-02-10`):

| Check | Result |
|---|---|
| Jetson 6/6 files | ✅ **452 MB** total; duration exactly 360.0 s; videos pull fine |
| Pi 48/48 files | ✅ **59.4 MB** total |
| Per-mic `audio.json` duration | ✅ all 360.7–361.9 s (target 360) — tight, no drift |
| Prosody/VAD CSV last `time_ms` | ✅ ~356–359 s across all 12 mics |
| Concurrent `log_save_stop` | ✅ all 12 ack "saving" in 2–30 ms |

Both durations pass with raw audio + raw video. **Full workflow (record →
concurrent stop → pull → validate) is production-ready on the hardware side.**
Remaining work is the our-side `external_media_coordinator` (Stage 1–3).", "oldString": "**6-minute pass — pending.**"}

