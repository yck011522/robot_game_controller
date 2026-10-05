# Record Player

A local, read-only exploration tool for the robot game's recordings. It never starts the controller, connects to the robots, or changes source recordings. All code and disposable environments are contained in `record_player`.

## Start on Windows

Double-click **`start.cmd`**, or run from the repository root:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File record_player/setup.ps1 -Start
```

Open **http://127.0.0.1:4317**. Keep the launch terminal running; Ctrl+C stops the server.

The setup script downloads a pinned portable Windows x64 Node.js into `.runtime`, checks its SHA256 against the official Node release checksum list, creates `.venv`, and installs pinned Python and npm dependencies. It does not change global PATH or install global npm packages. The first setup needs internet access; subsequent starts reuse the local installation without reinstalling packages unless the lockfiles change.

An existing **Python 3.10–3.13** installation is required. The script checks the user's Miniforge/Miniconda and `python`/`python3` commands. To choose an interpreter or port:

```powershell
powershell -ExecutionPolicy Bypass -File record_player/setup.ps1 -Python C:/Python312/python.exe -Start -Port 4318
```

On this computer setup was verified using the existing Miniforge Python 3.10.14. Runtime code uses only NumPy and PyArrow; the robot controller's larger environment is unnecessary.

For another computer, copy/clone the repository, copy the desired days under `recordings/games`, retain `recordings/games_index.csv`, and run the same script. Do **not** copy the `.venv` across computers. `package.json`, `package-lock.json`, `.node-version`, and `requirements.txt` declare the dependencies. The shared scene file at `src/subsystems/robot/assets/robot_cell_and_state.json` must remain present.

With an existing Node 22 installation, the standard project commands also work:

```powershell
cd record_player
npm ci
npm start
```

Create/install the Python `.venv` first using `setup.ps1`. On non-Windows systems, create `.venv` with `python3 -m venv .venv`, install `requirements.txt`, then use the same npm commands. The automatic Node bootstrap is Windows x64 only. The Node server defaults to `.venv/bin/python` on other platforms; `RECORD_PLAYER_PYTHON` can override its interpreter.

## Explore

- **Library:** all ledger games, locally playable games by default. Search dates, times, profiles, or flags; sort by table heading or the metric dropdown. Offline recordings remain visible with playback disabled when “On this computer” is unchecked.
- **Skeletons:** original camera orientation, P1 on the left. COCO-17 keypoints filtered by adjustable confidence. The default crop removes the upper 45% of the camera image; turn it off to inspect the entire frame.
- **Sound:** raw openSMILE loudness values per player, with a shared visual gain. Halos follow confident paired hands near provisional controller guides. `P3?` means a geometric guess, not a known player identity. Tracker IDs are not assumed to be player numbers.
- **Dials:** unwrapped signed dial lead in joint degrees, dial speed converted into joint degrees/s, and actual joint speed. Click any player card to inspect that joint's traces. Raw dial speed, torque, and actual angle appear beneath the plots.
- **Robot:** the URDF-derived kinematic tree and embedded visual meshes from the controller's curated COMPAS scene, including attached bucket tool, scoring buckets, pedestal, ground, and walls. Native body scale is respected. Ceiling/player barriers are hidden by default; every environment body can be toggled individually. Drag to orbit, wheel to zoom, right-drag to pan.
- **Timeline:** play/pause, 0.25–4× speeds, ±50 ms preview steps, scrubbing, and Jump to Play. Space toggles playback when a form control/button is not focused; arrows seek one second, Shift+arrows ten seconds. Clicking a plot seeks within its displayed range. Trace range can be full recording, Play-only (default), or 30 seconds around the playhead.
- **Weights:** three colored traces in grams, preserving negative sensor readings. Final CSV scores are labeled final scores, not reconstructed live scores.
- **Guides:** enable controller guides and drag the six positions. “Suggest guides from paired hands” uses confident lower-image wrist-pair observations and six X-position clusters. It declines when six adequately separated/supported positions are not found. Guides are saved per camera in browser local storage, shared across games; they are never written into recordings.

Use the **Evaluation guide** button, or [EVALUATION.md](EVALUATION.md), for a suggested first session.

## Timing, definitions, and limitations

| Display | Definition / interpretation |
|---|---|
| Recording zero | CSV `tutorial_entered_at`, converted using its explicit timezone |
| Play duration | `play_ended_at - play_entered_at`; separate from total recording duration |
| Dial lead | Recorded `dial_robot_deg - degrees(robot_actual.q_rad)`; no modulo/wrapping |
| Dial speed | `degrees(dial_vel_rad_s) × recorded signed gearing` |
| Gearing | Median of `dial_robot_deg / degrees(dial_pos_rad)` for nonzero angles, independently per joint; unavailable if insufficient data |
| Lead P95 A/B | 95th percentile of absolute lead, pooling valid observations across six joints on a uniform 20 Hz Play grid |
| Limited A/B | Percentage of valid 20 Hz Play-grid controller samples with `clamp_final < 0.99`; this is a team motion scale, not a per-joint collision classification |
| Coverage A/B | Percentage of Play-grid joint observations with both fresh dial and robot angles |
| Joint travel | Sum of the six recorded per-joint distance columns in the permanent CSV ledger, in radians; not recomputed by this viewer |
| Short play | Less than 60 seconds of Play, a descriptive flag rather than an exclusion |
| Low travel | Total A+B ledger travel under 2 rad, an experimental clue |
| Partial media | Fewer than two local camera-frame files or twelve loudness files; presence alone does not establish stream coverage |

Telemetry is reduced to at most 20 Hz by keeping the last sample in each 50 ms bin. Controller/global state retain up to 1 kHz and the cameras retain their original frames. All source samples remain in Parquet; very brief peaks can be missed in the preview and exploratory statistics. The viewer uses the latest sample at/before the playhead, never a future sample. Staleness cutoffs are 250 ms for telemetry/audio, 350 ms for camera frames, and one second for weights. Gaps break plots, clear skeletons, hide robot poses, and show missing numbers rather than holding forever. Explicit zero-person camera frames clear prior poses.

**Audio has relative `time_ms` only.** Neither its Parquet metadata nor the copied manifest provides the exact wall-clock capture start. The initial alignment assumes audio zero equals Tutorial entry. Positive “Audio offset” delays audio; adjust it during exploration. Skeleton/controller wall clocks are used as recorded; cross-device clock accuracy is not independently verified. Raw media playback is not implemented.

Loudness is not calibrated dB SPL. The gain controls display size only and uses the same mapping for every microphone. Hand/microphone association uses distance to the guides and can be wrong when people move, overlap, or are missed. Cluster suggestions are provisional; repeated detections are not independent calibration evidence. Larger dial lead or speed mismatch is not automatically a measure of skill, intention, or engagement. Tutorial alignment, force feedback, collisions, and telemetry corruption can all affect these readings.

The robot cells use the repository's current curated geometry, not a per-game scene snapshot or a reconstruction of actual balls/people. Both teams use the same local cell coordinate system. Collision flags/limits are replayed from the recording; this app does not recompute physics.

## Analyze newly copied recordings

Metrics are computed when a game opens. To populate sortable metrics for all local games in one pass:

```powershell
record_player/.venv/Scripts/python.exe record_player/data.py analyze
```

Click **Refresh index** afterward. On the supplied day, all 51 local games have already been analyzed. This took several minutes. Derived metrics are stored in `.cache`, keyed by source file size/mtime and converter version. They can be regenerated. The Node server holds up to four converted payloads in memory; restart it after changing source files or adapter code.

## Validation

```powershell
# From repository root:
record_player/.runtime/node-v22.22.0-win-x64/node.exe --test record_player/tests/math.test.mjs
record_player/.venv/Scripts/python.exe -m unittest discover -s record_player/tests -p test_*.py
```

Tests cover causal sample selection, stale/missing data, signed gearing, unwrapped lead, timestamp precision/sorting, empty camera frames, camera-independent scene units, rigid transforms, and recording-ID validation. Browser checks cover real and very short recordings, play/pause, scrubbing, player selection, trace modes, filtering/sorting, controller-guide suggestions, and environment visibility.

## Code map

| File | Responsibility |
|---|---|
| `setup.ps1`, `start.cmd` | Portable Node setup, repository-local Python venv, repeatable start |
| `server.mjs` | Loopback HTTP server, static files, Python worker invocation, bounded in-memory cache |
| `data.py` | CSV/Parquet conversion, source quality notes, exploratory metrics, curated scene export |
| `public/app.mjs` | Browser library, playback, canvas plots/skeletons, provisional controller guides |
| `public/math.mjs` | Shared units and timestamp-selection rules |
| `public/robot.mjs` | URDF-derived forward kinematics and Three.js scene rendering |

Dependencies come from [Node.js](https://nodejs.org/en/download/archive/v22.22.0), [Three.js](https://threejs.org/manual/pages/installation.html), and the Python package registry. The application has no runtime CDN, analytics, or remote data upload.
