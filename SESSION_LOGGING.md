# Session Logging Plan

Deferred until the game is playable. This document captures the design
decisions so implementation can proceed quickly later.

## Goals

1. Record every game session at 50 Hz for post-game analysis.
2. Correlate telemetry with room audio (12 microphones) and skeleton pose
   estimation on a shared timeline.
3. Keep file sizes small — static config is saved once, not every tick.

## Session Folder Structure

```
sessions/
  YYYY-MM-DD_HH-MM-SS/
    config.json            # Static settings snapshot (GameSettings minus volatile fields)
    metadata.json          # session_id, start_time_utc, duration_s, team names, scores
    telemetry.parquet      # 50 Hz time-series (see schema below)
    audio/
      mic_01.flac          # Per-microphone mono recordings (12 files)
      ...
      mic_12.flac
      audio_meta.json      # start_time_utc, sample_rate, mic-to-player mapping
    skeleton/
      poses.parquet        # 30 Hz skeleton keypoints per player (separate pipeline)
```

## Telemetry Schema (Parquet, 50 Hz)

All angular values in degrees. Motor IDs: 11–16 (Team 1), 21–26 (Team 2).

| Column pattern          | Count | Description                                      |
|--------------------------|-------|--------------------------------------------------|
| `t`                      | 1     | Wall-clock epoch (time.time()) for correlation   |
| `stage`                  | 1     | Current game stage (Sync, Idle, GameOn, etc.)    |
| `countdown_s`            | 1     | Seconds remaining in current stage               |
| `dial_deg_XX`            | 12    | Raw dial input in joint-space degrees             |
| `cmd_deg_XX`             | 12    | After gearing, before clamping                    |
| `clamp_deg_XX`           | 12    | After static range limits                         |
| `throttle_deg_XX`        | 12    | After rate limiter — what is sent to robot        |
| `robot_deg_XX`           | 12    | Actual position reported by robot                 |
| `error_deg_XX`           | 12    | throttle − robot_actual (proxy for force feedback)|
| `safe_min_deg_XX`        | 12    | Dynamic lower limit from collision detector       |
| `safe_max_deg_XX`        | 12    | Dynamic upper limit from collision detector       |
| `weight_XX`              | 6     | Bucket weights in grams (IDs 11–13, 21–23)       |
| `score_t1`, `score_t2`  | 2     | Running team scores                               |
| **Total**                | **~107** |                                                |

Estimated size: ~12,000 rows per 4-minute game × 107 float64 columns
≈ 10 MB raw → **200–400 KB** as Parquet with Snappy compression.

## Audio

- 12 mono channels, 48 kHz / 16-bit, FLAC lossless compression.
- ~4 MB/s total → **~960 MB per 4-minute game**.
- Separate files per mic so individual channels can be fed to speech-to-text
  (e.g. Whisper) for per-player transcription.

## Skeleton Pose Estimation

- Separate camera pipeline (MediaPipe / OpenPose), ~30 fps.
- Output: 33 keypoints × (x, y, z, confidence) × N players per frame.
- Stored as its own Parquet file with timestamps for alignment.

## Timeline Correlation

All streams share a common wall-clock epoch (`time.time()` / UTC).
Correlation at analysis time via `pandas.merge_asof()` or similar
timestamp-based join. No need for a single unified file.

## Implementation Notes

- **Logger class**: Self-threaded, receives snapshots from GameSettings at
  50 Hz via a `log_snapshot()` method that cherry-picks the volatile fields.
  Accumulates rows in a list, writes Parquet on game end (or every N seconds
  as a safety flush).
- **config.json**: Dumped once at session start from `GameSettings.snapshot()`,
  filtering to static-only fields.
- **metadata.json**: Written at session end with final scores, duration, etc.
- **No schema change needed**: GameSettings registry model stays as-is.
  The logger is a consumer, not a structural change.
- **Dependency**: `pyarrow` for Parquet I/O (add to requirements.txt when
  implementing).
