"""src/apps/haptic_io — per-team haptic input process.

Reads `subsystems.haptic_io.<team>` from the profile and dispatches to
the matching impl in `src/subsystems/haptic/`:

    sim_keyboard  → pygame keyboard window
    sim_scripted  → headless sine-wave producer (used by automated tests)
    real          → ESP32 boards over serial (not implemented yet)

Spawned as `python -m apps.haptic_io --profile <yaml> --proc haptic_io.a`.
"""
