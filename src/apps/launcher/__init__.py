"""Launcher / supervisor.

How the launcher works
----------------------
The launcher is one process that:

1. Reads the active profile YAML (CONFIG.md) — picked by `--profile` or
   by `config/launcher.yaml:default_profile`.
2. Spawns each enabled child as a separate OS process using the
   [SUPERVISOR.md §3](../../../docs/architecture/SUPERVISOR.md#3-spawn-contract)
   spawn contract — every child takes the same `--profile <path>
   --proc <name> [--instance <n>]` CLI, no env vars. This means any
   single process can be hand-launched by copying its argv out of the
   launcher log.
3. Subscribes to `heartbeat.*` on the bus (the broker is the first
   child, so subscriptions start working immediately after that
   spawns).
4. Waits for each child's first heartbeat to confirm it actually came
   up. Children that don't produce a heartbeat within 10 s abort the
   whole run with a non-zero exit code (no silent fall-through to a
   degraded system).
5. Prints a periodic status table (heartbeat age, reported `loop_hz`,
   bus-observed heartbeat rate) and watches for crashed children.
6. On Ctrl-C / SIGTERM, sends `CTRL_BREAK_EVENT` (Windows) or
   `SIGTERM` (POSIX) to every child in reverse startup order, then
   waits up to a grace period before SIGKILL.

What this slice does *not* do yet
---------------------------------
- Respawn (SUPERVISOR.md §5) — a crashed child currently brings the
  whole run down. Respawn lands in P12.
- The 5-in-60s circuit breaker (SUPERVISOR.md §6) — same deferral.
- Tier-by-tier startup beyond "broker first, then everything else" —
  P2 adds collision-broker and the heavier tiers, at which point the
  startup loop here grows to match SUPERVISOR.md §2.
"""
