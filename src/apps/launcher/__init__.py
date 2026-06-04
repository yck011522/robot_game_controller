"""Launcher / supervisor (P1 slice).

Spawns the bus broker, subscribes to `heartbeat.*`, prints liveness and
loop-hz for every child, and shuts everything down on Ctrl-C. Respawn
logic and the rest of [SUPERVISOR.md](../../../docs/architecture/SUPERVISOR.md)
land in later phases.
"""
