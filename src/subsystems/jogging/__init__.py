"""src/subsystems/jogging — joint planner implementations.

`in_process.InProcessPlanner` is the only impl shipped for P2: it runs
inside GameController's tick and skips the standalone-process overhead.
A `standalone` impl with its own process can be added later if 50 Hz
GC ticks ever stall behind collision-check round-trips.
"""
