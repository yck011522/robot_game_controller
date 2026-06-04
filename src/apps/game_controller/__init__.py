"""src/apps/game_controller — 50 Hz authoritative game loop (P2 slice).

For P2 the loop is pinned to the Play stage and runs only the team-A
dataflow: read latest `telem.haptic.a`, run the in-process planner
(with optional collision check), publish `cmd.robot.target.a`, and
emit a minimal `state.full` snapshot. Full state machine
(Idle/Tutorial/Play/Conclusion/Reset transitions, scoring, weights,
buttons, safety, etc.) lands in P4+.
"""
