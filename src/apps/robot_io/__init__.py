"""src/apps/robot_io — per-team UR10e process (sim or real).

Reads `subsystems.robot_io.<team>` from the profile.

    sim_pybullet → pybullet simulator (GUI or headless via
                   tuning.robot.headless, optionally overridden per team via
                   tuning.robot.headless_by_team: {a: bool, b: bool} — useful
                   for a two-team sim run, since pybullet only comfortably
                   supports one visible GUI window at a time; the other
                   team's process stays headless (DIRECT) in that case.)
    real_rtde    → real UR10e over RTDE TCP
"""
