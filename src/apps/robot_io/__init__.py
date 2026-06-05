"""src/apps/robot_io — per-team UR10e process (sim or real).

Reads `subsystems.robot_io.<team>` from the profile.

    sim_pybullet → pybullet simulator (GUI or headless via
                   tuning.robot.headless)
    real_rtde    → real UR10e over RTDE TCP
"""
