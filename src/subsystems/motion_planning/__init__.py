"""Standalone motion-planning utilities (experimental).

Status
------
This package is partially tested and partially functional. It is useful for
offline validation and iterative development, but it is not production-ready.
The package is intentionally not wired into the game controller yet.
"""

from .planner_core import (
    CollisionOracle,
    MotionPlannerBase,
    PlannerSettings,
    PlanResult,
    PlanStatus,
    discretize_joint_line,
    path_max_axis_step,
    path_from_trajectory,
    trajectory_from_path,
)
from .birrt_connect import BiRRTConnectPlanner

__all__ = [
    "PlannerSettings",
    "CollisionOracle",
    "MotionPlannerBase",
    "PlanResult",
    "PlanStatus",
    "BiRRTConnectPlanner",
    "discretize_joint_line",
    "path_max_axis_step",
    "path_from_trajectory",
    "trajectory_from_path",
]
