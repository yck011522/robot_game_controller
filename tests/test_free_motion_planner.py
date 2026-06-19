"""Focused tests for free-motion planner limits and return contracts.

Run with the repository's validated environment:
    $env:PYTHONPATH = "src"
    & C:\\Users\\yck01\\miniconda3\\envs\\game\\python.exe tests\\test_free_motion_planner.py
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC = REPO_ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from compas_fab.robots import JointTrajectory  # noqa: E402

from subsystems.motion_planning import (  # noqa: E402
    BiRRTConnectPlanner,
    PlannerSettings,
    PlanStatus,
)


class _AlwaysFreeOracle:
    """Collision oracle that accepts every configuration and edge."""

    def is_config_free(self, q_rad: list[float]) -> bool:
        """Return True for every endpoint."""
        return True

    def is_edge_free(self, points_rad: list[list[float]]) -> bool:
        """Return True for every discretized edge."""
        return True


class _BlockedEdgeOracle:
    """Collision oracle with valid endpoints but no valid motion edges."""

    def is_config_free(self, q_rad: list[float]) -> bool:
        """Return True so failures are caused by blocked paths, not endpoints."""
        return True

    def is_edge_free(self, points_rad: list[list[float]]) -> bool:
        """Return False for every attempted motion edge."""
        return False


class _WallGapOracle:
    """Synthetic obstacle requiring a detour through a gap in joint 1/2 space."""

    @staticmethod
    def _is_free(q_rad: list[float]) -> bool:
        """Reject a vertical wall around joint 1 below the joint 2 gap."""
        x = float(q_rad[0])
        y = float(q_rad[1])
        return not (-0.12 <= x <= 0.12 and y < 0.45)

    def is_config_free(self, q_rad: list[float]) -> bool:
        """Return whether one synthetic configuration avoids the wall."""
        return self._is_free(q_rad)

    def is_edge_free(self, points_rad: list[list[float]]) -> bool:
        """Return whether every discretized edge point avoids the wall."""
        return all(self._is_free(q) for q in points_rad)


Q_MIN = [-math.pi] * 6  # Symmetric lower test limits.
Q_MAX = [math.pi] * 6  # Symmetric upper test limits.
START = [0.0] * 6  # Collision-free test start.
GOAL = [math.radians(10.0), 0.0, 0.0, 0.0, 0.0, 0.0]  # Ten-degree test goal.


def _planner(oracle, **overrides) -> BiRRTConnectPlanner:
    """Create a deterministic planner with optional setting overrides."""
    values = {
        "max_iterations_per_attempt": 2,
        "trajectory_step_rad": math.radians(0.05),
        "attempt_timeout_s": 10.0,
        "max_restarts": 0,
        "total_timeout_s": 30.0,
        "smooth_iterations": 0,
        "rng_seed": 123,
    }
    values.update(overrides)
    return BiRRTConnectPlanner(
        q_min_rad=Q_MIN,
        q_max_rad=Q_MAX,
        collision_oracle=oracle,
        settings=PlannerSettings(**values),
    )


def test_direct_path_returns_compas_trajectory() -> None:
    """Direct planning returns the native compas trajectory type."""
    planner = _planner(_AlwaysFreeOracle())
    trajectory = planner.plan(START, GOAL)
    result = planner.plan_detailed(START, GOAL)
    assert isinstance(trajectory, JointTrajectory)
    assert result.status == PlanStatus.DIRECT_PATH
    assert result.attempts == 0
    assert len(result.path_rad) == 201


def test_zero_iterations_is_direct_only() -> None:
    """Zero expansion iterations reject a blocked direct path without RRT."""
    planner = _planner(_BlockedEdgeOracle(), max_iterations_per_attempt=0, max_restarts=4)
    result = planner.plan_detailed(START, GOAL)
    assert planner.plan(START, GOAL) is None
    assert result.status == PlanStatus.NO_DIRECT_PATH
    assert result.iterations == 0
    assert result.attempts == 0


def test_restart_and_iteration_limits_are_counted() -> None:
    """Each failed tree pair consumes its own configured iteration limit."""
    planner = _planner(_BlockedEdgeOracle(), max_iterations_per_attempt=2, max_restarts=2)
    result = planner.plan_detailed(START, GOAL)
    assert result.status == PlanStatus.ITERATION_LIMIT
    assert result.attempts == 3
    assert result.iterations == 6


def test_collision_sample_budget_has_distinct_status() -> None:
    """A direct line larger than the sample cap fails before oracle dispatch."""
    planner = _planner(_AlwaysFreeOracle(), max_collision_samples=100)
    result = planner.plan_detailed(START, GOAL)
    assert result.status == PlanStatus.COLLISION_BUDGET
    assert result.trajectory is None


def test_birrt_connect_finds_required_detour() -> None:
    """BiRRT-Connect solves a deterministic blocked-direct wall-with-gap case."""
    planner = BiRRTConnectPlanner(
        q_min_rad=[-1.2, -1.0, 0.0, 0.0, 0.0, 0.0],
        q_max_rad=[1.2, 1.0, 0.0, 0.0, 0.0, 0.0],
        collision_oracle=_WallGapOracle(),
        settings=PlannerSettings(
            max_iterations_per_attempt=500,
            extend_step_rad=0.15,
            trajectory_step_rad=0.02,
            goal_sample_rate=0.05,
            max_connect_steps=64,
            smooth_iterations=20,
            attempt_timeout_s=5.0,
            max_restarts=0,
            total_timeout_s=5.0,
            rng_seed=123,
        ),
    )
    start = [-1.0, -0.5, 0.0, 0.0, 0.0, 0.0]
    goal = [1.0, -0.5, 0.0, 0.0, 0.0, 0.0]
    result = planner.plan_detailed(start, goal)
    assert result.status == PlanStatus.PLANNED
    assert result.trajectory is not None
    assert result.connect_steps > 0
    assert any(q[1] >= 0.45 for q in result.path_rad)
    assert _WallGapOracle().is_edge_free(result.path_rad)


def main() -> int:
    """Run tests without requiring pytest."""
    test_direct_path_returns_compas_trajectory()
    test_zero_iterations_is_direct_only()
    test_restart_and_iteration_limits_are_counted()
    test_collision_sample_budget_has_distinct_status()
    test_birrt_connect_finds_required_detour()
    print("[test] free motion planner limits: OK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
