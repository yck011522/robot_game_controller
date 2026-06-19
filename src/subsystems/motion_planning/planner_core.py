"""Algorithm-neutral joint-space planning types and utilities.

Status
------
Experimental and not production-ready. This module is partially tested in
validation workflows and should not be treated as deployment-grade motion
planning behavior.

This module owns shared trajectory construction, collision accounting,
smoothing, tree storage, limits, and result contracts. The only concrete
search algorithm in this package is implemented in `birrt_connect.py`.
"""

from __future__ import annotations

import math
import random
import time
from dataclasses import dataclass
from enum import Enum
from typing import Protocol

from compas_fab.robots import JointTrajectory, JointTrajectoryPoint

from subsystems.robot.shared_compas_scene import UR10E_JOINT_NAMES

REVOLUTE_JOINT_TYPE = 0


class CollisionOracle(Protocol):
    """Planner-facing collision interface independent of process transport."""

    def is_config_free(self, q_rad: list[float]) -> bool:
        """Return True when one six-axis configuration is collision-free."""

    def is_edge_free(self, points_rad: list[list[float]]) -> bool:
        """Return True when every supplied edge sample is collision-free."""


class PlanStatus(str, Enum):
    """Machine-readable planning outcomes used by runtime and validation."""

    DIRECT_PATH = "direct_path"
    PLANNED = "planned"
    NO_DIRECT_PATH = "no_direct_path"
    START_IN_COLLISION = "start_in_collision"
    GOAL_IN_COLLISION = "goal_in_collision"
    ITERATION_LIMIT = "iteration_limit"
    ATTEMPT_TIMEOUT = "attempt_timeout"
    TOTAL_TIMEOUT = "total_timeout"
    COLLISION_BUDGET = "collision_budget"
    FINAL_VALIDATION_FAILED = "final_validation_failed"


class _CollisionBudgetExceeded(RuntimeError):
    """Internal control-flow exception raised before exceeding a sample cap."""


@dataclass(frozen=True)
class PlannerSettings:
    """Tunable settings shared by direct checks and BiRRT-Connect.

    `trajectory_step_rad` is deliberately shared by collision sampling and
    returned trajectory spacing, so every returned trajectory point has been
    collision checked.
    """

    max_iterations_per_attempt: int = 1000
    extend_step_rad: float = math.radians(4.0)
    trajectory_step_rad: float = math.radians(0.05)
    goal_sample_rate: float = 0.12
    max_connect_steps: int = 64
    smooth_iterations: int = 120
    corner_window: int = 20
    attempt_timeout_s: float = 2.0
    max_restarts: int = 4
    total_timeout_s: float = 10.0
    max_collision_samples: int = 0
    rng_seed: int | None = None


@dataclass
class PlanResult:
    """Detailed planning result used by validation and diagnostics."""

    success: bool
    status: PlanStatus
    trajectory: JointTrajectory | None
    path_rad: list[list[float]]
    sparse_path_rad: list[list[float]]
    corners: list[int]
    iterations: int
    attempts: int
    collision_samples: int
    elapsed_s: float
    message: str
    nodes_added: int = 0
    connect_steps: int = 0


@dataclass
class _Node:
    """One configuration in a search tree."""

    q: list[float]  # Six-axis configuration in radians.
    parent: int | None  # Parent node index, or None for the root.
    cost: float  # Accumulated Euclidean joint-space distance from root.


class _Tree:
    """Minimal parent-linked configuration tree used by BiRRT-Connect."""

    def __init__(self, root_q: list[float]) -> None:
        """Create a tree with one root configuration."""
        self.nodes = [_Node(q=list(root_q), parent=None, cost=0.0)]

    def nearest_index(self, q: list[float]) -> int:
        """Return the node nearest to `q` under Euclidean joint distance."""
        return min(range(len(self.nodes)), key=lambda i: _distance(self.nodes[i].q, q))

    def add_node(self, q: list[float], parent: int, cost: float) -> int:
        """Append one node and return its index."""
        index = len(self.nodes)
        self.nodes.append(_Node(q=list(q), parent=parent, cost=cost))
        return index

    def path_to_root(self, index: int) -> list[list[float]]:
        """Return configurations from root through the selected node."""
        path: list[list[float]] = []
        current: int | None = index
        while current is not None:
            node = self.nodes[current]
            path.append(list(node.q))
            current = node.parent
        path.reverse()
        return path


class MotionPlannerBase:
    """Shared mechanics for a concrete joint-space search algorithm."""

    def __init__(
        self,
        *,
        q_min_rad: list[float],
        q_max_rad: list[float],
        collision_oracle: CollisionOracle,
        settings: PlannerSettings | None = None,
    ) -> None:
        """Create a bounded six-axis planner using a supplied collision oracle."""
        self.q_min_rad = [float(v) for v in q_min_rad[:6]]  # Per-axis lower bounds.
        self.q_max_rad = [float(v) for v in q_max_rad[:6]]  # Per-axis upper bounds.
        self.oracle = collision_oracle  # Transport-independent collision interface.
        self.settings = settings or PlannerSettings()  # Search and validation tuning.
        self.rng = random.Random(self.settings.rng_seed)  # Deterministic optional sampler.
        self._collision_samples = 0  # Requested collision configurations this plan.
        if len(self.q_min_rad) != 6 or len(self.q_max_rad) != 6:
            raise ValueError("motion planner expects six joint limits")

    def plan(self, start_rad: list[float], goal_rad: list[float]) -> JointTrajectory | None:
        """Return a native joint trajectory, or None when planning fails."""
        result = self.plan_detailed(start_rad, goal_rad)
        return result.trajectory if result.success else None

    def plan_detailed(self, start_rad: list[float], goal_rad: list[float]) -> PlanResult:
        """Return a detailed result from the concrete search algorithm."""
        raise NotImplementedError

    def smooth_path(
        self,
        path_rad: list[list[float]],
        corners: list[int],
        *,
        deadline: float | None = None,
    ) -> list[list[float]]:
        """Shortcut a sparse path while preserving sampled collision freedom."""
        if len(path_rad) <= 2:
            return [list(q) for q in path_rad]
        path = [list(q) for q in path_rad]  # Mutable sparse path.
        corner_queue = list(corners)  # Kinks attempted before random shortcuts.
        for _ in range(max(0, int(self.settings.smooth_iterations))):
            if deadline is not None and time.perf_counter() >= deadline:
                break
            if len(path) <= 2:
                break
            if corner_queue:
                corner = max(1, min(len(path) - 2, corner_queue.pop(0)))
                half = max(2, self.settings.corner_window // 2)
                first = max(0, corner - half)
                last = min(len(path) - 1, corner + half)
            else:
                first = self.rng.randrange(0, len(path) - 2)
                last = self.rng.randrange(first + 2, len(path))
            shortcut = discretize_joint_line(
                path[first], path[last], self.settings.trajectory_step_rad
            )
            if not self._is_edge_free(shortcut):
                continue
            if _distance(path[first], path[last]) >= _path_cost(path[first:last + 1]):
                continue
            path = path[:first + 1] + path[last:]
            corner_queue.extend(find_corners(path))
        return path

    def _joined_path(
        self,
        start_tree: _Tree,
        goal_tree: _Tree,
        start_index: int,
        goal_index: int,
    ) -> list[list[float]]:
        """Join start-root and goal-root paths at their common bridge config."""
        start_side = start_tree.path_to_root(start_index)
        goal_side = goal_tree.path_to_root(goal_index)
        goal_side.reverse()
        if start_side and goal_side and _max_axis_delta(start_side[-1], goal_side[0]) <= 1e-12:
            return start_side + goal_side[1:]
        return start_side + goal_side

    def _sample(self) -> list[float]:
        """Sample uniformly inside the configured six-axis limits."""
        return [self.rng.uniform(lo, hi) for lo, hi in zip(self.q_min_rad, self.q_max_rad)]

    def _clamp(self, q_rad: list[float]) -> list[float]:
        """Clamp a six-axis vector to configured limits."""
        values = [float(v) for v in q_rad[:6]]
        while len(values) < 6:
            values.append(0.0)
        return [max(lo, min(hi, v)) for v, lo, hi in zip(values, self.q_min_rad, self.q_max_rad)]

    def _is_config_free(self, q_rad: list[float]) -> bool:
        """Check one configuration while enforcing the collision budget."""
        self._reserve_collision_samples(1)
        return self.oracle.is_config_free(q_rad)

    def _is_edge_free(self, points_rad: list[list[float]]) -> bool:
        """Check edge samples while enforcing the collision budget."""
        self._reserve_collision_samples(len(points_rad))
        return self.oracle.is_edge_free(points_rad)

    def _reserve_collision_samples(self, count: int) -> None:
        """Reserve requested samples or raise before exceeding the cap."""
        limit = max(0, int(self.settings.max_collision_samples))
        if limit and self._collision_samples + count > limit:
            raise _CollisionBudgetExceeded
        self._collision_samples += count

    def _success(
        self,
        t0: float,
        status: PlanStatus,
        trajectory: JointTrajectory,
        path_rad: list[list[float]],
        sparse_path_rad: list[list[float]],
        corners: list[int],
        iterations: int,
        attempts: int,
        message: str,
    ) -> PlanResult:
        """Build a successful detailed result with timing and counters."""
        return PlanResult(
            True, status, trajectory, path_rad, sparse_path_rad, corners,
            iterations, attempts, self._collision_samples,
            time.perf_counter() - t0, message,
        )

    def _failure(
        self,
        t0: float,
        status: PlanStatus,
        iterations: int,
        attempts: int,
        message: str,
        *,
        sparse_path_rad: list[list[float]] | None = None,
    ) -> PlanResult:
        """Build a failed detailed result with timing and counters."""
        return PlanResult(
            False, status, None, [], sparse_path_rad or [], [],
            iterations, attempts, self._collision_samples,
            time.perf_counter() - t0, message,
        )


def densify_path(path_rad: list[list[float]], max_step_rad: float) -> list[list[float]]:
    """Return a path whose adjacent points obey the per-axis step limit."""
    if not path_rad:
        return []
    dense = [list(path_rad[0])]
    for first, last in zip(path_rad, path_rad[1:]):
        dense.extend(discretize_joint_line(first, last, max_step_rad)[1:])
    return dense


def trajectory_from_path(path_rad: list[list[float]]) -> JointTrajectory:
    """Convert a radian path to a compas_fab joint trajectory."""
    joint_names = list(UR10E_JOINT_NAMES)
    joint_types = [REVOLUTE_JOINT_TYPE] * len(joint_names)
    points = [
        JointTrajectoryPoint(
            joint_values=list(q),
            joint_types=joint_types,
            joint_names=joint_names,
        )
        for q in path_rad
    ]
    return JointTrajectory(trajectory_points=points, joint_names=joint_names)


def path_from_trajectory(trajectory: JointTrajectory | None) -> list[list[float]]:
    """Extract radian joint vectors from a native trajectory."""
    if trajectory is None:
        return []
    return [
        [float(v) for v in point.joint_values[:6]]
        for point in (getattr(trajectory, "points", None) or [])
    ]


def find_corners(
    path_rad: list[list[float]],
    *,
    min_turn_rad: float = math.radians(0.5),
) -> list[int]:
    """Return sparse-path indices where normalized direction changes."""
    corners: list[int] = []
    for index in range(1, len(path_rad) - 1):
        previous = [path_rad[index][j] - path_rad[index - 1][j] for j in range(6)]
        following = [path_rad[index + 1][j] - path_rad[index][j] for j in range(6)]
        turn = math.sqrt(
            sum((a - b) * (a - b) for a, b in zip(_unit(previous), _unit(following)))
        )
        if turn > min_turn_rad:
            corners.append(index)
    return corners


def path_max_axis_step(path_rad: list[list[float]]) -> float:
    """Return the largest adjacent per-axis change in a path."""
    if len(path_rad) < 2:
        return 0.0
    return max(_max_axis_delta(a, b) for a, b in zip(path_rad, path_rad[1:]))


def discretize_joint_line(
    first: list[float],
    last: list[float],
    max_step_rad: float,
) -> list[list[float]]:
    """Discretize a straight joint-space edge including both endpoints."""
    span = _max_axis_delta(first, last)
    steps = max(1, int(math.ceil(span / max(max_step_rad, 1e-12))))
    return [
        [first[j] + (last[j] - first[j]) * (i / steps) for j in range(6)]
        for i in range(steps + 1)
    ]


def _steer(first: list[float], last: list[float], max_step_rad: float) -> list[float]:
    """Move toward `last` while limiting the largest per-axis change."""
    span = _max_axis_delta(first, last)
    if span <= max_step_rad:
        return list(last)
    ratio = max_step_rad / max(span, 1e-12)
    return [first[j] + (last[j] - first[j]) * ratio for j in range(6)]


def _distance(first: list[float], last: list[float]) -> float:
    """Return Euclidean six-axis joint-space distance."""
    return math.sqrt(sum((a - b) * (a - b) for a, b in zip(first, last)))


def _max_axis_delta(first: list[float], last: list[float]) -> float:
    """Return Chebyshev distance between two joint vectors."""
    return max(abs(a - b) for a, b in zip(first, last))


def _path_cost(path_rad: list[list[float]]) -> float:
    """Return total Euclidean length of a sparse path."""
    return sum(_distance(a, b) for a, b in zip(path_rad, path_rad[1:]))


def _unit(vector: list[float]) -> list[float]:
    """Return a unit vector, or zeros for a near-zero vector."""
    norm = math.sqrt(sum(value * value for value in vector))
    if norm <= 1e-12:
        return [0.0 for _ in vector]
    return [value / norm for value in vector]


__all__ = [
    "CollisionOracle",
    "MotionPlannerBase",
    "PlannerSettings",
    "PlanResult",
    "PlanStatus",
    "densify_path",
    "discretize_joint_line",
    "find_corners",
    "path_from_trajectory",
    "path_max_axis_step",
    "trajectory_from_path",
]
