"""Bidirectional RRT-Connect feasibility planner for robot joint space.

Status
------
Experimental and not production-ready. This implementation is partially
tested and currently used only for validation and iteration.

The planner grows one tree by a single random extension, then greedily grows
the opposite tree toward that new node until it reaches the node, encounters
a collision, reaches a time limit, or consumes `max_connect_steps`.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from enum import Enum

from .planner_core import (
    MotionPlannerBase,
    PlanResult,
    PlanStatus,
    _CollisionBudgetExceeded,
    _Tree,
    _distance,
    _max_axis_delta,
    _steer,
    densify_path,
    discretize_joint_line,
    find_corners,
    trajectory_from_path,
)


class _ConnectState(str, Enum):
    """Internal outcome of one greedy CONNECT operation."""

    REACHED = "reached"
    TRAPPED = "trapped"
    ADVANCED = "advanced"
    ATTEMPT_TIMEOUT = "attempt_timeout"
    TOTAL_TIMEOUT = "total_timeout"


@dataclass(frozen=True)
class _ConnectResult:
    """Greedy connection outcome and its final tree node."""

    state: _ConnectState  # Why CONNECT stopped.
    node_index: int | None  # Last added/reached node, if any.
    steps: int  # Successfully added greedy tree nodes.


class BiRRTConnectPlanner(MotionPlannerBase):
    """Find a feasible path with bidirectional greedy tree connection."""

    def plan_detailed(self, start_rad: list[float], goal_rad: list[float]) -> PlanResult:
        """Plan a path while preserving the common detailed result contract."""
        t0 = time.perf_counter()
        self._collision_samples = 0  # Requested collision configurations for this plan.
        start = self._clamp(start_rad)  # Start after configured joint-limit clamp.
        goal = self._clamp(goal_rad)  # Goal after configured joint-limit clamp.
        total_deadline = t0 + max(0.0, self.settings.total_timeout_s)

        try:
            if not self._is_config_free(start):
                return self._failure(
                    t0, PlanStatus.START_IN_COLLISION, 0, 0,
                    "start configuration is in collision",
                )
            if not self._is_config_free(goal):
                return self._failure(
                    t0, PlanStatus.GOAL_IN_COLLISION, 0, 0,
                    "goal configuration is in collision",
                )
            direct = discretize_joint_line(start, goal, self.settings.trajectory_step_rad)
            if self._is_edge_free(direct):
                trajectory = trajectory_from_path(direct)
                return self._success(
                    t0, PlanStatus.DIRECT_PATH, trajectory, direct,
                    [start, goal], [], 0, 0, "direct path",
                )
        except _CollisionBudgetExceeded:
            return self._failure(
                t0, PlanStatus.COLLISION_BUDGET, 0, 0,
                "collision sample budget exhausted",
            )

        if self.settings.max_iterations_per_attempt <= 0:
            return self._failure(
                t0, PlanStatus.NO_DIRECT_PATH, 0, 0,
                "direct path is blocked and random expansion is disabled",
            )

        total_iterations = 0
        attempts_run = 0
        total_nodes_added = 0
        total_connect_steps = 0
        last_status = PlanStatus.ITERATION_LIMIT
        last_message = "iteration limit reached without connecting the trees"
        max_attempts = max(1, self.settings.max_restarts + 1)

        for attempt_index in range(max_attempts):
            now = time.perf_counter()
            if now >= total_deadline:
                return self._failure_with_metrics(
                    t0, PlanStatus.TOTAL_TIMEOUT, total_iterations, attempts_run,
                    total_nodes_added, total_connect_steps,
                    "total planning timeout reached",
                )
            attempts_run += 1
            attempt_deadline = min(
                total_deadline,
                now + max(0.0, self.settings.attempt_timeout_s),
            )
            start_tree = _Tree(start)
            goal_tree = _Tree(goal)
            bridge: tuple[int, int] | None = None  # Connected start-tree and goal-tree indices.

            try:
                for iteration in range(1, self.settings.max_iterations_per_attempt + 1):
                    now = time.perf_counter()
                    if now >= total_deadline:
                        return self._failure_with_metrics(
                            t0, PlanStatus.TOTAL_TIMEOUT, total_iterations, attempts_run,
                            total_nodes_added, total_connect_steps,
                            "total planning timeout reached",
                        )
                    if now >= attempt_deadline:
                        last_status = PlanStatus.ATTEMPT_TIMEOUT
                        last_message = f"attempt {attempt_index + 1} timed out"
                        break

                    total_iterations += 1
                    active, passive = (
                        (start_tree, goal_tree) if iteration % 2 else (goal_tree, start_tree)
                    )
                    active_is_start = active is start_tree
                    opposite_root = goal if active_is_start else start
                    sample = (
                        opposite_root
                        if self.rng.random() < self.settings.goal_sample_rate
                        else self._sample()
                    )
                    active_index = self._extend_once(active, sample)
                    if active_index is None:
                        continue
                    total_nodes_added += 1

                    connection = self._connect_toward(
                        passive,
                        active.nodes[active_index].q,
                        attempt_deadline=attempt_deadline,
                        total_deadline=total_deadline,
                    )
                    total_connect_steps += connection.steps
                    total_nodes_added += connection.steps
                    if connection.state == _ConnectState.TOTAL_TIMEOUT:
                        return self._failure_with_metrics(
                            t0, PlanStatus.TOTAL_TIMEOUT, total_iterations, attempts_run,
                            total_nodes_added, total_connect_steps,
                            "total planning timeout reached during CONNECT",
                        )
                    if connection.state == _ConnectState.ATTEMPT_TIMEOUT:
                        last_status = PlanStatus.ATTEMPT_TIMEOUT
                        last_message = f"attempt {attempt_index + 1} timed out during CONNECT"
                        break
                    if connection.state != _ConnectState.REACHED or connection.node_index is None:
                        continue

                    if active_is_start:
                        bridge = (active_index, connection.node_index)
                    else:
                        bridge = (connection.node_index, active_index)
                    break
                else:
                    last_status = PlanStatus.ITERATION_LIMIT
                    last_message = f"attempt {attempt_index + 1} reached its iteration limit"
            except _CollisionBudgetExceeded:
                return self._failure_with_metrics(
                    t0, PlanStatus.COLLISION_BUDGET, total_iterations, attempts_run,
                    total_nodes_added, total_connect_steps,
                    "collision sample budget exhausted",
                )

            if bridge is None:
                continue

            sparse = self._joined_path(start_tree, goal_tree, bridge[0], bridge[1])
            corners = find_corners(sparse)
            try:
                smoothed = self.smooth_path(sparse, corners, deadline=total_deadline)
                dense = densify_path(smoothed, self.settings.trajectory_step_rad)
                if not self._is_edge_free(dense):
                    result = self._failure(
                        t0, PlanStatus.FINAL_VALIDATION_FAILED,
                        total_iterations, attempts_run,
                        "final trajectory collision check failed",
                        sparse_path_rad=smoothed,
                    )
                    return self._attach_metrics(result, total_nodes_added, total_connect_steps)
            except _CollisionBudgetExceeded:
                return self._failure_with_metrics(
                    t0, PlanStatus.COLLISION_BUDGET, total_iterations, attempts_run,
                    total_nodes_added, total_connect_steps,
                    "collision sample budget exhausted during smoothing",
                )
            trajectory = trajectory_from_path(dense)
            result = self._success(
                t0, PlanStatus.PLANNED, trajectory, dense, smoothed,
                find_corners(smoothed), total_iterations, attempts_run, "planned",
            )
            return self._attach_metrics(result, total_nodes_added, total_connect_steps)

        return self._failure_with_metrics(
            t0, last_status, total_iterations, attempts_run,
            total_nodes_added, total_connect_steps, last_message,
        )

    def _extend_once(self, tree: _Tree, target: list[float]) -> int | None:
        """Add one collision-free node toward `target`, without rewiring."""
        nearest_index = tree.nearest_index(target)
        nearest_q = tree.nodes[nearest_index].q
        q_new = _steer(nearest_q, target, self.settings.extend_step_rad)
        if _max_axis_delta(nearest_q, q_new) <= 1e-12:
            return nearest_index
        edge = discretize_joint_line(nearest_q, q_new, self.settings.trajectory_step_rad)
        if not self._is_edge_free(edge):
            return None
        cost = tree.nodes[nearest_index].cost + _distance(nearest_q, q_new)
        return tree.add_node(q_new, nearest_index, cost)

    def _connect_toward(
        self,
        tree: _Tree,
        target: list[float],
        *,
        attempt_deadline: float,
        total_deadline: float,
    ) -> _ConnectResult:
        """Greedily extend `tree` toward `target` until reached or trapped."""
        current_index = tree.nearest_index(target)
        if _max_axis_delta(tree.nodes[current_index].q, target) <= 1e-12:
            return _ConnectResult(_ConnectState.REACHED, current_index, 0)

        steps_added = 0
        for _ in range(max(1, int(self.settings.max_connect_steps))):
            now = time.perf_counter()
            if now >= total_deadline:
                return _ConnectResult(_ConnectState.TOTAL_TIMEOUT, current_index, steps_added)
            if now >= attempt_deadline:
                return _ConnectResult(_ConnectState.ATTEMPT_TIMEOUT, current_index, steps_added)

            current_q = tree.nodes[current_index].q
            q_new = _steer(current_q, target, self.settings.extend_step_rad)
            edge = discretize_joint_line(current_q, q_new, self.settings.trajectory_step_rad)
            if not self._is_edge_free(edge):
                return _ConnectResult(_ConnectState.TRAPPED, current_index, steps_added)
            cost = tree.nodes[current_index].cost + _distance(current_q, q_new)
            current_index = tree.add_node(q_new, current_index, cost)
            steps_added += 1
            if _max_axis_delta(q_new, target) <= 1e-12:
                return _ConnectResult(_ConnectState.REACHED, current_index, steps_added)

        return _ConnectResult(_ConnectState.ADVANCED, current_index, steps_added)

    @staticmethod
    def _attach_metrics(result: PlanResult, nodes_added: int, connect_steps: int) -> PlanResult:
        """Attach BiRRT-Connect growth counters to a common result."""
        result.nodes_added = nodes_added
        result.connect_steps = connect_steps
        return result

    def _failure_with_metrics(
        self,
        t0: float,
        status: PlanStatus,
        iterations: int,
        attempts: int,
        nodes_added: int,
        connect_steps: int,
        message: str,
    ) -> PlanResult:
        """Build a failed result and attach BiRRT-Connect growth counters."""
        result = self._failure(t0, status, iterations, attempts, message)
        return self._attach_metrics(result, nodes_added, connect_steps)


__all__ = ["BiRRTConnectPlanner"]
