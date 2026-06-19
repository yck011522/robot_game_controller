"""Time-bounded collision-certified shortcutting for rewind trajectories."""

from __future__ import annotations

import math
import random
import secrets
import threading
import time
from dataclasses import dataclass, replace
from typing import Callable, Sequence

from subsystems.motion_planning.collision_client import ParallelEdgeCheckResult
from subsystems.motion_planning.planner_core import discretize_joint_line


EdgeCheckFn = Callable[
    [list[list[list[float]]], int, int, float], ParallelEdgeCheckResult
]
ProgressFn = Callable[["ShortcutResult"], None]


@dataclass(frozen=True)
class ShortcutSettings:
    """Configuration for one team's bounded shortcut search."""

    enabled: bool = False
    optimization_budget_s: float = 3.0
    collision_step_rad: float = math.radians(1.0)
    collision_batch_size: int = 8
    worker_limit: int = 1
    random_seed: int | None = None


@dataclass(frozen=True)
class ShortcutCandidate:
    """One proposed replacement of an ordered path interval."""

    start_index: int
    end_index: int
    estimated_saving_s: float
    collision_samples: list[list[float]]


@dataclass(frozen=True)
class ShortcutResult:
    """Best path and cumulative optimization statistics at one instant."""

    path_rad: list[list[float]]
    status: str
    seed: int
    original_point_count: int
    shortened_point_count: int
    attempts: int
    accepted_shortcuts: int
    collision_rejections: int
    unresolved_candidates: int
    configurations_sent: int
    batches_sent: int
    worker_compute_ms: float
    elapsed_s: float
    original_duration_s: float
    shortened_duration_s: float


class JointTrajectoryShortcutter:
    """Repeatedly replace path intervals with certified straight joint edges."""

    def __init__(
        self,
        *,
        settings: ShortcutSettings,
        max_velocity_rad_s: Sequence[float],
        speed_fraction: float,
        edge_check_fn: EdgeCheckFn,
    ) -> None:
        self.settings = settings
        self._velocity_limits = [
            max(1e-9, abs(float(value)) * max(1e-6, float(speed_fraction)))
            for value in list(max_velocity_rad_s)[:6]
        ]
        while len(self._velocity_limits) < 6:
            self._velocity_limits.append(1.0)
        self._edge_check = edge_check_fn

    def optimize(
        self,
        path_rad: Sequence[Sequence[float]],
        *,
        cancel_event: threading.Event | None = None,
        progress_fn: ProgressFn | None = None,
    ) -> ShortcutResult:
        """Optimize until the wall-clock deadline without an iteration cap."""

        path = [[float(value) for value in q[:6]] for q in path_rad]
        seed = (
            int(self.settings.random_seed)
            if self.settings.random_seed is not None
            else secrets.randbits(64)
        )
        rng = random.Random(seed)
        started_s = time.perf_counter()
        deadline_s = started_s + max(0.0, float(self.settings.optimization_budget_s))
        original_duration_s = self._path_duration(path)
        result = ShortcutResult(
            path_rad=path,
            status="optimizing",
            seed=seed,
            original_point_count=len(path),
            shortened_point_count=len(path),
            attempts=0,
            accepted_shortcuts=0,
            collision_rejections=0,
            unresolved_candidates=0,
            configurations_sent=0,
            batches_sent=0,
            worker_compute_ms=0.0,
            elapsed_s=0.0,
            original_duration_s=original_duration_s,
            shortened_duration_s=original_duration_s,
        )

        while len(path) > 2 and time.perf_counter() < deadline_s:
            if cancel_event is not None and cancel_event.is_set():
                return replace(result, path_rad=path, status="cancelled")
            candidates = self._candidate_round(path, rng)
            if not candidates:
                continue
            checks = self._edge_check(
                [candidate.collision_samples for candidate in candidates],
                self.settings.collision_batch_size,
                self.settings.worker_limit,
                deadline_s,
            )
            attempts = result.attempts + len(candidates)
            collision_rejections = result.collision_rejections + sum(
                verdict is False for verdict in checks.free
            )
            unresolved = result.unresolved_candidates + sum(
                verdict is None for verdict in checks.free
            )
            valid = [
                candidate
                for candidate, verdict in zip(candidates, checks.free)
                if verdict is True
            ]
            accepted = result.accepted_shortcuts
            if valid:
                best = max(valid, key=lambda candidate: candidate.estimated_saving_s)
                path = path[: best.start_index + 1] + path[best.end_index :]
                accepted += 1
            elapsed_s = time.perf_counter() - started_s
            result = ShortcutResult(
                path_rad=path,
                status="optimizing",
                seed=seed,
                original_point_count=result.original_point_count,
                shortened_point_count=len(path),
                attempts=attempts,
                accepted_shortcuts=accepted,
                collision_rejections=collision_rejections,
                unresolved_candidates=unresolved,
                configurations_sent=result.configurations_sent + checks.configs_sent,
                batches_sent=result.batches_sent + checks.batches_sent,
                worker_compute_ms=result.worker_compute_ms + checks.compute_ms,
                elapsed_s=elapsed_s,
                original_duration_s=original_duration_s,
                shortened_duration_s=self._path_duration(path),
            )
            if progress_fn is not None:
                progress_fn(result)

        status = "cancelled" if cancel_event is not None and cancel_event.is_set() else "complete"
        return replace(
            result,
            path_rad=path,
            status=status,
            shortened_point_count=len(path),
            elapsed_s=time.perf_counter() - started_s,
            shortened_duration_s=self._path_duration(path),
        )

    def _candidate_round(
        self, path: list[list[float]], rng: random.Random
    ) -> list[ShortcutCandidate]:
        """Generate distinct duration-reducing candidates for one worker round."""

        desired_count = max(1, int(self.settings.worker_limit))
        candidates: list[ShortcutCandidate] = []
        seen: set[tuple[int, int]] = set()
        generation_attempts = 0
        generation_limit = max(32, desired_count * 16)
        while len(candidates) < desired_count and generation_attempts < generation_limit:
            generation_attempts += 1
            first, last = sorted(rng.sample(range(len(path)), 2))
            if last - first < 2 or (first, last) in seen:
                continue
            seen.add((first, last))
            old_duration = self._path_duration(path[first : last + 1])
            direct_duration = self._edge_duration(path[first], path[last])
            saving = old_duration - direct_duration
            if saving <= 1e-9:
                continue
            samples = discretize_joint_line(
                path[first], path[last], self.settings.collision_step_rad
            )
            candidates.append(
                ShortcutCandidate(
                    start_index=first,
                    end_index=last,
                    estimated_saving_s=saving,
                    collision_samples=samples,
                )
            )
        return candidates

    def _path_duration(self, path: Sequence[Sequence[float]]) -> float:
        """Estimate velocity-retimed duration of an ordered geometric path."""

        return sum(
            self._edge_duration(first, last)
            for first, last in zip(path, path[1:])
        )

    def _edge_duration(self, first: Sequence[float], last: Sequence[float]) -> float:
        """Return synchronized per-axis duration for one straight edge."""

        return max(
            abs(float(last[axis]) - float(first[axis])) / self._velocity_limits[axis]
            for axis in range(6)
        )
