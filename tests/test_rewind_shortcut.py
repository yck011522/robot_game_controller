"""Tests for deterministic, collision-certified rewind shortcutting.

Run:
    $env:PYTHONPATH = "src"
    C:/Users/yck01/miniconda3/envs/game/python.exe -m unittest tests.test_rewind_shortcut
"""

from __future__ import annotations

import math
import sys
import threading
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
SRC = REPO_ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from apps.game_controller.context import _rewind_shortcut_config  # noqa: E402
from subsystems.motion_planning.collision_client import (  # noqa: E402
    CollisionWorkerClient,
    ParallelEdgeCheckResult,
)
from subsystems.rewind.shortcut import (  # noqa: E402
    JointTrajectoryShortcutter,
    ShortcutSettings,
)


class RewindShortcutTests(unittest.TestCase):
    """Validate reproducibility and the collision-certification boundary."""

    @staticmethod
    def _zigzag_path() -> list[list[float]]:
        """Return a path with several duration-reducing straight shortcuts."""

        return [
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [1.0, 1.0, 0.0, 0.0, 0.0, 0.0],
            [2.0, 1.0, 0.0, 0.0, 0.0, 0.0],
        ]

    def _run_one_round(self, seed: int, *, collision_free: bool):
        """Run exactly one candidate round by cancelling from its callback."""

        cancel = threading.Event()

        def check_edges(edges, batch_size, max_in_flight, deadline_s):
            """Return the configured synthetic verdict for every candidate."""

            del batch_size, max_in_flight, deadline_s
            return ParallelEdgeCheckResult(
                free=[collision_free] * len(edges),
                configs_sent=sum(len(edge) for edge in edges),
                batches_sent=len(edges),
                compute_ms=1.0,
            )

        optimizer = JointTrajectoryShortcutter(
            settings=ShortcutSettings(
                enabled=True,
                optimization_budget_s=1.0,
                collision_step_rad=math.radians(1.0),
                collision_batch_size=8,
                worker_limit=4,
                random_seed=seed,
            ),
            max_velocity_rad_s=[1.0] * 6,
            speed_fraction=0.3,
            edge_check_fn=check_edges,
        )
        return optimizer.optimize(
            self._zigzag_path(),
            cancel_event=cancel,
            progress_fn=lambda result: cancel.set(),
        )

    def test_fixed_seed_produces_same_shortcut(self) -> None:
        """A validation seed makes candidate selection reproducible."""

        first = self._run_one_round(12345, collision_free=True)
        second = self._run_one_round(12345, collision_free=True)

        self.assertEqual(first.path_rad, second.path_rad)
        self.assertEqual(first.seed, 12345)
        self.assertLess(len(first.path_rad), len(self._zigzag_path()))
        self.assertLess(first.shortened_duration_s, first.original_duration_s)

    def test_rejected_shortcuts_leave_certified_path_unchanged(self) -> None:
        """No candidate may alter the path without a free collision verdict."""

        result = self._run_one_round(12345, collision_free=False)

        self.assertEqual(result.path_rad, self._zigzag_path())
        self.assertGreater(result.collision_rejections, 0)
        self.assertEqual(result.accepted_shortcuts, 0)

    def test_profile_defaults_to_fresh_production_seed(self) -> None:
        """Omitted seeds remain random while explicit validation seeds parse."""

        defaults = _rewind_shortcut_config({})
        configured = _rewind_shortcut_config(
            {"enabled": True, "random_seed": 99, "collision_batch_size": 8}
        )

        self.assertIsNone(defaults["random_seed"])
        self.assertEqual(configured["random_seed"], 99)
        self.assertTrue(configured["enabled"])


class _ImmediateCollisionTransport:
    """Minimal in-memory transport for exercising the parallel scheduler."""

    def __init__(self) -> None:
        self._sock = self
        self._next_request_id = 0
        self._replies: list[dict] = []
        self.sent_first_axes: list[list[float]] = []
        self.reset_count = 0

    def poll(self, timeout_ms: int) -> int:
        """Report a reply immediately; timeout is unused by this fake."""

        del timeout_ms
        return int(bool(self._replies))

    def _send_request(self, configs_rad: list[list[float]]) -> int:
        """Queue a matching synthetic reply; value 99 denotes collision."""

        self._next_request_id += 1
        self.sent_first_axes.append([q[0] for q in configs_rad])
        self._replies.append(
            {
                "request_id": self._next_request_id,
                "ok": True,
                "results": [
                    {"collision": math.isclose(q[0], 99.0)} for q in configs_rad
                ],
                "compute_ms": 0.1,
            }
        )
        return self._next_request_id

    def _recv_reply(self) -> dict:
        """Return replies in dispatch order for deterministic assertions."""

        return self._replies.pop(0)

    def _reset_socket(self) -> None:
        """Model discarding outstanding replies after fail-fast completion."""

        self.reset_count += 1
        self._replies.clear()


class ParallelEdgeSchedulerTests(unittest.TestCase):
    """Check bounded round-robin dispatch and per-edge early rejection."""

    def test_collision_stops_new_chunks_for_only_the_failed_edge(self) -> None:
        """A collision leaves other candidates running without queue flooding."""

        transport = _ImmediateCollisionTransport()
        edges = [
            [[99.0] + [0.0] * 5, [98.0] + [0.0] * 5],
            [[1.0] + [0.0] * 5, [2.0] + [0.0] * 5],
        ]

        result = CollisionWorkerClient.check_edges_parallel_until_collision(
            transport,
            edges,
            batch_size=1,
            max_in_flight=2,
            deadline_s=10**9,
        )

        self.assertEqual(result.free, [False, True])
        flattened = [value for batch in transport.sent_first_axes for value in batch]
        self.assertNotIn(98.0, flattened)
        self.assertIn(2.0, flattened)


if __name__ == "__main__":
    unittest.main()
