"""Tests for repeated rewind validation reporting and stage-gated motion.

Run:
    $env:PYTHONPATH = "src"
    C:/Users/yck01/miniconda3/envs/game/python.exe -m unittest tests.test_rewind_batch_validation
"""

from __future__ import annotations

import json
import gzip
import math
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

REPO_ROOT = Path(__file__).resolve().parent.parent
SRC = REPO_ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from apps.game_controller.batch_validation import (  # noqa: E402
    BatchValidationSession,
    BatchValidationSettings,
)
from core.config import load as load_profile  # noqa: E402
from apps.launcher.__main__ import _is_graceful_shutdown_request  # noqa: E402
from subsystems.haptic.random_trajectory import RandomTrajectoryHaptic  # noqa: E402
from subsystems.rewind.in_process import RewindController  # noqa: E402


class _RewindMetricsStub:
    """Return stable numeric metrics to the batch reporter."""

    def validation_metrics(self) -> dict:
        """Build the subset used by aggregate report calculations."""

        return {
            "shortcut_candidates_attempted": 10,
            "collision_free_candidates": 4,
            "applied_shortcuts": 2,
            "collision_rejected_candidates": 6,
            "deadline_unresolved_candidates": 0,
            "collision_checks_performed": 100,
            "original_dense_points": 1000,
            "remaining_sparse_points": 5,
            "remaining_dense_points": 80,
            "average_fail_fast_checked_percent": 25.0,
            "fail_fast_rejected_candidate_count": 6,
            "fail_fast_checked_fraction_sum": 1.5,
            "optimization_time_s": 3.0,
            "optimization_search_time_s": 2.75,
            "rewind_run_time_s": 8.0,
            "reset_total_time_s": 11.0,
        }


def _batch_haptic_profile() -> SimpleNamespace:
    """Build a minimal batch-enabled random haptic profile."""

    return SimpleNamespace(
        tuning={
            "haptic": {"gear_ratio": [0.1] * 6},
            "robot": {
                "q_limits_min_deg": [-180.0] * 6,
                "q_limits_max_deg": [180.0] * 6,
                "max_velocity_deg_s": [20.0] * 6,
            },
            "jogging": {"path_cutoff_deg": 3.0},
            "random_trajectory_validation": {
                "seed": 100,
                "enabled_on_start": True,
                "ui_enabled": False,
            },
            "batch_validation": {"enabled": True},
        }
    )


class BatchValidationTests(unittest.TestCase):
    """Exercise durable output, seed increments, and profile settings."""

    def test_two_games_write_jsonl_and_complete_summary(self) -> None:
        """Each game is appended while the summary is atomically refreshed."""

        logs_dir = REPO_ROOT / "logs"
        logs_dir.mkdir(exist_ok=True)
        with tempfile.TemporaryDirectory(dir=logs_dir) as directory:
            root = Path(directory)
            settings = BatchValidationSettings(
                enabled=True,
                game_count=2,
                base_seed=700,
                output_jsonl=str(root / "games.jsonl"),
                summary_json=str(root / "summary.json"),
            )
            session = BatchValidationSession(settings)
            teams = {"a": {"rewind": _RewindMetricsStub()}}

            session.mark_play_started()
            self.assertTrue(session.record_completed_game(teams))
            self.assertEqual(session.gameplay_seed(), 701)
            self.assertEqual(session.shortcut_seed(), 1_000_701)
            session.mark_play_started()
            self.assertFalse(session.record_completed_game(teams))

            lines = (root / "games.jsonl").read_text(encoding="utf-8").splitlines()
            records = [json.loads(line) for line in lines]
            summary = json.loads((root / "summary.json").read_text(encoding="utf-8"))

            self.assertEqual([item["gameplay_seed"] for item in records], [700, 701])
            self.assertEqual(summary["completed_game_count"], 2)
            self.assertTrue(summary["complete"])
            self.assertEqual(
                summary["teams"]["a"]["collision_free_candidate_percent"],
                40.0,
            )

    def test_random_haptic_moves_only_during_batch_play(self) -> None:
        """Tutorial and reset hold the target while play activates motion."""

        rig = RandomTrajectoryHaptic(team="a", profile=_batch_haptic_profile())
        rig.update_robot_actual([0.0] * 6)
        rig.update_state_full({"active_stage": "tutorial", "teams": {}})
        self.assertEqual(rig.robot_velocity_rad_s, [0.0] * 6)

        rig.update_state_full({"active_stage": "play", "teams": {}})
        self.assertTrue(any(value != 0.0 for value in rig.robot_velocity_rad_s))

        rig.update_state_full({"active_stage": "reset", "teams": {}})
        self.assertEqual(rig.robot_velocity_rad_s, [0.0] * 6)
        rig.reset_for_game(seed=101, game_index=2)
        self.assertEqual(rig.robot_velocity_rad_s, [0.0] * 6)

    def test_overnight_profile_has_confirmed_timing_and_count(self) -> None:
        """The operator profile encodes the agreed 100-game experiment."""

        profile = load_profile(
            REPO_ROOT
            / "config"
            / "profiles"
            / "dev_random_trajectory_rewind_batch.yaml"
        )
        self.assertEqual(profile.tuning["tutorial"]["duration_s"], 2)
        self.assertEqual(profile.tuning["game"]["duration_s"], 120)
        self.assertEqual(profile.tuning["game"]["rewind_speed_fraction"], 1.0)
        self.assertEqual(profile.tuning["batch_validation"]["game_count"], 100)
        self.assertEqual(
            profile.tuning["batch_validation"]["trajectory_output_dir"],
            "logs/sim_trajectory_recording",
        )

    def test_game_record_references_round_trip_gzip_trajectory(self) -> None:
        """Stored samples contain only relative time and six radian targets."""

        logs_dir = REPO_ROOT / "logs"
        logs_dir.mkdir(exist_ok=True)
        with tempfile.TemporaryDirectory(dir=logs_dir) as directory:
            root = Path(directory)
            settings = BatchValidationSettings(
                enabled=True,
                game_count=1,
                base_seed=800,
                output_jsonl=str(root / "games.jsonl"),
                summary_json=str(root / "summary.json"),
                trajectory_output_dir=str(root / "trajectories"),
            )
            rewind = RewindController(
                enabled=True,
                max_velocity_rad_s=[1.0] * 6,
                speed_fraction=1.0,
                arrival_tolerance_rad=math.radians(0.5),
            )
            rewind.start_recording([0.0] * 6, now_s=10.0)
            rewind.record_target([0.1] * 6, now_s=10.25)
            session = BatchValidationSession(settings)

            session.record_completed_game({"a": {"rewind": rewind}})

            record = json.loads(
                (root / "games.jsonl").read_text(encoding="utf-8").strip()
            )
            relative = record["gameplay_trajectory_files"]["a"]
            trajectory_path = REPO_ROOT / relative
            with gzip.open(trajectory_path, "rt", encoding="utf-8") as stream:
                payload = json.load(stream)

            self.assertEqual(payload["time_unit"], "seconds_from_play_start")
            self.assertEqual(payload["joint_unit"], "radians")
            self.assertEqual(len(payload["samples"]), 2)
            self.assertTrue(all(len(sample) == 7 for sample in payload["samples"]))
            self.assertEqual(payload["samples"][1][0], 0.25)

    def test_launcher_accepts_only_authoritative_batch_shutdown(self) -> None:
        """Unrelated publishers cannot terminate the supervised process tree."""

        self.assertTrue(
            _is_graceful_shutdown_request(
                "cmd.launcher.shutdown",
                {
                    "producer": "game_controller",
                    "reason": "batch_validation_complete",
                },
            )
        )
        self.assertFalse(
            _is_graceful_shutdown_request(
                "cmd.launcher.shutdown",
                {"producer": "untrusted", "reason": "batch_validation_complete"},
            )
        )


if __name__ == "__main__":
    unittest.main()
