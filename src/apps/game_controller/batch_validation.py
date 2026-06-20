"""Crash-resilient reporting and seed control for repeated virtual games."""

from __future__ import annotations

import json
import os
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from subsystems.rewind.trajectory_io import write_joint_trajectory_json_gz


_REPO_ROOT = Path(__file__).resolve().parents[3]
_SCHEMA_VERSION = 1
_SHORTCUT_SEED_OFFSET = 1_000_000


@dataclass(frozen=True)
class BatchValidationSettings:
    """Profile settings controlling repeated game validation and output."""

    enabled: bool = False
    game_count: int = 1
    base_seed: int = 12345
    auto_restart: bool = True
    shutdown_when_complete: bool = True
    output_jsonl: str = "logs/rewind_batch_validation.jsonl"
    summary_json: str = "logs/rewind_batch_validation_summary.json"
    trajectory_output_dir: str | None = None


def batch_validation_settings(node: Any) -> BatchValidationSettings:
    """Normalize ``tuning.batch_validation`` without affecting other profiles."""

    data = node if isinstance(node, dict) else {}
    return BatchValidationSettings(
        enabled=bool(data.get("enabled", False)),
        game_count=max(1, _int_value(data.get("game_count"), 1)),
        base_seed=_int_value(data.get("base_seed"), 12345),
        auto_restart=bool(data.get("auto_restart", True)),
        shutdown_when_complete=bool(data.get("shutdown_when_complete", True)),
        output_jsonl=str(
            data.get("output_jsonl") or "logs/rewind_batch_validation.jsonl"
        ),
        summary_json=str(
            data.get("summary_json")
            or "logs/rewind_batch_validation_summary.json"
        ),
        trajectory_output_dir=(
            str(data["trajectory_output_dir"])
            if data.get("trajectory_output_dir")
            else None
        ),
    )


class BatchValidationSession:
    """Own per-game seeds and durably write one report after every rewind."""

    def __init__(self, settings: BatchValidationSettings) -> None:
        self.settings = settings
        self.game_index = 1
        self.shutdown_requested = False
        self._play_started_wall_ns: int | None = None
        self._play_started_mono_s: float | None = None
        self._records: list[dict[str, Any]] = []
        self._run_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        self._jsonl_path = _resolve_output_path(settings.output_jsonl)
        self._summary_path = _resolve_output_path(settings.summary_json)
        self._trajectory_root = (
            _resolve_output_path(settings.trajectory_output_dir) / self._run_id
            if settings.trajectory_output_dir
            else None
        )
        self._prepare_outputs()

    @property
    def enabled(self) -> bool:
        """Return whether this session should override the normal lifecycle."""

        return self.settings.enabled

    @property
    def completed_game_count(self) -> int:
        """Return the number of records already flushed to the JSONL file."""

        return len(self._records)

    def gameplay_seed(self, team_index: int = 0) -> int:
        """Return the independently reproducible random-play seed for a team."""

        return self.settings.base_seed + self.game_index - 1 + team_index

    def shortcut_seed(self, team_index: int = 0) -> int:
        """Return a separate deterministic shortcut-search seed for a team."""

        return (
            self.settings.base_seed
            + _SHORTCUT_SEED_OFFSET
            + self.game_index
            - 1
            + team_index
        )

    def mark_play_started(self) -> None:
        """Capture wall and monotonic timestamps on tutorial-to-play entry."""

        self._play_started_wall_ns = time.time_ns()
        self._play_started_mono_s = time.perf_counter()

    def record_completed_game(self, teams: dict[str, dict]) -> bool:
        """Append one completed game and return whether another should start."""

        completed_wall_ns = time.time_ns()
        completed_mono_s = time.perf_counter()
        team_records: dict[str, dict[str, Any]] = {}
        trajectory_files: dict[str, str] = {}
        for team, state in teams.items():
            rewind = state["rewind"]
            team_records[team] = rewind.validation_metrics()
            trajectory_path = self._write_gameplay_trajectory(team, rewind)
            if trajectory_path is not None:
                trajectory_files[team] = trajectory_path
        record = {
            "schema_version": _SCHEMA_VERSION,
            "record_type": "game",
            "run_id": self._run_id,
            "game_index": self.game_index,
            "requested_game_count": self.settings.game_count,
            "gameplay_seed": self.gameplay_seed(),
            "shortcut_seed": self.shortcut_seed(),
            "play_started_wall_ns": self._play_started_wall_ns,
            "completed_wall_ns": completed_wall_ns,
            "play_and_reset_wall_time_s": _elapsed(
                self._play_started_mono_s, completed_mono_s
            ),
            "teams": team_records,
            "gameplay_trajectory_files": trajectory_files,
        }
        self._append_record(record)
        self._records.append(record)
        self._write_summary()
        has_next = self.game_index < self.settings.game_count
        if has_next:
            self.game_index += 1
            self._play_started_wall_ns = None
            self._play_started_mono_s = None
        return has_next and self.settings.auto_restart

    def close(self) -> None:
        """Refresh the aggregate summary during normal process teardown."""

        self._write_summary()

    def _prepare_outputs(self) -> None:
        """Create output directories and overwrite stale latest-run files."""

        self._jsonl_path.parent.mkdir(parents=True, exist_ok=True)
        self._summary_path.parent.mkdir(parents=True, exist_ok=True)
        self._jsonl_path.write_text("", encoding="utf-8")
        self._write_summary()

    def _write_gameplay_trajectory(self, team: str, rewind: Any) -> str | None:
        """Persist one team's dense play targets and return a repo-relative path."""

        if self._trajectory_root is None:
            return None
        gameplay_seed = self.gameplay_seed()
        filename = (
            f"game_{self.game_index:04d}_seed_{gameplay_seed}_team_{team}.json.gz"
        )
        path = self._trajectory_root / filename
        write_joint_trajectory_json_gz(rewind.recorded_trajectory, path)
        try:
            return path.relative_to(_REPO_ROOT).as_posix()
        except ValueError:
            return str(path)

    def _append_record(self, record: dict[str, Any]) -> None:
        """Append, flush, and fsync one compact JSONL game record."""

        line = json.dumps(record, separators=(",", ":"), sort_keys=True)
        with self._jsonl_path.open("a", encoding="utf-8", newline="\n") as stream:
            stream.write(line + "\n")
            stream.flush()
            os.fsync(stream.fileno())

    def _write_summary(self) -> None:
        """Atomically refresh aggregate totals and averages for the run."""

        summary = _aggregate_summary(
            self._records,
            run_id=self._run_id,
            requested_games=self.settings.game_count,
        )
        temporary = self._summary_path.with_suffix(self._summary_path.suffix + ".tmp")
        temporary.write_text(
            json.dumps(summary, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        temporary.replace(self._summary_path)


def _aggregate_summary(
    records: list[dict[str, Any]], *, run_id: str, requested_games: int
) -> dict[str, Any]:
    """Build per-team aggregate values from completed game records."""

    team_names = sorted(
        {
            team
            for record in records
            for team in (record.get("teams") or {}).keys()
        }
    )
    teams: dict[str, Any] = {}
    sum_keys = (
        "shortcut_candidates_attempted",
        "collision_free_candidates",
        "applied_shortcuts",
        "collision_rejected_candidates",
        "deadline_unresolved_candidates",
        "collision_checks_performed",
        "collision_configurations_planned",
        "collision_results_received",
        "collision_batches_dispatched",
        "worker_compute_ms_sum",
        "fail_fast_rejected_candidate_count",
        "fail_fast_checked_fraction_sum",
    )
    average_keys = (
        "original_dense_points",
        "remaining_sparse_points",
        "remaining_dense_points",
        "optimization_time_s",
        "optimization_search_time_s",
        "rewind_run_time_s",
        "reset_total_time_s",
    )
    for team in team_names:
        metrics = [
            record["teams"][team]
            for record in records
            if team in (record.get("teams") or {})
        ]
        totals = {key: _sum_numeric(metrics, key) for key in sum_keys}
        teams[team] = {
            "games": len(metrics),
            "totals": totals,
            "averages": {key: _average_numeric(metrics, key) for key in average_keys},
            "collision_free_candidate_percent": _ratio_percent(
                totals["collision_free_candidates"],
                totals["shortcut_candidates_attempted"],
            ),
            "applied_shortcut_percent": _ratio_percent(
                totals["applied_shortcuts"],
                totals["shortcut_candidates_attempted"],
            ),
            "average_fail_fast_checked_percent": _weighted_fail_fast_percent(
                totals["fail_fast_checked_fraction_sum"],
                totals["fail_fast_rejected_candidate_count"],
            ),
        }
    return {
        "schema_version": _SCHEMA_VERSION,
        "record_type": "summary",
        "run_id": run_id,
        "requested_game_count": requested_games,
        "completed_game_count": len(records),
        "complete": len(records) >= requested_games,
        "updated_wall_ns": time.time_ns(),
        "teams": teams,
    }


def _resolve_output_path(value: str) -> Path:
    """Resolve profile-relative output names against the repository root."""

    path = Path(value)
    return path if path.is_absolute() else _REPO_ROOT / path


def _int_value(value: Any, default: int) -> int:
    """Return an integer profile value or its documented default."""

    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def _elapsed(start_s: float | None, end_s: float) -> float | None:
    """Return elapsed seconds when a start timestamp has been captured."""

    return max(0.0, end_s - start_s) if start_s is not None else None


def _sum_numeric(records: list[dict[str, Any]], key: str) -> int | float:
    """Sum numeric non-boolean metric values under one key."""

    values = [
        record[key]
        for record in records
        if isinstance(record.get(key), (int, float))
        and not isinstance(record.get(key), bool)
    ]
    total = sum(values)
    all_integers = values and all(isinstance(value, int) for value in values)
    return int(total) if all_integers else total


def _average_numeric(records: list[dict[str, Any]], key: str) -> float | None:
    """Average numeric non-boolean values, ignoring unavailable metrics."""

    values = [
        float(record[key])
        for record in records
        if isinstance(record.get(key), (int, float))
        and not isinstance(record.get(key), bool)
    ]
    return sum(values) / len(values) if values else None


def _ratio_percent(numerator: float, denominator: float) -> float | None:
    """Return a percentage, or ``None`` for an empty candidate population."""

    return 100.0 * numerator / denominator if denominator > 0.0 else None


def _weighted_fail_fast_percent(
    fraction_sum: float, rejected_candidate_count: float
) -> float | None:
    """Return the rejected-candidate-weighted mean checked percentage."""

    if rejected_candidate_count <= 0.0:
        return None
    return 100.0 * fraction_sum / rejected_candidate_count
