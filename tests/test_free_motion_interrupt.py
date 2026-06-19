"""Verify Ctrl+C-safe JSON persistence for free-motion case generation."""

from __future__ import annotations

import importlib.util
import json
import math
import sys
import tempfile
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC = REPO_ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from subsystems.motion_planning import PlannerSettings  # noqa: E402


def _load_validation_tool():
    """Load the standalone tool as a module without invoking its CLI."""
    tool_path = REPO_ROOT / "tools" / "validate_free_motion_planner.py"
    spec = importlib.util.spec_from_file_location("validate_free_motion_planner", tool_path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class _InterruptingOracle:
    """Collision oracle that simulates Ctrl+C during the first check."""

    config_checks = 0  # Compatibility metric consumed by validation summaries.
    batch_checks = 0  # Compatibility metric consumed by validation summaries.

    def are_configs_free(
        self,
        configs_rad: list[list[float]],
        *,
        batch_size: int | None = None,
    ) -> list[bool]:
        """Raise KeyboardInterrupt during batched endpoint screening."""
        raise KeyboardInterrupt

    def is_config_free(self, q_rad: list[float]) -> bool:
        """Raise KeyboardInterrupt as if the operator pressed Ctrl+C."""
        raise KeyboardInterrupt

    def is_edge_free(self, points_rad: list[list[float]]) -> bool:
        """Raise KeyboardInterrupt if an edge check is reached unexpectedly."""
        raise KeyboardInterrupt


def test_generation_interrupt_writes_valid_partial_outputs() -> None:
    """Interrupted generation atomically writes both datasets and its run log."""
    tool = _load_validation_tool()
    with tempfile.TemporaryDirectory() as temp_dir:
        root = Path(temp_dir)
        easy_path = root / "easy.json"
        hard_path = root / "hard.json"
        log_path, interrupted = tool._classify_generated_cases(
            easy_path=easy_path,
            hard_path=hard_path,
            starts=10,
            seed=123,
            duration_s=60.0,
            goal_rad=[0.0] * 6,
            q_min_rad=[-math.pi] * 6,
            q_max_rad=[math.pi] * 6,
            oracle=_InterruptingOracle(),
            settings=PlannerSettings(max_iterations_per_attempt=0),
            log_every=1,
            sample_batch_size=4,
            out_dir=root,
        )
        assert interrupted is True
        assert json.loads(easy_path.read_text(encoding="utf-8"))["starts_deg"] == []
        assert json.loads(hard_path.read_text(encoding="utf-8"))["starts_deg"] == []
        assert json.loads(log_path.read_text(encoding="utf-8"))["interrupted"] is True
        assert not list(root.glob("*.tmp"))


def main() -> int:
    """Run the interruption test without requiring pytest."""
    test_generation_interrupt_writes_valid_partial_outputs()
    print("[test] free motion Ctrl+C persistence: OK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
