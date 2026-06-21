"""Focused tests for scoreboard reset scrolling and stage transitions.

Run:
    C:/Users/yck01/miniconda3/envs/game/python.exe -m unittest tests.test_scoreboard_controller
    C:/Users/yck01/miniconda3/envs/game/python.exe -m unittest tests.test_scoreboard_controller -v
"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent  # Repository root for imports.
SRC = REPO_ROOT / "src"  # Source directory containing runtime packages.
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from subsystems.scoreboard.controller import (  # noqa: E402
    ScoreboardConfig,
    ScoreboardController,
)
from subsystems.scoreboard.layout import ScoreboardLayout  # noqa: E402
from subsystems.scoreboard.transport import (  # noqa: E402
    MODE_SCROLL_UP,
    MODE_STATIC,
    cmd_scroll_continuous,
)


class _RecordingTransport:
    """Record scoreboard command lines without opening serial hardware."""

    def __init__(self) -> None:
        self.lines: list[bytes] = []  # Commands written by controller.pump().

    def write(self, line: bytes) -> bool:
        """Append one command line and report the simulated write as successful."""

        self.lines.append(line)
        return True


def _layout() -> ScoreboardLayout:
    """Return a complete six-panel layout matching the production team mapping."""

    bucket_displays = {
        "A1": 1,
        "A2": 2,
        "A3": 3,
        "B1": 4,
        "B2": 5,
        "B3": 6,
    }  # Bucket label to 1-based physical display index.
    return ScoreboardLayout(port="TEST", bucket_displays=bucket_displays)


def _state(stage: str, rewind_complete: bool) -> dict:
    """Build a one-team state snapshot with explicit rewind completion status."""

    return {
        "active_stage": stage,
        "teams": {
            "a": {
                "buckets": [120, 80, 40],
                "rewind": {"complete": rewind_complete},
                "conclusion": {"phase": "pause_before_sum", "done": False},
            }
        },
    }


class ScoreboardResetScrollTests(unittest.TestCase):
    """Verify continuous reset scrolling lasts until the game stage advances."""

    def test_continuous_scroll_command_encoding(self) -> None:
        """The transport helper must match the firmware's global command syntax."""

        self.assertEqual(cmd_scroll_continuous(True), b"/scrollcontinuous 1\n")
        self.assertEqual(cmd_scroll_continuous(False), b"/scrollcontinuous 0\n")

    def test_reset_scrolls_until_rewind_completion_advances_stage(self) -> None:
        """Reset stays scrolling across ticks and conclusion restores static mode."""

        transport = _RecordingTransport()  # Captures all emitted RS485 commands.
        controller = ScoreboardController(transport, _layout(), ScoreboardConfig())
        controller.initialize()
        controller.pump(now_mono=0.0)
        self.assertIn(b"/scrollcontinuous 1\n", transport.lines)

        controller.set_state(_state("reset", rewind_complete=False))
        controller.update(now_mono=1.0)
        controller.pump(now_mono=1.0)
        for display in (1, 2, 3):
            self.assertEqual(controller.desired_state(display).mode, MODE_SCROLL_UP)

        controller.set_state(_state("reset", rewind_complete=False))
        controller.update(now_mono=2.0)
        for display in (1, 2, 3):
            self.assertEqual(controller.desired_state(display).mode, MODE_SCROLL_UP)

        transport.lines.clear()
        controller.set_state(_state("conclusion", rewind_complete=True))
        controller.update(now_mono=3.0)
        controller.pump(now_mono=3.0)
        for display in (1, 2, 3):
            self.assertEqual(controller.desired_state(display).mode, MODE_STATIC)
            self.assertIn(f"/display/{display}/mode 0\n".encode("ascii"), transport.lines)


if __name__ == "__main__":
    unittest.main()
