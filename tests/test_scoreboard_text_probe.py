"""Unit tests for the hardware-free scoreboard probe command builder.

Run:
    C:/Users/yck01/miniconda3/envs/game/python.exe -m unittest tests.test_scoreboard_text_probe
"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent  # Repository root for tool import.
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from tools.scoreboard_text_probe import build_commands  # noqa: E402


class ScoreboardTextProbeTests(unittest.TestCase):
    """Verify each CLI mode produces the intended firmware command sequence."""

    def test_static_commands(self) -> None:
        """Static mode addresses only the selected display and does not scroll."""

        self.assertEqual(
            build_commands(2, "GAME OVER", "static"),
            [
                b"/display/2/text/enable 1\n",
                b"/display/2/mode 0\n",
                b'/display/2/text/stack "GAME OVER"\n',
            ],
        )

    def test_single_scroll_commands(self) -> None:
        """Single mode disables global repetition before starting display 2."""

        self.assertEqual(
            build_commands(2, "GAME,OVER", "single"),
            [
                b"/scrollcontinuous 0\n",
                b"/display/2/text/enable 1\n",
                b"/display/2/mode 1\n",
                b'/display/2/text/stack "GAME,OVER"\n',
            ],
        )

    def test_continuous_scroll_commands(self) -> None:
        """Continuous mode enables global repetition before starting display 2."""

        self.assertEqual(
            build_commands(2, "GAME,OVER", "continuous"),
            [
                b"/scrollcontinuous 1\n",
                b"/display/2/text/enable 1\n",
                b"/display/2/mode 1\n",
                b'/display/2/text/stack "GAME,OVER"\n',
            ],
        )


if __name__ == "__main__":
    unittest.main()
