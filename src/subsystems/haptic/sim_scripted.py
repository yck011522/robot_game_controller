"""Headless scripted haptic source.

Used by automated tests in place of `sim_keyboard` (which needs pygame
and a display). Publishes a low-frequency sine on every dial so the
game_controller and robot_io have something to react to without any
human input. All 6 dials are 90簞 out of phase from each other for
visual variety in any tap-recorded trace.
"""

from __future__ import annotations

import math
import time

# Hz at which we publish telem.haptic.<team> (matches BUS.md 禮6.2).
PUBLISH_HZ = 50.0

# Sine parameters per dial: small amplitude (so the resulting joint
# motion stays well within the URDF limits even at high gear_ratio),
# slow period (one full cycle every 4 s).
_AMPLITUDE_RAD = 0.05
_PERIOD_S = 4.0


class ScriptedHaptic:
    """Produces dial samples on demand. Stateless beyond `_t0`."""

    def __init__(self):
        self._t0 = time.perf_counter()

    def sample(self) -> dict:
        t = time.perf_counter() - self._t0
        # Six dials, each phase-shifted by ?/3 from the previous one.
        pos = [
            _AMPLITUDE_RAD * math.sin(2 * math.pi * t / _PERIOD_S + i * math.pi / 3)
            for i in range(6)
        ]
        vel = [
            _AMPLITUDE_RAD * (2 * math.pi / _PERIOD_S)
            * math.cos(2 * math.pi * t / _PERIOD_S + i * math.pi / 3)
            for i in range(6)
        ]
        return {
            "dial_pos_rad": pos,
            "dial_vel_rad_s": vel,
            "torque_ma": [0.0] * 6,  # sim source, no real force feedback to report
            "board_connected": [True] * 6,
            "board_loop_hz": [200] * 6,
        }
