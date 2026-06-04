"""Pygame keyboard haptic source.

Six-dial UR10e mapped to a four-row keyboard layout, mirroring the
`bullet_collision_keyboard_explorer.py` baseline so anyone used to
that tool feels at home:

    joint 0 (base):   fast+ '1'   slow+ 'q'   slow- 'a'   fast- 'z'
    joint 1 (shldr):  fast+ '2'   slow+ 'w'   slow- 's'   fast- 'x'
    joint 2 (elbow):  fast+ '3'   slow+ 'e'   slow- 'd'   fast- 'c'
    joint 3 (wrst1):  fast+ '4'   slow+ 'r'   slow- 'f'   fast- 'v'
    joint 4 (wrst2):  fast+ '5'   slow+ 't'   slow- 'g'   fast- 'b'
    joint 5 (wrst3):  fast+ '6'   slow+ 'y'   slow- 'h'   fast- 'n'

Held keys are summed algebraically per axis -- e.g. holding `q` and
`1` together moves at slow+fast deg/s in the positive direction;
holding `q` and `a` together cancels out. Rates are tuned in DIAL
deg/s; the jogging planner applies the gear ratio downstream, so a
gear=10 profile turns SLOW_DPS=10 of dial motion into 100 deg/s at the
joint -- this matches the explorer's "feel" for the default profile.

Velocity is derived from the per-tick position delta. Speed clamping
and acceleration shaping are NOT applied here; that responsibility
belongs to the jogging planner (where it can also see real hardware
state). Real haptic boards bypass this file entirely and are not
bounded by these constants.
"""

from __future__ import annotations

import math
import time

PUBLISH_HZ = 50.0

# Dial-space deg/s for the two speed tiers. With the default
# gear_ratio=10 these become 100 / 300 deg/s at the joint after the
# planner's gear multiply.
SLOW_DPS = 10.0
FAST_DPS = 30.0

SLOW_RAD_S = math.radians(SLOW_DPS)
FAST_RAD_S = math.radians(FAST_DPS)


class KeyboardHaptic:
    def __init__(self):
        import pygame
        pygame.init()
        self._screen = pygame.display.set_mode((520, 160))
        pygame.display.set_caption(
            "Haptic keyboard | 1-6 fast+  qwerty slow+  asdfgh slow-  zxcvbn fast-"
        )
        self._pygame = pygame
        self._pos = [0.0] * 6
        self._last_pos = list(self._pos)
        self._last_t = time.perf_counter()
        # Per joint: (fast_pos, slow_pos, slow_neg, fast_neg).
        K = pygame
        self._bindings = [
            (K.K_1, K.K_q, K.K_a, K.K_z),
            (K.K_2, K.K_w, K.K_s, K.K_x),
            (K.K_3, K.K_e, K.K_d, K.K_c),
            (K.K_4, K.K_r, K.K_f, K.K_v),
            (K.K_5, K.K_t, K.K_g, K.K_b),
            (K.K_6, K.K_y, K.K_h, K.K_n),
        ]

    def sample(self) -> dict:
        # Drain the pygame event queue so the window stays responsive.
        for _ in self._pygame.event.get():
            pass
        keys = self._pygame.key.get_pressed()
        now = time.perf_counter()
        dt = max(1e-3, now - self._last_t)
        self._last_t = now
        for i, (kfp, ksp, ksn, kfn) in enumerate(self._bindings):
            # Algebraic sum: e.g. fast+ and slow+ held -> +40 dps;
            # fast+ and fast- held -> 0; matches the explorer behavior.
            rate = 0.0
            if keys[kfp]:
                rate += FAST_RAD_S
            if keys[ksp]:
                rate += SLOW_RAD_S
            if keys[ksn]:
                rate -= SLOW_RAD_S
            if keys[kfn]:
                rate -= FAST_RAD_S
            self._pos[i] += rate * dt
        vel = [(p - lp) / dt for p, lp in zip(self._pos, self._last_pos)]
        self._last_pos = list(self._pos)
        # Refresh the window so it doesn't go "Not Responding" on Windows.
        self._screen.fill((20, 20, 20))
        self._pygame.display.flip()
        return {
            "dial_pos_rad": list(self._pos),
            "dial_vel_rad_s": vel,
            "board_connected": [True] * 6,
            "board_loop_hz": [int(PUBLISH_HZ)] * 6,
        }

    def close(self) -> None:
        try:
            self._pygame.quit()
        except Exception:
            pass
