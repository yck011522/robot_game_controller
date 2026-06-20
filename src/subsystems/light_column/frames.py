"""Pure color-frame helpers for the arena light columns.

A "frame" is just a list of :class:`Color` values, one per addressable LED
segment on a single strip (28 by default). These helpers are completely
side-effect free: they take colors plus a few scalars and return a new list.
They never touch serial hardware or controller state, which keeps the
animation math trivially unit-testable.

Two building blocks cover every animation the game needs today:

* :func:`solid` - one color for the whole strip (idle/reset/team fills).
* :func:`two_color_split` - a sharp bottom/top split with a *single*
  anti-aliased boundary LED (progress / speed / score bars).

:func:`scale` and :func:`mix` are small color utilities used by the breathing
effect and the boundary blend respectively.
"""

from __future__ import annotations

import math
from typing import NamedTuple


class Color(NamedTuple):
    """Logical RGB color, channels in 0-255.

    Stored as a plain immutable triple so frames are cheap to build and
    hashable for change-detection if ever needed. Use :func:`make_color`
    when channel values might fall outside 0-255 and need clamping.
    """

    r: int
    g: int
    b: int


def _clamp8(value: float) -> int:
    """Clamp and round a single channel value into the 0-255 byte range."""

    if value <= 0:
        return 0
    if value >= 255:
        return 255
    return int(round(value))


def make_color(r: float, g: float, b: float) -> Color:
    """Build a :class:`Color`, clamping each channel into 0-255."""

    return Color(_clamp8(r), _clamp8(g), _clamp8(b))


# Common named colors shared across renderers and tests.
BLACK = Color(0, 0, 0)
OFF = BLACK
WHITE = Color(255, 255, 255)
RED = Color(255, 0, 0)
GREEN = Color(0, 255, 0)
BLUE = Color(0, 0, 255)


def solid(color: Color, count: int) -> list[Color]:
    """Return a frame of ``count`` LEDs all set to ``color``."""

    return [color] * max(0, int(count))


def scale(color: Color, brightness: float) -> Color:
    """Scale a color toward black by ``brightness`` in [0, 1].

    ``brightness`` 1.0 returns the original color; 0.0 returns black. Used by
    the daydreaming "breathing" effect, where ``brightness`` is driven by a
    wall-clock sine wave.
    """

    amount = 0.0 if brightness < 0.0 else 1.0 if brightness > 1.0 else float(brightness)
    return Color(
        _clamp8(color.r * amount),
        _clamp8(color.g * amount),
        _clamp8(color.b * amount),
    )


def mix(first: Color, second: Color, fraction: float) -> Color:
    """Linearly blend ``first`` -> ``second`` by ``fraction`` in [0, 1].

    ``fraction`` 0.0 returns ``first``; 1.0 returns ``second``. Used only for
    the single boundary LED in :func:`two_color_split`.
    """

    amount = 0.0 if fraction < 0.0 else 1.0 if fraction > 1.0 else float(fraction)
    return Color(
        _clamp8(first.r + (second.r - first.r) * amount),
        _clamp8(first.g + (second.g - first.g) * amount),
        _clamp8(first.b + (second.b - first.b) * amount),
    )


def two_color_split(
    bottom: Color,
    top: Color,
    fraction_bottom: float,
    count: int,
) -> list[Color]:
    """Render a sharp bottom/top split bar with one anti-aliased boundary LED.

    The lower ``fraction_bottom`` portion of the column (measured from index 0,
    the physical bottom of the strip) is solid ``bottom``; the remainder is
    solid ``top``. Exactly one LED - the one straddling the boundary - is
    blended between the two colors by the sub-LED remainder, giving smooth
    visual resolution between the discrete segments without turning the whole
    bar into a gradient. The rest of the bar stays crisp.

    Args:
        bottom: Color for the filled (lower) portion. For a progress/speed bar
            this is the active color (team color); ``top`` is usually
            :data:`OFF`.
        top: Color for the empty (upper) portion.
        fraction_bottom: Fill fraction in [0, 1]. 1.0 -> entirely ``bottom``;
            0.0 -> entirely ``top``.
        count: Number of LEDs on the strip.

    Returns:
        A new list of ``count`` colors.
    """

    n = max(1, int(count))
    fill = 0.0 if fraction_bottom < 0.0 else 1.0 if fraction_bottom > 1.0 else float(
        fraction_bottom
    )
    exact = fill * n
    full = int(math.floor(exact))
    if full > n:
        full = n
    remainder = exact - full

    colors = [top] * n
    for index in range(full):
        colors[index] = bottom
    if full < n and remainder > 0.0:
        # The single boundary LED is `remainder` of the way toward `bottom`.
        colors[full] = mix(top, bottom, remainder)
    return colors
