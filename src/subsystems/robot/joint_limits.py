"""Shared UR10e joint-limit helpers.

Joint hard limits are defined only by the profile config.

Profiles should set `tuning.robot.q_limits_min_deg` /
`q_limits_max_deg` to the allowed motion subset for that setup.

There is intentionally no fallback to the compas robot model here.
If limits are missing from the profile, that is a configuration error.

Any consumer that needs a final safety clamp should go through this
module so planner-side and RobotIO-side behavior stays aligned.
"""

from __future__ import annotations

import math
from typing import Iterable


def resolve_joint_limits_rad(robot_tune: dict | None, *, axes: int = 6) -> tuple[list[float], list[float]]:
    """Resolve hard joint limits from profile tuning only.

    Preferred profile fields are `q_limits_min_deg` / `q_limits_max_deg`.
    Legacy `*_rad` fields are still accepted for back-compat, but the
    values must still come from the profile.
    """
    tune = robot_tune or {}
    if isinstance(tune.get("q_limits_min_deg"), (list, tuple)):
        q_min = _normalize_limits_deg(tune.get("q_limits_min_deg"), axes)
    elif isinstance(tune.get("q_limits_min_rad"), (list, tuple)):
        q_min = _normalize_limits_rad(tune.get("q_limits_min_rad"), axes)
    else:
        raise ValueError("tuning.robot.q_limits_min_deg is required")

    if isinstance(tune.get("q_limits_max_deg"), (list, tuple)):
        q_max = _normalize_limits_deg(tune.get("q_limits_max_deg"), axes)
    elif isinstance(tune.get("q_limits_max_rad"), (list, tuple)):
        q_max = _normalize_limits_rad(tune.get("q_limits_max_rad"), axes)
    else:
        raise ValueError("tuning.robot.q_limits_max_deg is required")
    return q_min, q_max


def clamp_joint_target_rad(q: Iterable[float], q_min: Iterable[float], q_max: Iterable[float], *, axes: int = 6) -> list[float]:
    """Clamp a joint target vector to the supplied hard limits."""
    values = [float(v) for v in list(q)[:axes]]
    while len(values) < axes:
        values.append(0.0)
    lower = [float(v) for v in list(q_min)[:axes]]
    upper = [float(v) for v in list(q_max)[:axes]]
    return [max(lo, min(hi, val)) for val, lo, hi in zip(values, lower, upper)]


def _normalize_limits_rad(values: object, axes: int) -> list[float]:
    if not isinstance(values, (list, tuple)):
        raise ValueError("joint limits must be a list or tuple")
    out = [float(v) for v in values[:axes]]
    if len(out) < axes:
        raise ValueError(f"joint limits must provide {axes} values")
    return out


def _normalize_limits_deg(values: object, axes: int) -> list[float]:
    if not isinstance(values, (list, tuple)):
        raise ValueError("joint limits must be a list or tuple")
    out = [math.radians(float(v)) for v in values[:axes]]
    if len(out) < axes:
        raise ValueError(f"joint limits must provide {axes} values")
    return out
