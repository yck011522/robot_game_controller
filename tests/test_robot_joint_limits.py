"""Focused regression tests for shared UR10e hard-limit helpers."""

from __future__ import annotations

import math
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
SRC = REPO_ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from subsystems.robot.joint_limits import (  # noqa: E402
    clamp_joint_target_rad,
    resolve_joint_limits_rad,
)


def main() -> int:
    try:
        resolve_joint_limits_rad({})
    except ValueError as exc:
        assert "q_limits_min_deg" in str(exc)
    else:
        raise AssertionError("missing joint limits should raise ValueError")

    q_min, q_max = resolve_joint_limits_rad(
        {
            "q_limits_min_deg": [-360.0, -360.0, -180.0, -360.0, -360.0, -360.0],
            "q_limits_max_deg": [360.0, 360.0, 180.0, 360.0, 360.0, 360.0],
        }
    )
    assert len(q_min) == 6
    assert len(q_max) == 6
    assert math.isclose(q_min[0], -2.0 * math.pi)
    assert math.isclose(q_max[0], 2.0 * math.pi)
    assert math.isclose(q_min[2], -1.0 * math.pi)
    assert math.isclose(q_max[2], 1.0 * math.pi)

    q = clamp_joint_target_rad([99.0, -99.0, 99.0, 0.1, -0.2, 0.3], q_min, q_max)
    assert math.isclose(q[0], q_max[0])
    assert math.isclose(q[1], q_min[1])
    assert math.isclose(q[2], q_max[2])
    assert math.isclose(q[3], 0.1)

    override_min, override_max = resolve_joint_limits_rad(
        {
            "q_limits_min_deg": [-10.0, -20.0, -30.0, -40.0, -50.0, -60.0],
            "q_limits_max_deg": [10.0, 20.0, 30.0, 40.0, 50.0, 60.0],
        }
    )
    q_override = clamp_joint_target_rad([9.0, -9.0, 0.5, 0.0, 0.0, 0.0], override_min, override_max)
    assert math.isclose(q_override[0], math.radians(10.0))
    assert math.isclose(q_override[1], math.radians(-20.0))
    assert math.isclose(q_override[2], 0.5)

    legacy_min, legacy_max = resolve_joint_limits_rad(
        {
            "q_limits_min_rad": [-1.0, -2.0, -3.0, -4.0, -5.0, -6.0],
            "q_limits_max_rad": [1.0, 2.0, 3.0, 4.0, 5.0, 6.0],
        }
    )
    assert legacy_min == [-1.0, -2.0, -3.0, -4.0, -5.0, -6.0]
    assert legacy_max == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]

    print("joint limit helper test passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())