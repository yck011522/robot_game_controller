"""Unit tests for the absolute-degree proximity zone builder.

These lock the red/green/grey band semantics that both the gamemaster UI and
the external display receivers rely on as a single ground truth.

Run:
    conda activate game
    $env:PYTHONPATH='src'; python -m pytest tests/test_prox_zones.py -q
"""

from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
SRC = REPO_ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from apps.game_controller import published_states as ps  # noqa: E402


# Standard probe fan: -10..-1 then 1..10 degrees (no 0 sample), 1 deg step.
_OFFSETS = [float(d) for d in (list(range(-10, 0)) + list(range(1, 11)))]


def _hits_at(*offsets: float) -> list[bool]:
    """Build a hit mask over _OFFSETS, True at the given offsets."""
    wanted = set(offsets)
    return [off in wanted for off in _OFFSETS]


def test_no_hits_spans_full_tested_window_with_no_red() -> None:
    zone = ps._prox_zone_for_axis(0.0, _OFFSETS, _hits_at(), age_ticks=0)
    assert zone["valid"] is True
    # Green fills the whole tested window (furthest probe +/- half a step).
    assert zone["free_min_deg"] == -10.5
    assert zone["free_max_deg"] == 10.5
    assert zone["blocked_above_till_deg"] is None
    assert zone["blocked_below_till_deg"] is None


def test_hit_above_only_red_above_green_below() -> None:
    # Nearest hit above is +4; +6 also hit but the band collapses to nearest.
    zone = ps._prox_zone_for_axis(0.0, _OFFSETS, _hits_at(4.0, 6.0), age_ticks=0)
    assert zone["free_max_deg"] == 3.5            # +4 - half step
    assert zone["blocked_above_till_deg"] == 10.5  # tested top edge
    assert zone["free_min_deg"] == -10.5           # free all the way down
    assert zone["blocked_below_till_deg"] is None


def test_hit_below_only_red_below_green_above() -> None:
    # Nearest hit below is -3 (closest to current); -7 also hit.
    zone = ps._prox_zone_for_axis(0.0, _OFFSETS, _hits_at(-3.0, -7.0), age_ticks=0)
    assert zone["free_min_deg"] == -2.5            # -3 + half step
    assert zone["blocked_below_till_deg"] == -10.5  # tested bottom edge
    assert zone["free_max_deg"] == 10.5
    assert zone["blocked_above_till_deg"] is None


def test_hits_both_sides_red_green_red() -> None:
    zone = ps._prox_zone_for_axis(0.0, _OFFSETS, _hits_at(-3.0, 4.0), age_ticks=0)
    assert zone["free_min_deg"] == -2.5
    assert zone["free_max_deg"] == 3.5
    assert zone["blocked_below_till_deg"] == -10.5
    assert zone["blocked_above_till_deg"] == 10.5


def test_absolute_anchor_offsets_all_edges() -> None:
    # A non-zero current angle shifts every edge by the anchor; no addition is
    # required by the receiver because the published edges are already absolute.
    zone = ps._prox_zone_for_axis(30.0, _OFFSETS, _hits_at(-3.0, 4.0), age_ticks=0)
    assert zone["free_min_deg"] == 27.5
    assert zone["free_max_deg"] == 33.5
    assert zone["blocked_below_till_deg"] == 19.5
    assert zone["blocked_above_till_deg"] == 40.5


def test_stale_axis_is_invalid() -> None:
    zone = ps._prox_zone_for_axis(0.0, _OFFSETS, _hits_at(4.0), age_ticks=13)
    assert zone["valid"] is False
    assert zone["free_min_deg"] is None
    assert zone["blocked_above_till_deg"] is None


def test_edges_are_clamped_to_joint_limits() -> None:
    # Current angle near the +5 deg upper limit: the +10.5 deg tested top edge
    # must clamp to the limit so no band advertises unreachable travel.
    zone = ps._prox_zone_for_axis(
        4.0, _OFFSETS, _hits_at(), age_ticks=0, q_min_deg=-5.0, q_max_deg=5.0
    )
    assert zone["valid"] is True
    assert zone["free_min_deg"] == -5.0   # would be -6.5, clamped to -5
    assert zone["free_max_deg"] == 5.0    # would be 14.5, clamped to +5
    assert zone["blocked_above_till_deg"] is None
    assert zone["blocked_below_till_deg"] is None


def test_red_band_outer_edge_clamped_to_limit() -> None:
    # Hit above at +4 from anchor 0; tested top edge 10.5 clamps to limit 8.
    zone = ps._prox_zone_for_axis(
        0.0, _OFFSETS, _hits_at(4.0), age_ticks=0, q_min_deg=-180.0, q_max_deg=8.0
    )
    assert zone["free_max_deg"] == 3.5            # inside the limit, unchanged
    assert zone["blocked_above_till_deg"] == 8.0   # 10.5 clamped to +8
    assert zone["blocked_below_till_deg"] is None


def test_payload_uses_supplied_joint_limits() -> None:
    team_state = {
        "last_q": [0.0] * 6,
        "last_prox_probe_offsets_deg": _OFFSETS,
        "last_prox_hits": [_hits_at() for _ in range(6)],
        "last_prox_age_ticks": [0] * 6,
    }
    limits = ([-3.0] * 6, [3.0] * 6)
    zones = ps._prox_zones_payload(team_state, limits)
    for z in zones:
        assert z["free_min_deg"] == -3.0   # clamped from -10.5
        assert z["free_max_deg"] == 3.0    # clamped from +10.5


def test_payload_anchors_each_axis_on_last_q() -> None:
    import math

    team_state = {
        "last_q": [math.radians(30.0), 0.0, 0.0, 0.0, 0.0, 0.0],
        "last_prox_probe_offsets_deg": _OFFSETS,
        "last_prox_hits": [
            _hits_at(4.0),       # axis 0: hit above
            _hits_at(),          # axis 1: clear
            _hits_at(),
            _hits_at(),
            _hits_at(),
            _hits_at(),
        ],
        "last_prox_age_ticks": [0, 0, 99, 0, 0, 0],  # axis 2 stale
    }
    zones = ps._prox_zones_payload(team_state)
    assert len(zones) == 6
    # Axis 0 anchored at 30 deg, red band above.
    assert round(zones[0]["free_max_deg"], 3) == 33.5
    assert round(zones[0]["blocked_above_till_deg"], 3) == 10.5 + 30.0
    # Axis 1 clear, full green window around 0 deg.
    assert zones[1]["blocked_above_till_deg"] is None
    assert zones[1]["blocked_below_till_deg"] is None
    # Axis 2 stale -> invalid.
    assert zones[2]["valid"] is False


# Dense near band (+/-1..10, 1deg) plus sparse far probes (+/-15,20,30,45,60),
# mirroring tuning.jogging.probe_far_offsets_deg. Kept sorted as the planner does.
_SPARSE_FAR = [15.0, 20.0, 30.0, 45.0, 60.0]
_OFFSETS_FAR = sorted(
    _OFFSETS + [-d for d in _SPARSE_FAR] + list(_SPARSE_FAR)
)


def _hits_at_far(*offsets: float) -> list[bool]:
    """Build a hit mask over _OFFSETS_FAR, True at the given offsets."""
    wanted = set(offsets)
    return [off in wanted for off in _OFFSETS_FAR]


def test_sparse_far_no_hits_green_reaches_far_edge() -> None:
    # With no hits the green window spans to the outermost sparse probe (+/-60)
    # plus half the *dense* step (0.5), so the broad range is visualized.
    zone = ps._prox_zone_for_axis(0.0, _OFFSETS_FAR, _hits_at_far(), age_ticks=0)
    assert zone["valid"] is True
    assert zone["free_min_deg"] == -60.5
    assert zone["free_max_deg"] == 60.5
    assert zone["blocked_above_till_deg"] is None
    assert zone["blocked_below_till_deg"] is None


def test_sparse_far_hit_uses_dense_half_step_and_reds_to_far_edge() -> None:
    # A single far hit at +20 collapses red from just inside +20 out to the
    # tested top edge (+60.5). half_step stays 0.5 (min dense spacing), so the
    # green/red boundary is +19.5 even though the far probes are 5-15deg apart.
    zone = ps._prox_zone_for_axis(0.0, _OFFSETS_FAR, _hits_at_far(20.0), age_ticks=0)
    assert zone["free_max_deg"] == 19.5
    assert zone["blocked_above_till_deg"] == 60.5
    assert zone["free_min_deg"] == -60.5
    assert zone["blocked_below_till_deg"] is None


def test_sparse_far_near_hit_dominates_far_hit() -> None:
    # Near hit (+3) and far hit (+45) both present: band collapses to nearest.
    zone = ps._prox_zone_for_axis(0.0, _OFFSETS_FAR, _hits_at_far(3.0, 45.0), age_ticks=0)
    assert zone["free_max_deg"] == 2.5
    assert zone["blocked_above_till_deg"] == 60.5
