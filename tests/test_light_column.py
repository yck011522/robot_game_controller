"""Automated tests for the arena light-column subsystem.

Run (no pytest in the env; use unittest):

    $env:PYTHONPATH = "src"
    & C:/Users/yck01/miniconda3/envs/game/python.exe -m unittest tests.test_light_column

Covers:
    * pure frame helpers (solid / scale / mix / two_color_split),
    * RS485 frame framing (build_strip_frame),
    * layout loading from the device config,
    * per-stage controller renderers (idle / play blackout / play bar /
      reset / daydream breathing / tutorial / conclusion winner crown),
    * the paced round-robin send scheduler (pump).

A FakeTransport stands in for real serial hardware: it mirrors the real
strip->port routing and records every frame written so the round-robin and
spacing behaviour can be asserted deterministically.
"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

_SRC = Path(__file__).resolve().parent.parent / "src"
if str(_SRC) not in sys.path:
    sys.path.insert(0, str(_SRC))

from subsystems.light_column import frames  # noqa: E402
from subsystems.light_column.controller import (  # noqa: E402
    LedColumnController,
    LightColumnConfig,
)
from subsystems.light_column.frames import (  # noqa: E402
    BLUE,
    OFF,
    RED,
    WHITE,
    Color,
    mix,
    scale,
    solid,
    two_color_split,
)
from subsystems.light_column.layout import load_light_column_layout  # noqa: E402
from subsystems.light_column.transport import build_strip_frame  # noqa: E402


class FakeTransport:
    """In-memory stand-in for LedTransport that records written frames.

    Mirrors the real strip->port routing (address order, channels 1 then 2)
    so round-robin assertions match production behaviour. ``writes`` collects
    ``(port, strip_id)`` tuples in send order, decoded from the frame bytes.
    """

    def __init__(self, layout) -> None:
        self._strips_for_port: dict[str, tuple[int, ...]] = {}
        for port, addresses in layout.controller_addresses_by_port.items():
            strips: list[int] = []
            for address in addresses:
                for channel in (1, 2):
                    strips.append(address * 10 + channel)
            self._strips_for_port[port] = tuple(strips)
        self._ports = tuple(layout.serial_ports)
        self.writes: list[tuple[str, int]] = []

    def ports(self) -> tuple[str, ...]:
        return self._ports

    def strips_for_port(self, port: str) -> tuple[int, ...]:
        return self._strips_for_port.get(port, ())

    def write(self, port: str, frame: bytes) -> bool:
        # Decode the strip id back out of the frame for assertion convenience.
        addr = (frame[5] << 8) | frame[6]
        channel = frame[7]
        self.writes.append((port, addr * 10 + channel))
        return True


def _make_controller():
    """Build a controller wired to a FakeTransport and the real layout."""

    layout = load_light_column_layout()
    config = LightColumnConfig()
    transport = FakeTransport(layout)
    controller = LedColumnController(transport, layout, config)
    return controller, transport, layout, config


class FramesTest(unittest.TestCase):
    def test_solid(self) -> None:
        self.assertEqual(solid(RED, 4), [RED, RED, RED, RED])
        self.assertEqual(solid(RED, 0), [])

    def test_scale_endpoints(self) -> None:
        self.assertEqual(scale(BLUE, 1.0), BLUE)
        self.assertEqual(scale(BLUE, 0.0), Color(0, 0, 0))
        self.assertEqual(scale(Color(200, 100, 50), 0.5), Color(100, 50, 25))

    def test_mix(self) -> None:
        self.assertEqual(mix(RED, BLUE, 0.0), RED)
        self.assertEqual(mix(RED, BLUE, 1.0), BLUE)
        self.assertEqual(mix(Color(0, 0, 0), Color(100, 0, 0), 0.5), Color(50, 0, 0))

    def test_two_color_split_full_and_empty(self) -> None:
        self.assertEqual(two_color_split(RED, OFF, 1.0, 4), [RED, RED, RED, RED])
        self.assertEqual(two_color_split(RED, OFF, 0.0, 4), [OFF, OFF, OFF, OFF])

    def test_two_color_split_sharp_half(self) -> None:
        # 2 of 4 filled, no fractional remainder -> crisp split, no blend LED.
        self.assertEqual(
            two_color_split(RED, OFF, 0.5, 4), [RED, RED, OFF, OFF]
        )

    def test_two_color_split_single_boundary_led(self) -> None:
        # 2.5 of 4 filled -> indices 0,1 solid, index 2 blended, index 3 empty.
        result = two_color_split(RED, OFF, 0.625, 4)
        self.assertEqual(result[0], RED)
        self.assertEqual(result[1], RED)
        self.assertEqual(result[2], mix(OFF, RED, 0.5))
        self.assertEqual(result[3], OFF)


class FrameFramingTest(unittest.TestCase):
    def test_build_strip_frame_structure(self) -> None:
        colors = [Color(1, 2, 3)] * 28
        frame = build_strip_frame(42, colors)
        # header
        self.assertEqual(frame[0:3], bytes([0xDD, 0x55, 0xEE]))
        # group broadcast 0
        self.assertEqual(frame[3:5], bytes([0x00, 0x00]))
        # device address 4, channel 2
        self.assertEqual(frame[5:7], bytes([0x00, 0x04]))
        self.assertEqual(frame[7], 0x02)
        # function (display) + WS2811 type + reserved
        self.assertEqual(frame[8], 0x99)
        self.assertEqual(frame[9], 0x02)
        self.assertEqual(frame[10:12], bytes([0x00, 0x00]))
        # payload length = 28 * 3 = 84
        self.assertEqual(frame[12:14], bytes([0x00, 0x54]))
        # repeat count
        self.assertEqual(frame[14:16], bytes([0x00, 0x01]))
        # first pixel in RGB wire order
        self.assertEqual(frame[16:19], bytes([1, 2, 3]))
        # tail
        self.assertEqual(frame[-2:], bytes([0xAA, 0xBB]))
        self.assertEqual(len(frame), 16 + 84 + 2)


class LayoutTest(unittest.TestCase):
    def test_layout_partitions_sixteen_strips(self) -> None:
        layout = load_light_column_layout()
        self.assertEqual(len(layout.all_strips), 16)
        self.assertEqual(layout.serial_ports, ("COM45", "COM39", "COM42"))
        self.assertEqual(layout.leds_per_strip, 28)
        # Team strips together cover every strip exactly once.
        combined = layout.team_strips["a"] + layout.team_strips["b"]
        self.assertEqual(sorted(combined), sorted(layout.all_strips))


class RendererTest(unittest.TestCase):
    def test_no_state_all_off(self) -> None:
        controller, _, layout, _ = _make_controller()
        controller.update(now_mono=1.0, now_wall=0.0)
        for strip_id in layout.all_strips:
            self.assertEqual(
                controller.strip_colors(strip_id), solid(OFF, layout.leds_per_strip)
            )

    def test_idle_solid_team_colors(self) -> None:
        controller, _, layout, config = _make_controller()
        controller.set_state({"active_stage": "idle", "teams": {}})
        controller.update(now_mono=1.0, now_wall=0.0)
        a_strip = layout.team_strips["a"][0]
        b_strip = layout.team_strips["b"][0]
        self.assertEqual(
            controller.strip_colors(a_strip), solid(config.team_colors["a"], 28)
        )
        self.assertEqual(
            controller.strip_colors(b_strip), solid(config.team_colors["b"], 28)
        )

    def test_play_startup_blackout(self) -> None:
        controller, _, layout, _ = _make_controller()
        state = {
            "active_stage": "play",
            "teams": {"a": {"collision": {"final_scalar": 0.5}}},
        }
        controller.set_state(state)
        # First update marks stage entry; elapsed 0 < blackout window.
        controller.update(now_mono=100.0, now_wall=0.0)
        a_strip = layout.team_strips["a"][0]
        self.assertEqual(controller.strip_colors(a_strip), solid(OFF, 28))

    def test_play_speed_bar(self) -> None:
        controller, _, layout, config = _make_controller()
        state = {
            "active_stage": "play",
            "teams": {"a": {"collision": {"final_scalar": 0.5}}},
        }
        controller.set_state(state)
        controller.update(now_mono=100.0, now_wall=0.0)  # enter stage
        controller.update(now_mono=100.6, now_wall=0.0)  # past blackout
        a_strip = layout.team_strips["a"][0]
        self.assertEqual(
            controller.strip_colors(a_strip),
            two_color_split(config.team_colors["a"], OFF, 0.5, 28),
        )

    def test_reset_all_white(self) -> None:
        controller, _, layout, _ = _make_controller()
        controller.set_state({"active_stage": "reset", "teams": {}})
        controller.update(now_mono=1.0, now_wall=0.0)
        for strip_id in layout.all_strips:
            self.assertEqual(controller.strip_colors(strip_id), solid(WHITE, 28))

    def test_daydream_breathing_range(self) -> None:
        controller, _, layout, config = _make_controller()
        controller.set_state({"active_stage": "daydreaming", "teams": {}})
        a_strip = layout.team_strips["a"][0]
        # phase 0 -> dimmest (min brightness).
        controller.update(now_mono=1.0, now_wall=0.0)
        dim = controller.strip_colors(a_strip)[0]
        self.assertEqual(dim, scale(config.team_colors["a"], config.breathing_min_brightness))
        # phase 0.5 (half of 3 s period) -> brightest (full color).
        controller.update(now_mono=2.0, now_wall=config.breathing_period_s / 2.0)
        bright = controller.strip_colors(a_strip)[0]
        self.assertEqual(bright, config.team_colors["a"])

    def test_tutorial_indicator_and_progress(self) -> None:
        controller, _, layout, config = _make_controller()
        # Player A1 (dial index 0) scrolled halfway through the tutorial range.
        state = {
            "active_stage": "tutorial",
            "teams": {
                "a": {
                    "haptic": {
                        "tutorial_progress_pct": [50.0, 0, 0, 0, 0, 0]
                    }
                }
            },
        }
        controller.set_state(state)
        controller.update(now_mono=1.0, now_wall=0.0)
        # Indicator strips show solid team color.
        for strip_id in layout.team_indicator_strips["a"]:
            self.assertEqual(
                controller.strip_colors(strip_id), solid(config.team_colors["a"], 28)
            )
        # Player A1's strip is a half-filled progress bar.
        a1_strip = layout.tutorial_player_strips["a"]["A1"]
        self.assertEqual(
            controller.strip_colors(a1_strip),
            two_color_split(config.team_colors["a"], OFF, 0.5, 28),
        )

    def test_conclusion_crowns_winner(self) -> None:
        controller, _, layout, config = _make_controller()
        state = {
            "active_stage": "conclusion",
            "teams": {
                "a": {"summed_score": 240, "buckets": [], "conclusion": {"done": True}},
                "b": {"summed_score": 100, "buckets": [], "conclusion": {"done": True}},
            },
        }
        controller.set_state(state)
        # Enter conclusion; both done -> latch completion timestamp.
        controller.update(now_mono=200.0, now_wall=0.0)
        # After the post-count hold, every strip becomes the winner's color.
        controller.update(now_mono=200.0 + config.conclusion_post_count_hold_s + 0.1, now_wall=0.0)
        winner = config.team_colors["a"]
        for strip_id in layout.all_strips:
            self.assertEqual(controller.strip_colors(strip_id), solid(winner, 28))


class PumpTest(unittest.TestCase):
    def test_pump_paces_and_round_robins(self) -> None:
        controller, transport, layout, config = _make_controller()
        controller.set_state({"active_stage": "idle", "teams": {}})
        controller.update(now_mono=0.0, now_wall=0.0)

        # First pump: one frame per open port.
        controller.pump(now_mono=0.0)
        self.assertEqual(len(transport.writes), len(layout.serial_ports))
        ports_hit = {port for port, _ in transport.writes}
        self.assertEqual(ports_hit, set(layout.serial_ports))

        # Immediate second pump: spacing not elapsed -> nothing sent.
        controller.pump(now_mono=0.0)
        self.assertEqual(len(transport.writes), len(layout.serial_ports))

        # After the inter-command delay: next strip per port (round-robin).
        before = len(transport.writes)
        controller.pump(now_mono=config.inter_command_delay_s)
        self.assertEqual(len(transport.writes) - before, len(layout.serial_ports))

        # The two frames sent on COM45 should be its first two strips in order.
        com45_writes = [strip for port, strip in transport.writes if port == "COM45"]
        self.assertEqual(com45_writes[:2], [11, 12])


if __name__ == "__main__":
    unittest.main()
