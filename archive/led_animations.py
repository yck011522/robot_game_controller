"""LED animation classes.

An animation is a time-based sequence that updates LED strip states over time.
Animations are composed and scheduled by LEDAnimationController.
"""

import time
import math
from abc import ABC, abstractmethod
from typing import Dict, List, Tuple, Optional
from dataclasses import dataclass

from led_serial import Color, LEDStripState, _LEDS_PER_STRIP, RED, BLUE, GREEN, WHITE, OFF


# ─────────────────────────────────────────────────────────────────────────────
# Base Animation Class
# ─────────────────────────────────────────────────────────────────────────────

class LEDAnimation(ABC):
    """Base class for all LED animations.
    
    An animation is associated with specific strip(s) and plays over a duration.
    Each frame call returns the updated state for those strips.
    """

    def __init__(self, strip_ids: List[int], duration_ms: float):
        """
        Args:
            strip_ids: List of strip IDs controlled by this animation
            duration_ms: Duration of animation in milliseconds
        """
        self.strip_ids = strip_ids
        self.duration_ms = duration_ms
        self._start_time_ms: Optional[float] = None

    def start(self, current_time_ms: float) -> None:
        """Mark animation as started at the given time."""
        self._start_time_ms = current_time_ms

    def get_elapsed_ms(self, current_time_ms: float) -> float:
        """Get elapsed time since start."""
        if self._start_time_ms is None:
            return 0.0
        return current_time_ms - self._start_time_ms

    def is_done(self, current_time_ms: float) -> bool:
        """Check if animation has finished."""
        return self.get_elapsed_ms(current_time_ms) >= self.duration_ms

    @abstractmethod
    def frame(self, current_time_ms: float) -> Dict[int, LEDStripState]:
        """
        Compute frame for all controlled strips at the given time.
        
        Args:
            current_time_ms: Current time in milliseconds
            
        Returns:
            Dict mapping strip_id → LEDStripState
        """
        pass


# ─────────────────────────────────────────────────────────────────────────────
# Fill Animation (for testing)
# ─────────────────────────────────────────────────────────────────────────────

class FillAnimation(LEDAnimation):
    """
    Simple animation: fill LEDs progressively from 0 to num_leds over duration.
    
    Use case: Score visualization (represents progress/fill level).
    For testing: Set to 100% or 28 LEDs to see a full fill animation.
    """

    def __init__(
        self,
        strip_ids: List[int],
        target_leds: int = _LEDS_PER_STRIP,
        duration_ms: float = 1000.0,
        color: Color = GREEN,
        off_color: Color = OFF,
    ):
        """
        Args:
            strip_ids: Strips to animate
            target_leds: Number of LEDs to reach (0–28)
            duration_ms: Duration in ms
            color: Color for filled LEDs
            off_color: Color for unfilled LEDs
        """
        super().__init__(strip_ids, duration_ms)
        self.target_leds = max(0, min(_LEDS_PER_STRIP, target_leds))
        self.color = color
        self.off_color = off_color

    def frame(self, current_time_ms: float) -> Dict[int, LEDStripState]:
        """Linearly interpolate from 0 to target_leds."""
        elapsed = self.get_elapsed_ms(current_time_ms)
        progress = min(1.0, elapsed / self.duration_ms)
        current_leds = int(progress * self.target_leds)

        states = {}
        for strip_id in self.strip_ids:
            state = LEDStripState(strip_id)
            state.leds = [self.color] * current_leds + [self.off_color] * (
                _LEDS_PER_STRIP - current_leds
            )
            states[strip_id] = state

        return states


# ─────────────────────────────────────────────────────────────────────────────
# Static Color Animation
# ─────────────────────────────────────────────────────────────────────────────

class ColorAnimation(LEDAnimation):
    """Static color animation: set strips to a fixed color instantly."""

    def __init__(self, strip_ids: List[int], color: Color, duration_ms: float = 100.0):
        """
        Args:
            strip_ids: Strips to set
            color: Fixed color
            duration_ms: How long to hold (default 100 ms, can be very short)
        """
        super().__init__(strip_ids, duration_ms)
        self.color = color

    def frame(self, current_time_ms: float) -> Dict[int, LEDStripState]:
        """Return all LEDs set to color."""
        states = {}
        for strip_id in self.strip_ids:
            state = LEDStripState(strip_id)
            state.leds = [self.color] * _LEDS_PER_STRIP
            states[strip_id] = state

        return states


# ─────────────────────────────────────────────────────────────────────────────
# Pulse Animation
# ─────────────────────────────────────────────────────────────────────────────

class PulseAnimation(LEDAnimation):
    """Pulse animation: brightness oscillates between two colors."""

    def __init__(
        self,
        strip_ids: List[int],
        color1: Color,
        color2: Color = OFF,
        duration_ms: float = 1000.0,
    ):
        """
        Args:
            strip_ids: Strips to animate
            color1: Primary color (at 0% and 100% of cycle)
            color2: Secondary color (at 50% of cycle)
            duration_ms: Full cycle duration in ms
        """
        super().__init__(strip_ids, duration_ms)
        self.color1 = color1
        self.color2 = color2

    def frame(self, current_time_ms: float) -> Dict[int, LEDStripState]:
        """Smoothly interpolate between colors based on sine wave."""
        elapsed = self.get_elapsed_ms(current_time_ms)
        # Repeat cycle indefinitely until is_done() is called
        cycle_progress = (elapsed % self.duration_ms) / self.duration_ms
        # Use sine wave for smooth pulsing (0 to 1 to 0)
        blend = (math.sin(cycle_progress * 2 * math.pi - math.pi / 2) + 1) / 2

        # Blend between color1 and color2
        r = int(self.color1.r * (1 - blend) + self.color2.r * blend)
        g = int(self.color1.g * (1 - blend) + self.color2.g * blend)
        b = int(self.color1.b * (1 - blend) + self.color2.b * blend)
        blend_color = Color(r, g, b)

        states = {}
        for strip_id in self.strip_ids:
            state = LEDStripState(strip_id)
            state.leds = [blend_color] * _LEDS_PER_STRIP
            states[strip_id] = state

        return states
