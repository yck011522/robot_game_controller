"""LED Animation Controller — thread-based animation manager.

The LEDAnimationController manages a sequence of animations and renders them to
LED strips at a steady frame rate (~50 Hz). It provides a register model interface:
the GameController can queue animations, change colors, or set static states without
managing threads or timing.

Usage:
    controller = LEDAnimationController(serial_port='COM3')
    controller.start()
    
    # Queue an animation
    anim = FillAnimation([11, 12], target_leds=28, duration_ms=2000, color=RED)
    controller.queue_animation(anim)
    
    # Or immediately set a static color on strips
    controller.set_strip_color(11, BLUE)
    
    controller.stop()
"""

import time
import threading
import logging
from typing import Optional, List, Dict
from collections import deque

from led_serial import LEDSystem, Color, LEDStripState, OFF
from led_animations import LEDAnimation, ColorAnimation

logger = logging.getLogger(__name__)

_RENDER_HZ = 50
_RENDER_INTERVAL = 1.0 / _RENDER_HZ


class LEDAnimationController:
    """Manages LED animations with a dedicated render thread."""

    def __init__(
        self,
        serial_port: Optional[str] = None,
        baudrate: int = 921600,
        inter_command_delay_s: float = 0.002,
        debug_hex: bool = False,
    ):
        """
        Args:
            serial_port: RS485 port (auto-detected if None)
            baudrate: RS485 baud rate (default 921600)
            inter_command_delay_s: Delay between RS485 packets for parser stability
            debug_hex: If True, logs outgoing packet bytes
        """
        self._serial = LEDSystem(
            serial_port=serial_port,
            baudrate=baudrate,
            inter_command_delay_s=inter_command_delay_s,
            debug_hex=debug_hex,
        )
        self._animation_queue: deque[LEDAnimation] = deque()
        self._current_animation: Optional[LEDAnimation] = None

        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._lock = threading.Lock()

        self._start_time_ms: Optional[float] = None
        self._frame_count = 0

    # ─────────────────────────────────────────────────────────────────────────
    # Lifecycle
    # ─────────────────────────────────────────────────────────────────────────

    def start(self) -> bool:
        """Start serial connection and render thread. Return True if successful."""
        if not self._serial.start():
            logger.error("Failed to start LED serial system")
            return False

        self._start_time_ms = int(time.time() * 1000)
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._render_loop,
            name="led-animation",
            daemon=True,
        )
        self._thread.start()
        logger.info("LEDAnimationController started")
        return True

    def stop(self) -> None:
        """Stop render thread and close serial connection."""
        self._stop_event.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        self._serial.stop()
        logger.info("LEDAnimationController stopped")

    # ─────────────────────────────────────────────────────────────────────────
    # Public API: Animation Queue
    # ─────────────────────────────────────────────────────────────────────────

    def queue_animation(self, animation: LEDAnimation) -> None:
        """Queue an animation to play after the current one finishes."""
        with self._lock:
            self._animation_queue.append(animation)

    def clear_animations(self) -> None:
        """Clear the animation queue (stops all pending animations)."""
        with self._lock:
            self._animation_queue.clear()
            self._current_animation = None

    # ─────────────────────────────────────────────────────────────────────────
    # Public API: Direct Strip Control
    # ─────────────────────────────────────────────────────────────────────────

    def set_strip_color(self, strip_id: int, color: Color) -> None:
        """Set a single strip to a static color (short-lived animation)."""
        self.queue_animation(ColorAnimation([strip_id], color, duration_ms=10.0))

    def set_strip_colors_batch(self, color_map: Dict[int, Color]) -> None:
        """
        Set multiple strips to colors in a single batch.
        
        Args:
            color_map: Dict mapping strip_id → Color
        """
        for strip_id, color in color_map.items():
            self._serial.set_strip_color(strip_id, color)

    def set_all_strips_color(self, color: Color) -> None:
        """Set all strips to a single color."""
        all_strips = list(self._serial.strips.keys())
        self.queue_animation(ColorAnimation(all_strips, color, duration_ms=10.0))

    def get_strip_state(self, strip_id: int) -> Optional[LEDStripState]:
        """Get current state of a strip (snapshot)."""
        try:
            return self._serial.get_strip_state(strip_id)
        except ValueError:
            return None

    def get_all_strip_states(self) -> Dict[int, LEDStripState]:
        """Get snapshot of all strips."""
        return self._serial.get_all_strip_states()

    # ─────────────────────────────────────────────────────────────────────────
    # Private: Render Loop
    # ─────────────────────────────────────────────────────────────────────────

    def _render_loop(self) -> None:
        """Render thread: advance animations and send frames at 50 Hz."""
        while not self._stop_event.is_set():
            start = time.time()

            self._render_frame()

            elapsed = time.time() - start
            if elapsed < _RENDER_INTERVAL:
                time.sleep(_RENDER_INTERVAL - elapsed)

    def _render_frame(self) -> None:
        """Render one frame: pop animations, compute state, send to serial."""
        current_time_ms = int((time.time() - self._start_time_ms / 1000.0) * 1000)

        with self._lock:
            # Advance to next animation if current is done
            if self._current_animation is None:
                if self._animation_queue:
                    self._current_animation = self._animation_queue.popleft()
                    self._current_animation.start(current_time_ms)
            else:
                if self._current_animation.is_done(current_time_ms):
                    if self._animation_queue:
                        self._current_animation = self._animation_queue.popleft()
                        self._current_animation.start(current_time_ms)
                    else:
                        self._current_animation = None

            # Render current animation
            if self._current_animation:
                states = self._current_animation.frame(current_time_ms)
                for strip_id, state in states.items():
                    self._serial.strips[strip_id].set_colors(state.leds)
                    self._serial._queue_strip_command(strip_id)

        self._frame_count += 1
