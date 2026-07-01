"""Shared admin-button mapping, edge detection, and lamp policy."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol


START_RESUME = "start_resume"
SKIP = "skip"
ESTOP = "estop"
BUTTON_NAMES = (START_RESUME, SKIP, ESTOP)


@dataclass(frozen=True)
class AdminButtonConfig:
    """Runtime configuration for one HY-IO4400S-4NN admin-button unit."""

    station_label: str
    slave_address: int
    input_start_address: int
    input_count: int
    resume_input_index: int
    skip_input_index: int
    estop_input_index: int
    resume_lamp_coil_address: int
    skip_cooldown_s: float


@dataclass(frozen=True)
class ButtonSignal:
    """One logical button level plus the edge/event emitted on this sample."""

    pressed: bool
    edge: str | None
    event_id: int | None


@dataclass(frozen=True)
class AdminButtonSnapshot:
    """One normalized hardware sample plus the desired resume-lamp state."""

    station_label: str
    raw_inputs: list[bool]
    buttons: dict[str, ButtonSignal]
    resume_lamp_on: bool
    errors: list[str]


class AdminButtonDriver(Protocol):
    """Minimal hardware/simulator contract used by the button runtime."""

    def connect(self) -> None:
        """Open any resources needed by the driver."""

    def read_inputs(self) -> tuple[list[bool], list[str]]:
        """Return raw digital-input bits and any read errors."""

    def write_resume_lamp(self, on: bool) -> list[str]:
        """Set the green resume lamp relay and return any write errors."""

    def close(self) -> None:
        """Release driver resources."""


class AdminButtonRuntime:
    """Poll admin buttons, normalize levels, debounce skip, and drive the lamp."""

    def __init__(self, driver: AdminButtonDriver, config: AdminButtonConfig) -> None:
        self.driver = driver  # Hardware/simulator object that owns Modbus I/O.
        self.config = config  # Channel mapping and operator-tunable cooldowns.
        self._prev_pressed = {name: False for name in BUTTON_NAMES}  # Prior tick levels for edge detection.
        self._event_seq = 0  # Monotonic id attached to accepted rise/fall edges.
        self._last_skip_rise_mono_s = -1.0e9  # Last accepted skip press time; enforces cooldown.
        self._last_lamp_on: bool | None = None  # Last commanded coil state; avoids repeated writes.

    def tick(self, *, paused: bool, now_mono_s: float) -> AdminButtonSnapshot:
        """Read inputs once, publish edge-aware state, and update the resume lamp.

        Called by ``apps.button_controller`` every process tick. The caller
        supplies the latest ``state.full.paused`` value so the physical green
        lamp mirrors the digital resume affordance while still staying off when
        the physical e-stop is asserted.
        """

        raw_inputs, read_errors = self.driver.read_inputs()
        errors = list(read_errors)
        pressed_by_name = self._logical_buttons(raw_inputs, has_error=bool(errors))
        buttons = {
            name: self._button_signal(name, pressed_by_name[name], now_mono_s)
            for name in BUTTON_NAMES
        }
        resume_lamp_on = bool(paused) and not bool(pressed_by_name[ESTOP]) and not errors
        if self._last_lamp_on is None or resume_lamp_on != self._last_lamp_on:
            errors.extend(self.driver.write_resume_lamp(resume_lamp_on))
            if not errors:
                self._last_lamp_on = resume_lamp_on
        return AdminButtonSnapshot(
            station_label=self.config.station_label,
            raw_inputs=list(raw_inputs),
            buttons=buttons,
            resume_lamp_on=resume_lamp_on,
            errors=errors,
        )

    def close(self) -> None:
        """Turn the resume lamp off and close the underlying driver."""

        try:
            self.driver.write_resume_lamp(False)
        finally:
            self.driver.close()

    def _logical_buttons(self, raw_inputs: list[bool], *, has_error: bool) -> dict[str, bool]:
        """Convert raw DI bits into logical pressed states with e-stop fail-safe."""

        def raw(index: int) -> bool:
            return bool(raw_inputs[index]) if 0 <= index < len(raw_inputs) else False

        # The green and white buttons are normally open: HIGH means pressed.
        # The e-stop is normally closed: LOW means pressed. Read errors also
        # force e-stop pressed so GameController fails paused.
        return {
            START_RESUME: raw(self.config.resume_input_index) and not has_error,
            SKIP: raw(self.config.skip_input_index) and not has_error,
            ESTOP: (not raw(self.config.estop_input_index)) or has_error,
        }

    def _button_signal(self, name: str, pressed: bool, now_mono_s: float) -> ButtonSignal:
        """Return edge metadata for a normalized button level."""

        previous = bool(self._prev_pressed.get(name, False))
        edge = None
        event_id = None
        if pressed != previous:
            candidate_edge = "rise" if pressed else "fall"
            if name == SKIP and candidate_edge == "rise":
                elapsed = now_mono_s - self._last_skip_rise_mono_s
                if elapsed >= self.config.skip_cooldown_s:
                    edge = candidate_edge
                    self._last_skip_rise_mono_s = now_mono_s
            else:
                edge = candidate_edge
            if edge is not None:
                self._event_seq += 1
                event_id = self._event_seq
        self._prev_pressed[name] = pressed
        return ButtonSignal(pressed=pressed, edge=edge, event_id=event_id)


def snapshot_to_payload(snapshot: AdminButtonSnapshot) -> dict:
    """Convert one runtime snapshot into the public ``telem.buttons`` shape."""

    station_payload = {
        name: {
            "pressed": signal.pressed,
            "edge": signal.edge,
            "event_id": signal.event_id,
        }
        for name, signal in snapshot.buttons.items()
    }
    return {
        "stations": {
            snapshot.station_label: station_payload,
        },
        "raw_inputs": {
            snapshot.station_label: snapshot.raw_inputs,
        },
        "resume_lamp_on": snapshot.resume_lamp_on,
        "errors": snapshot.errors,
    }

