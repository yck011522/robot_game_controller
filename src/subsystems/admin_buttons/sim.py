"""Simulated admin-button driver."""

from __future__ import annotations

from subsystems.admin_buttons.common import AdminButtonConfig


class SimAdminButtonUnit:
    """Admin-button simulator with no pressed buttons and a tracked lamp state."""

    def __init__(self, config: AdminButtonConfig) -> None:
        self.config = config  # Shared mapping so simulator mirrors real shape.
        self.resume_lamp_on = False  # Last requested lamp state for tests/debugging.

    def connect(self) -> None:
        """Open simulator resources."""

        return

    def read_inputs(self) -> tuple[list[bool], list[str]]:
        """Return idle raw inputs: NO buttons low, NC e-stop high."""

        raw_inputs = [False] * self.config.input_count
        if 0 <= self.config.estop_input_index < len(raw_inputs):
            raw_inputs[self.config.estop_input_index] = True
        return raw_inputs, []

    def write_resume_lamp(self, on: bool) -> list[str]:
        """Store the requested resume-lamp state."""

        self.resume_lamp_on = bool(on)
        return []

    def close(self) -> None:
        """Release simulator resources."""

        return

