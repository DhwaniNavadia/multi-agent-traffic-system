"""
simulation/signal.py

Traffic signal logic for ONE intersection.

In Phase-1:
- one direction is GREEN at a time (N/S/E/W)
- signal stays green for 'remaining_green' seconds
- when timer hits 0, the environment/agent can choose the next green direction
"""

from __future__ import annotations
from dataclasses import dataclass


@dataclass
class TrafficSignal:
    # Current green direction: 'N', 'S', 'E', 'W'
    green_direction: str = "N"

    # How many seconds of green are left
    remaining_green: int = 10

    def tick(self) -> None:
        """Advance the signal by 1 second."""
        if self.remaining_green > 0:
            self.remaining_green -= 1

    def is_expired(self) -> bool:
        """True if green time is finished and we need a new decision."""
        return self.remaining_green <= 0

    def set_green(self, direction: str, duration: int) -> None:
        """Apply a new green direction and duration."""
        self.green_direction = direction
        self.remaining_green = int(duration)
