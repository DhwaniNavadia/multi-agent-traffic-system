"""
agents/intersection_agent.py

Rule-based Intersection Agent (NO ML).

Controls ONE crossroads by deciding:
- which direction gets green
- how long green should last
while ensuring fairness (no starvation) and avoiding immediate repetition (cooldown).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Tuple

from config import (
    MIN_GREEN_TIME,
    MAX_GREEN_TIME,
    BASE_GREEN_TIME,
    GREEN_TIME_PER_VEHICLE,
    MAX_WAIT_THRESHOLD,
    DIRECTION_COOLDOWN,
)


@dataclass
class IntersectionAgent:
    """
    A simple rational agent:
    - Prioritizes lanes with higher congestion (queue + waiting)
    - Forces fairness if any lane waits too long
    - Applies a cooldown penalty to avoid giving green repeatedly to the same direction
    """

    # Weights for scoring (tunable)
    alpha_queue: float = 1.0
    beta_avg_wait: float = 0.5
    gamma_max_wait: float = 1.5

    def choose_next_phase(self, state: Dict[str, Dict[str, float]]) -> Tuple[str, int]:
        """
        Input state example:
        {
          "N": {"queue_length":..., "avg_wait_time":..., "max_wait_time":..., "since_green":...},
          ...
        }

        Returns:
          (direction, green_duration_seconds)
        """

        # 1) Fairness override: if any direction's max_wait exceeds threshold -> force it green
        forced = self._check_fairness_override(state)
        if forced is not None:
            duration = self._compute_green_time(queue_len=int(state[forced]["queue_length"]))
            return forced, duration

        # 2) Otherwise compute a priority score for each direction
        best_dir = None
        best_score = float("-inf")

        for d, info in state.items():
            q = info["queue_length"]
            avg_w = info["avg_wait_time"]
            max_w = info["max_wait_time"]
            since_green = info.get("since_green", 999999.0)

            score = (self.alpha_queue * q) + (self.beta_avg_wait * avg_w) + (self.gamma_max_wait * max_w)

            # 3) Cooldown penalty:
            # If this direction got green very recently, avoid selecting it again immediately.
            if since_green < DIRECTION_COOLDOWN:
                score -= 1000  # strong penalty to prevent immediate repetition

            if score > best_score:
                best_score = score
                best_dir = d

        assert best_dir is not None

        # 4) Compute dynamic green time for chosen direction
        duration = self._compute_green_time(queue_len=int(state[best_dir]["queue_length"]))
        return best_dir, duration

    def _check_fairness_override(self, state: Dict[str, Dict[str, float]]) -> str | None:
        """
        If any lane has waited too long, return that direction to force green.
        Otherwise return None.
        """
        worst_dir = None
        worst_wait = -1.0

        for d, info in state.items():
            mw = info["max_wait_time"]
            if mw > worst_wait:
                worst_wait = mw
                worst_dir = d

        if worst_wait >= MAX_WAIT_THRESHOLD:
            return worst_dir

        return None

    def _compute_green_time(self, queue_len: int) -> int:
        """
        Green time formula:
            green = BASE_GREEN_TIME + queue_len * GREEN_TIME_PER_VEHICLE
        clipped between MIN_GREEN_TIME and MAX_GREEN_TIME.
        """
        green = BASE_GREEN_TIME + queue_len * GREEN_TIME_PER_VEHICLE
        green = max(MIN_GREEN_TIME, green)
        green = min(MAX_GREEN_TIME, green)
        return int(green)
