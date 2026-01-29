"""
agents/intersection_agent.py

2-phase Intersection Agent (NO ML).
Chooses between phases:
- "NS" (N+S green)
- "EW" (E+W green)

Uses pressure + hysteresis to avoid oscillations.
Adds a soft max-wait emergency switch to reduce peak waiting time.
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
    # Pressure weights (tunable)
    W_QUEUE: float = 1.0
    W_MAXWAIT: float = 0.25  # big wait should matter, but not dominate

    # Hysteresis margin: require other phase to be better by this much to switch
    SWITCH_MARGIN: float = 3.0

    # NEW: Soft emergency threshold to reduce max-wait spikes (seconds)
    SOFT_MAX_WAIT: float = 10.0

    def choose_next_phase(self, state: Dict[str, Dict[str, float]]) -> Tuple[str, int]:
        # 1) Fairness override (hard safety)
        forced_phase = self._fairness_phase(state)
        if forced_phase is not None:
            q = self._phase_queue(state, forced_phase)
            mw = self._phase_max_wait(state, forced_phase)
            return forced_phase, self._compute_green_time(q, mw)

        # 2) Soft emergency switch (reduce max-wait spikes)
        worst_dir = max(["N", "S", "E", "W"], key=lambda d: float(state[d]["max_wait_time"]))
        worst_wait = float(state[worst_dir]["max_wait_time"])
        if worst_wait >= self.SOFT_MAX_WAIT:
            soft_phase = "NS" if worst_dir in ("N", "S") else "EW"
            q = self._phase_queue(state, soft_phase)
            mw = self._phase_max_wait(state, soft_phase)

            # In emergency, keep green shorter so we relieve waiting quickly then re-evaluate
            duration = min(self._compute_green_time(q, mw), 10)
            return soft_phase, duration

        # 3) Pressure computation
        ns_pressure = self._pressure(state, "NS")
        ew_pressure = self._pressure(state, "EW")

        # 4) Cooldown on phases
        phase_info = state.get("_phase", {})
        ns_since = float(phase_info.get("NS_since_green", 999999.0))
        ew_since = float(phase_info.get("EW_since_green", 999999.0))

        if ns_since < DIRECTION_COOLDOWN:
            ns_pressure -= 1000
        if ew_since < DIRECTION_COOLDOWN:
            ew_pressure -= 1000

        # 5) Decide best pressure phase
        best = "NS" if ns_pressure >= ew_pressure else "EW"

        # 6) Hysteresis: if pressures are close, prefer the phase that has been waiting longer
        if abs(ns_pressure - ew_pressure) < self.SWITCH_MARGIN:
            ns_since = float(phase_info.get("NS_since_green", 999999.0))
            ew_since = float(phase_info.get("EW_since_green", 999999.0))
            best = "NS" if ns_since >= ew_since else "EW"

        q = self._phase_queue(state, best)
        mw = self._phase_max_wait(state, best)
        return best, self._compute_green_time(q, mw)

    def _pressure(self, state: Dict[str, Dict[str, float]], phase: str) -> float:
        q = self._phase_queue(state, phase)
        mw = self._phase_max_wait(state, phase)
        return (self.W_QUEUE * q) + (self.W_MAXWAIT * mw)

    def _fairness_phase(self, state: Dict[str, Dict[str, float]]) -> str | None:
        worst_dir = None
        worst_wait = -1.0
        for d in ["N", "S", "E", "W"]:
            mw = float(state[d]["max_wait_time"])
            if mw > worst_wait:
                worst_wait = mw
                worst_dir = d

        if worst_wait >= MAX_WAIT_THRESHOLD:
            return "NS" if worst_dir in ("N", "S") else "EW"
        return None

    def _phase_queue(self, state: Dict[str, Dict[str, float]], phase: str) -> int:
        if phase == "NS":
            return int(state["N"]["queue_length"] + state["S"]["queue_length"])
        return int(state["E"]["queue_length"] + state["W"]["queue_length"])

    def _phase_max_wait(self, state: Dict[str, Dict[str, float]], phase: str) -> float:
        if phase == "NS":
            return float(max(state["N"]["max_wait_time"], state["S"]["max_wait_time"]))
        return float(max(state["E"]["max_wait_time"], state["W"]["max_wait_time"]))

    def _compute_green_time(self, queue_len: int, max_wait: float) -> int:
        """
        Slightly conservative greens to reduce max wait:
        shorter cycles = lower average wait, but not too short to hurt throughput.
        """
        green = BASE_GREEN_TIME + queue_len * GREEN_TIME_PER_VEHICLE

        # small wait boost
        if max_wait >= 50:
            green += 4
        if max_wait >= 100:
            green += 4

        green = max(MIN_GREEN_TIME, green)
        green = min(MAX_GREEN_TIME, green)
        return int(green)
