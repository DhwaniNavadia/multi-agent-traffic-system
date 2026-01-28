"""
simulation/environment.py

This module defines the simulation environment (traffic world).
For Phase-1, we simulate ONE 4-way crossroads with 4 incoming lanes:
North (N), South (S), East (E), West (W).

No cameras / IoT: traffic density is generated using ARRIVAL_RATES from config.py.
"""

from __future__ import annotations

import random
from dataclasses import dataclass, field
from typing import Dict, List

from config import (
    TIME_STEP,
    SIMULATION_TIME,
    ARRIVAL_RATES,
    VEHICLE_DISCHARGE_RATE,
)


# -------------------------
# Basic data models
# -------------------------

@dataclass
class Vehicle:
    """
    A simple vehicle model for simulation (no physics).
    """
    vid: int
    direction: str           # 'N', 'S', 'E', 'W'
    spawn_time: int          # time when vehicle was created
    wait_time: int = 0       # accumulated waiting time in seconds


@dataclass
class Lane:
    """
    Each direction has one incoming lane represented as a queue of vehicles.
    """
    direction: str
    queue: List[Vehicle] = field(default_factory=list)

    def queue_length(self) -> int:
        return len(self.queue)

    def add_vehicle(self, v: Vehicle) -> None:
        self.queue.append(v)

    def increment_waiting(self) -> None:
        """
        Increase waiting time for all vehicles currently in this lane queue.
        """
        for v in self.queue:
            v.wait_time += TIME_STEP

    def pop_vehicles(self, count: int) -> List[Vehicle]:
        """
        Remove up to 'count' vehicles from the front of the queue (they pass the intersection).
        """
        passed = self.queue[:count]
        self.queue = self.queue[count:]
        return passed


# -------------------------
# Environment (Simulation Engine)
# -------------------------

class TrafficEnvironment:
    """
    Simulation engine for one crossroads.
    Controls time, spawns vehicles, moves vehicles based on current green direction,
    and keeps basic statistics.
    """

    def __init__(self, seed: int | None = 42):
        self.time: int = 0
        self.vehicle_id_counter: int = 1

        # Create one lane per direction
        self.lanes: Dict[str, Lane] = {d: Lane(d) for d in ["N", "S", "E", "W"]}

        # Current green direction (Phase-1: only one direction is green at a time)
        self.green_direction: str = "N"  # initial default

        # Stats
        self.total_spawned: int = 0
        self.total_passed: int = 0
        self.passed_vehicles: List[Vehicle] = []  # store for metrics later

        if seed is not None:
            random.seed(seed)

    # ---------- Inputs (Synthetic traffic generation) ----------

    def spawn_vehicles(self) -> None:
        """
        Spawn vehicles based on per-direction arrival probabilities.

        For each direction:
        - generate random number r in [0,1)
        - if r < ARRIVAL_RATES[direction], spawn one vehicle in that lane this second
        """
        for d, rate in ARRIVAL_RATES.items():
            r = random.random()
            if r < rate:
                v = Vehicle(
                    vid=self.vehicle_id_counter,
                    direction=d,
                    spawn_time=self.time,
                )
                self.vehicle_id_counter += 1
                self.lanes[d].add_vehicle(v)
                self.total_spawned += 1

    # ---------- State sensing (what agent will read later) ----------

    def get_state(self) -> Dict[str, Dict[str, float]]:
        """
        Returns the observable state for decision-making.

        For each direction, provide:
        - queue_length
        - avg_wait_time
        - max_wait_time

        (In Phase-2, the intersection agent will use this to decide signal timing.)
        """
        state: Dict[str, Dict[str, float]] = {}

        for d, lane in self.lanes.items():
            q = lane.queue_length()
            if q == 0:
                avg_wait = 0.0
                max_wait = 0.0
            else:
                waits = [v.wait_time for v in lane.queue]
                avg_wait = sum(waits) / q
                max_wait = max(waits)

            state[d] = {
                "queue_length": float(q),
                "avg_wait_time": float(avg_wait),
                "max_wait_time": float(max_wait),
            }

        return state

    # ---------- Vehicle movement based on signal ----------

    def step_movement(self) -> None:
        """
        Move vehicles for the current time step.

        Rules (simple and explainable):
        - Vehicles in red lanes stay and their wait_time increments
        - Vehicles in the green lane can pass (up to VEHICLE_DISCHARGE_RATE per second)
        """
        for d, lane in self.lanes.items():
            if d == self.green_direction:
                passed = lane.pop_vehicles(VEHICLE_DISCHARGE_RATE)
                self.total_passed += len(passed)
                self.passed_vehicles.extend(passed)
            else:
                lane.increment_waiting()

    # ---------- Debug/Logging (console output) ----------

    def print_status(self) -> None:
        """
        Print a small status line for debugging and to verify the simulator works.
        """
        q_info = " ".join([f"{d}={self.lanes[d].queue_length():2d}" for d in ["N", "S", "E", "W"]])
        print(f"t={self.time:3d} | Green={self.green_direction} | {q_info} | spawned={self.total_spawned} passed={self.total_passed}")

    # ---------- Main simulation loop ----------

    def run(self, verbose: bool = True) -> None:
        """
        Run simulation for SIMULATION_TIME seconds.

        Current Phase-1 behavior:
        - green_direction is fixed (default 'N') just to validate simulation mechanics.

        Next step (Phase-2):
        - we will connect this environment to IntersectionAgent to set green_direction dynamically.
        """
        for t in range(0, SIMULATION_TIME, TIME_STEP):
            self.time = t

            # 1) Generate vehicles
            self.spawn_vehicles()

            # 2) Move vehicles according to current green signal
            self.step_movement()

            # 3) Print status
            if verbose:
                self.print_status()
