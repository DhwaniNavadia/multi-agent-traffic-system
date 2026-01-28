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

from simulation.signal import TrafficSignal


# -------------------------
# Basic data models
# -------------------------

@dataclass
class Vehicle:
    """A simple vehicle model for simulation (no physics)."""
    vid: int
    direction: str           # 'N', 'S', 'E', 'W'
    spawn_time: int
    wait_time: int = 0


@dataclass
class Lane:
    """Each direction has one incoming lane represented as a queue of vehicles."""
    direction: str
    queue: List[Vehicle] = field(default_factory=list)

    def queue_length(self) -> int:
        return len(self.queue)

    def add_vehicle(self, v: Vehicle) -> None:
        self.queue.append(v)

    def increment_waiting(self) -> None:
        for v in self.queue:
            v.wait_time += TIME_STEP

    def pop_vehicles(self, count: int) -> List[Vehicle]:
        passed = self.queue[:count]
        self.queue = self.queue[count:]
        return passed


# -------------------------
# Environment (Simulation Engine)
# -------------------------

class TrafficEnvironment:
    """
    Simulation engine for ONE crossroads.
    """

    def __init__(self, seed: int | None = 42):
        self.time: int = 0
        self.vehicle_id_counter: int = 1

        # Create one lane per direction
        self.lanes: Dict[str, Lane] = {
            "N": Lane("N"),
            "S": Lane("S"),
            "E": Lane("E"),
            "W": Lane("W"),
        }

        # Traffic signal
        self.signal = TrafficSignal(
            green_direction="N",
            remaining_green=10
        )

        # Statistics
        self.total_spawned: int = 0
        self.total_passed: int = 0
        self.passed_vehicles: List[Vehicle] = []

        if seed is not None:
            random.seed(seed)

    # ---------- Traffic generation ----------

    def spawn_vehicles(self) -> None:
        for d, rate in ARRIVAL_RATES.items():
            if random.random() < rate:
                v = Vehicle(
                    vid=self.vehicle_id_counter,
                    direction=d,
                    spawn_time=self.time
                )
                self.vehicle_id_counter += 1
                self.lanes[d].add_vehicle(v)
                self.total_spawned += 1

    # ---------- State sensing ----------

    def get_state(self) -> Dict[str, Dict[str, float]]:
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

    # ---------- Vehicle movement ----------

    def step_movement(self) -> None:
        for d, lane in self.lanes.items():
            if d == self.signal.green_direction:
                passed = lane.pop_vehicles(VEHICLE_DISCHARGE_RATE)
                self.total_passed += len(passed)
                self.passed_vehicles.extend(passed)
            else:
                lane.increment_waiting()

    # ---------- Debug output ----------

    def print_status(self) -> None:
        q_info = " ".join(
            f"{d}={self.lanes[d].queue_length():2d}"
            for d in ["N", "S", "E", "W"]
        )
        print(
            f"t={self.time:3d} | "
            f"Green={self.signal.green_direction}({self.signal.remaining_green:2d}s) | "
            f"{q_info} | spawned={self.total_spawned} passed={self.total_passed}"
        )

    # ---------- Main simulation loop ----------

    def run(self, verbose: bool = True) -> None:
        for t in range(0, SIMULATION_TIME, TIME_STEP):
            self.time = t

            # 1) Spawn vehicles
            self.spawn_vehicles()

            # 2) Move vehicles
            self.step_movement()

            # 3) Update signal timer
            self.signal.tick()

            # 4) Print status
            if verbose:
                self.print_status()
