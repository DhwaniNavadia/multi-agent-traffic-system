"""
simulation/environment.py

ONE 4-way crossroads simulation with 2-phase traffic signal:
- "NS": North + South green
- "EW": East + West green

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
from agents.intersection_agent import IntersectionAgent


PHASE_TO_DIRS = {
    "NS": ["N", "S"],
    "EW": ["E", "W"],
}


# -------------------------
# Basic data models
# -------------------------

@dataclass
class Vehicle:
    """A simple vehicle model for simulation (no physics)."""
    vid: int
    direction: str
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


class TrafficEnvironment:
    """
    Simulation engine for ONE crossroads using 2-phase control (NS/EW).
    """

    def __init__(self, seed: int | None = 42):
        self.time: int = 0
        self.vehicle_id_counter: int = 1

        self.lanes: Dict[str, Lane] = {
            "N": Lane("N"),
            "S": Lane("S"),
            "E": Lane("E"),
            "W": Lane("W"),
        }

        self.signal = TrafficSignal(green_phase="NS", remaining_green=10)

        self.last_green_time: Dict[str, int] = {"NS": -999999, "EW": -999999}
        self.last_green_time[self.signal.green_phase] = 0

        self.agent = IntersectionAgent()

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
                    spawn_time=self.time,
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

        state["_phase"] = {
            "NS_since_green": float(self.time - self.last_green_time["NS"]),
            "EW_since_green": float(self.time - self.last_green_time["EW"]),
        }

        return state

    # ---------- Helpers ----------

    def _phase_queue_sum(self, phase: str) -> int:
        return sum(self.lanes[d].queue_length() for d in PHASE_TO_DIRS[phase])

    def _should_gap_out(self) -> bool:
        return self._phase_queue_sum(self.signal.green_phase) == 0

    def _global_max_wait(self) -> float:
        return float(max(self.get_state()[d]["max_wait_time"] for d in ["N", "S", "E", "W"]))

    # ---------- Vehicle movement ----------

    def step_movement(self) -> None:
        green_dirs = set(PHASE_TO_DIRS[self.signal.green_phase])

        for d, lane in self.lanes.items():
            if d in green_dirs:
                passed = lane.pop_vehicles(VEHICLE_DISCHARGE_RATE)
                self.total_passed += len(passed)
                self.passed_vehicles.extend(passed)
            else:
                lane.increment_waiting()

    # ---------- Debug output ----------

    def print_status(self) -> None:
        q_info = " ".join(f"{d}={self.lanes[d].queue_length():2d}" for d in ["N", "S", "E", "W"])
        print(
            f"t={self.time:3d} | "
            f"Phase={self.signal.green_phase}({self.signal.remaining_green:2d}s) | "
            f"{q_info} | spawned={self.total_spawned} passed={self.total_passed}"
        )

    # ---------- Summary metrics ----------

    def print_summary(self) -> None:
        if self.passed_vehicles:
            avg_wait = sum(v.wait_time for v in self.passed_vehicles) / len(self.passed_vehicles)
            max_wait = max(v.wait_time for v in self.passed_vehicles)
        else:
            avg_wait = 0.0
            max_wait = 0

        total_queue = sum(self.lanes[d].queue_length() for d in ["N", "S", "E", "W"])
        throughput = self.total_passed / max(1, SIMULATION_TIME)

        print("\n====== SIMULATION SUMMARY ======")
        print(f"Total spawned vehicles : {self.total_spawned}")
        print(f"Total passed vehicles  : {self.total_passed}")
        print(f"Throughput (veh/sec)   : {throughput:.3f}")
        print(f"Total vehicles in queue: {total_queue}")
        print(f"Avg wait (passed)      : {avg_wait:.2f} sec")
        print(f"Max wait (passed)      : {max_wait} sec")
        print("================================\n")

    # ---------- Main simulation loop ----------

    def run(self, verbose: bool = True, mode: str = "agent", fixed_cycle: int = 10) -> None:
        phases = ["NS", "EW"]
        fixed_index = 0

        for t in range(0, SIMULATION_TIME, TIME_STEP):
            self.time = t

            # 1) Spawn vehicles
            self.spawn_vehicles()

            # 2) Move vehicles (use current phase for this second)
            self.step_movement()

            # 3) Tick timer
            self.signal.tick()

            # 4) Gap-out AFTER movement: if phase now empty, end green
            if self._should_gap_out():
                self.signal.remaining_green = 0

            # 5) Safety cut: if max waiting is rising too much, end phase early
            # (this forces re-decision next step without breaking throughput)
            if self._global_max_wait() >= 12.0:
                self.signal.remaining_green = 0

            # 6) Switch only when expired
            if self.signal.is_expired():
                if mode == "agent":
                    state = self.get_state()
                    next_phase, duration = self.agent.choose_next_phase(state)
                    self.signal.set_green(next_phase, duration)
                    self.last_green_time[next_phase] = self.time
                elif mode == "fixed":
                    fixed_index = (fixed_index + 1) % len(phases)
                    next_phase = phases[fixed_index]
                    self.signal.set_green(next_phase, fixed_cycle)
                    self.last_green_time[next_phase] = self.time
                else:
                    raise ValueError("mode must be 'agent' or 'fixed'")

            if verbose:
                self.print_status()

        self.print_summary()
