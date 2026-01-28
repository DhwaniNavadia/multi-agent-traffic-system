"""
config.py

Global configuration parameters for the
Multi-Agent Intelligent Traffic Management System.

All simulation, traffic, and signal timing parameters
are defined here to keep the system modular and tunable.
"""

# =========================
# SIMULATION PARAMETERS
# =========================

# Duration of one simulation step (in seconds)
TIME_STEP = 1

# Total simulation time (in seconds)
SIMULATION_TIME = 300  # 5 minutes


# =========================
# TRAFFIC GENERATION
# =========================

# Vehicle arrival probability per second for each direction
# (simulates traffic density without cameras)
ARRIVAL_RATES = {
    "N": 0.6,  # North
    "S": 0.4,  # South
    "E": 0.3,  # East
    "W": 0.2   # West
}


# =========================
# SIGNAL TIMING LIMITS
# =========================

# Minimum and maximum green signal duration (seconds)
MIN_GREEN_TIME = 10
MAX_GREEN_TIME = 60

# Base green time added regardless of traffic
BASE_GREEN_TIME = 10

# Extra green time added per waiting vehicle
GREEN_TIME_PER_VEHICLE = 2


# =========================
# FAIRNESS PARAMETERS
# =========================

# Maximum allowed waiting time before forcing green
MAX_WAIT_THRESHOLD = 90  # seconds

# Minimum time before the same direction can get green again
DIRECTION_COOLDOWN = 5  # seconds


# =========================
# MOVEMENT PARAMETERS
# =========================

# Vehicles that can pass per second when signal is green
VEHICLE_DISCHARGE_RATE = 1
