from simulation.environment import TrafficEnvironment

if __name__ == "__main__":
    env = TrafficEnvironment(seed=42)
    env.run(verbose=True)
