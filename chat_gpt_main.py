import threading
import time

class GlobalPlanner(threading.Thread):
    def __init__(self, map, agents):
        super().__init__()
        self.map = map
        self.agents = agents
        self.running = True

    def run(self):
        while self.running:
            print("Global planner updating goals...")
            for agent in self.agents:
                agent.goal = self.assign_goal(agent)
            time.sleep(1)  # Update every second

    def assign_goal(self, agent):
        # Assign a goal (example: nearest unexplored cell)
        return agent.find_nearest_unexplored()

class Agent(threading.Thread):
    def __init__(self, id, start, map):
        super().__init__()
        self.id = id
        self.position = start
        self.map = map
        self.goal = None
        self.running = True

    def run(self):
        while self.running:
            if self.goal:
                print(f"Agent {self.id} planning path...")
                self.plan_path()
                print(f"Agent {self.id} executing step...")
                self.execute_step()
            time.sleep(0.5)  # Execute every 0.5 seconds

    def plan_path(self):
        # Plan a path to the current goal
        self.path = self.a_star(self.position, self.goal)

    def execute_step(self):
        # Move one step along the path
        if self.path:
            self.position = self.path.pop(0)

# Example simulation
if __name__ == "__main__":
    # Initialize map and agents
    map = [[0, 0, -1], [0, 1, 0], [0, 0, 0]]  # Simplified map
    agents = [Agent(id=1, start=(0, 0), map=map), Agent(id=2, start=(2, 2), map=map)]
    global_planner = GlobalPlanner(map, agents)

    # Start threads
    global_planner.start()
    for agent in agents:
        agent.start()

    # Let the simulation run for 5 seconds
    time.sleep(5)

    # Stop all threads
    global_planner.running = False
    for agent in agents:
        agent.running = False

    global_planner.join()
    for agent in agents:
        agent.join()

    print("Simulation complete.")
