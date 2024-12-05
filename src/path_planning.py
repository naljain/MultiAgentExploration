import numpy as np
import copy
import heapq
from collections import deque
import numpy as np
from queue import PriorityQueue


# TODO : add hybrid CBS functions

class PathFinding:
    def __init__(self, map, agents, start_pos, goal_pos, three_d=False):
        """
        map : current explored map
        agents : num of agents present
        start_pos : dictionary of agent ID and corresponding starting position
        goal_pos : dictionary of agent ID and corresponding goal position from global planner

        Map constants :

        UNKNOWN = -1
        FREE = 0
        OCCUPIED = 1

        1) sensor gets input of map
        2) agent plans where to go based on sensor info

        """

        self.map = map
        self.agents = agents
        self.three_d = three_d
        self.start_pos = start_pos
        print(start_pos)
        self.goal_pos = goal_pos

        self.grid = copy.deepcopy(map)
        if three_d:
            self.x_max, self.y_max, self.z_max = self.grid.shape
        else:
            self.x_max, self.y_max = self.grid.shape

    def get_neighbours(self, pos):
        # [up, down, left, right, up-left, up-right, down-left, down-right]
        # [(0, 1), (0, -1), (-1, 0), (1, 0), (-1, 1), (1, 1), (-1, -1), (1, -1)]
        x, y = pos

        directions = [(0, 1), (0, -1), (-1, 0), (1, 0), (-1, 1), (1, 1),
                      (-1, -1), (1, -1)]
        for dir in directions:
            dx, dy = dir
            new_x = dx + x
            new_y = dy + y
            # if outside wall limits, continue
            if new_x >= self.x_max or new_x < 0 or new_y >= self.y_max or new_y < 0:
                continue
            # if new_x, new_y is in obstacle, continue
            if self.grid[new_x][new_y] == 1:
                continue
            yield dir

    def valid_move(self):
        # check if at walls
        # check if obstacle
        pass

    def heuristic(self, p1, p2):
        # manhattan distance
        if self.three_d:
            x1, y1, z1 = p1
            x2, y2, z2 = p2
            d = np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2 + (z2 - z1) ** 2)
        else:
            x1, y1 = p1
            x2, y2 = p2
            d = np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        return d

    def a_star(self, agent_id):
        # Node is defined as n = (f, g, (x, y), parent)

        rows = self.x_max
        cols = self.y_max

        open_list = []
        closed_list = set()

        start = self.start_pos[agent_id]
        goal = self.goal_pos[agent_id]
        x0, y0 = start
        xf, yf = goal

        node_init = (0, 0, (x0, y0), None)
        heapq.heappush(open_list, node_init)

        while open_list:
            f, g, pos, parent = heapq.heappop(open_list)
            xc, yc = pos
            if pos == goal:
                # Path reconstruction
                path = []
                while parent:
                    path.append(pos)  # Append the current position
                    pos, g, parent = parent[2], parent[1], parent[3]  # Backtrack to parent
                path.append(start)  # Append the start position
                return path[::-1]  # Reverse the path to get start-to-goal order

            if pos in closed_list:
                continue

            closed_list.add(pos)
            for valid_dir in self.get_neighbours(pos):
                dx, dy = valid_dir
                nx = dx + xc
                ny = dy + yc
                if (nx, ny) not in closed_list:
                    new_g = g + 1
                    new_f = self.heuristic((nx, ny), goal) + new_g
                    heapq.heappush(open_list, (
                    new_f, new_g, (nx, ny), (f, g, pos, parent)))
        return None

    def jps(self, agent_id):
        pass

    # def cbs(self):
    #     # return dictionary of waypoints for each agent
    #     pass
    def cbs(self):
        """
        Conflict-Based Search for Multi-Agent Pathfinding.

        Returns:
            A dictionary with agent IDs as keys and their conflict-free paths as values.
        """
        # Initialize conflict tree (CT)
        CT = []  # Priority queue for CT nodes (sorted by cost)
        counter = 0  # Unique counter for tie-breaking in the heap

        # Initialize root node
        root = {
            "paths": {},  # Optimal paths for all agents
            "constraints": [],  # List of constraints [(agent_id, (x, y), t)]
            "cost": 0,  # Total cost of the solution
        }

        # Compute initial paths for all agents
        for agent_id in range(1, self.agents + 1):
            path = self.a_star(
                agent_id)  # Find a path using A* without constraints
            if path is None:
                return None  # No solution exists
            root["paths"][agent_id] = path
            root["cost"] += len(path)  # Total cost is the sum of all path lengths

        # Add the root node to the conflict tree
        heapq.heappush(CT, (root["cost"], counter, root))
        counter += 1

        while CT:
            # Get the CT node with the lowest cost
            _, _, node = heapq.heappop(CT)

            # Check for conflicts
            conflict = self.detect_conflict(node["paths"])
            if conflict is None:
                # No conflicts; return the paths
                return node["paths"]

            # Branch to resolve the conflict
            agent1, agent2, loc, time = conflict

            for agent_id in [agent1, agent2]:
                # Create a new constraint to resolve the conflict
                new_constraint = (agent_id, loc, time)

                # Create a new child node with the additional constraint
                child = {
                    "paths": copy.deepcopy(node["paths"]),
                    "constraints": node["constraints"] + [new_constraint],
                    "cost": 0,
                }

                # Recompute the path for the constrained agent
                self.apply_constraints(child["constraints"])
                new_path = self.a_star(agent_id)
                if new_path is None:
                    continue  # If no valid path, discard this branch
                child["paths"][agent_id] = new_path

                # Update the total cost
                child["cost"] = sum(
                    len(path) for path in child["paths"].values())

                # Add the child node to the conflict tree
                heapq.heappush(CT, (child["cost"], counter, child))
                counter += 1

        return None  # No solution found

    def detect_conflict(self, paths):
        """
        Detects a conflict in the paths.

        Args:
            paths (dict): A dictionary of agent paths.

        Returns:
            A tuple (agent1, agent2, (x, y), t) representing the conflict, or None if no conflict exists.
        """
        times = max(len(path) for path in paths.values())
        for t in range(times):
            positions = {}
            for agent, path in paths.items():
                if t < len(path):
                    pos = path[t]
                else:
                    pos = path[-1]  # Agent stays at goal if path is shorter
                if pos in positions:
                    return positions[pos], agent, pos, t  # Conflict detected
                positions[pos] = agent
        return None  # No conflicts

    def apply_constraints(self, constraints):
        """
        Modifies the grid or the planning logic to respect constraints.

        Args:
            constraints (list): List of constraints [(agent_id, (x, y), t)].
        """
        self.constraints = {}
        for agent_id, loc, time in constraints:
            if agent_id not in self.constraints:
                self.constraints[agent_id] = {}
            self.constraints[agent_id][time] = loc

    def a_star_with_constraints(self, agent_id):
        """
        A* pathfinding with constraints for a specific agent.
        """
        # Define constraints
        agent_constraints = self.constraints.get(agent_id, {})

        def is_constrained(pos, time):
            return time in agent_constraints and agent_constraints[time] == pos

        # Standard A* logic with an additional check for constraints
        open_list = []
        closed_list = set()
        start = self.start_pos[agent_id]
        goal = self.goal_pos[agent_id]
        heapq.heappush(open_list, (0, 0, start, None))

        while open_list:
            f, g, pos, parent = heapq.heappop(open_list)
            if pos == goal:
                path = []
                while parent:
                    path.append(pos)
                    pos, g, parent = parent[2], parent[1], parent[3]
                path.append(start)
                return path[::-1]

            if (pos, g) in closed_list:
                continue
            closed_list.add((pos, g))

            for dx, dy in self.get_neighbours(pos):
                nx, ny = pos[0] + dx, pos[1] + dy
                new_pos = (nx, ny)
                new_g = g + 1
                if not is_constrained(new_pos, new_g):
                    new_f = new_g + self.heuristic(new_pos, goal)
                    heapq.heappush(open_list, (
                    new_f, new_g, new_pos, (f, g, pos, parent)))

        return None


