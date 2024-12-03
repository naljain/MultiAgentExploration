import numpy as np
import copy
import heapq
from collections import deque
import numpy as np
from queue import PriorityQueue

'''
1) sensor gets input of map 
2) agent plans where to go based on sensor info 


'''
class PathFinding(map, agents):
    def __init__(self, map, agents, start_pos, goal_pos, three_d = False):
        """
        map : current explored map
        agents : num of agents present
        start_pos : dictionary of agent ID and corresponding starting position
        goal_pos : dictionary of agent ID and corresponding goal position from global planner

        Map constants :

        UNKNOWN = -1
        FREE = 0
        OCCUPIED = 1
        """

        self.map = map
        self.agents = agents
        self.three_d = three_d
        self.start_pos = start_pos
        self.goal_pos = goal_pos

        self.grid = copy.deepcopy(map)
        if three_d:
            self.x, self.y, self.z = self.grid.shape
        else:
            self.x, self.y = self.grid.shape

    def get_neighbours(self):
        # [up, down, left, right, up-left, up-right, down-left, down-right]
        # [(0, 1), (0, -1), (-1, 0), (1, 0), (-1, 1), (1, 1), (-1, -1), (1, -1)]

        pass

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
        # Node is defined as n = (f, g, (x,y), parent)

        rows = self.x
        cols = self.y

        open_list = []
        closed_list = set()

        start = self.start_pos[agent_id]
        goal = self.goal_pos[agent_id]
        x0, y0 = start
        xf, yf = goal

        node_init = (0, 0, (x0, y0), None)
        heapq.heappush(open_list, node_init)

        while open_list:
            pos,  = heapq.heappop(open_list)
            if start == goal:
                path = []
                # TODO : Reconstruct path
                return path

            if

        pass

    def jps(self, agent_id):
        pass

    def cbs(self):
        # return dictionary of waypoints for each agent
        pass

