import path_planning
import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN
import numpy as np
import random
import threading
import queue


class GlobalPlanner():
    """
        Explore Map constants :

        UNKNOWN = -1
        FREE = 0
        OCCUPIED = 1
        BLOATED AREA = 2
        TODO : collision check, goal reassignment?
        TODO : check if same goal assigned to multiple agents
        TODO : when number of goals < num of agents
    """

    def __init__(self, global_map, agents, start_pos, time_step, bloat_size,
                 unknown_travel):
        # Map of actual environment
        self.global_map = global_map
        self.x_max, self.y_max = self.global_map.shape
        # print(self.x_max, self.y_max)
        self.start_pos = start_pos

        ### FOR TESTING ONLY - REMOVE WHEN CLUSTERING/GOAL PICKING DONE ###
        self.explored_map = global_map
        self.explored_map[:, 50:] = -1
        # self.explored_map[40:80, 30:45] = -1

        ### UNCOMMENT THIS AFTER ###
        # Map maintained by global planner as agents explore and upate
        # self.explored_map = np.ones(global_map.shape) * -1
        # self.explored_map[0][0] = 0

        assert (self.explored_map.shape == self.global_map.shape)

        self.agents = agents
        self.time_step = time_step
        self.bloat_size = bloat_size
        self.unknown_travel = unknown_travel

        # [up, down, left, right, up-left, up-right, down-left, down-right]
        self.directions = [(0, 1), (0, -1), (-1, 0), (1, 0), (-1, 1), (1, 1),
                           (-1, -1), (1, -1)]
        self.frontier = []
        self.clusters = None

    # TODO BELLOW FUNCTIONS
    def bloat_obstacles(self):
        for x in range(self.x_max):
            for y in range(self.y_max):
                if self.explored_map[x][y] == 1:
                    # for dx, dy in self.directions:
                    #     for b in range(self.bloat_size):
                    #         nx, ny = dx * (b+1) + x, dy * (b+1) + y
                    #         if self.explored_map[nx][ny] != 1:
                    #             self.explored_map[nx][ny] = 2

                    for dx in range(-self.bloat_size, self.bloat_size + 1):
                        for dy in range(-self.bloat_size, self.bloat_size + 1):
                            nx, ny = x + dx, y + dy
                            # Check bounds and ensure it's not already an obstacle
                            if 0 <= nx < self.x_max and 0 <= ny < self.y_max:
                                # Use Euclidean distance to ensure a circular bloat
                                if dx ** 2 + dy ** 2 <= self.bloat_size ** 2:
                                    if self.explored_map[nx][ny] != 1:
                                        self.explored_map[nx][ny] = 2

    def upadate_maps(self):
        # takes the local map of the drone that reaches frontier goal and makes global map == local map
        pass

    def send_ref_traj(self):
        # calls on path planning to get waypoints
        # converts to spline
        # sends to agents
        pass

    # Functions for generating new goals
    def find_frontier(self):
        map = self.explored_map
        frontier = []
        for x in range(self.x_max):
            for y in range(self.y_max):
                if map[x][y] == 0:
                    for dx, dy in self.directions:
                        new_x, new_y = dx + x, dy + y
                        # Check if nx, ny outside of map bounds
                        if new_x >= self.x_max or new_x < 0 or new_y >= self.y_max or new_y < 0:
                            continue
                        if map[new_x][new_y] == -1:
                            if (x, y) not in frontier:
                                frontier.append((x, y))
        return frontier

    def find_clusters_dbscan(self):
        """
        Finds clusters of unexplored cells using DBSCAN.
        """
        map = self.explored_map
        unexplored = [(x, y) for x in range(len(map)) for y in
                      range(len(map[0])) if map[x][y] == -1]
        if not unexplored:
            return []

        unexplored_array = np.array(unexplored)

        clustering = DBSCAN(eps=2, min_samples=2).fit(unexplored_array)

        # Extract clusters
        clusters = {}
        for point, label in zip(unexplored, clustering.labels_):
            if label == -1:  # Noise (unclustered points)
                continue
            if label not in clusters:
                clusters[label] = []
            clusters[label].append(point)
        return list(clusters.values())

    def split_large_clusters(self, clusters, max_size=5):
        """
        Split large clusters into smaller ones if they exceed a maximum size.
        """

        new_clusters = []
        for cluster in clusters:
            if len(cluster) <= max_size:
                new_clusters.append(cluster)
            else:
                # Split the cluster into smaller chunks
                for i in range(0, len(cluster), max_size):
                    new_clusters.append(cluster[i:i + max_size])
        self.clusters = new_clusters
        # print(new_clusters)
        return new_clusters

    def compute_centroid(self, cluster):
        x_coords, y_coords = zip(*cluster)
        centroid_x = np.mean(x_coords)
        centroid_y = np.mean(y_coords)
        return centroid_x, centroid_y

    def find_closest_frontier(self, frontier, centroid):
        """
        Find the frontier cell closest to a given centroid.
        """
        closest_cell = None
        min_distance = float('inf')
        for cell in frontier:
            # print(cell, centroid)
            distance = np.sqrt(
                (centroid[0] - cell[0]) ** 2 + (centroid[1] - cell[1]) ** 2)
            if distance < min_distance:
                min_distance = distance
                closest_cell = cell
        return closest_cell

    def assign_frontier_to_clusters(self, frontier, clusters):
        """
        Assign the closest frontier cell to each cluster based on its centroid.

        Args:
            clusters (list of lists): List of clusters, where each cluster is a list of (x, y) coordinates.
            frontier (list of tuples): List of frontier cells [(x1, y1), ...].

        Returns:
            dict: A mapping of each cluster's index to its closest frontier cell.
        """
        cluster_to_frontier = {}
        for idx, cluster in enumerate(clusters):
            centroid = self.compute_centroid(cluster)
            closest_frontier = self.find_closest_frontier(frontier, centroid)
            cluster_to_frontier[idx] = closest_frontier
        return cluster_to_frontier

    def generate_goals(self, frontier, clusters):
        goals_pos = {}
        goals = []
        for i in range(self.agents):
            centroid = self.compute_centroid(clusters[i])
            xg, yg = self.find_closest_frontier(frontier, centroid)
            goals_pos[i + 1] = (xg, yg)
            frontier.remove((xg, yg))
            goals.append((xg, yg))

            if self.explored_map[xg][yg] == 1 or self.explored_map[xg][
                yg] == 2:
                raise Exception("Goal generated inside obstacle")

        return goals_pos, goals

    def run_planner(self, plotting=False):
        """
        for loop:
            1) generate global goals
            2) feed into path planning to generate reference trajectory/waypoints
               from the most up-to-date map
            3) send reference traj to controller
            4) get updates (maps) at regular frequency from agents
            5) merge maps together to get most up-to-date map
            6) regenerate goals
        """
        # Generate goals
        num_agents = self.agents
        self.bloat_obstacles()  # BLOATING OBSTACLES HAS TO BE BEFORE FRONTIER
        frontier = self.find_frontier()
        clusters = self.find_clusters_dbscan()
        if len(clusters) <= num_agents:
            cluster_size = sum(
                len(sublist) for sublist in clusters) // num_agents
            clusters = self.split_large_clusters(clusters, cluster_size)
        start_pos = self.start_pos
        goal_pos, goals = self.generate_goals(frontier, clusters)
        print(goal_pos)
        path_plan = path_planning.PathFinding(self.explored_map, self.agents,
                                              start_pos, goal_pos,
                                              self.unknown_travel)
        waypoints = path_plan.cbs()
        # waypoints = path_plan.all_a_star()
        print(waypoints)
        if plotting:
            self.visualise_frontier(clusters, frontier, start_pos, goal_pos,
                                    waypoints)

    def worker_fn(self, plotting):
        return self.run_planner(plotting=True)

    def visualise_frontier(self, clusters, frontier, start, goals, waypoints):

        def get_cmap(n, name='hsv'):
            '''Returns a function that maps each index in 0, 1, ..., n-1 to a distinct
            RGB color; the keyword argument name must be a standard mpl colormap name.'''
            return plt.cm.get_cmap(name, n)

        rows, cols = self.x_max, self.y_max
        fig, ax = plt.subplots(
            figsize=(10, 10))  # Scale figure size

        # Create a color map for visualizing the states
        color_map = {
            -1: "lightblue",  # Unexplored space
            0: "white",  # Explored free space
            1: 'slategrey',  # obstacle
            2: 'pink'  # bloated obstacle
        }
        # dbscan = self.find_clusters_dbscan()
        # cmap = get_cmap(len(dbscan))
        dbscan = clusters

        # Plot the grid with different colors for each state
        for x in range(rows):
            for y in range(cols):
                cell_value = self.explored_map[x][y]
                color = color_map.get(cell_value,
                                      "black")  # Default to black for other values
                if (x, y) in self.frontier:
                    color = 'pink'
                ax.add_patch(plt.Rectangle((y, x), 1, 1, color=color))
        i = 0
        # for cluster in dbscan:
        #     r = random.random()
        #     b = random.random()
        #     g = random.random()
        #     c = (r,g,b)
        #     for (x, y) in cluster:
        #         ax.scatter(y + 0.5, x + 0.5, color = c)
        #     i += 1

        for i in range(self.agents):
            goal_x, goal_y = goals[i + 1]
            start_x, start_y = start[i + 1]
            ax.scatter(goal_y + 0.5, goal_x + 0.5, color='red')
            ax.scatter(start_y + 0.5, start_x + 0.5, color='blue')

        for i in range(self.agents):
            path = waypoints[i + 1]
            if path:
                path_x = [y + 0.5 for x, y in path]
                path_y = [x + 0.5 for x, y in path]
                ax.plot(path_x, path_y, color='blue', linewidth=2,
                        label='Path', zorder=2)

        # Set axis limits to include all cells
        ax.set_xlim(0, cols)
        ax.set_ylim(0, rows)

        # Customize the plot
        ax.invert_yaxis()
        ax.set_xticks(range(cols))
        ax.set_yticks(range(rows))
        ax.grid(which="major", color="black", linestyle='-', linewidth=0.1)
        ax.tick_params(left=False, bottom=False, labelleft=False,
                       labelbottom=False)

        # Add legend
        from matplotlib.patches import Patch
        legend_elements = [
            Patch(facecolor="white", label="Explored"),
            Patch(facecolor="lightblue", label="Unexplored"),
        ]
        # ax.legend(handles=legend_elements, loc="upper right")

        # Show the plot
        print('plotting')
        plt.show()


if __name__ == "__main__":
    map = np.loadtxt('test_map')
    num_agents = 3
    start_pos = {1: (0, 0), 2: (10, 0), 3: (20, 0)}  # , 4: (9, 0), 5: (11,0)}
    time_step = 1

    bloat_val = 4  # BLOAT_VAL > RADIUS OF DRONE
    unknown_travel = True

    planner = GlobalPlanner(map, num_agents, start_pos, time_step, bloat_val,
                            unknown_travel)
    planner.run_planner(plotting=True)
