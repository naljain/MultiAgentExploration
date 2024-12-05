import numpy as np
import path_planning
import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN
import numpy as np

class GlobalPlanner():
    """
        Map constants :

        UNKNOWN = -1
        FREE = 0
        OCCUPIED = 1 (with obstacle) ??
    """
    def __init__(self, global_map, agents, time_step):
        # Map of actual environment
        self.global_map = global_map
        self.x_max, self.y_max = self.global_map.shape
        print(self.x_max, self.y_max)
        # Map maintained by global planner as agents explore and upate
        # self.explored_map = np.ones(global_map.shape) * -1
        # self.explored_map[0][0] = 0

        ### FOR TESTING ONLY - REMOVE WHEN CLUSTERING/GOAL PICKING DONE ###
        self.explored_map = global_map

        assert (self.explored_map.shape == self.global_map.shape)

        self.agents = agents
        self.time_step = time_step

        # [up, down, left, right, up-left, up-right, down-left, down-right]
        self.directions = [(0, 1), (0, -1), (-1, 0), (1, 0), (-1, 1), (1, 1), (-1, -1), (1, -1)]
        self.frontier = []


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
        for x in range(self.x_max):
            for y in range(self.y_max):
                if map[x][y] == 0:
                    for dx, dy in self.directions:
                        new_x, new_y = dx + x, dy + y
                        # Check if nx, ny outside of map bounds
                        if new_x >= self.x_max or new_x < 0 or new_y >= self.y_max or new_y < 0:
                            continue
                        if map[new_x][new_y] == -1:
                            if (x,y) not in self.frontier:
                                self.frontier.append((x,y))
        return self.frontier

    def find_clusters_dbscan(self):
        """
        Finds clusters of unexplored cells using DBSCAN.

        Args:
            map: 2D list representing the map (-1 = unexplored, 0 = free, 1 = obstacle).

        Returns:
            List of clusters, where each cluster is a list of (x, y) positions.
        """
        map = self.explored_map
        unexplored = [(x, y) for x in range(len(map)) for y in
                      range(len(map[0])) if map[x][y] == -1]
        if not unexplored:
            return []

        # DBSCAN requires numerical input; convert (x, y) tuples to an array
        unexplored_array = np.array(unexplored)

        # Apply DBSCAN
        clustering = DBSCAN(eps=2, min_samples=2).fit(unexplored_array)

        # Extract clusters
        clusters = {}
        for point, label in zip(unexplored, clustering.labels_):
            if label == -1:  # Noise (unclustered points)
                continue
            if label not in clusters:
                clusters[label] = []
            clusters[label].append(point)

        print(list(clusters.values()))
        return list(clusters.values())

    def generate_goals(self):
        pass




    def find_uknown_clusters(self):
        pass

    def find_centroid(self):
        pass

    def check_collision(self):
        pass

    def reassign_goal(self):
        pass

    def run_planner(self):
        """
        for loop:
            1) generate global goals to feed into path planning to generate reference trajectory/waypoints
               from the most up-to-date map
            2) send reference traj to controller
            3) get updates (maps) at regular frequency from agents
            4) merge maps together to get most up-to-date map
            5) regenerate goals
        """
        pass

    def visualise_frontier(self):
        rows, cols = self.x_max, self.y_max
        fig, ax = plt.subplots(
            figsize=(cols * 0.6, rows * 0.6))  # Scale figure size

        # Create a color map for visualizing the states
        color_map = {
            -1: "lightblue",  # Unexplored space
            0: "white",  # Explored free space
        }
        dbscan = self.find_clusters_dbscan()

        # Plot the grid with different colors for each state
        for x in range(rows):
            for y in range(cols):
                cell_value = self.explored_map[x][y]
                color = color_map.get(cell_value,"black")  # Default to black for other values
                if (x, y) in self.frontier:
                    color = 'pink'
                ax.add_patch(plt.Rectangle((y, x), 1, 1, color=color))
        for cluster in dbscan:
            for (x, y) in cluster:
                ax.scatter(y + 0.5, x + 0.5)

        # Set axis limits to include all cells
        ax.set_xlim(0, cols)
        ax.set_ylim(0, rows)

        # Customize the plot
        ax.invert_yaxis()
        ax.set_xticks(range(cols))
        ax.set_yticks(range(rows))
        ax.grid(which="major", color="black", linestyle='-', linewidth=0.5)
        ax.tick_params(left=False, bottom=False, labelleft=False,
                       labelbottom=False)

        # Add legend
        from matplotlib.patches import Patch
        legend_elements = [
            Patch(facecolor="white", label="Explored"),
            Patch(facecolor="lightblue", label="Unexplored"),
        ]
        ax.legend(handles=legend_elements, loc="upper right")

        # Show the plot
        print('plotting')
        plt.show()


if __name__ == "__main__":
    map1 = [[-1, 0, -1, -1, -1],
           [0, 0,  0, -1, -1],
           [0, 0,  0, -1, -1],
           [-1, 0, 0, 0, 0],
           [-1, -1, 0, 0, 0],
           [0, 0, 0, 0, 0]]

    map2 = np.zeros((10,10))
    map2[:,7:] = -1
    print(map2)

    map1 = np.asarray(map1)

    planner = GlobalPlanner(map1, 1, 1)
    f = planner.find_frontier()
    print(f)

    planner.find_clusters_dbscan()
    planner.visualise_frontier()
#

