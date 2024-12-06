import numpy as np
import matplotlib.pyplot as plt

def plot_2d_map(map_array, start, goals, num_agents):
    """
    Visualize a 2D map represented as a numpy array.

    Parameters:
        map_array (np.array): 2D numpy array where 0 represents empty space and 1 represents obstacles.
        title (str): Title of the plot.
    """
    # Create the figure and axis
    plt.figure(figsize=(8, 8))

    # Use a colormap to distinguish between empty space and obstacles
    cmap = plt.get_cmap(
        "gray_r")  # "gray_r" inverts the grayscale (0=white, 1=black)
    plt.imshow(map_array, cmap=cmap, origin="upper")

    for i in range(num_agents):
        goal_x, goal_y = goals[i + 1]
        start_x, start_y = start[i + 1]
        plt.scatter(goal_y + 0.5, goal_x + 0.5, color='red')
        plt.scatter(start_y + 0.5, start_x + 0.5, color='blue')

    # Add a grid for better visualization
    plt.grid(visible=True, color="black", linestyle="--", linewidth=0.5)
    plt.xticks(ticks=np.arange(-0.5, map_array.shape[1], 1), labels=[])
    plt.yticks(ticks=np.arange(-0.5, map_array.shape[0], 1), labels=[])

    # Add title and axis labels
    plt.xlabel("X-axis")
    plt.ylabel("Y-axis")

    # Adjust grid to overlay properly
    plt.gca().set_xticks(np.arange(-0.5, map_array.shape[1], 1), minor=True)
    plt.gca().set_yticks(np.arange(-0.5, map_array.shape[0], 1), minor=True)
    plt.gca().grid(which="minor", color="black", linestyle="--", linewidth=0.5)
    plt.gca().tick_params(axis="both", which="minor", length=0)




    # Display the plot
    plt.show()


def bloat_obstacles(map, bloat_size):
    for x in range(len(map)):
        for y in range(len(map[0])):
            if map[x][y] == 1:
                # for dx, dy in self.directions:
                #     for b in range(self.bloat_size):
                #         nx, ny = dx * (b+1) + x, dy * (b+1) + y
                #         if self.explored_map[nx][ny] != 1:
                #             self.explored_map[nx][ny] = 2

                for dx in range(-bloat_size, bloat_size + 1):
                    for dy in range(-bloat_size, bloat_size + 1):
                        nx, ny = x + dx, y + dy
                        # Check bounds and ensure it's not already an obstacle
                        if 0 <= nx < len(map) and 0 <= ny < len(map[0]):
                            # Use Euclidean distance to ensure a circular bloat
                            if dx**2 + dy**2 <= bloat_size**2:
                                if map[nx][ny] != 1:
                                    map[nx][ny] = 2
    return map

goal = {1: (16, 49), 2: (49, 49), 3: (83, 49)}




start = {1: (0, 0), 2: (3, 0), 3: (6, 0)}


# {1: (16, 49), 2: (44, 49), 3: (97, 49)}
# {1: (0, 0), 2: (3, 0), 3: (6, 0)}
map = np.loadtxt('test_map')
mapbloat = bloat_obstacles(map, 2)
plot_2d_map(mapbloat, start, goal, 3)