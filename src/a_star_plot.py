import path_planning
import matplotlib.pyplot as plt
import numpy as np

def visualize_path_matplotlib(grid, path, start, goal):
    """
    Visualize the grid and the path taken using Matplotlib.

    :param grid: 2D list or array (0 = free, 1 = obstacle).
    :param path: List of tuples representing the path (x, y).
    :param start: Tuple representing the start position (x, y).
    :param goal: Tuple representing the goal position (x, y).
    """
    rows, cols = len(grid), len(grid[0])

    # Create a blank canvas
    grid_canvas = np.array(grid)

    # Start and goal points
    sx, sy = start
    gx, gy = goal

    # Create a figure
    fig, ax = plt.subplots(figsize=(cols, rows))

    # Draw grid with obstacles
    ax.imshow(grid_canvas, cmap='gray_r', origin='upper')

    # Highlight the start and goal points
    ax.scatter(sy, sx, color='green', label='Start', s=100, zorder=3)  # Start
    ax.scatter(gy, gx, color='red', label='Goal', s=100, zorder=3)  # Goal

    # Plot the path if it exists
    if path:
        path_x = [y for x, y in path]
        path_y = [x for x, y in path]
        ax.plot(path_x, path_y, color='blue', linewidth=2, label='Path', zorder=2)

    # Customize the plot
    ax.set_xticks(range(cols))
    ax.set_yticks(range(rows))
    ax.grid(which="major", color="black", linestyle='-', linewidth=0.5)
    ax.set_xticks(np.arange(-0.5, cols, 1), minor=True)
    ax.set_yticks(np.arange(-0.5, rows, 1), minor=True)
    ax.grid(which="minor", color="black", linestyle='-', linewidth=0.05)
    ax.tick_params(left=False, bottom=False, labelleft=False, labelbottom=False)
    ax.legend(loc="upper right")

    # Show the plot
    plt.show()


if __name__ == "__main__":
    # Example grid: 0 = free, 1 = obstacle
    grid = [
    [0, 1, 0, 0, 0, 1, 0],
    [0, 1, 0, 1, 0, 1, 0],
    [0, 1, 0, 1, 0, 1, 0],
    [0, 0, 0, 1, 0, 0, 0],
    [1, 1, 0, 0, 0, 1, 1],
    [0, 0, 0, 1, 0, 0, 0],
]
    grid = np.asarray(grid)
    print('start')
    # Example path
    start = {1:(0, 0)}
    goal = {1:(5, 6)}

    a = path_planning.PathFinding(grid, 1, start, goal)
    path = a.a_star(1)
    print('FINAL PATH' , path)


    # path = [(0, 0), (0, 1), (0, 2), (1, 2), (2, 2), (3, 3)]  # Example A* output

    # Visualize the path
    visualize_path_matplotlib(grid, path, start[1], goal[1])


# grid_3 = [
#     [0, 0, 1, 0, 0],
#     [1, 0, 1, 0, 1],
#     [1, 0, 0, 0, 1],
#     [0, 1, 1, 0, 0],
#     [0, 0, 0, 1, 0],
# ]
# start_3 = (0, 0)
# goal_3 = (4, 4)

# grid_9 = [
#     [0, 1, 0, 0, 0, 1, 0],
#     [0, 1, 0, 1, 0, 1, 0],
#     [0, 1, 0, 1, 0, 1, 0],
#     [0, 0, 0, 1, 0, 0, 0],
#     [1, 1, 0, 0, 0, 1, 1],
#     [0, 0, 0, 1, 0, 0, 0],
# ]
# start_9 = (0, 0)
# goal_9 = (5, 6)