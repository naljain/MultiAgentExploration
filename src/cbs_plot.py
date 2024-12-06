import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import path_planning

def visualize_cbs(grid, agent_paths, interval=700):
    """
    Visualize Conflict-Based Search (CBS) paths using Matplotlib animation.
    After the animation, display the complete paths as lines.

    :param grid: 2D list or array representing the environment (0 = free, 1 = obstacle).
    :param agent_paths: Dictionary with agent IDs as keys and their paths as values.
                        Each path is a list of (x, y) positions.
    :param interval: Time (ms) between animation frames.
    """
    rows, cols = len(grid), len(grid[0])

    # Initialize figure and axis
    fig, ax = plt.subplots(figsize=(cols, rows))
    ax.set_xlim(-0.5, cols - 0.5)
    ax.set_ylim(-0.5, rows - 0.5)
    ax.set_xticks(range(cols))
    ax.set_yticks(range(rows))
    ax.grid(color='black', linestyle='-', linewidth=0.5)
    ax.tick_params(left=False, bottom=False, labelleft=False, labelbottom=False)

    # Draw the grid with obstacles
    for x in range(rows):
        for y in range(cols):
            if grid[x][y] == 1:
                ax.add_patch(plt.Rectangle((y - 0.5, x - 0.5), 1, 1, color='black'))

    # Colors for agents
    agent_colors = plt.cm.get_cmap("tab10", len(agent_paths))

    # Initialize agent positions
    agent_positions = {agent_id: path[0] for agent_id, path in agent_paths.items()}
    agent_start_goal = {agent_id: (path[0], path[-1]) for agent_id, path in agent_paths.items()}

    # Initialize patches for agents
    agent_patches = {}
    for agent_id, (x, y) in agent_positions.items():
        color = agent_colors(agent_id - 1)
        patch = ax.add_patch(plt.Circle((y, x), 0.3, color=color, label=f"Agent {agent_id}"))
        agent_patches[agent_id] = patch

        # Mark start and goal positions
        start, goal = agent_start_goal[agent_id]
        sx, sy = start
        gx, gy = goal
        ax.add_patch(plt.Circle((sy, sx), 0.2, color=color, alpha=0.6, label=f"Start {agent_id}"))
        ax.add_patch(plt.Circle((gy, gx), 0.2, color=color, alpha=1.0, edgecolor='black', label=f"Goal {agent_id}"))

    # Update function for animation
    def update(frame):
        for agent_id, path in agent_paths.items():
            if frame < len(path):
                x, y = path[frame]
                agent_patches[agent_id].set_center((y, x))

    # Draw complete paths after animation
    def draw_trails():
        for agent_id, path in agent_paths.items():
            color = agent_colors(agent_id - 1)
            path_x = [y for x, y in path]
            path_y = [x for x, y in path]
            ax.plot(path_x, path_y, color=color, linestyle='--', linewidth=2, label=f"Path {agent_id}")
        # ax.legend(loc='upper right')  # Update legend
        plt.draw()

    # Animation
    frames = max(len(path) for path in agent_paths.values())

    def animation_done(*args):
        draw_trails()

    ani = animation.FuncAnimation(fig, update, frames=frames, interval=interval, repeat=False)
    ani.event_source.add_callback(animation_done)  # Tie the callback to the animation ending

    # Show the plot
    plt.show()


if __name__ == "__main__":
    grid = [
        [0, 0, 0, 0, 0],
        [0, 1, 1, 0, 0],
        [0, 0, 0, 0, 0],
        [0, 0, 1, 1, 0],
        [0, 0, 0, 0, 0],
    ]
    start_pos = {
        1: (0, 0),  # Agent 1 starts at (0, 0)
        2: (4, 0),  # Agent 2 starts at (4, 0)
        3: (0, 4),  # Agent 3 starts at (0, 4)
    }

    goal_pos = {
        1: (4, 4),  # Agent 1's goal is (4, 4)
        2: (0, 4),  # Agent 2's goal is (0, 4)
        3: (0, 2),  # Agent 3's goal is (4, 0)
    }

    planner = path_planning.PathFinding(np.asarray(grid), agents=3,
                                        start_pos=start_pos, goal_pos=goal_pos)
    print('start planning')
    solution = planner.cbs()
    print('end planning')
    if solution:
        visualize_cbs(grid, solution)
    else:
        print("No solution found.")
#
# grid_3 = [
#     [0, 0, 1, 0, 0, 0, 1],
#     [1, 0, 1, 0, 1, 0, 1],
#     [1, 0, 0, 0, 1, 0, 1],
#     [0, 0, 1, 0, 0, 0, 1],
#     [1, 0, 1, 1, 1, 0, 0],
#     [1, 0, 0, 0, 0, 1, 0],
# ]
# start_3 = {1: (0, 0), 2: (5, 6)}
# goal_3 = {1: (5, 6), 2: (0, 0)}
#
#
#   grid = [
#         [0, 0, 0, 0, 0],
#         [0, 1, 1, 0, 0],
#         [0, 0, 0, 0, 0],
#         [0, 0, 1, 1, 0],
#         [0, 0, 0, 0, 0],
#     ]
#
#     # Start and goal positions for 3 agents
#     start_pos = {
#         1: (0, 0),  # Agent 1 starts at (0, 0)
#         2: (4, 0),  # Agent 2 starts at (4, 0)
#         3: (0, 4),  # Agent 3 starts at (0, 4)
#     }
#
#     goal_pos = {
#         1: (4, 4),  # Agent 1's goal is (4, 4)
#         2: (0, 4),  # Agent 2's goal is (0, 4)
#         3: (4, 0),  # Agent 3's goal is (4, 0)
#     }