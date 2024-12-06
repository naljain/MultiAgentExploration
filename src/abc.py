import numpy as np
import matplotlib.pyplot as plt

def plot_2d_map(map_array, title="2D Map Visualization"):
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

    # Add a grid for better visualization
    plt.grid(visible=True, color="black", linestyle="--", linewidth=0.5)
    plt.xticks(ticks=np.arange(-0.5, map_array.shape[1], 1), labels=[])
    plt.yticks(ticks=np.arange(-0.5, map_array.shape[0], 1), labels=[])

    # Add title and axis labels
    plt.title(title)
    plt.xlabel("X-axis")
    plt.ylabel("Y-axis")

    # Adjust grid to overlay properly
    plt.gca().set_xticks(np.arange(-0.5, map_array.shape[1], 1), minor=True)
    plt.gca().set_yticks(np.arange(-0.5, map_array.shape[0], 1), minor=True)
    plt.gca().grid(which="minor", color="black", linestyle="--", linewidth=0.5)
    plt.gca().tick_params(axis="both", which="minor", length=0)

    # Display the plot
    plt.show()

map = np.loadtxt('test_map')
plot_2d_map(map)