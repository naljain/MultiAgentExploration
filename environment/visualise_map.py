import matplotlib.pyplot as plt
import map
import numpy as np
from mpl_toolkits.mplot3d import Axes3D


def plot_3d_map(map_array, title="3D Map Visualization"):
    """
    Visualize a 3D map with filled obstacles and goals.

    Parameters:
        map_array (np.array): 3D numpy array where:
                              - 0 represents empty space
                              - 1 represents obstacles
                              - 3 represents goals
        title (str): Title of the plot.
    """
    # Create a 3D figure
    fig = plt.figure(figsize=(10, 10))
    ax = fig.add_subplot(111, projection='3d')

    # Create a boolean array for filled regions (True for obstacles and goals)
    filled_map = (map_array == 1) | (map_array == 3)

    # Define colors for obstacles and goals
    colors = np.empty(map_array.shape, dtype=object)
    colors[map_array == 1] = 'black'  # Obstacles
    colors[map_array == 3] = 'red'    # Goals

    # Plot the voxel grid
    ax.voxels(filled_map, facecolors=colors, edgecolor='k', linewidth=0.2)

    # Set labels and title
    ax.set_xlabel("X-axis")
    ax.set_ylabel("Y-axis")
    ax.set_zlabel("Z-axis")
    ax.set_title(title)

    # Show plot
    plt.show()



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



# Example usage
if __name__ == "__main__":
    # Create a sample 2D numpy array (map)
    e1 = map.Environment((30, 30, 30), 8, 4, three_d= True)
    e2 = map.Environment((10, 10), 8, 4, three_d= False)

    e2.build_map()
    # plot_3d_map(e1.map, title="Sample 2D Map")
    plot_2d_map(e2.map, title="Sample 2D Map")
    print(e2.map)
    # print(e1.map)




