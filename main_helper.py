import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap

def plot_array_as_grid(array):
    """
    Plots a 2D NumPy array as a grid with specified color mappings:
    -1: black, 0: white, 1: blue.
    """
    # Define the custom colormap
    cmap = ListedColormap(['black', 'white', 'blue'])

    # Normalize the values to match the colormap indices
    norm_array = (array + 1).astype(int)  # Map -1 -> 0, 0 -> 1, 1 -> 2

    # Plot the grid
    plt.imshow(norm_array, cmap=cmap, origin='upper')

    # Add gridlines
    plt.grid(visible=True, color='gray', linestyle='-', linewidth=0.5)

    # Remove axis ticks for a cleaner look
    plt.xticks([])
    plt.yticks([])

    # Show the plot
    plt.show()