import random
import numpy as np


class Environment:
    def __init__(self, size, obstacles, three_d = False, wind = False,):
        self.size = size
        self.x, self.y = size
        self.obstacles = obstacles
        self.three_d = three_d
        self.wind = wind

    def build_map(self):
        # if self.three_d:
        #     # np.array( x , y . z)
        # else:
            # np.array (x , y)
        pass

    def generate_obstacles(self):
        # if self.three_d:
        #     # 3d obstacles
        # else:
            # 2d obstacles
        pass

    def generate_goals(self):
        pass

