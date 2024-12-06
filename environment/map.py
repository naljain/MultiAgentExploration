import math
import random
import numpy as np

"""
Map is represented as either [x,y] or [x,y,z] array where:

0 : open space
1 : obstacle
3 : goals

should be in a different map instance
5 : discovered area
6 : undiscovered area 

# TODO : 
a) not sure if 5, 6 should be here or on another map copy
b) add walls so we know when we reach the edge of the environment 
"""


# Create the figure and axis


class Environment:
    def __init__(self, size, obstacles, goals, three_d=False, wind=False):
        self.size = size
        if three_d:
            self.x, self.y, self.z = size
        else:
            self.x, self.y = size

        self.obstacles = obstacles
        self.three_d = three_d
        self.wind = wind
        self.obstacle_shapes = ['rectangle', 'circle', 'square']

        # create map array
        self.map = np.zeros(self.size)
        self.goals = np.zeros(goals)

    def build_map(self):
        self.generate_obstacles()
        self.generate_goals()

    def generate_obstacles(self):
        # TODO extend to different obstacle sizes, simplifying to only
        #  square obstacles right now

        for _ in range(self.obstacles):
            # Generate obstacle size
            obj_size = random.randint(2, min(self.x, self.y) // 4)
            obj_height = random.randint(2,
                                        self.z) if self.three_d else 1

            # Generate obstacle center coordinates
            obj_centre_x = random.randint(obj_size // 2,
                                          self.x - obj_size // 2 - 1)
            obj_centre_y = random.randint(obj_size // 2,
                                          self.y - obj_size // 2 - 1)

            if self.three_d:
                obj_centre_z = random.randint(obj_height // 2,
                                              self.z - obj_height // 2 - 1)

                # Ensure obstacle fits within bounds
                self.map[
                    obj_centre_x - obj_size // 2: obj_centre_x + obj_size // 2,
                    obj_centre_y - obj_size // 2: obj_centre_y + obj_size // 2,
                    obj_centre_z - obj_height // 2: obj_centre_z + obj_height // 2
                    ] = 1
                # print('height', obj_height)
                # print('size', obj_size)
                # print('X', obj_centre_x - obj_size // 2, obj_centre_x + obj_size // 2)
                # print('Y', obj_centre_y - obj_size // 2, obj_centre_y + obj_size // 2)
                # print('Z', obj_centre_z - obj_height // 2, obj_centre_z + obj_height // 2)

            else:
                self.map[
                obj_centre_x - obj_size // 2: obj_centre_x + obj_size // 2,
                obj_centre_y - obj_size // 2: obj_centre_y + obj_size // 2
                ] = 1

    def generate_goals(self):
        for i in range(len(self.goals)):
            if self.three_d:
                goal_x = random.randint(1, self.x - 1)
                goal_y = random.randint(1, self.y - 1)
                goal_z = random.randint(1, self.z - 1)
                self.map[goal_x, goal_y, goal_z] = 3

            else:
                goal_x = random.randint(1, self.x-1)
                goal_y = random.randint(1, self.y-1)
                self.map[goal_x, goal_y] = 3

            # TODO add check if goal is created on top of obstacle / unreachable

        def save_map(self, name, path):
            pass

    def check_collision(self, pose):
        pass

    def check_valid_move(self, move):
        pass
