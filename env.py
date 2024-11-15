class Environment:
    def __init__(self, size, obstacles, time_step, three_d = False):
        self.size = size
        self.x, self.y = size
        self.time_step = time_step
        self.obstacles = obstacles
        self.three_d = three_d

    def build_grid(self):
        pass

    def generate_obstacles(self):
        pass

    def generate_goals(self):
        pass

