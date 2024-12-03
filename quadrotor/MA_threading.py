import threading
import queue

from rotorpy.vehicles.multirotor import Multirotor
from rotorpy.vehicles.crazyflie_params import quad_params
from rotorpy.controllers.quadrotor_control import SE3Control
from rotorpy.trajectories.hover_traj import HoverTraj
from rotorpy.trajectories.circular_traj import CircularTraj, ThreeDCircularTraj
from rotorpy.trajectories.lissajous_traj import TwoDLissajous
from rotorpy.trajectories.speed_traj import ConstantSpeed
from rotorpy.trajectories.minsnap import MinSnap 
from rotorpy.world import World
from rotorpy.utils.animate import animate
from rotorpy.simulate import merge_dicts

import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
import os
import yaml

# Replace multiprocessing.Pool with a custom ThreadPool
class ThreadPool:
    def __init__(self, num_threads):
        self.num_threads = num_threads
        self.tasks = queue.Queue()
        self.results = []
        self.lock = threading.Lock()

    def worker(self):
        while True:
            task = self.tasks.get()
            if task is None:
                break
            result = worker_fn(task)
            with self.lock:
                self.results.append(result)
            self.tasks.task_done()

    def map(self, func, iterable):
        self.results = []
        threads = []
        for _ in range(self.num_threads):
            t = threading.Thread(target=self.worker)
            t.start()
            threads.append(t)

        for item in iterable:
            self.tasks.put(item)

        for _ in range(self.num_threads):
            self.tasks.put(None)

        for t in threads:
            t.join()

        return self.results

# Modify the run_sim function to use a shared data structure
def run_sim(trajectory, t_offset, t_final=10, t_step=1/100, shared_data=None):
    mav = Multirotor(quad_params)
    controller = SE3Control(quad_params)

    # Init mav at the first waypoint for the trajectory.
    x0 = {'x': trajectory.update(t_offset)['x'],
        'v': np.zeros(3,),
        'q': np.array([0, 0, 0, 1]), # [i,j,k,w]
        'w': np.zeros(3,),
        'wind': np.array([0,0,0]),  # Since wind is handled elsewhere, this value is overwritten
        'rotor_speeds': np.array([1788.53, 1788.53, 1788.53, 1788.53])}
    
    time = [0]
    states = [x0]
    flats = [trajectory.update(time[-1] + t_offset)]
    controls = [controller.update(time[-1], states[-1], flats[-1])]

    while True:
        if time[-1] >= t_final:
            break
        time.append(time[-1] + t_step)
        states.append(mav.step(states[-1], controls[-1], t_step))
        flats.append(trajectory.update(time[-1] + t_offset))
        controls.append(controller.update(time[-1], states[-1], flats[-1]))

        # Update shared data
        if shared_data is not None:
            shared_data[threading.get_ident()] = states[-1]['x']

    time        = np.array(time, dtype=float)    
    states      = merge_dicts(states)
    controls    = merge_dicts(controls)
    flats       = merge_dicts(flats)

    return time, states, controls, flats

# Modify the worker_fn to include shared_data
def worker_fn(cfg):
    return run_sim(*cfg[:-1], shared_data=cfg[-1])

# Main code
if __name__ == "__main__":
    world = World.grid_forest(n_rows=2, n_cols=2, width=0.5, height=3, spacing=4)


    # Create a shared data structure for position sharing
    shared_data = {}
    dt = 1/100
    tf = 10

    # Modify the config_list to include shared_data
    config_list = [
        (TwoDLissajous(A=1, B=1, a=2, b=1, x_offset=1.5, y_offset=2, height=2.0), 0, tf, dt, shared_data),
        (TwoDLissajous(A=1, B=1, a=2, b=1, x_offset=1.75, y_offset=2, height=2.0), 0.5, tf, dt, shared_data),
        (TwoDLissajous(A=1, B=1, a=2, b=1, x_offset=2.0, y_offset=2, height=2.0), 1.0, tf, dt, shared_data),
        (TwoDLissajous(A=1, B=1, a=2, b=1, x_offset=2.25, y_offset=2, height=2.0), 1.5, tf, dt, shared_data),
        (TwoDLissajous(A=1, B=1, a=2, b=1, x_offset=2.50, y_offset=2, height=2.0), 2.0, tf, dt, shared_data)
    ]

    # Use ThreadPool instead of multiprocessing.Pool
    with ThreadPool(num_threads=len(config_list)) as pool:
        results = pool.map(worker_fn, config_list)

    # Concatentate all the relevant states/inputs for animation. 
    all_pos = []
    all_rot = []
    all_wind = []
    all_time = results[0][0]

    for r in results:
        all_pos.append(r[1]['x'])
        all_wind.append(r[1]['wind'])
        all_rot.append(Rotation.from_quat(r[1]['q']).as_matrix())

    all_pos = np.stack(all_pos, axis=1)
    all_wind = np.stack(all_wind, axis=1)
    all_rot = np.stack(all_rot, axis=1)

    # Check for collisions.
    collisions = find_collisions(all_pos, epsilon=2e-1)

    # Animate. 
    ani = animate(all_time, all_pos, all_rot, all_wind, animate_wind=False, world=world, filename=None)

    # Plot the positions of each agent in 3D, alongside collision events (when applicable)
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    colors = plt.cm.tab10(range(all_pos.shape[1]))
    for mav in range(all_pos.shape[1]):
        ax.plot(all_pos[:, mav, 0], all_pos[:, mav, 1], all_pos[:, mav, 2], color=colors[mav])
        ax.plot([all_pos[-1, mav, 0]], [all_pos[-1, mav, 1]], [all_pos[-1, mav, 2]], '*', markersize=10, markerfacecolor=colors[mav], markeredgecolor='k')
    world.draw(ax)
    for event in collisions:
        ax.plot([all_pos[event['timestep'], event['agents'][0], 0]], [all_pos[event['timestep'], event['agents'][0], 1]], [all_pos[event['timestep'], event['agents'][0], 2]], 'rx', markersize=10)
    ax.set_xlabel("x, m")
    ax.set_ylabel("y, m")
    ax.set_zlabel("z, m")

    plt.show()