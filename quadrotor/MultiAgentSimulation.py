import threading
import queue
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
import copy
class ThreadPoolManager:
    def __init__(self, num_threads, worker_fn=None):
        self.pool = ThreadPool(num_threads=num_threads, worker_fn=worker_fn)

    def __enter__(self):
        return self.pool
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.pool.close()
        self.pool.join()

class ThreadPool:
    def __init__(self, num_threads, worker_fn=None):
        self.num_threads = num_threads
        self.tasks = queue.Queue()
        self.results = []
        self.lock = threading.Lock()
        self.workers = []
        self.worker_fn = worker_fn

    def worker(self):
        while True:
            task = self.tasks.get()
            if task is None:
                self.tasks.task_done()
                break
            result = self.worker_fn(task)
            with self.lock:
                self.results.append(result)
            self.tasks.task_done()

    def map(self, func, iterable):
        self.results = []
        self.workers = []
        for _ in range(self.num_threads):
            t = threading.Thread(target=self.worker)
            t.start()
            self.workers.append(t)

        for item in iterable:
            self.tasks.put(item)
        
        self.tasks.join()
        return self.results
    
    def close(self):
        for _ in range(self.num_threads):
            self.tasks.put(None)

    def join(self):
        for worker in self.workers:
            worker.join()


from rotorpy.vehicles.multirotor import Multirotor
from rotorpy.vehicles.crazyflie_params import quad_params
from rotorpy.controllers.quadrotor_control import SE3Control
from rotorpy.world import World
from rotorpy.simulate import merge_dicts
from rotorpy.utils.animate import animate
from rotorpy.trajectories.hover_traj import HoverTraj
from rotorpy.trajectories.lissajous_traj import TwoDLissajous
from rotorpy.trajectories.minsnap import MinSnap
from rotorpy.sensors.range_sensors import TwoDRangeSensor
from matplotlib.animation import FuncAnimation


class MultiAgentSimulation:
    """
    MultiAgentSimulation is a class that simulates multiple agents in parallel using a thread pool.
    The main purpose of the thread pool is to allow for shared data between instances. 
    The primary goal is to pass position data of agents which serve as 'obstacles' to other agents. 
    Each agent can then measure range data from the world and pass it to a motion planner.
    """
    def __init__(self, world, num_agents, t_final=10, t_step=1/100, config_list=[], config_defaults="",):
        """Initializes the MultiAgentSimulation class with variables for the high-level multi-simulation and constant parameters for each instance.

        Args:
            num_agents (int): Desired number of agents.
            t_final (float, optional): duration of the sim. Defaults to 10. [seconds]
            t_step (float, optional): timestep for the sim. Defaults to 1/100. [seconds]
            config_list (list, optional): list of configurations that define instances. Includes time_offset, trajectory. Defaults to [].
            config_defaults (str, optional): Default configuration for each instance. ['Hover', 'Cool' >B-)] Defaults to "".
            visualize (bool, optional): Plot simulation. Defaults to False.
        """        
        self.world = world
        self.num_agents = num_agents
        self.t_final = t_final
        self.t_step = t_step
        self.config_list = config_list
        self.config_defaults = config_defaults
    
    def load_config(self, config_list, shared_data=None):
        """Loads the configuration list that defines the parameters for each instance. Empty list loads a default configuration based on number of agents.

        Args:
            config_list (dict): list of configurations that define instances. Includes time_offset, trajectory.
        """        
        if shared_data is None:
            shared_data = {}

        if not config_list:
            # Default
            generated_configs = []
            for i in range(self.num_agents):
                generated_configs.append((HoverTraj(x0=np.array([i, i, 5])), 0, self.t_final, self.t_step, shared_data))
        elif config_list == "Hover":
            # Hover
            generated_configs = []
            for i in range(self.num_agents):
                generated_configs.append((HoverTraj(x0=np.array([i, i, 5])), 0, self.t_final, self.t_step, shared_data))
        elif config_list == "Cool":
            # Figure 8
            generated_configs = []
            x_offset = 1.5
            time_offset = 0
            for i in range(self.num_agents):
                generated_configs.append((TwoDLissajous(A=1, B=1, a=2, b=1, x_offset=x_offset, y_offset=2, height=2.0), time_offset, self.t_final, self.t_step, shared_data))
                x_offset += 0.25
                time_offset += 0.5
        else:
            # Custom
            generated_configs = []
            for i in range(len(generated_configs)):
                generated_configs.append((config_list[i][0], config_list[i][1], self.t_final, self.t_step, shared_data))

        return generated_configs
    
    def worker_fn(self, cfg):
        return self.single_agent_sim(*cfg[:-1], shared_data=cfg[-1])
    
    def world_update(self,world, position_data):
        for pos in position_data['positions']:
            # Replace position of other agents wtih 0.1x0.1x0.2m blocks in this world instance.
            block = {'extents': [pos[0], pos[0]+0.1, pos[1], pos[1]+0.1, pos[2]-0.1, pos[2]+0.1], 'color': [1, 0, 0]}
            world.world['blocks'].append(block)
            return world
            
    def single_agent_sim(self, trajectory, t_offset, t_final=10, t_step=1/100, shared_data=None):
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
        world = copy.deepcopy(self.world)
        if shared_data is not None:
            shared_data[threading.get_ident()] = {'time':[], 'positions':[], 'sensor_data':[]}
        # if self.range_sensor:
        #     # Sensor intrinsics


        while True:
            # Update shared data
            if shared_data is not None:
                shared_data[threading.get_ident()]['positions'].append(states[-1]['x'])
                shared_data[threading.get_ident()]['time'].append(time[-1])
                world = self.world_update(world, (shared_data.copy()).pop(threading.get_ident()))
                # if self.range_sensor:

                

            if time[-1] >= t_final:
                break
            time.append(time[-1] + t_step)
            states.append(mav.step(states[-1], controls[-1], t_step))
            flats.append(trajectory.update(time[-1] + t_offset))
            controls.append(controller.update(time[-1], states[-1], flats[-1]))

            # Update shared data
            # if shared_data is not None:
            #     shared_data[threading.get_ident()] = states[-1]['x']

        time        = np.array(time, dtype=float)    
        states      = merge_dicts(states)
        controls    = merge_dicts(controls)
        flats       = merge_dicts(flats)

        return time, states, controls, flats

    def find_collisions(self, all_positions, epsilon=1e-1):
        """
        Checks if any two agents get within epsilon meters of any other agent. 
        Inputs:
            all_positions: the position vs time for each agent concatenated into one array. 
            epsilon: the distance threshold constituting a collision. 
        Outputs:
            collisions: a list of dictionaries where each dict describes the time of a collision, agents involved, and the location. 
        """

        N, M, _ = all_positions.shape
        collisions = []

        for t in range(N):
            # Get positions. 
            pos_t = all_positions[t]

            dist_sq = np.sum((pos_t[:, np.newaxis, :] - pos_t[np.newaxis, :, :])**2, axis=-1)

            # Set diagonal to a large value to avoid false positives. 
            np.fill_diagonal(dist_sq, np.inf)

            close_pairs = np.where(dist_sq < epsilon**2)

            for i, j in zip(*close_pairs):
                if i < j: # avoid duplicate pairs.
                    collision_info = {
                        "timestep": t,
                        "agents": (i, j),
                        "location": pos_t[i]
                    }
                    collisions.append(collision_info)

        return collisions
    
    def run_sim(self, sensor_parameters= {'angular_fov': 360, 'angular_resolution': 1, 'fixed_heading': True, 'noise_density': 0.005}, 
                range_sensor_plot=False, visualize=False):
        data = {'number_agents':self.num_agents}
        with ThreadPoolManager(num_threads=self.num_agents,worker_fn=self.worker_fn) as pool:
            config_list = self.load_config(self.config_list, shared_data=data)
            results = pool.map(pool.worker_fn, config_list)
        
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
        collisions = self.find_collisions(all_pos, epsilon=2e-1)

        if visualize:
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
        
        # if range_sensor:
        #     # Sensor intrinsics
        #     angular_fov = 360
        #     angular_resolution = 1
        #     fixed_heading = True
        #     noise_density = 0.005

        #     drone_id = 0

        if range_sensor_plot:
            (fig, axes) = plt.subplots(nrows=1, ncols=2, num="Ray Intersections Test")
            N, M, _ = all_pos.shape
            ani = FuncAnimation(fig, update_plot, frames=range(0, N), repeat=True)
            plt.show()

# Main code
if __name__ == "__main__":
    world = World.grid_forest(n_rows=2, n_cols=2, width=0.5, height=3, spacing=4)
    sensor_parameters = {'angular_fov': 360, 'angular_resolution': 1, 'fixed_heading': True, 'noise_density': 0.005}

    sim = MultiAgentSimulation(world=world, num_agents=2, t_final=10, t_step=1/100, config_list="Cool")
    sim.run_sim()
