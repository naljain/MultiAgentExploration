import threading
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
import copy
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
from rotorpy.utils.plotter import plot_map


class MultiAgentSimulation:
    """
    MultiAgentSimulation is a class that simulates multiple agents in parallel using a thread pool.
    The main purpose of the thread pool is to allow for shared data between instances. 
    The primary goal is to pass position data of agents which serve as 'obstacles' to other agents. 
    Each agent can then measure range data from the world and pass it to a motion planner.
    """
    def __init__(self,thread_manager, world, num_agents, t_final=10, t_step=1/100, config_list=[], config_defaults="",):
        """Initializes the MultiAgentSimulation class with variables for the high-level multi-simulation and constant parameters for each instance.

        Args:
            num_agents (int): Desired number of agents.
            t_final (float, optional): duration of the sim. Defaults to 10. [seconds]
            t_step (float, optional): timestep for the sim. Defaults to 1/100. [seconds]
            config_list (list, optional): list of configurations that define instances. Includes time_offset, trajectory. Defaults to [].
            config_defaults (str, optional): Default configuration for each instance. ['Hover', 'Cool' >B-)] Defaults to "".
            visualize (bool, optional): Plot simulation. Defaults to False.
        """
        self.Manager = thread_manager        
        self.world = world
        self.num_agents = num_agents
        self.t_final = t_final
        self.t_step = t_step
        self.config_list = config_list
        self.config_defaults = config_defaults
        self.deg2rad = np.pi/180
        
        # Rotorpy World properties
        bounds = self.world.world['bounds']['extents']
        obstacles = self.world.world['blocks']

        # Initializing Numpy arrays for planner

        # get size of world
        x_min, x_max = bounds[0], bounds[1]
        y_min, y_max = bounds[2], bounds[3]
        # z_min, z_max = bounds[4], bounds[5]
        size_x = x_max - x_min
        size_y = y_max - y_min
        self.world_positions = np.zeros((size_x, size_y))
        # Prepared for sensor updating later
        self.sensor_reading = np.zeros((size_x, size_y))
        # get obstacle positions in world
        for obstacle in obstacles:
            x_min, x_max = obstacle['extents'][0], obstacle['extents'][1]
            y_min, y_max = obstacle['extents'][2], obstacle['extents'][3]
            # z_min, z_max = obstacle['extents'][4], obstacle['extents'][5]

            # Converts world to map representation
            self.world_positions[x_min:x_max, y_min:y_max] = 1
    
    def load_config(self, config_list, shared_data=None, sensor_parameters=None):
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
            for i in range(len(config_list)):
                generated_configs.append((config_list[i][0], config_list[i][1], self.t_final, self.t_step, shared_data))
        
        # Place sensor parameters in config structure
        # Sensor parameters can be vary for each 
        for i in range(len(generated_configs)):
            generated_configs[i] = (*generated_configs[i], sensor_parameters)

        return generated_configs
    
    def worker_fn(self, cfg):
        # return self.single_agent_sim(*cfg[:-2], shared_data=cfg[-2], sensor_parameters=cfg[-1])
        return self.single_agent_sim(*cfg)
    
    def sense_from_world(self, world, position_data, thread_id, sensor_parameters):
        current_position = {'x':position_data[thread_id]['positions'][-1]}
        position_data_copy = position_data.copy()
        position_data_copy.pop(thread_id)
        other_drone_positions = position_data_copy
        if len(other_drone_positions) != 0:
            for pos in other_drone_positions.values():
                # Replace position of other agents wtih 0.1x0.1x0.2m blocks in this world instance.
                pos = pos['positions'][-1]
                block = {'extents': [pos[0], pos[0]+0.1, pos[1], pos[1]+0.1, pos[2]-0.1, pos[2]+0.1], 'color': [0, 1, 0]}
                world.world['blocks'].append(block)
            # For now, TwoDRangeSensor is instantiated every time step because the rays cast in the world are calculated on instantiation.
            range_sensor = TwoDRangeSensor(world, sampling_rate=100, angular_fov=sensor_parameters['angular_fov'], angular_resolution=sensor_parameters['angular_resolution'], fixed_heading=sensor_parameters['fixed_heading'], noise_density=sensor_parameters['noise_density'])
            sensor_data = range_sensor.measurement(current_position)

            # # Remove the blocks that were added to the world
            # world.world['blocks'] = world.world['blocks'][:prev_obstacles]
            return sensor_data, world
        else:
            return None, None
    def single_agent_sim(self, trajectory, t_offset, t_final=10, t_step=1/100, shared_data=None, sensor_parameters=None):
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
        orig_world = copy.deepcopy(self.world)
        # Record number of obstacles in the world before adding the other agents.
        prev_obstacles = len(self.world.world['blocks'])
        if shared_data is not None:
            shared_data[threading.get_ident()] = {'time':[], 'positions':[], 'orientations':[],
                                                   'sensor_data':[], 'world':[],
                                                   'world_map_represenation':[], 'sensor_map_represenation':[]}

        while True:
            # Update shared data
            if shared_data is not None:
                shared_data[threading.get_ident()]['positions'].append(states[-1]['x'])
                shared_data[threading.get_ident()]['orientations'].append(states[-1]['q'])
                shared_data[threading.get_ident()]['time'].append(time[-1])
                shared_data[threading.get_ident()]['world_map_represenation'].append(self.world_positions)
                if sensor_parameters:
                    sensor_data, updated_world = self.sense_from_world(orig_world, shared_data, threading.get_ident(), sensor_parameters)
                    shared_data[threading.get_ident()]['sensor_data'].append(sensor_data)
                    shared_data[threading.get_ident()]['world'].append(copy.deepcopy(updated_world))
                    if sensor_data is not None:
                        shared_data[threading.get_ident()]['sensor_map_represenation'].append(self.range_sensor_to_map_representation(states[-1] , sensor_data))
                    # Remove the blocks that were added to the world
                    if orig_world is not None:
                        orig_world.world['blocks'] = orig_world.world['blocks'][:prev_obstacles]

            if time[-1] >= t_final:
                break
            time.append(time[-1] + t_step)
            states.append(mav.step(states[-1], controls[-1], t_step))
            flats.append(trajectory.update(time[-1] + t_offset))
            controls.append(controller.update(time[-1], states[-1], flats[-1]))

        time        = np.array(time, dtype=float)    
        states      = merge_dicts(states)
        controls    = merge_dicts(controls)
        flats       = merge_dicts(flats)

        return time, states, controls, flats, shared_data

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
    def plot_range(self, axes, world, pos, ranges, range_sensor, detected_edges=None):
        random_color = np.random.uniform(low=0, high=1, size=(100, 3))
        axes[0].plot(pos[0], pos[1], 'ro', zorder=10, label="Robot Position")
        if detected_edges is None:
            axes[0].plot((pos[0:2] + ranges[:,np.newaxis]*range_sensor.ray_vectors)[:,0], (pos[0:2] + ranges[:,np.newaxis]*range_sensor.ray_vectors)[:,1], 'ko', markersize=3, label="Ray Intersection Points")
        else:
            for (i,edge) in enumerate(detected_edges):
                axes[0].plot((pos[0:2] + ranges[edge,np.newaxis]*range_sensor.ray_vectors[edge])[:,0], (pos[0:2] + ranges[edge,np.newaxis]*range_sensor.ray_vectors[edge])[:,1], color=random_color[i], marker='o', markersize=3, linestyle='none', label="Ray Intersection Points")
        line_objects = []
        for i in range(range_sensor.N_rays):
            xvals = [pos[0], (pos[0:2] + ranges[:,np.newaxis]*range_sensor.ray_vectors)[i,0]]
            yvals = [pos[1], (pos[0:2] + ranges[:,np.newaxis]*range_sensor.ray_vectors)[i,1]]
            line = axes[0].plot(xvals, yvals, 'k-', linewidth=0.5, alpha=0.5)

        if detected_edges is None:
            axes[1].plot(range_sensor.ray_angles, range_sensor.ranges, 'r.', linewidth=1.5)
        else:
            axes[1].plot(range_sensor.ray_angles, range_sensor.ranges, 'k.', linewidth=1.5, alpha=0.1)
            for (i,edge) in enumerate(detected_edges):
                axes[1].plot(range_sensor.ray_angles[edge], range_sensor.ranges[edge], color=random_color[i], linestyle='none', marker='.')
        axes[1].plot([-180, 180], [range_sensor.Dmin, range_sensor.Dmin], 'k--', linewidth=1)
        axes[1].plot([-180, 180], [range_sensor.Dmax, range_sensor.Dmax], 'k--', linewidth=1)

        return
    def update_range_plot(self, t, data, axes, sensor_parameters):
        for ax in axes:
            ax.clear()
        # Get positions.
        pos_t = data['positions'][t]
        state = {'x': pos_t}
        data_world = data['world'][t]
        if data_world is None:
            return
        range_sensor = TwoDRangeSensor(data_world, sampling_rate=100, angular_fov=sensor_parameters['angular_fov'], angular_resolution=sensor_parameters['angular_resolution'], fixed_heading=sensor_parameters['fixed_heading'], noise_density=sensor_parameters['noise_density'])
        ranges = range_sensor.measurement(state)
        plot_map(axes[0], data_world.world)
        self.plot_range(axes, self.world, pos_t, ranges, range_sensor)
    def range_sensor_to_map_representation(self, position_data, sensor_data):
        map = copy.deepcopy(self.sensor_reading)
        x = int(position_data['x'][0])
        y = int(position_data['x'][1])
        # orientation = Rotation.fromposition_data['q'] # If fixed heading, no need to adjust
        
        for theta, r in enumerate(sensor_data):
            x_new = np.ceil(x + r*np.cos(theta*self.deg2rad)).astype(int)
            y_new = np.ceil(y + r*np.sin(theta*self.deg2rad)).astype(int)
            if x_new >= 0 and x_new < map.shape[0] and y_new >= 0 and y_new < map.shape[1]:
                map[x_new, y_new] = 1
        return map

    def run_sim(self, sensor_parameters = {'angular_fov': 360, 'angular_resolution': 1, 'fixed_heading': True, 'noise_density': 0.005}, 
                range_sensor_plot=False, visualize=False, planner=False, planner_fcn=None):
        data = {}
        if planner:
            assert planner_fcn is not None, "Planner function must be provided if planner is True."
            with self.Manager(num_threads=self.num_agents,worker_fn=self.worker_fn, planner=planner, planner_fnc=planner_fcn) as pool:
                config_list = self.load_config(self.config_list, shared_data=data, sensor_parameters=sensor_parameters)
                config_list.append('planner')
                results = pool.map(config_list)
        else:
            with self.Manager(num_threads=self.num_agents,worker_fn=self.worker_fn) as pool:
                config_list = self.load_config(self.config_list, shared_data=data, sensor_parameters=sensor_parameters)
                results = pool.map(pool.worker_fn, config_list)
        
            # Concatentate all the relevant states/inputs for animation. 
        all_pos = []
        all_rot = []
        all_wind = []
        all_time = results[-1][0]

        for r in results[1:]:
            all_pos.append(r[1]['x'])
            all_wind.append(r[1]['wind'])
            all_rot.append(Rotation.from_quat(r[1]['q']).as_matrix())

        shared_data = results[-1][-1]
        all_pos = np.stack(all_pos, axis=1)
        all_wind = np.stack(all_wind, axis=1)
        all_rot = np.stack(all_rot, axis=1)

        # Check for collisions.
        collisions = self.find_collisions(all_pos, epsilon=2e-1)

        if visualize:
                # Animate. 
            ani = animate(all_time, all_pos, all_rot, all_wind, animate_wind=False, world=self.world, filename=None)

            # Plot the positions of each agent in 3D, alongside collision events (when applicable)
            fig = plt.figure()
            ax = fig.add_subplot(projection='3d')
            colors = plt.cm.tab10(range(all_pos.shape[1]))
            for mav in range(all_pos.shape[1]):
                ax.plot(all_pos[:, mav, 0], all_pos[:, mav, 1], all_pos[:, mav, 2], color=colors[mav])
                ax.plot([all_pos[-1, mav, 0]], [all_pos[-1, mav, 1]], [all_pos[-1, mav, 2]], '*', markersize=10, markerfacecolor=colors[mav], markeredgecolor='k')
            self.world.draw(ax)
            for event in collisions:
                ax.plot([all_pos[event['timestep'], event['agents'][0], 0]], [all_pos[event['timestep'], event['agents'][0], 1]], [all_pos[event['timestep'], event['agents'][0], 2]], 'rx', markersize=10)
            ax.set_xlabel("x, m")
            ax.set_ylabel("y, m")
            ax.set_zlabel("z, m")

            plt.show()

        if range_sensor_plot and sensor_parameters:
            for data in shared_data:
                N = len(shared_data[data]['positions'])
                (fig, axes) = plt.subplots(nrows=1, ncols=2, num="Ray Intersections Test")
                ani = FuncAnimation(fig, self.update_range_plot, frames=range(0, N), repeat=False, fargs=(shared_data[data], axes, sensor_parameters))
                plt.show()

        return shared_data
# Main code
if __name__ == "__main__":
    import sys
    import os 
    cwd = os.getcwd()
    sys.path.insert(0, cwd)
    from simulation_threading.ThreadPool import ThreadPoolManager


    world = World.grid_forest(n_rows=2, n_cols=2, width=1, height=3, spacing=4)
    sensor_parameters = {'angular_fov': 360, 'angular_resolution': 1, 'fixed_heading': True, 'noise_density': 0.005}
    # sensor_parameters = None

    sim = MultiAgentSimulation(thread_manager=ThreadPoolManager, world=world, num_agents=3, t_final=10, t_step=1/100, config_list="Cool")
    results = sim.run_sim(sensor_parameters=sensor_parameters, range_sensor_plot=True, visualize=True)
    print(results)
