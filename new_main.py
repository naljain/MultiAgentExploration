import numpy as np
import time
from src.global_planner import GlobalPlanner
from quadrotor.MultiAgentSimulation import MultiAgentSimulation
from rotorpy.world import World
from rotorpy.trajectories.minsnap import MinSnap
import numpy as np
from src.global_planner import GlobalPlanner
from src.path_planning import PathFinding

from simulation_threading.ThreadPool import ThreadPoolManager
from rotorpy.sensors.range_sensors import TwoDRangeSensor

#sim params
map = np.loadtxt('src/test_map2')
# world = World.grid_forest(n_rows=2, n_cols=2, width=2, height=3, spacing=20)
world = World.from_file('./environment/dummy.json')
sensor_parameters = {'angular_fov': 360, 'angular_resolution': 1, 'fixed_heading': True, 'noise_density': 0.005}

t = 0
map_unexplored = True
num_agents = 2
agents = {}
Manager = ThreadPoolManager


# plannar params
agents = {1: 123, 2 : 222}  # , 2: 343, 3: 4444, 4: 444245, 5:13434} # dict = {agent num : agent id}
time_step = 1

bloat_val = 4  # BLOAT_VAL > RADIUS OF DRONE
unknown_travel = True
senor_range = 3  # 30 cm

# quad agent params
config_list = []


while map_unexplored:
    if t == 0:
        Planner = GlobalPlanner(map, agents, time_step, bloat_val, senor_range,
                                unknown_travel)
        initial_pose = Planner.start_pos
        goal_pose = Planner.run_planner()
        for i in range(num_agents):

            x0 = np.array([initial_pose[i + 1][0]/10, initial_pose[i + 1][1]/10, 0])
            xf = np.array([goal_pose[i + 1][0]/10, goal_pose[i + 1][1]/10, 0])
            config_list.append((MinSnap(points=np.row_stack((x0, xf)),
                                        v_avg=1.0, verbose=False), 0))
        sim = MultiAgentSimulation(thread_manager=Manager, world=world,
                                   num_agents=num_agents, t_final=10,
                                   t_step=1 / 100, config_list=config_list,map_resolution=10)
        range_sensor = TwoDRangeSensor(world, sampling_rate=100, angular_fov=sensor_parameters['angular_fov'], angular_resolution=sensor_parameters['angular_resolution'], fixed_heading=sensor_parameters['fixed_heading'], noise_density=sensor_parameters['noise_density'])
        # range_sensor.map_resolution = 0.1
        range_sensor.Dmax = 200
        sensor_data = []
        map_representations = []
        from src.abc import plot_2d_map
        for i in range(num_agents):
            x0 = {'x': np.array([initial_pose[i + 1][0]/10, initial_pose[i + 1][1]/10, 0])}
            sensor_data.append(range_sensor.measurement(x0))
            map_representations.append(sim.range_sensor_to_map_representation(x0, sensor_data[i]))
            plot_2d_map(map_representations[i], initial_pose, goal_pose, 2)

        t = 1
        break

results = sim.run_sim(sensor_parameters=sensor_parameters, range_sensor_plot=True, visualize=True, planner=True, planner_fcn=Planner.worker_fn)