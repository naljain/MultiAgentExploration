from quadrotor.MultiAgentSimulation import MultiAgentSimulation
from rotorpy.world import World
from rotorpy.trajectories.minsnap import MinSnap
import numpy as np

world = World.grid_forest(n_rows=2, n_cols=2, width=0.5, height=3, spacing=4)
sensor_parameters = {'angular_fov': 360, 'angular_resolution': 1, 'fixed_heading': True, 'noise_density': 0.005}
# sensor_parameters = None
num_agents = 3
# Create Custom Configurations
config_list = []
Nc = num_agents

np.random.seed(42)  # For reproducibility
initial_positions = np.random.uniform(low=1, high=3, size=(num_agents, 2))
final_positions = np.random.uniform(low=1, high=3, size=(num_agents, 2))

t_offset = 0
for i in range(num_agents):
    # x = [X, Y, Z]
    x0 = np.array([initial_positions[i][0], initial_positions[i][1], 0])
    xf = np.array([final_positions[i][0], final_positions[i][1], 0])
    config_list.append((MinSnap(points=np.row_stack((x0, xf)), v_avg=1.0, verbose=False), t_offset))
    t_offset += 0.5

# sim = MultiAgentSimulation(world=world, num_agents=num_agents, t_final=10, t_step=1/100, config_list="Cool")
sim = MultiAgentSimulation(world=world, num_agents=num_agents, t_final=10, t_step=1/100, config_list=config_list)
sim.run_sim(sensor_parameters=sensor_parameters, range_sensor_plot=True, visualize=True)