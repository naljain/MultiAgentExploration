from quadrotor.MultiAgentSimulation import MultiAgentSimulation
from rotorpy.world import World

world = World.grid_forest(n_rows=2, n_cols=2, width=0.5, height=3, spacing=4)
sensor_parameters = {'angular_fov': 360, 'angular_resolution': 1, 'fixed_heading': True, 'noise_density': 0.005}
# sensor_parameters = None

sim = MultiAgentSimulation(world=world, num_agents=3, t_final=10, t_step=1/100, config_list="Cool")
sim.run_sim(sensor_parameters=sensor_parameters, range_sensor_plot=True, visualize=True)