import threading
"""
Imports
"""
# The simulator is instantiated using the Environment class
from rotorpy.environments import Environment

# Vehicles. Currently there is only one. 
# There must also be a corresponding parameter file. 
from rotorpy.vehicles.multirotor import Multirotor
from rotorpy.vehicles.crazyflie_params import quad_params
# from rotorpy.vehicles.hummingbird_params import quad_params  # There's also the Hummingbird

# You will also need a controller (currently there is only one) that works for your vehicle. 
from rotorpy.controllers.quadrotor_control import SE3Control

# And a trajectory generator
from rotorpy.trajectories.hover_traj import HoverTraj
from rotorpy.trajectories.circular_traj import CircularTraj, ThreeDCircularTraj
from rotorpy.trajectories.lissajous_traj import TwoDLissajous
from rotorpy.trajectories.speed_traj import ConstantSpeed
from rotorpy.trajectories.minsnap import MinSnap

# You can optionally specify a wind generator, although if no wind is specified it will default to NoWind().
from rotorpy.wind.default_winds import NoWind, ConstantWind, SinusoidWind, LadderWind
from rotorpy.wind.dryden_winds import DrydenGust, DrydenGustLP
from rotorpy.wind.spatial_winds import WindTunnel

# You can also optionally customize the IMU and motion capture sensor models. If not specified, the default parameters will be used. 
from rotorpy.sensors.imu import Imu
from rotorpy.sensors.external_mocap import MotionCapture

# You can also specify a state estimator. This is optional. If no state estimator is supplied it will default to null. 
from rotorpy.estimators.wind_ukf import WindUKF

# Also, worlds are how we construct obstacles. The following class contains methods related to constructing these maps. 
from rotorpy.world import World

# Reference the files above for more documentation. 

# Other useful imports
import numpy as np                  # For array creation/manipulation
import matplotlib.pyplot as plt     # For plotting, although the simulator has a built in plotter
from scipy.spatial.transform import Rotation  # For doing conversions between different rotation descriptions, applying rotations, etc. 
import os                           # For path generation
import time
class MultiAgentSimulator:
    def __init__(self, num_drones, initial_positions, simulation_time):
        self.num_drones = num_drones
        self.lock = threading.Lock()
        self.positions = [None] * num_drones
        self.simulation_time = simulation_time
        self.drones = []
        self.stop_event = threading.Event()
        self.initial_positions = initial_positions
        try:
            assert len(initial_positions) == num_drones
        except AssertionError:
            print("Initial positions must be provided for each drone")
            return

    def run_drone(self, drone_id, initial_position):
        world = World.from_file(os.path.abspath(os.path.join(os.path.dirname(__file__),'rotorpy','rotorpy','worlds','double_pillar.json')))
        sim_instance = Environment(vehicle=Multirotor(quad_params),           # vehicle object, must be specified. 
                           controller=SE3Control(quad_params),        # controller object, must be specified.
                           trajectory=CircularTraj(radius=2),         # trajectory object, must be specified.
                           wind_profile=SinusoidWind(),               # OPTIONAL: wind profile object, if none is supplied it will choose no wind. 
                           sim_rate     = 100,                        # OPTIONAL: The update frequency of the simulator in Hz. Default is 100 Hz.
                           imu          = None,                       # OPTIONAL: imu sensor object, if none is supplied it will choose a default IMU sensor.
                           mocap        = None,                       # OPTIONAL: mocap sensor object, if none is supplied it will choose a default mocap.  
                           estimator    = None,                       # OPTIONAL: estimator object
                           world        = world,                      # OPTIONAL: the world, same name as the file in rotorpy/worlds/, default (None) is empty world
                           safety_margin= 0.25                        # OPTIONAL: defines the radius (in meters) of the sphere used for collision checking
                       )
        # drone = Quadrotor(world)
        # simulator = Simulator(drone, world)
        start_time = time.time()
        control_input = {'cmd_motor_speeds':0,
            'cmd_motor_thrusts':0,
            'cmd_thrust':0,
            'cmd_moment':0,
            'cmd_q':0,
            'cmd_w':0,
            'cmd_v':0,
            'cmd_acc': 0}
        x0 = initial_position
        sim_instance.vehicle.initial_state = x0
        position = x0
        while not self.stop_event.is_set() and (time.time() - start_time) < self.simulation_time:
            position = sim_instance.vehicle.step(position, control_input, 0.01)
            with self.lock:
                self.positions[drone_id] = position
            # Update obstacles based on other drone positions
            obstacles = [pos for i, pos in enumerate(self.positions) if i != drone_id]
            # world.update_obstacles(obstacles)
            time.sleep(0.01)
            print(f"Drone {drone_id} position: {position}")

    def run_simulation(self):
        threads = []

        for i in range(self.num_drones):
            thread = threading.Thread(target=self.run_drone, args=(i, self.initial_positions[i]))
            threads.append(thread)
            thread.start()
        
        time.sleep(self.simulation_time)
        self.stop_event.set()

        for thread in threads:
            thread.join()
        
        print("Simulation finished")
        print("Drone 0 positions:", self.positions[0])
        print("Drone 1 positions:", self.positions[1])
        print("Drone 2 positions:", self.positions[2])
# Usage
if __name__ == '__main__':
    x0_1 = {'x': np.array([0,0,0]),
    'v': np.zeros(3,),
    'q': np.array([0, 0, 0, 1]), # [i,j,k,w]
    'w': np.zeros(3,),
    'wind': np.array([0,0,0]),  # Since wind is handled elsewhere, this value is overwritten
    'rotor_speeds': np.array([1788.53, 1788.53, 1788.53, 1788.53])}

    x0_2 = {'x': np.array([1,1,0]),
    'v': np.zeros(3,),
    'q': np.array([0, 0, 0, 1]), # [i,j,k,w]
    'w': np.zeros(3,),
    'wind': np.array([0,0,0]),  # Since wind is handled elsewhere, this value is overwritten
    'rotor_speeds': np.array([1788.53, 1788.53, 1788.53, 1788.53])}

    x0_3 = {'x': np.array([-1,-1,0]),
    'v': np.zeros(3,),
    'q': np.array([0, 0, 0, 1]), # [i,j,k,w]
    'w': np.zeros(3,),
    'wind': np.array([0,0,0]),  # Since wind is handled elsewhere, this value is overwritten
    'rotor_speeds': np.array([1788.53, 1788.53, 1788.53, 1788.53])}

    inital_positions = [x0_1, x0_2, x0_3]

    simulator = MultiAgentSimulator(num_drones=3, initial_positions=inital_positions, simulation_time=1)
    simulator.run_simulation()