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

# Also, worlds are how we construct obstacles. The following class contains methods related to constructing these maps. 
from rotorpy.world import World

import os
import numpy as np
class Quadrotor():
    def __init__(self, world=None, x0=None):
        self.x0 = x0

        # Class instantiates the enviroment and quadrotor

        # world = World.from_file(os.path.abspath(os.path.join(os.path.dirname(''),'..','MultiAgentExploration','rotorpy','rotorpy','worlds','double_pillar.json')))

        self.sim_instance = Environment(vehicle=Multirotor(quad_params),           # vehicle object, must be specified. 
                           controller=SE3Control(quad_params),        # controller object, must be specified.
                           trajectory=HoverTraj,         # trajectory object, must be specified.
                           wind_profile=None,               # OPTIONAL: wind profile object, if none is supplied it will choose no wind. 
                           sim_rate     = 100,                        # OPTIONAL: The update frequency of the simulator in Hz. Default is 100 Hz.
                           imu          = None,                       # OPTIONAL: imu sensor object, if none is supplied it will choose a default IMU sensor.
                           mocap        = None,                       # OPTIONAL: mocap sensor object, if none is supplied it will choose a default mocap.  
                           estimator    = None,                       # OPTIONAL: estimator object
                           world        = world,                      # OPTIONAL: the world, same name as the file in rotorpy/worlds/, default (None) is empty world
                           safety_margin= 0.25                        # OPTIONAL: defines the radius (in meters) of the sphere used for collision checking
                       )
    
    def discrete_time_dynamics(self, x_t, u_t, delta_t):
        '''Dynamics of the quadrotor. Returns (x_{t+delta_t}, x_dot) given (x_t, u_t, delta_t)'''
        return self. sim_instance. vehicle.step(x_t, u_t, delta_t), self.sim_instance.vehicle.statedot(x_t, u_t, delta_t)