from rotorpy.vehicles.crazyflie_params import quad_params
from rotorpy.vehicles.multirotor import Multirotor
from quadrotor.mpc_template import MPC_Controller
from src.global_planner import GlobalPlanner
import numpy as np

# plannar params
agents = {1: 123}  # , 2: 343, 3: 4444, 4: 444245, 5:13434} # dict = {agent num : agent id}
time_step = 1

bloat_val = 4  # BLOAT_VAL > RADIUS OF DRONE
unknown_travel = True
senor_range = 3  # 30 cm
map = np.loadtxt('src/test_map2')
Planner = GlobalPlanner(map, agents, time_step, bloat_val, senor_range,
                            unknown_travel)
initial_pose = Planner.start_pos
goal_pose = Planner.run_planner()

traj = np.array([(0, 0), (1, 1), (2, 2), (2, 3)])

# Create a multirotor object
vehicle = Multirotor(quad_params, control_abstraction='cmd_motor_thrusts')
x0 = vehicle.initial_state
# Create a MPC controller object
controller = MPC_Controller(map, vehicle)
state = np.concatenate([x0['x'], x0['v']])
u = controller.compute_mpc_feedback(state,traj)