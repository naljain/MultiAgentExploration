from rotorpy.vehicles.crazyflie_params import quad_params
from rotorpy.vehicles.multirotor import Multirotor
from quadrotor.mpc_other_nal_dynamics import MPC_Controller
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
traj = np.array([(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
                 (1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
                 (2, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
                 (2, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)])

# Create a multirotor object
vehicle = Multirotor(quad_params, control_abstraction='cmd_ctbm')
x0 = vehicle.initial_state
# Create a MPC controller object
controller = MPC_Controller(map, vehicle)
state = np.concatenate([x0['x'], x0['v']])
u = controller.compute_mpc_feedback(state,traj)

contol_step = {'cmd_thrust': u[0], 'cmd_moment': u[1:]}
init_state = {'x': np.array([0, 0, 0]), 'v': np.zeros(3,), 'q' : np.array([0,0,0,1]), 'w': np.zeros(3,), 'wind': np.array([0, 0, 0]), 'rotor_speeds': np.array([0, 0, 0,0])}

print(vehicle.step(init_state, contol_step, 1))

print(contol_step)
print('U is' , u)