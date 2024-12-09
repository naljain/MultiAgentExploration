from rotorpy.vehicles.crazyflie_params import quad_params
from rotorpy.vehicles.multirotor import Multirotor
from quadrotor.mpc_other_nal_dynamics import MPC_Controller
from src.global_planner import GlobalPlanner
import numpy as np
import time
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
traj = np.array([(0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0, 0, 0),
                 (1, 1, 0.1, 0, 0, 0, 0, 0, 0, 0, 0, 0),
                 (2, 2, 0.1, 0, 0, 0, 0, 0, 0, 0, 0, 0),
                 (2, 3, 0.1, 0, 0, 0, 0, 0, 0, 0, 0, 0)])

# Create a multirotor object
vehicle = Multirotor(quad_params, control_abstraction='cmd_ctbm')
x0 = vehicle.initial_state
# Create a MPC controller object
controller = MPC_Controller(map, vehicle)

# u = controller.compute_mpc_feedback(x0,traj)
# contol_step = {'cmd_thrust': u[0], 'cmd_moment': u[1:]}
#
# next_x = (vehicle.step(x0, contol_step, 0.1))
# u_next = controller.compute_mpc_feedback(next_x,traj)
# u_control_next = {'cmd_thrust': u_next[0], 'cmd_moment': u_next[1:]}
# print(u_next)
# print('next_x_1', next_x)
#
# next_x_2 = (vehicle.step(next_x, u_control_next, 0.1))
# u_next_2 = controller.compute_mpc_feedback(next_x_2,traj)
# print('next_x_2', next_x_2)
# print(u_next_2)

init_x = x0
u_mpc_next = controller.compute_mpc_feedback(init_x,traj)
u_control_next = {'cmd_thrust': u_mpc_next[0], 'cmd_moment': u_mpc_next[1:]}
next_state_prev = vehicle.step(init_x, u_control_next, 0.1)
counter = 0
while True:
    counter +=1
    # if counter > 3: break
    next_state = vehicle.step(next_state_prev, u_control_next, 0.1)
    u_mpc_next = controller.compute_mpc_feedback(next_state,traj)
    u_control_next = {'cmd_thrust': u_mpc_next[0], 'cmd_moment': u_mpc_next[1:]}
    next_state_prev = next_state
    if counter > 100:
        time.sleep(2)
        print(next_state)

#
#
#
