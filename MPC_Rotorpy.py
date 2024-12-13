from rotorpy.vehicles.crazyflie_params import quad_params
from rotorpy.vehicles.multirotor import Multirotor
from quadrotor.Dynamics_Rotorpy import MPC_RotorPy
from src.global_planner import GlobalPlanner
import numpy as np
import time
from mpl_toolkits.mplot3d import Axes3D
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
traj = np.array([(1, 1, 1, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0),
                 (1, 1, 1, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0),
                 (1, 1, 1, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0),
                 (1, 1, 1, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0)])

# Create a multirotor object
vehicle = Multirotor(quad_params, control_abstraction='cmd_motor_speeds', aero=True)
# Create a MPC controller object
controller = MPC_RotorPy(map, vehicle)

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
# Vehicle Initial State
vehicle.initial_state = {'x': np.array([0,0,0]),
                                            'v': np.zeros(3,),
                                            'q': np.array([0, 0, 0, 1]), # [i,j,k,w]
                                            'w': np.zeros(3,),
                                            'wind': np.array([0,0,0]),  # Since wind is handled elsewhere, this value is overwritten
                                            'rotor_speeds': np.array([1788.53, 1788.53, 1788.53, 1788.53])}

vehicle.initial_state = {'x': np.array([0.0001,0.0001,0.0001]),
                                            'v': np.array([0.000001,0.0000001,0.0000001]),
                                            'q': np.array([0.000001, 0.000001, 0.000001, 1]), # [i,j,k,w]
                                            'w': np.array([0.0000001,0.000001,0.0000001]),
                                            'wind': np.array([0,0,0]),  # Since wind is handled elsewhere, this value is overwritten
                                            'rotor_speeds': np.array([1788.53, 1788.53, 1788.53, 1788.53])}
vehicle.motor_noise = 0.0
init_x = vehicle.initial_state
u_mpc_next = controller.compute_mpc_feedback(init_x,traj)
u_control_next = {'cmd_motor_speeds': u_mpc_next} # Negative keeps it from falling faster. 
next_state = vehicle.step(init_x, u_control_next, 0.1)
counter = 0
to_plot = []
while True:
    counter +=1
    # if counter > 3: break
    u_mpc_next = controller.compute_mpc_feedback(next_state,traj)
    u_control_next = {'cmd_motor_speeds': u_mpc_next}
    next_state = vehicle.step(next_state, u_control_next, 0.1)
    to_plot.append(next_state)
    if counter > 10:
        break
# u_test = {'cmd_motor_speeds': np.array([1788.53, 1788.53, 1788.53, 1788.53])}
# while True:
#     counter +=1
#     next_state = vehicle.step(next_state, u_test, 0.1)
#     to_plot.append(next_state)
#     print('Position:', next_state['x'])
#     print('Angle:', next_state['q'])

#     if counter > 30:
#         break
import matplotlib.pyplot as plt

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

x_vals = [state['x'][0] for state in to_plot]
y_vals = [state['x'][1] for state in to_plot]
z_vals = [state['x'][2] for state in to_plot]

ax.plot(x_vals, y_vals, z_vals, label='Trajectory')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.legend()

plt.show()