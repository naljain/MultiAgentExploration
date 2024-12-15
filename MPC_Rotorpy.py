from rotorpy.vehicles.crazyflie_params import quad_params
from rotorpy.vehicles.multirotor import Multirotor
from quadrotor.Dynamics_Rotorpy import MPC_RotorPy
from src.global_planner import GlobalPlanner
import numpy as np
import time
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation as R
import copy
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
# traj = np.array([(1, 1, 1, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0),
#                  (1, 1, 1, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0),
#                  (1, 1, 1, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0),
#                  (1, 1, 1, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0)])
traj = np.array([(0.5, 0.8, 0.5, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1788.53, 1788.53, 1788.53, 1788.53),
                 (0.5, 0.8, 0.5, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1788.53, 1788.53, 1788.53, 1788.53),
                 (0.5, 0.8, 0.5, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1788.53, 1788.53, 1788.53, 1788.53),
                 (0.5, 0.8, 0.5, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1788.53, 1788.53, 1788.53, 1788.53)])
from rotorpy.trajectories.minsnap import MinSnap
from rotorpy.controllers.quadrotor_control import SE3Control
controller_se3 = SE3Control(quad_params)
# traj_min = MinSnap(points=np.array([[0 , 0, 0.5],[0.5, 0.8, 0.5]]), v_avg=0.5, verbose=False)
waypoints_minsnap = np.array([[0, 0, 0.5, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1788.53, 1788.53, 1788.53, 1788.53],
                            [0.1, 0.1, 0.5, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1788.53, 1788.53, 1788.53, 1788.53],
                            [0.2, 0.2, 0.5, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1788.53, 1788.53, 1788.53, 1788.53],
                            [0.2, 0.3 ,0.5, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1788.53, 1788.53, 1788.53, 1788.53],
                            [0.4, 0.4, 0.5, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1788.53, 1788.53, 1788.53, 1788.53],
                            [0.5, 0.5, 0.5, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1788.53, 1788.53, 1788.53, 1788.53],
                            [0.6, 0.6, 0.5, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1788.53, 1788.53, 1788.53, 1788.53],
                            [0.7, 0.7, 0.5, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1788.53, 1788.53, 1788.53, 1788.53],
                            [0.8, 0.8, 0.5, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1788.53, 1788.53, 1788.53, 1788.53],
                            [0.9, 0.9, 0.5, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1788.53, 1788.53, 1788.53, 1788.53],
                            [1, 1, 0.5, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1788.53, 1788.53, 1788.53, 1788.53],
                            [1.1, 1.1, 0.5, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1788.53, 1788.53, 1788.53, 1788.53],
                            [1.2, 1.2, 0.5, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1788.53, 1788.53, 1788.53, 1788.53],
                            [1.3, 1.3, 0.5, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1788.53, 1788.53, 1788.53, 1788.53],
                            [1.4, 1.4, 0.5, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1788.53, 1788.53, 1788.53, 1788.53],
                            [1.5, 1.5, 0.5, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1788.53, 1788.53, 1788.53, 1788.53],
                            [1.6, 1.6, 0.5, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1788.53, 1788.53, 1788.53, 1788.53],
                            [1.7, 1.7, 0.5, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1788.53, 1788.53, 1788.53, 1788.53],
                            [1.8, 1.8, 0.5, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1788.53, 1788.53, 1788.53, 1788.53],
                            [1.9, 1.9, 0.5, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1788.53, 1788.53, 1788.53, 1788.53],
                            [2, 2, 0.5, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1788.53, 1788.53, 1788.53, 1788.53],
                            [2.1, 2.1, 0.5, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1788.53, 1788.53, 1788.53, 1788.53],
                            [2.2, 2.2, 0.5, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1788.53, 1788.53, 1788.53, 1788.53],
                            [2.3, 2.3, 0.5, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1788.53, 1788.53, 1788.53, 1788.53],
                            [2.4, 2.4, 0.5, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1788.53, 1788.53, 1788.53, 1788.53],
                            [2.5, 2.5, 0.5, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1788.53, 1788.53, 1788.53, 1788.53],
                            [2.6, 2.6, 0.5, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1788.53, 1788.53, 1788.53, 1788.53],
                            [2.7, 2.7, 0.5, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1788.53, 1788.53, 1788.53, 1788.53],
                            [2.8, 2.8, 0.5, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1788.53, 1788.53, 1788.53, 1788.53],
                            [2.9, 2.9, 0.5, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1788.53, 1788.53, 1788.53, 1788.53],
                            [3, 3, 0.5, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1788.53, 1788.53, 1788.53, 1788.53]])

waypoints_plot = copy.copy(waypoints_minsnap)
traj_min = MinSnap(points=waypoints_minsnap[:,:3], v_avg=0.5, verbose=False)
# Create a multirotor object
vehicle = Multirotor(quad_params, control_abstraction='cmd_motor_speeds')
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
# vehicle.initial_state = {'x': np.array([0,0,0]),
#                                             'v': np.zeros(3,),
#                                             'q': np.array([0, 0, 0, 1]), # [i,j,k,w]
#                                             'w': np.zeros(3,),
#                                             'wind': np.array([0,0,0]),  # Since wind is handled elsewhere, this value is overwritten
#                                             'rotor_speeds': np.array([1788.53, 1788.53, 1788.53, 1788.53])}

vehicle.initial_state = {'x': np.array([0.0001,0.0001,0.5]),
                                            'v': np.array([0.000001,0.0000001,0.0000001]),
                                            'q': np.array([0.000001, 0.000001, 0.000001, 1]), # [i,j,k,w]
                                            'w': np.array([0.0000001,0.000001,0.0000001]),
                                            'wind': np.array([0,0,0]),  # Since wind is handled elsewhere, this value is overwritten
                                            'rotor_speeds': np.array([1788.53, 1788.53, 1788.53, 1788.53])}
vehicle.motor_noise = 0.0
init_x = vehicle.initial_state
t_step = 0.05
times=[0]
states_se3 = [init_x]
states_mpc = [init_x]
# flats = [traj_min.update(times[-1])]
# controls = [controller_se3.update(times[-1], states[-1], flats[-1])]
# controller.take_init_guess(controls[-1])
# u_ref = np.array([controls[-1]['cmd_motor_speeds'], controls[-1]['cmd_motor_speeds'], controls[-1]['cmd_motor_speeds'],controls[-1]['cmd_motor_speeds'],controls[-1]['cmd_motor_speeds']])
# u_mpc_next = controller.compute_mpc_feedback(init_x,waypoints_minsnap, u_ref)
# u_control_next = {'cmd_motor_speeds': u_mpc_next} 
# next_state = vehicle.step(init_x, u_control_next, 0.05)

counter = 0
# times.append(times[-1] + t_step)
N = 4
flats = []
controls = []
while (times[-1] < 60 and len(waypoints_minsnap) > N): #or np.linalg.norm(states[-1]['x'] - waypoints_minsnap[-1]) > 0.2:
    for i in range(N):
        flats.append(traj_min.update(times[-1]))
        controls.append(controller_se3.update(times[-1], states_se3[-1], flats[-1]))
        times.append(times[-1] + t_step)
        states_se3.append(vehicle.step(states_se3[-1], controls[-1], t_step))
    controller.take_init_guess(controls[-N])
    u_ref = [control['cmd_motor_speeds'] for control in controls[:N-1]]
    # try:
    u_mpc_next = controller.compute_mpc_feedback(states_mpc[-1],waypoints_minsnap[:4],u_ref)
    # except:
    #     break
    u_control_next = {'cmd_motor_speeds': u_mpc_next} 
    # u_control_next = {'cmd_motor_speeds': u_ref[0]}
    states_mpc.append(vehicle.step(states_mpc[-1], u_control_next, t_step))
    controls = controls[1:]
    # states.append(next_state)
    # try:
    #     flats.append(traj_min.update(times[-1]))
    # except:
    #     break
    # controls.append(controller_se3.update(times[-1], states[-1], flats[-1]))

    # controller.take_init_guess(controls[-1])
    # u_ref = np.array([controls[-1]['cmd_motor_speeds'], controls[-1]['cmd_motor_speeds'], controls[-1]['cmd_motor_speeds'],controls[-1]['cmd_motor_speeds'],controls[-1]['cmd_motor_speeds']])
    # u_mpc_next = controller.compute_mpc_feedback(next_state,waypoints_minsnap,u_ref)
    # u_control_next = {'cmd_motor_speeds': u_mpc_next}
    # next_state = vehicle.step(next_state, u_control_next, 0.05)
    # times.append(times[-1] + t_step)
    print('Time:', times[-1])
    print('Error:', np.linalg.norm(states_mpc[-1]['x'] - waypoints_minsnap[-1][:3]))
    waypoints_minsnap = waypoints_minsnap[1:]
    print('Waypoints left:', len(waypoints_minsnap))
    if waypoints_minsnap.size == 1:
        break
    # if np.linalg.norm(current_state["x"] - traj[0][0:3]) < 0.1:  # Threshold in meters
    #     print('true remove traj waypoint')
    #     # break
    #     traj.pop(0)  # Remove the waypoint if reached
    #     if not traj:
    #         break
    #     time.sleep(2)

# u_test = {'cmd_motor_speeds': np.array([1788.53, 1788.53, 1788.53, 1788.53])}

# while True:
#     counter +=1
#     next_state = vehicle.step(states[-1], controls[-1], t_step)
#     states.append(next_state)
#     times.append(times[-1] + t_step)
#     flats.append(traj_min.update(times[-1]))
#     controls.append(controller_se3.update(times[-1], states[-1], flats[-1]))
    
#     to_plot.append(next_state)
#     print('Position:', next_state['x'])
#     print('Angle:', next_state['q'])

#     if counter > 1000:
#         break
import matplotlib.pyplot as plt

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot for states_mpc
x_vals_mpc = [state['x'][0] for state in states_mpc]
y_vals_mpc = [state['x'][1] for state in states_mpc]
z_vals_mpc = [state['x'][2] for state in states_mpc]
quat_vals_mpc = [state['q'] for state in states_mpc]

# Convert quaternions to roll, pitch, yaw for states_mpc
rpy_vals_mpc = [R.from_quat(quat).as_euler('xyz', degrees=True) for quat in quat_vals_mpc]
roll_vals_mpc = [rpy[0] for rpy in rpy_vals_mpc]
pitch_vals_mpc = [rpy[1] for rpy in rpy_vals_mpc]
yaw_vals_mpc = [rpy[2] for rpy in rpy_vals_mpc]

ax.plot(x_vals_mpc, y_vals_mpc, z_vals_mpc, label='MPC Trajectory')
ax.scatter(waypoints_plot[:, 0], waypoints_plot[:, 1], waypoints_plot[:, 2], color='r', label='Waypoints')

# Plot for states_se3
x_vals_se3 = [state['x'][0] for state in states_se3]
y_vals_se3 = [state['x'][1] for state in states_se3]
z_vals_se3 = [state['x'][2] for state in states_se3]
quat_vals_se3 = [state['q'] for state in states_se3]

# Convert quaternions to roll, pitch, yaw for states_se3
rpy_vals_se3 = [R.from_quat(quat).as_euler('xyz', degrees=True) for quat in quat_vals_se3]
roll_vals_se3 = [rpy[0] for rpy in rpy_vals_se3]
pitch_vals_se3 = [rpy[1] for rpy in rpy_vals_se3]
yaw_vals_se3 = [rpy[2] for rpy in rpy_vals_se3]

ax.plot(x_vals_se3, y_vals_se3, z_vals_se3, label='SE3 Trajectory', linestyle='dashed')

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_xlim(-2, 5)
ax.set_ylim(-2, 5)
ax.legend()

# 2D plot of x and y for both trajectories
plt.figure()
plt.plot(x_vals_mpc, y_vals_mpc, label='2D MPC Trajectory')
plt.plot(x_vals_se3, y_vals_se3, label='2D SE3 Trajectory', linestyle='dashed')
plt.scatter(waypoints_plot[:, 0], waypoints_plot[:, 1], color='r', label='Waypoints')
plt.xlabel('X')
plt.ylabel('Y')
plt.xlim(-0.1, 1)
plt.ylim(-0.1, 1)
plt.legend()
plt.show()
