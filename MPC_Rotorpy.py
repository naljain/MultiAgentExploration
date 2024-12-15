from rotorpy.vehicles.crazyflie_params import quad_params
from rotorpy.vehicles.multirotor import Multirotor
from quadrotor.Dynamics_Rotorpy import MPC_RotorPy
from src.global_planner import GlobalPlanner
import numpy as np
import time
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation as R
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
                            [0.5, 0.5, 0.5, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1788.53, 1788.53, 1788.53, 1788.53]])
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
states = [init_x]
flats = [traj_min.update(times[-1])]
controls = [controller_se3.update(times[-1], states[-1], flats[-1])]
# u_se3 = controller_se3.update(0, init_x, traj_min.update(0))
controller.take_init_guess(controls[-1])
u_ref = np.array([controls[-1]['cmd_motor_speeds'], controls[-1]['cmd_motor_speeds'], controls[-1]['cmd_motor_speeds'],controls[-1]['cmd_motor_speeds'],controls[-1]['cmd_motor_speeds']])
u_mpc_next = controller.compute_mpc_feedback(init_x,waypoints_minsnap, u_ref)
u_control_next = {'cmd_motor_speeds': u_mpc_next} 
next_state = vehicle.step(init_x, u_control_next, 0.05)

counter = 0
times.append(times[-1] + t_step)
while times[-1] < 2: #or np.linalg.norm(states[-1]['x'] - waypoints_minsnap[-1]) > 0.2:
    
    states.append(next_state)
    try:
        flats.append(traj_min.update(times[-1]))
    except:
        break
    controls.append(controller_se3.update(times[-1], states[-1], flats[-1]))

    controller.take_init_guess(controls[-1])
    u_ref = np.array([controls[-1]['cmd_motor_speeds'], controls[-1]['cmd_motor_speeds'], controls[-1]['cmd_motor_speeds'],controls[-1]['cmd_motor_speeds'],controls[-1]['cmd_motor_speeds']])
    u_mpc_next = controller.compute_mpc_feedback(next_state,waypoints_minsnap,u_ref)
    u_control_next = {'cmd_motor_speeds': u_mpc_next}
    next_state = vehicle.step(next_state, u_control_next, 0.05)
    times.append(times[-1] + t_step)
    print('Time:', times[-1])
    print('Error:', np.linalg.norm(states[-1]['x'] - waypoints_minsnap[-1][:3]))

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

x_vals = [state['x'][0] for state in states]
y_vals = [state['x'][1] for state in states]
z_vals = [state['x'][2] for state in states]
quat_vals = [state['q'] for state in states]

# Convert quaternions to roll, pitch, yaw
rpy_vals = [R.from_quat(quat).as_euler('xyz', degrees=True) for quat in quat_vals]
roll_vals = [rpy[0] for rpy in rpy_vals]
pitch_vals = [rpy[1] for rpy in rpy_vals]
yaw_vals = [rpy[2] for rpy in rpy_vals]

ax.plot(x_vals, y_vals, z_vals, label='Trajectory')
ax.scatter(waypoints_minsnap[:, 0], waypoints_minsnap[:, 1], waypoints_minsnap[:, 2], color='r', label='Waypoints')
# ax.quiver(x_vals, y_vals, z_vals, roll_vals, pitch_vals, yaw_vals, length=0.1, normalize=True, color='r', label='Orientation')

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_xlim(-2, 5)
ax.set_ylim(-2, 5)
ax.legend()

# 2D plot of x and y
plt.figure()
plt.plot(x_vals, y_vals, label='2D Trajectory')
plt.scatter(waypoints_minsnap[:, 0], waypoints_minsnap[:, 1], color='r', label='Waypoints')
plt.xlabel('X')
plt.ylabel('Y')
plt.xlim(-0.1, 1)
plt.ylim(-0.1, 1)
plt.legend()
plt.show()

plt.show()
