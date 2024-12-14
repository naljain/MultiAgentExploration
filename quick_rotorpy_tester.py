from rotorpy.vehicles.multirotor import Multirotor
from rotorpy.vehicles.crazyflie_params import quad_params
from rotorpy.controllers.quadrotor_control import SE3Control
from rotorpy.trajectories.minsnap import MinSnap
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation as R
counter = 0
t_offset = 0
t_step = 0.05
trajectory = MinSnap(points=np.array([[0 , 0, 0.5],[0.5, 0.5, 0.5]]), v_avg=0.5, verbose=False)
mav = Multirotor(quad_params)
controller = SE3Control(quad_params)
x0 = {'x': trajectory.update(t_offset)['x'],
    'v': np.zeros(3,),
    'q': np.array([0, 0, 0, 1]), # [i,j,k,w]
    'w': np.zeros(3,),
    'wind': np.array([0,0,0]),  # Since wind is handled elsewhere, this value is overwritten
    'rotor_speeds': np.array([1788.53, 1788.53, 1788.53, 1788.53])}

time = [0]
states = [x0]
flats = [trajectory.update(time[-1] + t_offset)]
controls = [controller.update(time[-1], states[-1], flats[-1])]
while time[-1] < 10:
    time.append(time[-1] + t_step)
    states.append(mav.step(states[-1], controls[-1], t_step))
    flats.append(trajectory.update(time[-1] + t_offset))
    controls.append(controller.update(time[-1], states[-1], flats[-1]))

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
# ax.quiver(x_vals, y_vals, z_vals, roll_vals, pitch_vals, yaw_vals, length=0.1, normalize=True, color='r', label='Orientation')

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_xlim(-0, 1)
ax.set_ylim(-0, 1)
ax.legend()

plt.show()