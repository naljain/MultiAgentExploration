"""
Imports
"""
import numpy as np
from scipy.spatial.transform import Rotation  # This is a useful library for working with attitude.
import matplotlib.pyplot as plt
from numpy.linalg import inv
from numpy.linalg import cholesky
from math import sin, cos
from rotorpy.vehicles.multirotor import Multirotor
from pydrake.solvers import SnoptSolver

import math
from scipy.interpolate import interp1d
from scipy.integrate import ode
from scipy.integrate import solve_ivp
from scipy.linalg import expm
from scipy.linalg import solve_continuous_are
from pydrake.solvers import MathematicalProgram, Solve, OsqpSolver
import pydrake.symbolic as sym
from pydrake.symbolic import Expression


from pydrake.all import MonomialBasis, OddDegreeMonomialBasis, Variables

class MPC_Controller(object):
    """
    The controller is implemented as a class with two required methods: __init__() and update().
    The __init__() is used to instantiate the controller, and this is where any model parameters or
    controller gains should be set.
    In update(), the current time, state, and desired flat outputs are passed into the controller at
    each simulation step. The output of the controller depends on the control abstraction for Multirotor...
        if cmd_motor_speeds: the output dict should contain the key 'cmd_motor_speeds'
        if cmd_motor_thrusts: the output dict should contain the key 'cmd_rotor_thrusts'
        if cmd_vel: the output dict should contain the key 'cmd_v'
        if cmd_ctatt: the output dict should contain the keys 'cmd_thrust' and 'cmd_q'
        if cmd_ctbr: the output dict should contain the keys 'cmd_thrust' and 'cmd_w'
        if cmd_ctbm: the output dict should contain the keys 'cmd_thrust' and 'cmd_moment'
    """
    def __init__(self, map,  vehicle):
        """
        Use this constructor to save vehicle parameters, set controller gains, etc.
        Parameters:
            vehicle_params, dict with keys specified in a python file under /rotorpy/vehicles/

        """
        self.vehicle = vehicle
        self.map = map
        self.d_safe = 10
        self.Q = np.diag([2, 2, 1, 0, 0, 0,0,0,0,0,0,0])
        # self.Q[0,0] = 1
        # self.Q[]
        self.R = np.eye(4)
        self.thrust_d = np.array([0.2943, 0, 0, 0])


    def update(self, state):
        """
        This function receives the current time, true state, and desired flat
        outputs. It returns the command inputs.

        Inputs:
            t, present time in seconds
            state, a dict describing the present state with keys
                x, position, m
                v, linear velocity, m/s
                q, quaternion [i,j,k,w]
                w, angular velocity, rad/s
            flat_output, a dict describing the present desired flat outputs with keys
                x,        position, m
                x_dot,    velocity, m/s
                x_ddot,   acceleration, m/s**2
                x_dddot,  jerk, m/s**3
                x_ddddot, snap, m/s**4
                yaw,      yaw angle, rad
                yaw_dot,  yaw rate, rad/s

        Outputs:
            control_input, a dict describing the present computed control inputs with keys

                key, description, unit, (applicable control abstraction)
                cmd_motor_speeds, the commanded speed for each motor, rad/s, (cmd_motor_speeds)
                cmd_thrust, the collective thrust of all rotors, N, (cmd_ctatt, cmd_ctbr, cmd_ctbm)
                cmd_moment, the control moments on each boxy axis, N*m, (cmd_ctbm)
                cmd_q, desired attitude as a quaternion [i,j,k,w], , (cmd_ctatt)
                cmd_w, desired angular rates in body frame, rad/s, (cmd_ctbr)
                cmd_v, desired velocity vector in world frame, m/s (cmd_vel)
        """

        # Only some of these are necessary depending on your desired control abstraction.
        state = [state['x'], state['v']]
        u_from_mpc = self.compute_mpc_feedback(state)
        cmd_motor_speeds = np.zeros((4,))
        cmd_motor_thrusts = u_from_mpc # needs to have dimensions of 4.
        cmd_thrust = 0
        cmd_moment = np.zeros((3,))
        cmd_q = np.array([0,0,0,1])
        cmd_w = np.zeros((3,))
        cmd_v = np.zeros((3,))

        control_input = {'cmd_motor_speeds':cmd_motor_speeds,
                         'cmd_motor_thrusts':cmd_motor_thrusts,
                         'cmd_thrust':cmd_thrust,
                         'cmd_moment':cmd_moment,
                         'cmd_q':cmd_q,
                         'cmd_w':cmd_w,
                         'cmd_v':cmd_v}
        return control_input



    def downsample_traj(self, waypoints, N):
        v_avg = 1
        # TODO this is def wrong bc we allow diagnol movements so need to fix this but for simplicity right now
        length_traj = len(waypoints) # in cm
        T_traj = length_traj/v_avg
        pass


    def add_intial_state_constraint(self, prog, x, x_current):
        """
        this x_current should come from position data of the agent, not
        from waypoints as the prev agent could have ended not on the waypoint
        for the prev traj segment
        """
        # n_x = x_current.shape[0]
        # print(x_current)
        state = np.concatenate([x_current['x'], x_current['v']])
        n_x = state.shape[0]
        print(n_x)
        for i in range(3):
            prog.AddBoundingBoxConstraint(state[i], state[i], x[0, i])


    def add_final_state_constraints(self):
        """
        i dont think we need to put the v_N =0 and a_N = 0,
        but if we do it would come here
        :return:
        """
        pass


    def add_input_saturation_constraint(self, prog, x, u, N):
        n_u = u.shape[1]
        # TODO : get u min and u max from rotorpy
        u_min = -10
        u_max = 10
        #in homework they have u_d here, not sure if we need that?
        for j in range(n_u):
            for i in range(N-1):
                prog.AddBoundingBoxConstraint(u_min, u_max, u[i, j])

    def symbolic_quadrotor_dynamics(self,x, u, mass, inertia, gravity=9.81):
        """
        Define symbolic quadrotor dynamics for pydrake.

        Parameters:
            mass: Mass of the quadrotor (float).
            inertia: Inertia matrix (3x3 NumPy array).
            gravity: Gravitational acceleration (default: 9.81).

        Returns:
            A tuple containing:
                - Symbolic variables for state (p, v, theta, omega).
                - Symbolic variables for control (F, tau).
                - Symbolic expressions for state derivatives (state_dot).
        """
        # Define symbolic state variables
        p = x[0:3]  # Position: [x, y, z]
        v = x[3:6]  # Velocity: [vx, vy, vz]
        theta = x[6:9] # Orientation: [roll, pitch, yaw]
        omega = x[9:12] # Angular velocity: [wx, wy, wz]

        # Define symbolic control inputs
        F = u[0]  # Thrust
        tau = u[1:4]  # Torques: [tau_x, tau_y, tau_z]

        # Gravity vector
        g = np.array([0, 0, -gravity])

        # Rotation matrix from body to world frame (roll-pitch-yaw)
        roll, pitch, yaw = theta
        R_x = np.array([
            [1, 0, 0],
            [0, np.cos(roll), -np.sin(roll)],
            [0, np.sin(roll), np.cos(roll)],
        ])
        R_y = np.array([
            [np.cos(pitch), 0, np.sin(pitch)],
            [0, 1, 0],
            [-np.sin(pitch), 0, np.cos(pitch)],
        ])
        R_z = np.array([
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw), np.cos(yaw), 0],
            [0, 0, 1],
        ])
        R = R_z @ R_y @ R_x  # Total rotation matrix

        # Translational dynamics
        thrust_world = R @ np.array([0, 0, F])  # Thrust in world frame
        linear_accel = thrust_world / mass + g  # Linear acceleration

        # Rotational dynamics
        omega_cross = np.cross(omega, inertia @ omega)  # Cross-product term
        angular_accel = np.linalg.inv(inertia) @ (np.array(tau) - omega_cross)

        # State derivatives
        p_dot = v  # Position derivative = velocity
        v_dot = linear_accel  # Velocity derivative = linear acceleration
        theta_dot = omega  # Orientation derivative = angular velocity
        omega_dot = angular_accel  # Angular velocity derivative = angular acceleration

        # Combine all state derivatives
        state_dot = np.concatenate([p_dot, v_dot, theta_dot, omega_dot])

        # return p, v, theta, omega, F, tau, state_dot
        return state_dot

    def add_dynamic_constraints(self, prog, x, u, rotorpy_model, T, N, mass, inertia):
        for k in range(N-1):
            xk = x[k]
            # print('len of xk in dyan co', len(xk))
            uk = u[k]
            x_dot = self.symbolic_quadrotor_dynamics(xk, uk, mass, inertia)
            # xk_1 = xk + x_dot*T
            xk_1 = [xk[i] + x_dot[i] * T for i in range(12)]
            # print('len of xk+1 in dyan co', len(xk_1))
            # prog.AddConstraint(xk_1 - xk == 0)
            for i in range(12):
                # prog.AddConstraint((xk_1[i] - xk[i]), np.array([0]))
                prog.AddConstraint((xk_1[i] - xk[i]) == 0)
                # prog.AddBoundingBoxConstraint(0, 0, xk_1[i] - xk[i], lb=np.array([0]), ub=np.array([0]))
        # prog.AddLinearEqualityConstraint((x[k + 1]), xk_1)



    # def add_dynamics_constraint(self, prog, x, u, N, T):
    #     state_for_rotorpy = {'x': x[0:3], 'v':x[3:6], 'q':x[6:10],}
    #     multirotor = self.vehicle
    #     for i in range(N-1):
    #
    #         controls = np.array([Expression(var) for var in u[i]])
    #         input_for_rotorpy = {'cmd_motor_thrusts': controls}
    #         dynamics = multirotor.statedot(state_for_rotorpy, input_for_rotorpy, T)
    #         statedot = np.array([dynamics['vdot'], dynamics['wdot']])
    #         prog.AddLinearEqualityConstraint((x[i+1]) - statedot, np.zeros(6)) # 6 elements in state??



    def barrier_dist(self, p_i, p_j):
        x_i, y_i = p_i
        x_j, y_j = p_j
        d = ((x_j - x_i) ** 2 + (y_j - y_i)**2) ** 0.5
        return d

    def add_barrier_obstacle_constraint(self, prog, x, N):
        map = self.map
        all_obstacles_x, all_obstacles_y = np.where(map == 1)
        len_all_obstacles = len(all_obstacles_x)
        d_safe = self.d_safe

        for k in range(N):
            xk, yk = x[k, 0:2]
            epsilon = 0
            min_dist = math.inf
            closest_obs = []

            # to make it run faster only look at moving window of map
            for i in range(len_all_obstacles):
                x, y = all_obstacles_x[i], all_obstacles_y[i]

                d = self.barrier_dist((xk,yk), (x,y))
                if d < min_dist:
                    min_dist = d
                    closest_obs = [(x, y)]
                    continue
                if d == min_dist:
                    closest_obs.append((x,y))

            for obs in closest_obs:
                barrier_cost = -np.log((min_dist - d_safe + epsilon))**2
                prog.AddCost(barrier_cost)

    def add_cost(self, prog, x, x_ref, u, N):
        u_ref = 0
        for k in range(N-1):
            # print((x[k] - x_ref[k]).T @ self.Q @ (x[k] - x_ref[k]))
            prog.AddQuadraticCost(
                (x[k] - x_ref[k]).T @ self.Q @ (x[k] - x_ref[k]) + (u[k] - self.thrust_d).T @ self.R @ (u[k]- self.thrust_d)
            )
            # prog.AddQuadraticCost((x[k] - x_ref[k]).T @ self.Q @ (x[k] - x_ref[k]))

        prog.AddQuadraticCost((x[N - 1] - x_ref[N - 1]).T @ self.Q @ (x[N - 1] - x_ref[N - 1]))

    def compute_mpc_feedback(self, x_current, x_ref):

        # QP params
        N = 4  # prediction horizon TODO NEEDS TO BE TUNED
        T = 0.1 # time step

        # initialise mathematical program
        prog = MathematicalProgram()

        # initialise decision variables
        x = np.zeros((N, 12), dtype= "object")
        for i in range(N):
            x[i] = prog.NewContinuousVariables(12, 'x_' + str(i))
        u = np.zeros((N-1, 4), dtype = "object")
        for i in range(N-1):
            u[i] = prog.NewContinuousVariables(4, 'u_' + str(i))

        # add constraints
        self.add_intial_state_constraint(prog, x, x_current)
        # self.add_input_saturation_constraint(prog, x, u, N)

        # self.add_dynamics_constraint(prog, x, u, N, T)

        # RotorPy model
        rotorpy_model = self.vehicle
        mass = rotorpy_model.mass
        Ixx = rotorpy_model.Ixx
        Iyy = rotorpy_model.Iyy
        Izz = rotorpy_model.Izz
        Ixy = rotorpy_model.Ixy
        Ixz = rotorpy_model.Ixz
        Iyz = rotorpy_model.Iyz
        inertia = rotorpy_model.inertia



        # Add dynamic constraints
        self.add_dynamic_constraints(prog, x, u, rotorpy_model, T, N, mass, inertia)


        # add cost
        # self.add_barrier_obstacle_constraint(prog, x, N)
        # TODO input x_ref
        self.add_cost(prog, x, x_ref, u, N)

        # solve the QP
        solver = SnoptSolver() # can be SNOPT if this doesnt work
        result = solver.Solve(prog)
        if result.is_success():
            print("Solution found!")
        else:
            print("Solver failed.")        # get u_mpc
        # u_mpc = result.GetSolution(u[0])
        u_mpc = result.GetSolution(u[0]) # this is from hw, so need to see if this is correct
        print(u_mpc)
        return u_mpc
    # def symbolic_dynamics(x, u, rotorpy_model):





