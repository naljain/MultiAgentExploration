import numpy as np
import matplotlib.pyplot as plt
from numpy.linalg import inv
from numpy.linalg import cholesky
from math import sin, cos
import math
from scipy.interpolate import interp1d
from scipy.integrate import ode
from scipy.integrate import solve_ivp
from scipy.linalg import expm
from scipy.linalg import solve_continuous_are

from pydrake.solvers import MathematicalProgram, Solve, OsqpSolver
import pydrake.symbolic as sym

from pydrake.all import MonomialBasis, OddDegreeMonomialBasis, Variables


class MPC(object):
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

    def __init__(self, vehicle_params):
        """
        Use this constructor to save vehicle parameters, set controller gains, etc.
        Parameters:
            vehicle_params, dict with keys specified in a python file under /rotorpy/vehicles/

        """
        pass

    def update(self, t, state, flat_output):
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
        cmd_motor_speeds = np.zeros((4,))
        cmd_motor_thrusts = np.zeros((4,))
        cmd_thrust = 0
        cmd_moment = np.zeros((3,))
        cmd_q = np.array([0, 0, 0, 1])
        cmd_w = np.zeros((3,))
        cmd_v = np.zeros((3,))

        control_input = {'cmd_motor_speeds': cmd_motor_speeds,
                         'cmd_motor_thrusts': cmd_motor_thrusts,
                         'cmd_thrust': cmd_thrust,
                         'cmd_moment': cmd_moment,
                         'cmd_q': cmd_q,
                         'cmd_w': cmd_w,
                         'cmd_v': cmd_v}
        return control_input















class Quadrotor(object):
    def __init__(self, Q, R, Qf):
        self.g = 9.81
        self.m = 1
        self.a = 0.25
        self.I = 0.0625
        self.Q = Q
        self.R = R
        self.Qf = Qf

        # Input limits
        self.umin = 0
        self.umax = 5.5

        self.n_x = 6
        self.n_u = 2

    def x_d(self):
        # Nominal state
        return np.array([0, 0, 0, 0, 0, 0])

    def u_d(self):
        # Nominal input
        return np.array([self.m * self.g / 2, self.m * self.g / 2])

    def continuous_time_full_dynamics(self, x, u):
        # Dynamics for the quadrotor
        g = self.g
        m = self.m
        a = self.a
        I = self.I

        theta = x[2]
        ydot = x[3]
        zdot = x[4]
        thetadot = x[5]
        u0 = u[0]
        u1 = u[1]

        xdot = np.array([ydot,
                         zdot,
                         thetadot,
                         -sin(theta) * (u0 + u1) / m,
                         -g + cos(theta) * (u0 + u1) / m,
                         a * (u0 - u1) / I])
        return xdot

    def continuous_time_linearized_dynamics(self):
        # Dynamics linearized at the fixed point
        # This function returns A and B matrix
        A = np.zeros((6, 6))
        A[:3, -3:] = np.identity(3)
        A[3, 2] = -self.g;

        B = np.zeros((6, 2))
        B[4, 0] = 1 / self.m;
        B[4, 1] = 1 / self.m;
        B[5, 0] = self.a / self.I
        B[5, 1] = -self.a / self.I

        return A, B

    def discrete_time_linearized_dynamics(self, T):
        # Discrete time version of the linearized dynamics at the fixed point
        # This function returns A and B matrix of the discrete time dynamics
        A_c, B_c = self.continuous_time_linearized_dynamics()
        A_d = np.identity(6) + A_c * T;
        B_d = B_c * T;

        return A_d, B_d

    def add_initial_state_constraint(self, prog, x, x_current):
        # TODO: impose initial state constraint.
        # Use AddBoundingBoxConstraint
        # Constrain such that x0 = x_current
        n_x = x_current.shape[0]
        x_d = self.x_d()

        for i in range(n_x):
            prog.AddBoundingBoxConstraint(x_current[i] - x_d[i],
                                          x_current[i] - x_d[i], x[0, i])

    def add_input_saturation_constraint(self, prog, x, u, N):
        # TODO: impose input limit constraint.
        # Use AddBoundingBoxConstraint
        # The limits are available through self.umin and self.umax
        n_u = u.shape[1]
        ud = self.u_d()
        for j in range(n_u):
            for i in range(N - 1):
                prog.AddBoundingBoxConstraint(self.umin - ud[j],
                                              self.umax - ud[j], u[i, j])
                # prog.AddBoundingBoxConstraint(-self.umin, -self.umax, u[i, j])

    def add_dynamics_constraint(self, prog, x, u, N, T):
        # TODO: impose dynamics constraint.
        # Use AddLinearEqualityConstraint(expr, value)
        A, B = self.discrete_time_linearized_dynamics(T)
        xd = self.x_d()
        ud = self.u_d()
        for i in range(N - 1):
            prog.AddLinearEqualityConstraint(
                (x[i + 1]) - (A @ (x[i]) + B @ (u[i])), np.zeros(6))

    def add_cost(self, prog, x, u, N):
        # TODO: add cost.
        xd = self.x_d()
        ud = self.u_d()
        # for i in range(N-1):
        #     prog.AddQuadraticCost((x[i] - xd).T @ self.Q @ (x[i] - xd) + (u[i] - ud).T @ self.R @ (u[i] - ud))
        # prog.AddQuadraticCost((x[N-1] - xd).T @ self.Qf @ (x[N-1] - xd))
        for i in range(N - 1):
            prog.AddQuadraticCost(
                (x[i]).T @ self.Q @ (x[i]) + (u[i]).T @ self.R @ (u[i]))
        prog.AddQuadraticCost((x[N - 1]).T @ self.Qf @ (x[N - 1]))

    def compute_mpc_feedback(self, x_current, use_clf=False):
        '''
        This function computes the MPC controller input u
        '''

        # Parameters for the QP
        N = 10
        T = 0.1

        # Initialize mathematical program and decalre decision variables
        prog = MathematicalProgram()
        x = np.zeros((N, 6), dtype="object")
        for i in range(N):
            x[i] = prog.NewContinuousVariables(6, "x_" + str(i))
        u = np.zeros((N - 1, 2), dtype="object")
        for i in range(N - 1):
            u[i] = prog.NewContinuousVariables(2, "u_" + str(i))

        # Add constraints and cost
        self.add_initial_state_constraint(prog, x, x_current)
        self.add_input_saturation_constraint(prog, x, u, N)
        self.add_dynamics_constraint(prog, x, u, N, T)
        self.add_cost(prog, x, u, N)

        # Placeholder constraint and cost to satisfy QP requirements
        # TODO: Delete after completing this function
        # prog.AddQuadraticCost(0)
        # prog.AddLinearEqualityConstraint(0, 0)

        # Solve the QP
        solver = OsqpSolver()
        result = solver.Solve(prog)

        # u_mpc = np.zeros(2)
        u_mpc = result.GetSolution(u[0]) + self.u_d()
        # TODO: retrieve the controller input from the solution of the optimization problem
        # and use it to compute the MPC input u
        # You should make use of result.GetSolution(decision_var) where decision_var
        # is the variable you want

        return u_mpc

    def compute_lqr_feedback(self, x):
        '''
        Infinite horizon LQR controller
        '''
        A, B = self.continuous_time_linearized_dynamics()
        S = solve_continuous_are(A, B, self.Q, self.R)
        K = -inv(self.R) @ B.T @ S
        u = self.u_d() + K @ x;
        return u