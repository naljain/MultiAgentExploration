# import casadi as ca
# import numpy as np
# import math
# import time
# import matplotlib.pyplot as plt
#
#
# class MPCWaypointFollower:
#     def __init__(self, waypoints):
#         """
#         Initialize the MPC controller with a set of waypoints.
#
#         Parameters:
#             waypoints: List of [x, y] waypoints to follow
#         """
#         self.waypoints = waypoints
#         self.current_waypoint_index = 0
#
#         # Quadrotor parameters
#         self.mq = 1.0  # Mass of the quadrotor
#         self.g = 9.8  # Gravity
#         self.U1 = self.mq * self.g
#
#         # MPC Parameters
#         self.T = 0.02  # Time step
#         self.N = 10  # Horizon length
#
#         # Define CasADi optimization problem
#         self.opti = ca.Opti()
#         self.opt_controls = self.opti.variable(self.N, 2)  # [phi, theta]
#         self.phid = self.opt_controls[:, 0]
#         self.thed = self.opt_controls[:, 1]
#
#         self.opt_states = self.opti.variable(self.N + 1, 4)  # [x, y, dx, dy]
#         self.x = self.opt_states[:, 0]
#         self.y = self.opt_states[:, 1]
#         self.dx = self.opt_states[:, 2]
#         self.dy = self.opt_states[:, 3]
#
#         # Reference trajectories
#         self.opt_u_ref = self.opti.parameter(self.N, 2)
#         self.opt_x_ref = self.opti.parameter(self.N + 1, 4)
#
#         # Initial condition constraint
#         self.opti.subject_to(self.opt_states[0, :] == self.opt_x_ref[0, :])
#
#         # Dynamics constraints
#         for i in range(self.N):
#             x_next = self.opt_states[i, :] + self._dynamics(
#                 self.opt_states[i, :], self.opt_controls[i, :]
#             ) * self.T
#             self.opti.subject_to(self.opt_states[i + 1, :] == x_next)
#
#         # Cost function
#         Q = np.diag([20.0, 20.0, 1.0, 1.0])  # State error weights
#         R = np.diag([1.0, 1.0])  # Control effort weights
#
#         self.obj = 0
#         for i in range(self.N):
#             state_error = self.opt_states[i, :] - self.opt_x_ref[i + 1, :]
#             control_error = self.opt_controls[i, :] - self.opt_u_ref[i, :]
#             self.obj += ca.mtimes([state_error, Q, state_error.T])
#             self.obj += ca.mtimes([control_error, R, control_error.T])
#         self.opti.minimize(self.obj)
#
#         # Control limits
#         self.opti.subject_to(self.opti.bounded(-0.5, self.phid, 0.5))
#         self.opti.subject_to(self.opti.bounded(-0.5, self.thed, 0.5))
#
#         opts_setting = {"ipopt.max_iter": 2000, "ipopt.print_level": 0,
#                         "print_time": 0}
#         self.opti.solver("ipopt", opts_setting)
#
#         # Internal state for MPC
#         self.current_state = np.array([0.0, 0.0, 0.0, 0.0])  # [x, y, dx, dy]
#         self.u0 = np.zeros((self.N, 2))
#         self.next_trajectories = np.zeros((self.N + 1, 4))
#         self.next_controls = np.zeros((self.N, 2))
#
#     def _dynamics(self, state, control):
#         """
#         Compute the quadrotor dynamics for given state and control.
#         """
#         # Explicitly extract elements from CasADi matrices
#         x = state[0]
#         y = state[1]
#         dx = state[2]
#         dy = state[3]
#
#         phi = control[0]
#         theta = control[1]
#
#         # Dynamics equations
#         ddx = ca.sin(theta) * self.U1 / self.mq
#         ddy = ca.sin(phi) * self.U1 / self.mq
#
#         # Return a row vector (1x4)
#         return ca.horzcat(dx, dy, ddx, ddy)
#
#     def _generate_trajectory(self, current_state):
#         """
#         Generate a trajectory from the current position to the next waypoint.
#
#         Parameters:
#             current_state: [x, y, dx, dy], current state of the quadrotor
#         """
#         if self.current_waypoint_index >= len(self.waypoints):
#             # No more waypoints to follow
#             return np.tile(current_state, (self.N + 1, 1)), np.zeros(
#                 (self.N, 2))
#
#         # Get the current waypoint
#         target = self.waypoints[self.current_waypoint_index]
#         x_target, y_target = target
#
#         # Compute trajectory to waypoint
#         trajectory = np.zeros((self.N + 1, 4))
#         controls = np.zeros((self.N, 2))
#
#         for i in range(self.N + 1):
#             alpha = min(1.0, i / self.N)
#             trajectory[i, :2] = current_state[:2] + alpha * (
#                         np.array(target) - current_state[:2])
#             trajectory[i, 2:] = 0.0  # Assume zero velocity
#
#         for i in range(self.N):
#             dx_target = (x_target - current_state[0]) / self.N
#             dy_target = (y_target - current_state[1]) / self.N
#             phi = math.asin(dy_target * self.mq / self.U1)
#             theta = math.asin(dx_target * self.mq / self.U1)
#             controls[i, :] = [phi, theta]
#
#         # Check if waypoint is reached
#         if np.linalg.norm(current_state[:2] - np.array(target)) < 0.1:
#             self.current_waypoint_index += 1
#
#         return trajectory, controls
#
#     def run(self, sim_time=70.0):
#         """
#         Run the MPC controller simulation for a given duration.
#         """
#         t0 = 0
#         x_c = [self.current_state.copy()]  # History of states
#         u_c = []  # History of controls
#         t_c = [t0]  # Time history
#
#         while t0 < sim_time:
#             # Generate desired trajectory and commands
#             next_trajectories, next_controls = self._generate_trajectory(
#                 self.current_state)
#
#             # Update optimization parameters
#             self.opti.set_value(self.opt_x_ref, next_trajectories)
#             self.opti.set_value(self.opt_u_ref, next_controls)
#             self.opti.set_initial(self.opt_controls, self.u0)
#             self.opti.set_initial(self.opt_states, next_trajectories)
#
#             # Solve the optimization problem
#             try:
#                 sol = self.opti.solve()
#                 u_res = sol.value(self.opt_controls)
#             except RuntimeError:
#                 u_res = np.zeros((self.N, 2))  # Fallback
#
#             # Apply control to update state
#             phi, theta = u_res[0, :]
#             dx = math.sin(theta) * self.U1 / self.mq * self.T
#             dy = math.sin(phi) * self.U1 / self.mq * self.T
#             self.current_state[:2] += self.current_state[2:] * self.T
#             self.current_state[2:] = [dx, dy]
#
#             # Log data
#             x_c.append(self.current_state.copy())
#             u_c.append([phi, theta])
#             t_c.append(t0)
#
#             # Advance time
#             t0 += self.T
#
#         return np.array(x_c), np.array(u_c), np.array(t_c)
#
#
# if __name__ == "__main__":
#     # Define waypoints
#     waypoints = [[1, 1], [2, 2], [3, 1], [4, 0]]
#
#     # Initialize controller
#     mpc = MPCWaypointFollower(waypoints)
#
#     # Run simulation
#     x_c, u_c, t_c = mpc.run()
#
#     # Plot results
#     plt.figure()
#     plt.plot(x_c[:, 0], x_c[:, 1], label="Trajectory")
#     plt.scatter(*zip(*waypoints), color="red", label="Waypoints")
#     plt.xlabel("x [m]")
#     plt.ylabel("y [m]")
#     plt.legend()
#     plt.grid()
#     plt.show()

#
# import casadi as ca
# import numpy as np
# import math
# import time
# import matplotlib.pyplot as plt
#
#
# class MPCWaypointFollower:
#     def __init__(self, waypoints):
#         """
#         Initialize the MPC controller with a set of waypoints.
#
#         Parameters:
#             waypoints: List of [x, y] waypoints to follow
#         """
#         self.waypoints = waypoints
#         self.current_waypoint_index = 0
#
#         # Quadrotor parameters
#         self.mq = 1.0  # Mass of the quadrotor
#         self.g = 9.8  # Gravity
#         self.U1 = self.mq * self.g
#
#         # MPC Parameters
#         self.T = 0.02  # Time step
#         self.N = 10  # Horizon length
#
#         # Define CasADi optimization problem
#         self.opti = ca.Opti()
#         self.opt_controls = self.opti.variable(self.N, 2)  # [phi, theta]
#         self.phid = self.opt_controls[:, 0]
#         self.thed = self.opt_controls[:, 1]
#
#         self.opt_states = self.opti.variable(self.N + 1, 4)  # [x, y, dx, dy]
#         self.x = self.opt_states[:, 0]
#         self.y = self.opt_states[:, 1]
#         self.dx = self.opt_states[:, 2]
#         self.dy = self.opt_states[:, 3]
#
#         # Reference trajectories
#         self.opt_u_ref = self.opti.parameter(self.N, 2)
#         self.opt_x_ref = self.opti.parameter(self.N + 1, 4)
#
#         # Initial condition constraint
#         self.opti.subject_to(self.opt_states[0, :] == self.opt_x_ref[0, :])
#
#         # Dynamics constraints
#         for i in range(self.N):
#             x_next = self.opt_states[i, :] + self._dynamics(
#                 self.opt_states[i, :], self.opt_controls[i, :]
#             ) * self.T
#             self.opti.subject_to(self.opt_states[i + 1, :] == x_next)
#
#         # Cost function
#         Q = np.diag([20.0, 20.0, 1.0, 1.0])  # State error weights
#         R = np.diag([1.0, 1.0])  # Control effort weights
#
#         self.obj = 0
#         for i in range(self.N):
#             state_error = self.opt_states[i, :] - self.opt_x_ref[i + 1, :]
#             control_error = self.opt_controls[i, :] - self.opt_u_ref[i, :]
#             self.obj += ca.mtimes([state_error, Q, state_error.T])
#             self.obj += ca.mtimes([control_error, R, control_error.T])
#         self.opti.minimize(self.obj)
#
#         # Control limits
#         self.opti.subject_to(self.opti.bounded(-0.5, self.phid, 0.5))
#         self.opti.subject_to(self.opti.bounded(-0.5, self.thed, 0.5))
#
#         opts_setting = {"ipopt.max_iter": 2000, "ipopt.print_level": 0,
#                         "print_time": 0}
#         self.opti.solver("ipopt", opts_setting)
#
#         # Internal state for MPC
#         self.current_state = np.array([0.0, 0.0, 0.0, 0.0])  # [x, y, dx, dy]
#         self.u0 = np.zeros((self.N, 2))
#         self.next_trajectories = np.zeros((self.N + 1, 4))
#         self.next_controls = np.zeros((self.N, 2))
#
#     def _dynamics(self, state, control):
#         """
#         Compute the quadrotor dynamics for given state and control.
#         """
#         # Explicitly extract elements from CasADi matrices
#         x = state[0]
#         y = state[1]
#         dx = state[2]
#         dy = state[3]
#
#         phi = control[0]
#         theta = control[1]
#
#         # Dynamics equations
#         ddx = ca.sin(theta) * self.U1 / self.mq
#         ddy = ca.sin(phi) * self.U1 / self.mq
#
#         # Return a row vector (1x4)
#         return ca.horzcat(dx, dy, ddx, ddy)
#
#     def _generate_trajectory(self, current_state):
#         """
#         Generate a trajectory from the current position to the next waypoint.
#
#         Parameters:
#             current_state: [x, y, dx, dy], current state of the quadrotor
#         """
#         if self.current_waypoint_index >= len(self.waypoints):
#             # No more waypoints to follow
#             return np.tile(current_state, (self.N + 1, 1)), np.zeros(
#                 (self.N, 2))
#
#         # Get the current waypoint
#         target = self.waypoints[self.current_waypoint_index]
#         x_target, y_target = target
#
#         # Compute trajectory to waypoint
#         trajectory = np.zeros((self.N + 1, 4))
#         controls = np.zeros((self.N, 2))
#
#         for i in range(self.N + 1):
#             alpha = min(1.0, i / self.N)
#             trajectory[i, :2] = current_state[:2] + alpha * (
#                         np.array(target) - current_state[:2])
#             trajectory[i, 2:] = 0.0  # Assume zero velocity
#
#         for i in range(self.N):
#             dx_target = (x_target - current_state[0]) / self.N
#             dy_target = (y_target - current_state[1]) / self.N
#             phi = math.asin(dy_target * self.mq / self.U1)
#             theta = math.asin(dx_target * self.mq / self.U1)
#             controls[i, :] = [phi, theta]
#
#         # Check if waypoint is reached
#         if np.linalg.norm(current_state[:2] - np.array(target)) < 0.1:
#             self.current_waypoint_index += 1
#
#         return trajectory, controls
#
#     def run(self, sim_time=150.0):
#         """
#         Run the MPC controller simulation for a given duration.
#         """
#         t0 = 0
#         x_c = [self.current_state.copy()]  # History of states
#         u_c = []  # History of controls
#         t_c = [t0]  # Time history
#
#         while t0 < sim_time:
#             # Generate desired trajectory and commands
#             next_trajectories, next_controls = self._generate_trajectory(
#                 self.current_state)
#
#             # Update optimization parameters
#             self.opti.set_value(self.opt_x_ref, next_trajectories)
#             self.opti.set_value(self.opt_u_ref, next_controls)
#             self.opti.set_initial(self.opt_controls, self.u0)
#             self.opti.set_initial(self.opt_states, next_trajectories)
#
#             # Solve the optimization problem
#             try:
#                 sol = self.opti.solve()
#                 u_res = sol.value(self.opt_controls)
#             except RuntimeError:
#                 u_res = np.zeros((self.N, 2))  # Fallback
#
#             # Apply control to update state
#             phi, theta = u_res[0, :]
#             dx = math.sin(theta) * self.U1 / self.mq * self.T
#             dy = math.sin(phi) * self.U1 / self.mq * self.T
#             self.current_state[:2] += self.current_state[2:] * self.T
#             self.current_state[2:] = [dx, dy]
#
#             # Log data
#             x_c.append(self.current_state.copy())
#             u_c.append([phi, theta])
#             t_c.append(t0)
#
#             # Advance time
#             t0 += self.T
#
#         return np.array(x_c), np.array(u_c), np.array(t_c)
#
#
# if __name__ == "__main__":
#     # Define waypoints
#     waypoints = [[1, 1], [2, 2], [3, 1], [4, 0]]
#
#     # Initialize controller
#     mpc = MPCWaypointFollower(waypoints)
#
#     # Run simulation
#     x_c, u_c, t_c = mpc.run()
#
#     # Plot results
#     plt.figure()
#     plt.plot(x_c[:, 0], x_c[:, 1], label="Trajectory")
#     plt.scatter(*zip(*waypoints), color="red", label="Waypoints")
#     plt.xlabel("x [m]")
#     plt.ylabel("y [m]")
#     plt.legend()
#     plt.grid()
#     plt.show()

import casadi as ca
import numpy as np
import math
import matplotlib.pyplot as plt



class MPCWaypointFollowerWithObstacleAvoidance:
    def __init__(self, waypoints, obstacle_map):
        """
        Initialize the MPC controller with waypoints and obstacle avoidance.

        Parameters:
            waypoints: List of [x, y] waypoints to follow
            obstacle_map: 2D numpy array with 0 (free) and 1 (obstacle)
        """
        self.waypoints = waypoints
        self.current_waypoint_index = 0
        self.map = obstacle_map

        # Quadrotor parameters
        self.mq = 1.0  # Mass of the quadrotor
        self.g = 9.8  # Gravity
        self.U1 = self.mq * self.g

        # MPC Parameters
        self.T = 0.02  # Time step
        self.N = 10  # Horizon length

        # Define CasADi optimization problem
        self.opti = ca.Opti()
        self.opt_controls = self.opti.variable(self.N, 2)  # [phi, theta]
        self.phid = self.opt_controls[:, 0]
        self.thed = self.opt_controls[:, 1]

        self.opt_states = self.opti.variable(self.N + 1, 4)  # [x, y, dx, dy]
        self.x = self.opt_states[:, 0]
        self.y = self.opt_states[:, 1]
        self.dx = self.opt_states[:, 2]
        self.dy = self.opt_states[:, 3]

        # Reference trajectories
        self.opt_u_ref = self.opti.parameter(self.N, 2)
        self.opt_x_ref = self.opti.parameter(self.N + 1, 4)

        # Initial condition constraint
        self.opti.subject_to(self.opt_states[0, :] == self.opt_x_ref[0, :])

        # Dynamics constraints
        for i in range(self.N):
            x_next = self.opt_states[i, :] + self._dynamics(
                self.opt_states[i, :], self.opt_controls[i, :]
            ) * self.T
            self.opti.subject_to(self.opt_states[i + 1, :] == x_next)

        # Cost function
        Q = np.diag([5.8, 5.8, 1.0, 1.0])  # State error weights
        R = np.diag([1.0, 1.0])  # Control effort weights

        self.obj = 0
        for i in range(self.N):
            state_error = self.opt_states[i, :] - self.opt_x_ref[i + 1, :]
            control_error = self.opt_controls[i, :] - self.opt_u_ref[i, :]
            self.obj += ca.mtimes([state_error, Q, state_error.T])
            self.obj += ca.mtimes([control_error, R, control_error.T])

            # Add obstacle avoidance via barrier function
            self.obj += self._add_obstacle_barrier(self.opt_states[i, :])

        self.opti.minimize(self.obj)

        # Control limits
        self.opti.subject_to(self.opti.bounded(-0.5, self.phid, 0.5))
        self.opti.subject_to(self.opti.bounded(-0.5, self.thed, 0.5))

        opts_setting = {"ipopt.max_iter": 2000, "ipopt.print_level": 0,
                        "print_time": 0}
        self.opti.solver("ipopt", opts_setting)

        # Internal state for MPC
        self.current_state = np.array([0.0, 0.0, 0.0, 0.0])  # [x, y, dx, dy]
        self.u0 = np.zeros((self.N, 2))
        self.next_trajectories = np.zeros((self.N + 1, 4))
        self.next_controls = np.zeros((self.N, 2))

    def _dynamics(self, state, control):
        """
        Compute the quadrotor dynamics for given state and control.
        """
        x = state[0]
        y = state[1]
        dx = state[2]
        dy = state[3]

        phi = control[0]
        theta = control[1]

        ddx = ca.sin(theta) * self.U1 / self.mq
        ddy = ca.sin(phi) * self.U1 / self.mq

        return ca.horzcat(dx, dy, ddx, ddy)

    def _add_obstacle_barrier(self, state):
        """
        Add a weighted barrier function to the cost for obstacle avoidance.
        """
        x, y = state[0], state[1]
        barrier_cost = 0
        epsilon = 1e-6  # Small value to avoid division by zero
        lambda_barrier = 0.9 # Barrier weight, adjust as needed
        d_safe = 0.05

        for i in range(self.map.shape[0]):
            for j in range(self.map.shape[1]):
                if self.map[i, j] == 1:  # Obstacle cell
                    obs_x, obs_y = i, j
                    distance = ca.sqrt(
                        (x - obs_x) ** 2 + (y - obs_y) ** 2 + epsilon)
                    barrier_cost += lambda_barrier / distance

        return barrier_cost

    # def _add_obstacle_barrier(self, state):
    #     """
    #     Add a radial barrier function to penalize proximity to obstacles.
    #     """
    #     x, y = state[0], state[1]
    #     barrier_cost = 0
    #     lambda_barrier = 1.0  # Weight for the barrier function
    #     r_safe = 0.05  # Safe radius around obstacles
    #     epsilon = 1e-6  # Small value to avoid division by zero
    #
    #     for i in range(self.map.shape[0]):
    #         for j in range(self.map.shape[1]):
    #             if self.map[i, j] == 1:  # Obstacle cell
    #                 obs_x, obs_y = i, j
    #                 distance = ca.sqrt(
    #                     (x - obs_x) ** 2 + (y - obs_y) ** 2 + epsilon)
    #
    #                 # Quadratic penalty for distances below r_safe
    #                 barrier_cost += lambda_barrier * ca.fmax(0, (
    #                             1 / distance ** 2) - (1 / r_safe ** 2))
    #
    #     return barrier_cost

    def _generate_trajectory(self, current_state):
        """
        Generate a trajectory from the current position to the next waypoint.
        """
        if self.current_waypoint_index >= len(self.waypoints):
            return np.tile(current_state, (self.N + 1, 1)), np.zeros((self.N, 2))

        target = self.waypoints[self.current_waypoint_index]
        x_target, y_target = target

        trajectory = np.zeros((self.N + 1, 4))
        controls = np.zeros((self.N, 2))

        for i in range(self.N + 1):
            alpha = min(1.0, i / self.N)
            trajectory[i, :2] = current_state[:2] + alpha * (np.array(target) - current_state[:2])
            trajectory[i, 2:] = 0.0  # Assume zero velocity

        for i in range(self.N):
            dx_target = (x_target - current_state[0]) / self.N
            dy_target = (y_target - current_state[1]) / self.N
            phi = math.asin(dy_target * self.mq / self.U1)
            theta = math.asin(dx_target * self.mq / self.U1)
            controls[i, :] = [phi, theta]

        if np.linalg.norm(current_state[:2] - np.array(target)) < 0.1:
            self.current_waypoint_index += 1

        return trajectory, controls

    def run(self, sim_time=500.0):
        """
        Run the MPC controller simulation for a given duration.
        """
        t0 = 0
        x_c = [self.current_state.copy()]  # History of states
        u_c = []  # History of controls
        t_c = [t0]  # Time history

        while t0 < sim_time:
            next_trajectories, next_controls = self._generate_trajectory(self.current_state)

            self.opti.set_value(self.opt_x_ref, next_trajectories)
            self.opti.set_value(self.opt_u_ref, next_controls)
            self.opti.set_initial(self.opt_controls, self.u0)
            self.opti.set_initial(self.opt_states, next_trajectories)

            try:
                sol = self.opti.solve()
                u_res = sol.value(self.opt_controls)
            except RuntimeError:
                u_res = np.zeros((self.N, 2))  # Fallback

            phi, theta = u_res[0, :]
            dx = math.sin(theta) * self.U1 / self.mq * self.T
            dy = math.sin(phi) * self.U1 / self.mq * self.T
            self.current_state[:2] += self.current_state[2:] * self.T
            self.current_state[2:] = [dx, dy]

            x_c.append(self.current_state.copy())
            u_c.append([phi, theta])
            t_c.append(t0)
            t0 += self.T

        return np.array(x_c), np.array(u_c), np.array(t_c)


if __name__ == "__main__":
    waypoints = [(0, 0), (1, 1), (2, 2), (3, 3), (4, 3), (5, 4), (6, 5), (6, 6), (6, 7)]
    obstacle_map = np.zeros((10, 10))
    obstacle_map[3:5, 4:5] = 1  # Add an obstacle at (5, 5)

    mpc = MPCWaypointFollowerWithObstacleAvoidance(waypoints, obstacle_map)
    x_c, u_c, t_c = mpc.run()

    plt.figure()
    plt.plot(x_c[:, 0], x_c[:, 1], label="Trajectory")
    plt.scatter(*zip(*waypoints), color="red", label="Waypoints")
    plt.imshow(obstacle_map.T, origin="lower", cmap="Greys", alpha=0.3)
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.legend()
    plt.grid()
    plt.show()
