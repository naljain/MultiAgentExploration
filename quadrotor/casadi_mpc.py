class MPCWaypointFollowerWithObstacleAvoidance:
    def __init__(self, waypoints, obstacle_map, quad_params):
        """
        Initialize the MPC controller with waypoints and obstacle avoidance.

        Parameters:
            waypoints: List of [x, y, z] waypoints to follow
            obstacle_map: 2D numpy array with 0 (free) and 1 (obstacle)
            quad_params: Parameters of the multirotor dynamics
        """
        self.waypoints = waypoints
        self.current_waypoint_index = 0
        self.map = obstacle_map
        self.dynamics = CasADiMultirotorDynamics(quad_params)

        # MPC Parameters
        self.T = 0.02  # Time step
        self.N = 10  # Horizon length

        # State and control dimensions
        self.n_states = 13  # x, v, q, w
        self.n_controls = 4  # Rotor speeds

        # CasADi optimization problem
        self.opti = ca.Opti()
        self.opt_controls = self.opti.variable(self.N, self.n_controls)  # Rotor speeds
        self.opt_states = self.opti.variable(self.N + 1, self.n_states)  # Full state vector

        # Reference trajectories
        self.opt_x_ref = self.opti.parameter(self.N + 1, self.n_states)
        self.opt_u_ref = self.opti.parameter(self.N, self.n_controls)

        # Initial condition constraint
        self.opti.subject_to(self.opt_states[0, :] == self.opt_x_ref[0, :])

        # Dynamics constraints
        for i in range(self.N):
            x_next = self.opt_states[i, :] + self._dynamics(self.opt_states[i, :], self.opt_controls[i, :]) * self.T
            self.opti.subject_to(self.opt_states[i + 1, :] == x_next)

        # Cost function
        Q = np.diag([10.0] * self.n_states)  # State error weights
        R = np.diag([1.0] * self.n_controls)  # Control effort weights

        self.obj = 0
        for i in range(self.N):
            state_error = self.opt_states[i, :] - self.opt_x_ref[i + 1, :]
            control_error = self.opt_controls[i, :] - self.opt_u_ref[i, :]
            self.obj += ca.mtimes([state_error, Q, state_error.T])
            self.obj += ca.mtimes([control_error, R, control_error.T])
            self.obj += self._add_obstacle_barrier(self.opt_states[i, :])

        self.opti.minimize(self.obj)

        # Control limits
        self.opti.subject_to(self.opti.bounded(0, self.opt_controls, 1000))  # Rotor speeds

        opts_setting = {"ipopt.max_iter": 2000, "ipopt.print_level": 0, "print_time": 0}
        self.opti.solver("ipopt", opts_setting)

        # Internal state for MPC
        self.current_state = np.zeros(self.n_states)
        self.u0 = np.zeros((self.N, self.n_controls))

    def _dynamics(self, state, control):
        """
        Use CasADiMultirotorDynamics to compute the state derivatives symbolically.
        """
        return self.dynamics.statedot(state, control)

    def _add_obstacle_barrier(self, state):
        """
        Add obstacle avoidance barrier cost.
        """
        x, y = state[0], state[1]
        barrier_cost = 0
        lambda_barrier = 100.0  # Barrier weight
        r_safe = 1.0  # Safe radius
        epsilon = 1e-6


        for i in range(self.map.shape[0]):
            for j in range(self.map.shape[1]):
                if self.map[i, j] == 1:  # Obstacle cell
                    obs_x, obs_y = i, j
                    distance = ca.sqrt((x - obs_x)**2 + (y - obs_y)**2 + epsilon)
                    barrier_cost += lambda_barrier * ca.fmax(0, (1 / distance**2) - (1 / r_safe**2))
        return barrier_cost

    def run(self, sim_time=10.0):
        """
        Run the MPC controller simulation.
        """
        t0 = 0
        state_history = [self.current_state.copy()]
        control_history = []
        time_history = [t0]

        while t0 < sim_time:
            next_trajectories = np.tile(self.current_state, (self.N + 1, 1))
            next_controls = np.zeros((self.N, self.n_controls))

            # Set optimization parameters
            self.opti.set_value(self.opt_x_ref, next_trajectories)
            self.opti.set_value(self.opt_u_ref, next_controls)
            self.opti.set_initial(self.opt_controls, self.u0)
            self.opti.set_initial(self.opt_states, next_trajectories)

            # Solve the optimization problem
            sol = self.opti.solve()
            u_res = sol.value(self.opt_controls)
            state_dot = self._dynamics(self.current_state, u_res[0, :]).full().flatten()
            self.current_state += state_dot * self.T  # Euler step

            # Store results
            state_history.append(self.current_state.copy())
            control_history.append(u_res[0, :])
            time_history.append(t0)
            t0 += self.T

        return np.array(state_history), np.array(control_history), np.array(time_history)
