import numpy as np
import casadi as ca


class MPC_RotorPy(object):
    def __init__(self, map, vehicle):
        """
        MPC controller for a quadrotor using CasADi / IPOPT.

        State  (17): [pos(3), vel(3), quat(4)[i,j,k,w], omega(3), rotor_speeds(4)]
        Control (4): cmd_motor_speeds
        """
        self.vehicle = vehicle
        self.map = map

        # Cost weights
        self.Q = np.diag([10, 10, 5, 1, 1, 1, 3, 3, 3, 3, 1, 1, 1, 1, 1, 1, 1])
        self.R = np.eye(4)

        # Vehicle parameters (cached as numpy for DM conversion)
        self.tau_m            = vehicle.tau_m
        self.rotor_geometry   = vehicle.rotor_geometry   # (num_rotors, 3)
        self.k_eta            = vehicle.k_eta
        self.num_rotors       = vehicle.num_rotors
        self.k_flap           = vehicle.k_flap
        self.k_m              = vehicle.k_m
        self.rotor_dir        = vehicle.rotor_dir        # (num_rotors,)
        self.weight           = vehicle.weight           # (3,) world-frame weight force
        self.mass             = vehicle.mass
        self.inertia          = vehicle.inertia          # (3,3)
        self.inv_inertia      = vehicle.inv_inertia      # (3,3)
        self.gravity          = 9.81
        self.rotor_speed_min  = vehicle.rotor_speed_min
        self.rotor_speed_max  = vehicle.rotor_speed_max
        self.drag_matrix      = vehicle.drag_matrix      # (3,3)
        self.rotor_drag_matrix = vehicle.rotor_drag_matrix  # (3,3)

        # Default initial guess (hover)
        self.u_guess = np.array([1788.53, 1788.53, 1788.53, 1788.53])

    # ------------------------------------------------------------------
    # Utilities
    # ------------------------------------------------------------------
    def _hat(self, v):
        """Skew-symmetric matrix for a 3-vector (CasADi-compatible)."""
        return ca.vertcat(
            ca.horzcat(   0, -v[2],  v[1]),
            ca.horzcat( v[2],    0, -v[0]),
            ca.horzcat(-v[1],  v[0],    0),
        )

    def _cross3(self, a, b):
        """Cross product of two 3-vectors (CasADi-compatible)."""
        return ca.vertcat(
            a[1]*b[2] - a[2]*b[1],
            a[2]*b[0] - a[0]*b[2],
            a[0]*b[1] - a[1]*b[0],
        )

    # ------------------------------------------------------------------
    # Dynamics
    # ------------------------------------------------------------------
    def quadrotor_dynamics(self, x, u):
        """
        CasADi symbolic quadrotor dynamics (forward Euler compatible).

        Parameters
        ----------
        x : (17,) CasADi / numpy vector
            [pos(3), vel(3), quat(4)[i,j,k,w], omega(3), rotor_speeds(4)]
        u : (4,) CasADi / numpy vector
            cmd_motor_speeds

        Returns
        -------
        x_dot : (17,1) CasADi expression
        """
        p           = x[0:3]
        v           = x[3:6]
        q           = x[6:10]   # quaternion [i, j, k, w]
        omega       = x[10:13]
        rotor_speed = x[13:17]
        cmd_speed   = u[0:4]

        # --- Normalize quaternion ---
        q = q / (ca.norm_2(q) + 1e-7)
        qi, qj, qk, qw = q[0], q[1], q[2], q[3]

        # --- Rotation matrix body → world ---
        R_mat = ca.vertcat(
            ca.horzcat(1 - 2*qj**2 - 2*qk**2,  2*qi*qj - 2*qw*qk,  2*qi*qk + 2*qj*qw),
            ca.horzcat(  2*qi*qj + 2*qw*qk,  1 - 2*qi**2 - 2*qk**2,  2*qj*qk - 2*qi*qw),
            ca.horzcat(  2*qi*qk - 2*qj*qw,    2*qj*qk + 2*qw*qi,  1 - 2*qi**2 - 2*qj**2),
        )

        # --- Rotor speed derivative (first-order lag) ---
        rotor_accel = (1.0 / self.tau_m) * (rotor_speed - cmd_speed)

        # --- Position derivative ---
        p_dot = v

        # --- Quaternion derivative with normalization stabilisation ---
        # G maps body angular velocity to quaternion rate: q_dot = 0.5 * G^T * omega
        G = ca.vertcat(
            ca.horzcat( qw,  qk, -qj, -qi),
            ca.horzcat(-qk,  qw,  qi, -qj),
            ca.horzcat( qj, -qi,  qw, -qk),
        )
        q_dot = 0.5 * G.T @ omega
        q_dot = q_dot - (ca.dot(q, q) - 1.0) * q  # normalization correction

        # --- Body-frame airspeed and rotor local airspeeds ---
        body_airspeed = R_mat.T @ v
        omega_hat = self._hat(omega)

        # Accumulate body forces and moments per rotor
        D = ca.DM(self.drag_matrix) @ (-ca.norm_2(body_airspeed) * body_airspeed)
        FtotB = D
        MtotB = ca.DM.zeros(3, 1)

        for i in range(self.num_rotors):
            r_i  = ca.DM(self.rotor_geometry[i, :])               # rotor position (3,)
            rs_i = rotor_speed[i]                                  # rotor speed (scalar)
            ls_i = body_airspeed + omega_hat @ r_i                 # local airspeed (3,)

            # Thrust (body z direction)
            T_i = ca.vertcat(0, 0, self.k_eta * rs_i**2)

            # Rotor drag force
            H_i = -rs_i * (ca.DM(self.rotor_drag_matrix) @ ls_i)

            # Flapping moment: -k_flap * rs * (ls × e3)
            #   ls × e3 = [ls_y, -ls_x, 0]
            M_flap_i = -self.k_flap * rs_i * self._cross3(ls_i, ca.DM([0.0, 0.0, 1.0]))

            # Yaw (reaction) moment
            M_yaw_i = ca.vertcat(0, 0, float(self.rotor_dir[i]) * self.k_m * rs_i**2)

            # Accumulate
            FtotB = FtotB + T_i + H_i
            MtotB = MtotB + self._cross3(r_i, T_i + H_i) + M_flap_i + M_yaw_i

        # --- Velocity derivative ---
        Ftot  = R_mat @ FtotB
        v_dot = (ca.DM(self.weight) + Ftot) / self.mass

        # --- Angular velocity derivative ---
        w_dot = ca.DM(self.inv_inertia) @ (
            MtotB - omega_hat @ (ca.DM(self.inertia) @ omega)
        )

        return ca.vertcat(p_dot, v_dot, q_dot, w_dot, rotor_accel)

    # ------------------------------------------------------------------
    # MPC
    # ------------------------------------------------------------------
    def take_init_guess(self, u_se3):
        self.u_guess = u_se3['cmd_motor_speeds']

    def compute_mpc_feedback(self, x_current, x_ref, u_ref):
        """
        Solve the MPC problem and return the first optimal control input.

        Parameters
        ----------
        x_current : dict
            Current vehicle state from rotorpy (keys: 'x','v','q','w','rotor_speeds').
        x_ref : array (N, 17)
            Reference state trajectory over the horizon.
        u_ref : array (N-1, 4)
            Reference control trajectory over the horizon.

        Returns
        -------
        u_mpc : (4,) numpy array  –  cmd_motor_speeds for the next step
        """
        N   = 4      # prediction horizon
        T   = 0.05   # time step [s]
        n_x = 17
        n_u = 4

        current_state = np.concatenate([
            x_current['x'], x_current['v'], x_current['q'],
            x_current['w'], x_current['rotor_speeds'],
        ])

        opti = ca.Opti()

        # Decision variables
        X = opti.variable(n_x, N)    # (17, N)  – state trajectory
        U = opti.variable(n_u, N-1)  # (4, N-1) – control trajectory

        # --- Initial state constraint ---
        opti.subject_to(X[:, 0] == current_state)

        # --- Dynamics constraints (forward Euler) ---
        for k in range(N - 1):
            x_dot = self.quadrotor_dynamics(X[:, k], U[:, k])
            opti.subject_to(X[:, k+1] == X[:, k] + x_dot * T)

        # --- Input saturation ---
        opti.subject_to(opti.bounded(self.rotor_speed_min, U, self.rotor_speed_max))

        # --- Cost ---
        Q_ca = ca.DM(self.Q)
        R_ca = ca.DM(self.R)
        cost = 0
        for k in range(N - 1):
            dx = X[:, k] - ca.DM(x_ref[k])
            du = U[:, k] - ca.DM(u_ref[k])
            cost += ca.mtimes([dx.T, Q_ca, dx]) + ca.mtimes([du.T, R_ca, du])
        dx_N = X[:, N-1] - ca.DM(x_ref[N-1])
        cost += ca.mtimes([dx_N.T, Q_ca, dx_N])
        opti.minimize(cost)

        # --- Initial guess ---
        opti.set_initial(U, np.tile(self.u_guess, (N-1, 1)).T)
        opti.set_initial(X, np.tile(current_state, (N, 1)).T)

        # --- Solver (IPOPT, silent) ---
        opti.solver('ipopt', {
            'ipopt.print_level': 0,
            'print_time': 0,
            'ipopt.max_iter': 200,
            'ipopt.tol': 1e-4,
            'ipopt.acceptable_tol': 1e-3,
        })

        try:
            sol   = opti.solve()
            u_mpc = sol.value(U[:, 0])
            print("MPC solution:", u_mpc)
        except Exception as e:
            print(f"MPC solver failed: {e}  →  returning initial guess")
            u_mpc = self.u_guess

        return u_mpc
