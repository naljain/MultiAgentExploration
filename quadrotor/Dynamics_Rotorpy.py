import numpy as np
from pydrake.solvers import MathematicalProgram, SnoptSolver, SolverOptions
from pydrake.symbolic import Expression, if_then_else, sqrt
import math
class MPC_RotorPy(object):
    def __init__(self, map,  vehicle):
        """
        Use this constructor to save vehicle parameters, set controller gains, etc.
        Parameters:
            vehicle_params, dict with keys specified in a python file under /rotorpy/vehicles/

        """
        self.vehicle = vehicle
        self.map = map
        self.d_safe = 0.1
        self.Q = np.diag([10, 10, 5, 1, 1, 1, 3, 3, 3, 3, 1, 1, 1, 1,1,1,1])

        self.R = np.eye(4)
        self.thrust_d = np.array([1788.53, 1788.53, 1788.53, 1788.53])
        self.tau_m = vehicle.tau_m
        self.rotor_geometry = vehicle.rotor_geometry
        self.k_eta = vehicle.k_eta
        self.num_rotors = vehicle.num_rotors
        self.k_flap = vehicle.k_flap
        self.k_m = vehicle.k_m
        self.rotor_dir = vehicle.rotor_dir
        self.weight = vehicle.weight
        self.mass = vehicle.mass
        self.inertia = vehicle.inertia
        self.inv_inertia = vehicle.inv_inertia
        self.gravity = 9.81
        self.rotor_speed_min = vehicle.rotor_speed_min
        self.rotor_speed_max = vehicle.rotor_speed_max
        self.drag_matrix = vehicle.drag_matrix
        self.rotor_drag_matrix = vehicle.rotor_drag_matrix

    def hat_map(cls, s):
        """
        Given vector s in R^3, return associate skew symmetric matrix S in R^3x3
        In the vectorized implementation, we assume that s is in the shape (N arrays, 3)
        """
        if len(s.shape) > 1:  # Vectorized implementation
            return np.array([[ np.zeros(s.shape[0]), -s[:,2],  s[:,1]],
                             [ s[:,2],     np.zeros(s.shape[0]), -s[:,0]],
                             [-s[:,1],  s[:,0],     np.zeros(s.shape[0])]])
        else:
            return np.array([[    0, -s[2],  s[1]],
                             [ s[2],     0, -s[0]],
                             [-s[1],  s[0],     0]])
    # def einsum_replacement(cls, tensor_ijk, matrix_ik):
    #     result_j = np.zeros(tensor_ijk.shape[1])
        
    #     for i in range(tensor_ijk.shape[0]):
    #         for k in range(tensor_ijk.shape[2]):
    #             result_j += tensor_ijk[i, :, k] * matrix_ik[i, k]
        
    #     return result_j
    def einsum_replacement(cls, tensor_ijk, matrix_ik):
        result_j = [Expression(0) for _ in range(tensor_ijk.shape[1])]
        
        for i in range(tensor_ijk.shape[0]):
            for k in range(matrix_ik.shape[1]):
                for j in range(tensor_ijk.shape[1]):
                    result_j[j] += tensor_ijk[i, j, k] * matrix_ik[i, k]
    
        return result_j
    
    def take_init_guess(self, u_se3):
        self.u_guess = u_se3['cmd_motor_speeds']
        
    def add_intial_state_constraint(self, prog, x, x_current):
        """
        this x_current should come from position data of the agent, not
        from waypoints as the prev agent could have ended not on the waypoint
        for the prev traj segment
        """
        # state = np.concatenate([x_current['x'], x_current['v'], x_current['q'], x_current['w']])
        # state_x_v = np.concatenate([x_current['x'], x_current['v']])
        # n_x_v = state_x_v.shape[0]
        # state_p_w = np.concatenate([x_current['q'], x_current['w']])
        # state_p_w = np.concatenate([x_current['w']])
        # n_p_w = state_p_w.shape[0]
        # print(n_x)
        state = np.concatenate([x_current['x'], x_current['v'], x_current['q'], x_current['w']])
        n_x = state.shape[0]
        rotor_state = x_current['rotor_speeds']
        n_rotor = rotor_state.shape[0]
        for i in range(n_x):
            prog.AddBoundingBoxConstraint(state[i], state[i], x[0, i])
        for i in range(n_rotor):
            prog.AddBoundingBoxConstraint(self.rotor_speed_min, self.rotor_speed_max, x[0, i+n_x])
        # for i in range(n_p_w):
        #     prog.AddBoundingBoxConstraint(state_p_w[i], state_p_w[i], x[0, i+n_x_v])
    
    def add_input_saturation_constraint(self, prog, x, u, N):
        n_u = u.shape[1]

        #in homework they have u_d here, not sure if we need that?
        for j in range(n_u):
            for i in range(N-1):
                prog.AddBoundingBoxConstraint(self.rotor_speed_min, self.rotor_speed_max, u[i, j])

    def symbolic_quadrotor_dynamics(self,x, u):
        """
        Define symbolic quadrotor dynamics for pydrake. Written for individual motor speeds or "cmd_motor_speeds" as control input.

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
        # convert u into controls
        
        # Define symbolic state variables
        p = x[0:3]  # Position: [x, y, z]
        v = x[3:6]  # Velocity: [vx, vy, vz]
        q = x[6:10] # Orientation: Quaterniom[i, j, k, w]
        omega = x[10:13] # Angular velocity: [wx, wy, wz]
        rotor_speed = x[13:17] # Rotor speeds: [w1, w2, w3, w4]

        # Define symbolic control inputs
        cmd_speed = u[0:4]  # Motor speeds: [w1, w2, w3, w4]


        # Gravity vector
        g = np.array([0, 0, -self.gravity])

        # Rotation matrix from body to world frame (Quats)
        i, j, k, w = q
        # Normalize quaternion
        q = np.array([i, j, k, w])
        norm_q = sqrt(i**2 + j**2 + k**2 + w**2)
        epsilon = Expression(1e-6)
        default_quaternion = np.array([Expression(0), Expression(0), Expression(0), Expression(1)])
        # for i in range(4):
        #     q[i] = if_then_else(
        #         norm_q > epsilon,
        #         q[i] / norm_q,
        #     default_quaternion[i])
        q = q / (np.linalg.norm(q) + 0.0000001)

        # R, Rotation Matrix
        R = np.array([
            [1 - 2*q[1]**2 - 2*q[2]**2, 2*q[0]*q[1] - 2*q[3]*q[2], 2*q[0]*q[2] + 2*q[1]*q[3]],
            [2*q[0]*q[1] + 2*q[3]*q[2], 1 - 2*q[0]**2 - 2*q[2]**2, 2*q[1]*q[2] - 2*q[0]*q[3]],
            [2*q[0]*q[2] - 2*q[1]*q[3], 2*q[1]*q[2] + 2*q[3]*q[0], 1 - 2*q[0]**2 - 2*q[1]**2]
        ])
        
        # Rotor speed derivatives
        rotor_accel = (1/self.tau_m)*(rotor_speed - cmd_speed)

        # Position derivatives
        x_dot = v*1
        
        # Orientation derivatives
        (q0, q1, q2, q3) = q
        G = np.array([[q3, q2, -q1, -q0],
                      [-q2, q3, q0, -q1],
                      [q1, -q0, q3, -q2]])
        quat_dot = 0.5 * G.T @ omega
        quat_err = np.sum(q**2) - 1
        quat_err_grad = 2* q
        quat_dot = quat_dot - 0.5 * quat_err * quat_err_grad

        # Compute Body Wrench, No wind though
        body_airspeed_vector = R.T@(v)
        hat_body_rates = self.hat_map(omega)@(self.rotor_geometry.T)
        local_airspeeds = body_airspeed_vector[:, np.newaxis] + hat_body_rates
        # Thrust of each rotor, assuming that the rotors all point in the body z direction
        T = np.array([0, 0, self.k_eta])[:, np.newaxis]*rotor_speed**2

        # No aero, F that S
        D = -np.linalg.norm(body_airspeed_vector)*self.drag_matrix@body_airspeed_vector
        H = -rotor_speed*(self.rotor_drag_matrix@local_airspeeds)
        M_flap = -self.k_flap*rotor_speed*((self.hat_map(local_airspeeds.T).transpose(2,0,1))@np.array([0,0,1])).T
        # D = np.zeros(3,)
        # H = np.zeros((3,self.num_rotors))
        # M_flap = -self.k_flap*rotor_speed*((self.hat_map(hat_body_rates.T).transpose(2,0,1))@np.array([0,0,1])).T

        # Compute moments due to rotor thrusts, again none of that air shit
        # M_force = -self.einsum_replacement(self.hat_map(self.rotor_geometry), T+H)
        M_force = [-x for x in self.einsum_replacement(self.hat_map(self.rotor_geometry), T+H)]
        M_yaw = self.rotor_dir*(np.array([0,0,self.k_m])[:, np.newaxis]*rotor_speed**2)

        # Sum all elements to compute the the total body wrench
        FtotB = np.sum(T + H, axis=1) + D
        MtotB = M_force + np.sum(M_yaw + M_flap, axis=1)

        Ftot = R@FtotB

        # Velocity Derviatices
        v_dot = (self.weight + Ftot)/self.mass

        #Angular Velocity derivative
        w_hat = self.hat_map(omega)
        w_dot = self.inv_inertia @ (MtotB - w_hat @(self.inertia @ omega))

        state_dot = np.concatenate([x_dot, v_dot, quat_dot, w_dot, rotor_accel])

        # return derivatives
        return state_dot
    
    def add_dynamic_constraints(self, prog, x, u, T, N):
        n_x = len(x[0])
        for k in range(N-1):
            xk = x[k]
            # print('len of xk in dyan co', len(xk))
            uk = u[k]
            x_dot = self.symbolic_quadrotor_dynamics(xk, uk)
            # xk_1 = xk + x_dot*T
            xk_1 = [xk[i] + x_dot[i] * T for i in range(n_x)]
            # print('len of xk+1 in dyan co', len(xk_1))
            # prog.AddConstraint(xk_1 - xk == 0)
            for i in range(n_x):
                # Create formula Object for Contraint
                constraint = (Expression(x[k+1][i]) == xk_1[i])
                prog.AddConstraint(constraint)
    #             # prog.AddConstraint((xk_1[i] - xk[i]), np.array([0]))
    #             # prog.AddConstraint((xk_1[i] - xk[i]) == 0)
    #             # prog.AddBoundingBoxConstraint(0, 0, xk_1[i] - xk[i], lb=np.array([0]), ub=np.array([0]))
    #     # prog.AddLinearEqualityConstraint((x[k + 1]), xk_1)
    # def add_dynamic_constraints(self, prog, x, u, T, N):
    #     for i in range(N-1):
    #         x_dot = self.symbolic_quadrotor_dynamics(x[i], u[i])
    #         prog.AddConstraint(x[i+1] == (x[i] + x_dot * T))
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

    def add_distance_constraint(self, prog, x, N):
        avoid = np.array([0.2, 0.2])
        for k in range(N):
            xk, yk = x[k, 0:2]
            d = self.barrier_dist((xk, yk), avoid)
            barrier_cost = -np.log(abs(d - self.d_safe))**2
            prog.AddCost(barrier_cost)

    def add_cost(self, prog, x, x_ref, u, u_ref, N):
        
        for k in range(N-1):
            # print((x[k] - x_ref[k]).T @ self.Q @ (x[k] - x_ref[k]))
            prog.AddQuadraticCost(
                (x[k] - x_ref[k]).T @ self.Q @ (x[k] - x_ref[k]) + (u[k]-u_ref[k]).T @ self.R @ (u[k]-u_ref[k])
            )
            # prog.AddQuadraticCost(
            #     (x[k] - x_ref[k]).T @ self.Q @ (x[k] - x_ref[k])
            #     )

        prog.AddQuadraticCost((x[N-1] - x_ref[N-1]).T @ self.Q @ (x[N-1] - x_ref[N-1]))

    def compute_mpc_feedback(self, x_current, x_ref, u_ref):
        # x_test = np.array([0, 0, 0.1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1788.53, 1788.53, 1788.53, 1788.53])
        # u_test = np.array([0, 0, 0, 0])
        # self.symbolic_quadrotor_dynamics(x_test, u_test)

        current_state = np.concatenate([x_current['x'], x_current['v'], x_current['q'], x_current['w'], x_current['rotor_speeds']])
        # QP params
        N = 6  # prediction horizon TODO NEEDS TO BE TUNED
        T = 0.05 # time step

        # initialise mathematical program
        self.prog = MathematicalProgram()

        # initialise decision variables
        x = np.zeros((N, 17), dtype= "object")
        for i in range(N):
            x[i] = self.prog.NewContinuousVariables(17, 'x_' + str(i))
        u = np.zeros((N-1, 4), dtype = "object")
        for i in range(N-1):
            u[i] = self.prog.NewContinuousVariables(4, 'u_' + str(i))

        # add constraints
        self.add_intial_state_constraint(self.prog, x, x_current)

        self.add_input_saturation_constraint(self.prog, x, u, N)

        # Add dynamic constraints
        self.add_dynamic_constraints(self.prog, x, u, T, N)


        # add cost
        # self.add_barrier_obstacle_constraint(prog, x, N)
        self.add_distance_constraint(self.prog, x, N)
        
        # TODO input x_ref
        self.add_cost(self.prog, x, x_ref, u,u_ref, N)

        # solve the QP
        solver = SnoptSolver()
        solver_id = solver.id()
        self.prog.SetSolverOption(solver_id=solver_id, solver_option='Print file', option_value='./snopt.out')
        # prog.SetInitialGuess(u, [[1788.53, 1788.53, 1788.53, 1788.53],
        #                                  [1788.53, 1788.53, 1788.53, 1788.53],
        #                                  [1788.53, 1788.53, 1788.53, 1788.53]]
        # )
        # self.prog.SetInitialGuess(u, [[0.00001, 0.00001, 0.00001, 0.00001],
        #              [0.00001, 0.00001, 0.00001, 0.00001],
        #              [0.00001, 0.00001, 0.00001, 0.00001]]
        # )
        self.prog.SetInitialGuess(u, [self.u_guess, self.u_guess, self.u_guess])
        self.prog.SetInitialGuess(x, [[0.000001, 0.000001, 0.000001, 0.000001, 0.000001, 0.000001, 0.000001, 0.000001, 0.000001, 0.000001, 0.000001, 0.000001, 0.000001, 0.000001, 0.000001, 0.000001, 0.000001], # had to do this so it doesn't error out
                 [0.000001, 0.000001, 0.000001, 0.000001, 0.000001, 0.000001, 0.000001, 0.000001, 0.000001, 0.000001, 0.000001, 0.000001, 0.000001, 0.000001, 0.000001, 0.000001, 0.000001],
                 [0.000001, 0.000001, 0.000001, 0.000001, 0.000001, 0.000001, 0.000001, 0.000001, 0.000001, 0.000001, 0.000001, 0.000001, 0.000001, 0.000001, 0.000001, 0.000001, 0.000001],
                 [0.000001, 0.000001, 0.000001, 0.000001, 0.000001, 0.000001, 0.000001, 0.000001, 0.000001, 0.000001, 0.000001, 0.000001, 0.000001, 0.000001, 0.000001, 0.000001, 0.000001]])
        result = solver.Solve(self.prog)
        if result.is_success():
            print("Solution found!")
        else:
            print("Solver failed.")        # get u_mpc
        # u_mpc = result.GetSolution(u[0])
        u_mpc = result.GetSolution(u[0]) # this is from hw, so need to see if this is correct
        print(u_mpc)
        return u_mpc
    # def symbolic_dynamics(x, u, rotorpy_model):
