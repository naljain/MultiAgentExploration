import numpy as np
from pydrake.symbolic import Expression, Variable
from pydrake.math import RotationMatrix
from pydrake.common.eigen_geometry import Quaternion

class Quad_dynamics:
    def __init__(self, vehicle):
         self.tau_m = vehicle.tau_m
    @staticmethod
    def _s_dot_fn(t: Variable, s: np.ndarray, cmd_rotor_speeds: np.ndarray) -> np.ndarray:
        # Unpack state
        x = s[0:3]
        v = s[3:6]
        q = np.array
        w = s[10:13]
        rotor_speeds = s[13:]

        # Create RotationMatrix from quaternion
        R = RotationMatrix(Quaternion(q))

        # Rotor speed derivative
        rotor_accel = (1/self.tau_m) * (cmd_rotor_speeds - rotor_speeds)

        # Position derivative
        x_dot = v

        # Orientation derivative
        q_dot = Quad_dynamics.quat_dot(q, w)  # Implement this function using pydrake

        # Compute airspeed vector in the body frame
        body_airspeed_vector = R.inverse() @ (v - wind)

        # Compute total wrench in the body frame
        FtotB, MtotB = Quad_dynamics.compute_body_wrench(w, rotor_speeds, body_airspeed_vector)  # Implement this function

        # Rotate the force from the body frame to the inertial frame
        Ftot = R @ FtotB

        # Velocity derivative
        v_dot = (self.weight + Ftot) / self.mass

        # Angular velocity derivative
        w_hat = Multirotor.hat_map(w)  # Implement this function
        w_dot = self.inv_inertia @ (MtotB - w_hat @ (self.inertia @ w))

        # Wind derivative (currently zero)
        wind_dot = Expression([0, 0, 0])

        # Pack into vector of derivatives
        s_dot = np.empty(16 + self.num_rotors, dtype=object)
        s_dot[0:3] = x_dot
        s_dot[3:6] = v_dot
        s_dot[6:10] = q_dot
        s_dot[10:13] = w_dot
        s_dot[13:16] = wind_dot
        s_dot[16:] = rotor_accel

        return s_dot

    @staticmethod
    def compute_body_wrench(body_rates, rotor_speeds, body_airspeed_vector):
            # Get the local airspeeds for each rotor
            local_airspeeds = body_airspeed_vector[:, np.newaxis] + Multirotor.hat_map(body_rates) @ (self.rotor_geometry.T)

            # Compute the thrust of each rotor
            T = np.array([Expression(0), Expression(0), self.k_eta])[:, np.newaxis] * rotor_speeds**2

            # Add in aero wrenches (if applicable)
            if self.aero:
                # Parasitic drag force acting at the CoM
                D = -Multirotor._norm(body_airspeed_vector) * self.drag_matrix @ body_airspeed_vector
                # Rotor drag (aka H force) acting at each propeller hub
                H = -rotor_speeds * (self.rotor_drag_matrix @ local_airspeeds)
                # Pitching flapping moment acting at each propeller hub
                M_flap = -self.k_flap * rotor_speeds * ((Quad_dynamics.hat_map(local_airspeeds.T).transpose(2, 0, 1)) @ np.array([Expression(0), Expression(0), Expression(1)])).T
            else:
                D = np.array([Expression(0), Expression(0), Expression(0)])
                H = np.zeros((3, self.num_rotors), dtype=object)
                M_flap = np.zeros((3, self.num_rotors), dtype=object)

            # Compute the moments due to the rotor thrusts, rotor drag (if applicable), and rotor drag torques
            M_force = -Quad_dynamics.einsum_symbolic('ijk, ik->j', Quad_dynamics.hat_map(self.rotor_geometry), T+H)
            M_yaw = self.rotor_dir * (np.array([Expression(0), Expression(0), self.k_m])[:, np.newaxis] * rotor_speeds**2)

            # Sum all elements to compute the total body wrench
            FtotB = Quad_dynamics.sum_symbolic(T + H, axis=1) + D
            MtotB = M_force + Quad_dynamics.sum_symbolic(M_yaw + M_flap, axis=1)

            return (FtotB, MtotB)

    @staticmethod
    def hat_map(w):
            return RotationMatrix.Hat(w).matrix()

    @staticmethod
    def _norm(v):
            return (v[0]**2 + v[1]**2 + v[2]**2)**0.5

    @staticmethod
    def einsum_symbolic(subscripts, *operands):
            # Implement a simplified einsum for symbolic expressions
            # This is a basic implementation and may need to be expanded for more complex cases
            if subscripts == 'ijk, ik->j':
                result = np.zeros(operands[0].shape[1], dtype=object)
                for j in range(operands[0].shape[1]):
                    for i in range(operands[0].shape[0]):
                        for k in range(operands[0].shape[2]):
                            result[j] += operands[0][i,j,k] * operands[1][i,k]
                return result
            else:
                raise NotImplementedError(f"einsum operation '{subscripts}' not implemented for symbolic expressions")

    @staticmethod
    def sum_symbolic(arr, axis=None):
            if axis is None:
                return sum(arr.flatten())
            elif axis == 0:
                return np.array([sum(arr[:,i]) for i in range(arr.shape[1])])
            elif axis == 1:
                return np.array([sum(arr[i,:]) for i in range(arr.shape[0])])
            else:
                raise ValueError("Unsupported axis for sum_symbolic")

    from pydrake.symbolic import Expression, Variable
    import numpy as np

    def quat_dot(quat, omega):
        """
        Parameters:
            quat: Quaternion [i,j,k,w]
            omega: angular velocity of body in body axes

        Returns:
            quat_dot: [i,j,k,w]
        """
        # Extract quaternion components
        q0, q1, q2, q3 = quat[0], quat[1], quat[2], quat[3]

        # Construct G matrix using symbolic expressions
        G = np.array([
            [ q3,  q2, -q1, -q0],
            [-q2,  q3,  q0, -q1],
            [ q1, -q0,  q3, -q2]
        ])

        # Compute quaternion derivative
        quat_dot = 0.5 * G.T @ omega

        # Augment to maintain unit quaternion
        quat_err = sum(q**2 for q in quat) - 1
        quat_err_grad = 2 * quat
        quat_dot = quat_dot - quat_err * quat_err_grad

        return quat_dot