import casadi as ca
import numpy as np
from rotorpy.vehicles.crazyflie_params import quad_params
from rotorpy.vehicles.multirotor import Multirotor
from quadrotor.mpc_template import MPC_Controller
from src.global_planner import GlobalPlanner
from rotorpy.vehicles.crazyflie_params import quad_params
from rotorpy.vehicles.multirotor import Multirotor
from quadrotor.mpc_template import MPC_Controller
from src.global_planner import GlobalPlanner
import numpy as np

# plannar params
agents = {1: 123}  # , 2: 343, 3: 4444, 4: 444245, 5:13434} # dict = {agent num : agent id}
time_step = 1

bloat_val = 4  # BLOAT_VAL > RADIUS OF DRONE
unknown_travel = True
senor_range = 3  # 30 cm
map = np.loadtxt('../src/test_map2')
Planner = GlobalPlanner(map, agents, time_step, bloat_val, senor_range,
                            unknown_travel)
initial_pose = Planner.start_pos
goal_pose = Planner.run_planner()

traj = np.array([(0, 0), (1, 1), (2, 2), (2, 3)])

# Initialize RotorPy model
rotorpy_model = Multirotor(quad_params, control_abstraction='cmd_motor_thrusts')

# Define symbolic variables for states and controls
# Define symbolic variables for states and controls
x = ca.MX.sym("x", 12)  # State vector (12 variables for a quadrotor)
u = ca.MX.sym("u", 4)   # Control inputs (4 variables for rotor thrusts)

# Define CasADi-compatible RotorPy dynamics
def rotorpy_dynamics(state, control, rotorpy_model):
    """
    Convert RotorPy dynamics into a CasADi-compatible function.

    Parameters:
        state: CasADi MX symbolic state vector
        control: CasADi MX symbolic control vector
        rotorpy_model: Instance of RotorPy Multirotor class

    Returns:
        CasADi symbolic state derivative (state_dot)
    """
    # Ensure state and control are symbolic vectors
    state_vector = ca.vertcat(*[state[i] for i in range(state.size1())])  # Safe indexing
    control_vector = ca.vertcat(*[control[i] for i in range(control.size1())])  # Safe indexing

    # Convert symbolic matrices to a format RotorPy can evaluate numerically
    state_dot_np = rotorpy_model.statedot(control_vector, state_vector, 0.1)

    # Ensure the returned value is CasADi-compatible
    state_dot = ca.vertcat(*state_dot_np)
    return state_dot


# Create a CasADi function for the dynamics
f_dynamics = ca.Function("f_dynamics", [x, u], [rotorpy_dynamics(x, u, rotorpy_model)])
