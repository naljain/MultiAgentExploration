def add_dynamics_constraint(self, prog, x, u, N, T):
    state_for_rotorpy = {'x': x[0:3], 'v':x[3:6], 'q':x[6:10],}
    input_for_rotorpy = {'cmd_motor_thrusts': u}
    multirotor = self.vehicle
    dynamics = multirotor.statedot(state_for_rotorpy, input_for_rotorpy, T)
    for i in range(N-1):
        prog.AddQuadraticCost((x[i]).T @ self.Q @ (x[i]) + (u[i]).T @ self.R @ (u[i]))
    prog.AddQuadraticCost((x[N-1]).T @ self.Qf @ (x[N-1]))