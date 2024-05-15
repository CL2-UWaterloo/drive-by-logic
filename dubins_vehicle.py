#!/usr/bin/python3

from utils import Coords2D, State, CarLikeRobot, LimoBot
from problem import Problem
from executor import Executor

from dataclasses import asdict
from casadi import *

class DubinCar(Problem):

    def __init__(self, robot : CarLikeRobot):
        super(DubinCar, self).__init__()
        self.robot = robot
        self.prep_robot_information()
        self.number_of_states = 15

    def prep_robot_information(self):
        self.minimum_turning_radius = self.robot.get_minimum_turning_radius()
        self.wheel_base = self.robot.get_wheel_base()
        self.max_linear_velocity = self.robot.get_max_linear_velocity()
        self.max_acceleration = self.robot.get_max_acceleration()
        self.max_steering_angle = self.robot.get_max_steering_angle()

    def prep_problem(self, *args, **kwargs):
        """
        Introduce variables. n states (Including the first and last) therefore, n+1 states.
        """
        self.states : list[State] = []

        # iterate from 0 (initial) to n (final)
        for i in range(self.number_of_states + 1):
            idx = str(i)
            state = State(
                SX.sym("x" + idx), SX.sym("y" + idx), SX.sym("th" + idx),
                SX.sym("v" + idx), SX.sym("phi" + idx), SX.sym("t" + idx)
            )
            for _, value in asdict(state).items():
                self.set_variable(value.name(), value)
            self.states.append(state)

    def prep_constraints(self, *args, **kwargs):
        self.initial_state = kwargs["init"]
        self.final_state = kwargs["final"]

        X0 = self.states[0]; Xn = self.states[-1]

        # Boundary Conditions
        self.set_equality_constraint("xi", X0.x, init.x)
        self.set_equality_constraint("yi", X0.y, init.y)
        self.set_equality_constraint("thi", X0.theta, init.th)

        self.set_equality_constraint("xf", Xn.x, final.x, 0.2)
        self.set_equality_constraint("yf", Xn.y, final.y, 0.2)
        self.set_equality_constraint("thf", Xn.theta, final.th)

        # Kinematic Constraints

        # Initial Conditions: start at rest and t = 0
        self.set_equality_constraint("vi", X0.v, 0)
        self.set_equality_constraint("phii", X0.phi, 0)
        self.set_equality_constraint("ti", X0.t, 0)

        # Final Conditions: stop to rest
        self.set_equality_constraint("vf", Xn.v, 0)
        self.set_equality_constraint("phif", Xn.phi, 0)

        # Set kinematics between each state according to simple car
        # https://msl.cs.uiuc.edu/planning/node658.html
        for i in range(1, self.number_of_states+1):
            idx = str(i)
            Xi = self.states[i]; Xi_1 = self.states[i-1]

            # Physical and mathematical constraints

            self.set_constraint("t"+idx, Xi.t, 0)
            self.set_constraint("th_kin"+idx, Xi.theta, -pi, pi)
            self.set_constraint("v_max"+idx, Xi.v, 0, self.max_linear_velocity)
            self.set_constraint("phi_max"+idx, Xi.phi, -self.max_steering_angle, self.max_steering_angle)

            # Bicycle model

            dt = Xi.t - Xi_1.t

            dx = Xi.x - Xi_1.x
            dy = Xi.y - Xi_1.y
            dth = Xi.theta - Xi_1.theta

            self.set_equality_constraint("cycle"+idx, dy*(cos(Xi_1.theta)+cos(Xi.theta)) - dx*(sin(Xi_1.theta)+sin(Xi.theta)), 0)
            self.set_constraint("dt"+idx, dt, 0, 2) # s

            # Velocity and Steering Angle constraints

            d = power(dx, 2) + power(dy, 2)

            self.set_equality_constraint("v"+idx, d - power(Xi.v, 2)*power(dt, 2), 0)
            self.set_equality_constraint("phi"+idx, atan2(dth*self.wheel_base, Xi.v*dt) - Xi.phi, 0)

            # Curvature Constraints

            p = self.wheel_base/tan(Xn.phi)
            self.set_constraint("p_max"+idx, p - self.minimum_turning_radius, 0)

            # Acceleration Constraints

            if (i - 2 >= 0):
                Xi_2 = self.states[i-2]
                dt_1 = Xi_1.t - Xi_2.t
                a = (2*(Xi.v - Xi_1.v))/(dt + dt_1)
                self.set_constraint("a"+idx, a, 0, self.max_acceleration)

        # Time Constraints
        time_sum = 0
        for i in range(0, len(self.states) - 1):
            time_sum += (self.states[i+1].t - self.states[i].t)

        self.set_equality_constraint("time_sum", time_sum - Xn.t + X0.t, 0)

    def objective(self, *args, **kwargs):
        ##### Minimize Path Length
        def length(stateA : State, stateB : State):
            return power((stateA.x - stateB.x), 2) + power((stateA.y - stateB.y), 2)

        path_length = 0
        for i in range(1, len(self.states)):
            path_length += length(self.states[i], self.states[i-1])

        return path_length

        ##### Minimize time of trajectory
        # time_squared_sum = 0
        # for i in range(0, len(self.states) - 1):
        #     time_squared_sum += power((self.states[i+1].t - self.states[i].t), 2)
        
        # return time_squared_sum

    def initial_guess(self, *args, **kwargs):
        guess_variables = []
        for i in self.get_variables():
            guess_variables.append(DM_rand())

        return vertcat(*guess_variables)

    def solve(self, *args, **kwargs):
        constraints = self.get_constraints()
        nlp = {
            "x" : vertcat(*self.get_variables()),
            "f": self.objective(),
            "g": vertcat(*constraints[0])
        }

        opts = {
            "ipopt": {
                "hessian_approximation": "limited-memory",
                "max_iter": 5000
            },
            "jit": True,
            "compiler": "shell"
        }

        sol = nlpsol("Solver", "ipopt", nlp, opts)

        if "warm_start" not in kwargs.keys():
            return sol(
                x0 = self.initial_guess(),
                lbg = vertcat(*constraints[1]),
                ubg = vertcat(*constraints[2])
            ), sol
        else:
            return sol(
                x0 = kwargs["warm_start"],
                lbg = vertcat(*constraints[1]),
                ubg = vertcat(*constraints[2])
            ), sol

if __name__ == "__main__":
    init = Coords2D(0.0, 0.0, 0.0)
    final = Coords2D(1.0, 1.0, 0.0)

    lm = LimoBot()
    dc = DubinCar(lm)

    ex = Executor(dc)
    ex.prep(init=init, final=final)
    solution, solver = ex.solve(warm_start=True, warming_iterations=3)
    decision_variables = solution["x"]

    import matplotlib.pyplot as plt

    note_x = []
    note_y = []

    note_phi = []

    for i in range(0, decision_variables.shape[0], 6):
        x = decision_variables[i]; y = decision_variables[i+1]; th = decision_variables[i+2]
        v = decision_variables[i+3]; phi = decision_variables[i+4]; t = decision_variables[i+5]

        print(x, y, th, v, phi, t)

        note_x.append(float(x))
        note_y.append(float(y))
        
        note_phi.append(float(phi))


    plt.plot(note_x, note_y)
    plt.show()