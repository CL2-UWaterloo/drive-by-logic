#!/usr/bin/python3

from utils import *
from problem import Problem
from executor import Executor

from dataclasses import dataclass, asdict
from casadi import *

@dataclass
class State:
    x : SX
    y : SX
    theta : SX
    t : SX

class DubinCar(Problem):

    def __init__(self, robot : CarLikeRobot):
        super(DubinCar, self).__init__()
        self.robot = robot
        self.prep_robot_information()

        # Formulation Parameters
        self.number_of_states = 5
        self.dt_max = 3 # s

    def prep_robot_information(self):
        self.minimum_turning_radius = self.robot.get_minimum_turning_radius()
        self.wheel_base = self.robot.get_wheel_base()
        self.max_linear_velocity = self.robot.get_max_linear_velocity()
        self.max_acceleration = self.robot.get_max_acceleration()
        self.max_steering_angle = self.robot.get_max_steering_angle()

    def prep_problem(self, *args, **kwargs):
        self.states : list[State] = []

        for i in range(self.number_of_states):
            idx = str(i)
            state = State(
                SX.sym("x"+idx), SX.sym("y" + idx),
                SX.sym("th" + idx), SX.sym("t" + idx)
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

        # TODO : Add tolerances here
        self.set_equality_constraint("xf", Xn.x, final.x)
        self.set_equality_constraint("yf", Xn.y, final.y)
        self.set_equality_constraint("thf", Xn.theta, final.th)

        for i in range(self.number_of_states - 1):
            idx = str(i); Xi = self.states[i]; Xip1 = self.states[i+1]

            # Time Constraint
            # It is allowed for a state to take upto dt_max seconds to transition
            self.set_constraint("t"+idx, Xi.t, 0, self.dt_max)

            # Bicycle Model Constraint
            dy = Xip1.y - Xi.y; dx = Xip1.x - Xi.x; dth = Xip1.theta - Xi.theta
            self.set_equality_constraint(
                "cycle"+idx,
                dy*(cos(Xip1.theta) + cos(Xi.theta)) - dx*(sin(Xip1.theta) + sin(Xi.theta)),
                0
            )

            # Curvature Constraint
            di2 = power(dy, 2) + power(dx, 2)
            pi2 = di2/power(2*sin(dth/2), 2)
            ri2 = di2/dth

            self.set_constraint(
                "curvature"+idx,
                ri2, power(self.minimum_turning_radius, 2)
            )

            # Velocity Constraints
            self.set_constraint(
                "velocity"+idx,
                (pi2*power(dth, 2))/power(Xi.t, 2),
                0, power(self.max_linear_velocity, 2)
            )

            # if i + 2 <= self.number_of_states - 1 and i - 2 <= 0:
            #     # vi = 
            #     # ai = (2*(Xip1.v - Xi.v))/(Xip1.t + Xi.t)
            #     self.set_constraint(
            #         "acceleration"+idx,
            #         ai, 0, self.max_acceleration
            #     )                

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
        # for state in self.states:
        #     time_squared_sum += power(state.t, 2)

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
    solution, solver = ex.solve()
    decision_variables = solution["x"]

    import matplotlib.pyplot as plt

    note_x = []
    note_y = []

    note_phi = []

    for i in range(0, decision_variables.shape[0], 4):
        x = decision_variables[i]; y = decision_variables[i+1]; th = decision_variables[i+2]
        t = decision_variables[i+3]

        print(x, y, th, t)

        note_x.append(float(x))
        note_y.append(float(y))
        note_phi.append(float(th))


    plt.plot(note_x, note_y)
    plt.show()