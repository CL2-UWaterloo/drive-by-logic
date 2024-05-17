#!/usr/bin/python3

from utils import *
from problem import Problem
from executor import Executor

from dataclasses import dataclass, asdict
from casadi import *

@dataclass
class State:
    x : SX | float
    y : SX | float
    theta : SX | float
    sigma : SX | float
    t : SX | float

class DubinCar(Problem):

    def __init__(self, robot : CarLikeRobot):
        super(DubinCar, self).__init__()
        self.robot = robot
        self.prep_robot_information()

        # Formulation Parameters
        self.number_of_states = 4
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
                SX.sym("x"+idx), SX.sym("y" + idx), SX.sym("th" + idx),
                SX.sym("s" + idx), SX.sym("t" + idx)
            )
            for _, value in asdict(state).items():
                self.set_variable(value.name(), value)
            self.states.append(state)

    def prep_constraints(self, *args, **kwargs):
        self.initial_state = kwargs["init"]
        self.final_state = kwargs["final"]

        X0 = self.states[0]; Xn = self.states[-1]

        # Boundary Conditions
        self.set_equality_constraint("x0", X0.x, init.x)
        self.set_equality_constraint("y0", X0.y, init.y)
        self.set_equality_constraint("th0", X0.theta, init.th)

        # TODO : Add tolerances here
        self.set_equality_constraint("xn", Xn.x, final.x)
        self.set_equality_constraint("yn", Xn.y, final.y)
        self.set_equality_constraint("thn", Xn.theta, final.th)

        k = 1/self.minimum_turning_radius
        v = 1

        for i in range(1, self.number_of_states):
            idx = str(i)
            Xi = self.states[i]; Xim1 = self.states[i-1]

            self.set_constraint("t"+idx, Xi.t, 0)
            self.set_constraint("s"+idx, Xi.sigma, -1, 1)

            arc_phase = Xi.sigma*k*v*Xi.t*0.5
            self.set_equality_constraint("x"+idx, Xi.x - Xim1.x - v*Xi.t*sinc(arc_phase)*cos(Xim1.theta + arc_phase), 0)
            self.set_equality_constraint("y"+idx, Xi.y - Xim1.y - v*Xi.t*sinc(arc_phase)*sin(Xim1.theta + arc_phase), 0)
            self.set_equality_constraint("th"+idx, Xi.theta - Xim1.theta - 2*arc_phase, 0)

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
    final = Coords2D(0.0, 10.0, 0.0)

    lm = LimoBot()
    dc = DubinCar(lm)

    ex = Executor(dc)
    ex.prep(init=init, final=final)
    solution, solver = ex.solve()
    decision_variables = solution["x"]

    import matplotlib.pyplot as plt

    states : list[State] = []

    for i in range(0, decision_variables.shape[0], 5):
        x = decision_variables[i]; y = decision_variables[i+1]; th = decision_variables[i+2]
        s = decision_variables[i+3]; t = decision_variables[i+4]

        print(x, y, th, s, t)

        states.append(
            State(
                float(x), float(y), float(th),
                float(s), float(t)
            )
        )

    note_x = []
    note_y = []

    m = 10
    v = 1
    k = 1/lm.minimum_turning_radius

    prev_state : State = None
    for state in states:
        if not prev_state:
            prev_state = state
            continue

        for i in range(m + 1):
            t = (i/m)*state.t
            arc_phase = state.sigma*k*v*0.5
            x = prev_state.x + v*t*sinc(arc_phase*t)*cos(prev_state.theta+arc_phase*t)
            y = prev_state.y + v*t*sinc(arc_phase*t)*sin(prev_state.theta+arc_phase*t)

            note_x.append(x)
            note_y.append(y)

        prev_state = state

    plt.plot(note_x, note_y)
    plt.show()