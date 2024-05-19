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
        self.number_of_states = 5

        # Stage : Intermediate State
        self.number_of_stages = 4

        self.t_max = 40 # s

    def prep_robot_information(self):
        self.minimum_turning_radius = self.robot.get_minimum_turning_radius()
        self.wheel_base = self.robot.get_wheel_base()
        self.max_linear_velocity = self.robot.get_max_linear_velocity()
        self.max_acceleration = self.robot.get_max_acceleration()
        self.max_steering_angle = self.robot.get_max_steering_angle()

        self.k = 1/self.minimum_turning_radius
        self.v = self.max_linear_velocity*0.5

    def prep_problem(self, *args, **kwargs):
        self.states : list[list[State]] = []

        for i in range(self.number_of_states):
            state_idx = str(i)
            state = []
            for j in range(self.number_of_stages):
                idx = state_idx + str(j)
                stage = State(
                    SX.sym("x"+idx), SX.sym("y" + idx), SX.sym("th" + idx),
                    SX.sym("s" + idx), SX.sym("t" + idx)
                )
                for _, value in asdict(stage).items():
                    self.set_variable(value.name(), value)
                state.append(stage)
            self.states.append(state)

    def prep_constraints(self, *args, **kwargs):
        self.initial_state = kwargs["init"]
        self.final_state = kwargs["final"]
        self.obstacles = kwargs["obstacles"]

        X0 = self.states[0][0]; Xn = self.states[-1][-1]

        # Boundary Conditions
        self.set_equality_constraint("x0", X0.x, init.x)
        self.set_equality_constraint("y0", X0.y, init.y)
        self.set_equality_constraint("th0", X0.theta, init.th)

        # TODO : Add tolerances here
        self.set_equality_constraint("xn", Xn.x, final.x)
        self.set_equality_constraint("yn", Xn.y, final.y)
        self.set_equality_constraint("thn", Xn.theta, final.th)

        time_squared_sum = 0
        for i in range(self.number_of_states):
            state_idx = str(i)

            for j in range(1, self.number_of_stages):
                idx = state_idx+str(j)     
                Xi = self.states[i][j]; Xim1 = self.states[i][j-1]

                self.set_constraint("t"+idx, Xi.t, 0)
                self.set_constraint("s"+idx, Xi.sigma, -1, 1)

                arc_phase = Xi.sigma*self.k*self.v*Xi.t*0.5
                self.set_equality_constraint("x"+idx, Xi.x - Xim1.x - self.v*Xi.t*sinc(arc_phase)*cos(Xim1.theta + arc_phase), 0)
                self.set_equality_constraint("y"+idx, Xi.y - Xim1.y - self.v*Xi.t*sinc(arc_phase)*sin(Xim1.theta + arc_phase), 0)
                self.set_equality_constraint("th"+idx, Xi.theta - Xim1.theta - 2*arc_phase, 0)

                time_squared_sum += power(Xi.t, 2)

            if i+1 != self.number_of_states:
                self.set_equality_constraint("continuity_x"+state_idx, self.states[i][-1].x - self.states[i+1][0].x, 0)
                self.set_equality_constraint("continuity_y"+state_idx, self.states[i][-1].y - self.states[i+1][0].y, 0)
                self.set_equality_constraint("continuity_th"+state_idx, self.states[i][-1].theta - self.states[i+1][0].theta, 0)

        self.set_constraint("time_sum", time_squared_sum, 0, self.t_max)

        state_id = 0
        for state in self.states:
            stage_id = 0
            for j in range(1, self.number_of_stages):
                traj_id = 0

                trajectory = self.trajectory(
                    state=state[j-1],
                    v=self.v,
                    k=self.k,
                    n=20,
                    s=state[j].sigma,
                    t=state[j].t
                )

                for intermediate_state in trajectory:
                    idx = str(state_id)+str(stage_id)+str(traj_id)
                    self.set_constraint(
                        "intermediate_x"+idx,
                        intermediate_state.x
                    )
                    self.set_constraint(
                        "intermediate_y"+idx,
                        intermediate_state.y
                    )
                    
                    # Check for obstacles
                    obs_id = 0
                    for obstacle in self.obstacles:
                        # Distance from center of the circle
                        check_obstacle = lambda x, y : power(obstacle[0] - x, 2) + power(obstacle[1] - y, 2) - power(obstacle[2], 2)
                        idx = idx + str(obs_id)
                        self.set_constraint(
                            "check_obs"+idx,
                            check_obstacle(intermediate_state.x, intermediate_state.y),
                            0
                        )
                        obs_id += 1

                    traj_id += 1
                stage_id += 1
            state_id += 1

    @staticmethod
    def trajectory(*args, **kwargs) -> list[State]:
        state : State = kwargs["state"]
        v = kwargs["v"]; k = kwargs["k"]
        n = kwargs["n"]; s = kwargs["s"]
        t = kwargs["t"]

        arc_phase = s*k*v*0.5

        trajectory = []

        for i in range(n+1):
            dt = t*(i/n)

            dx = state.x + v*dt*sinc(arc_phase*dt)*cos(state.theta+arc_phase*dt)
            dy = state.y + v*dt*sinc(arc_phase*dt)*sin(state.theta+arc_phase*dt)
            dth = state.theta + 2*arc_phase

            trajectory.append(State(
                dx, dy, dth, s, dt
            ))

        return trajectory

    def objective(self, *args, **kwargs):
        def length(stateA : State, stateB : State):
            return power((stateA.x - stateB.x), 2) + power((stateA.y - stateB.y), 2)

        path_length = 0
        for i in range(1, len(self.states)):
            path_length += length(self.states[i][0], self.states[i-1][0])

        return path_length

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
                # x0 = self.initial_guess(),
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
    final = Coords2D(1.0, 10.0, 0.0)

    # X, Y, R -> Circular Obstacles
    obstacles = [
        # [1.0, 5.0, 2.0]
    ]

    lm = LimoBot()
    dc = DubinCar(lm)

    ex = Executor(dc)
    ex.prep(init=init, final=final, obstacles=obstacles)
    solution, solver = ex.solve()
    decision_variables = solution["x"]
    constraints = solution["g"]

    import matplotlib.pyplot as plt

    intermediate_values = dc.get_constraint_idx_by_pattern("intermediate")

    note_x = []; note_y = []

    for i in range(0, len(intermediate_values), 2):
        x_idx = intermediate_values[i]; y_idx = intermediate_values[i+1]
        note_x.append(float(constraints[x_idx])); note_y.append(float(constraints[y_idx]))

    plt.plot(note_x, note_y)
    plt.show()