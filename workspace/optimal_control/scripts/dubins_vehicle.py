#!/usr/bin/python3

from optimal_control.utils import *
from optimal_control.problem import Problem
from optimal_control.executor import Executor

from dataclasses import dataclass, asdict
from casadi import *

# TODO: Refactor Name from state to anything else
@dataclass
class State:
    # Position Information
    x : SX | float
    y : SX | float
    theta : SX | float

    # Below information encodes how the robot got to this state
    # Formulation inspired from: https://ieeexplore.ieee.org/document/9143597

    # Segment 1
    s0 : SX | float
    t0 : SX | float
    
    # Segment 2
    s1 : SX | float
    t1 : SX | float

    # Segment 3
    s2 : SX | float
    t2 : SX | float

    # Velocity and Curvature
    # To imitate Dubin's formulation of a car, the velocity and curvature are constant
    v : SX | float
    k : SX | float

# TODO: Should obstacle checking be in terms of forward kinematics of the car model?

class DubinCar(Problem):

    def __init__(self, robot : CarLikeRobot):
        super(DubinCar, self).__init__()
        self.robot = robot
        self.prep_robot_information()

        # Formulation Parameters
        self.number_of_states = 3

        self.granularity = 5

        self.t_max = 20 # s

    def prep_robot_information(self):
        self.minimum_turning_radius = self.robot.get_minimum_turning_radius()
        self.wheel_base = self.robot.get_wheel_base()
        self.max_linear_velocity = self.robot.get_max_linear_velocity()
        self.max_acceleration = self.robot.get_max_acceleration()
        self.max_steering_angle = self.robot.get_max_steering_angle()

        self.k = 1/self.minimum_turning_radius
        self.v = self.max_linear_velocity*0.5

    def prep_problem(self, *args, **kwargs):
        self.states : list[State] = []

        for i in range(self.number_of_states):
            idx = str(i)
            
            # Make State
            state = State(
                SX.sym("x"+idx), SX.sym("y"+idx), SX.sym("th"+idx),
                SX.sym("s0"+idx), SX.sym("t0"+idx),
                SX.sym("s1"+idx), SX.sym("t1"+idx),
                SX.sym("s2"+idx), SX.sym("t2"+idx),
                SX.sym("v"+idx), SX.sym("k"+idx)
            )
            
            # Declare Decision Variables
            for _, value in asdict(state).items():
                self.set_variable(value.name(), value)

            self.states.append(state)

    def prep_constraints(self, *args, **kwargs):
        # Get Initial, Final states, and Obstacles from arguments.
        self.initial_state = kwargs["init"]
        self.final_state = kwargs["final"]
        self.obstacles = kwargs["obstacles"]

        # The first state is X0 and the last is Xn
        X0 = self.states[0]; Xn = self.states[-1]

        # Boundary Conditions
        self.set_equality_constraint("x0", X0.x, init.x)
        self.set_equality_constraint("y0", X0.y, init.y)
        self.set_equality_constraint("th0", X0.theta, init.th)

        # Final Boundary Conditions are subject to change based on problem description

        # TODO : Add tolerances here to represent a goal region instead of pose
        self.set_equality_constraint("xn", Xn.x, final.x)
        self.set_equality_constraint("yn", Xn.y, final.y)

        # Orientation is the same for any point reached within goal region
        self.set_equality_constraint("thn", Xn.theta, final.th)

        # Defining parametric equations for x, y
        def parametric_x(l, k, th):
            arc_phase = k*l*0.5
            return l*sinc(arc_phase)*cos(th + arc_phase)

        def parametric_y(l, k, th):
            arc_phase = k*l*0.5
            return l*sinc(arc_phase)*sin(th + arc_phase)

        # Iterate through all states to establish constraints
        time_sum = 0
        for i in range(1, self.number_of_states):
            idx = str(i)

            # Current state and previous state (Xim1 = Xi-1)
            Xi = self.states[i]; Xim1 = self.states[i-1]

            # Velocity Selection Constraint
            self.set_constraint(
                "v"+idx, Xi.v, 0, self.max_linear_velocity
            )

            # Acceleration Constraint
            self.set_constraint(
                "a"+idx, Xi.v - Xim1.v, 0, self.max_acceleration
            )

            # Curvature Selection Constraint
            self.set_constraint(
                "k"+idx, Xi.k, 0, self.k
            )

            # Smoothness Constraint
            # self.set_constraint(
            #     "k_dot"+idx, Xi.k - Xim1.k, 0, 0.1
            # )

            # Time Selection Constraint
            self.set_constraint(
                "t0"+idx, Xi.t0, 0
            )
            self.set_constraint(
                "t1"+idx, Xi.t1, 0
            )
            self.set_constraint(
                "t2"+idx, Xi.t2, 0
            )

            # Sigma Selection Constraint
            self.set_constraint(
                "s0"+idx, Xi.s0, -1, 1
            )
            self.set_constraint(
                "s1"+idx, Xi.s1, -1, 1
            )
            self.set_constraint(
                "s2"+idx, Xi.s2, -1, 1
            )

            # Start with establishing the equality constraint between final and initial positions
            self.set_equality_constraint(
                "x"+idx,
                Xi.x - Xim1.x \
                    - parametric_x(Xi.v*Xi.t0, Xi.k*Xi.s0, Xim1.theta) \
                    - parametric_x(Xi.v*Xi.t1, Xi.k*Xi.s1, Xim1.theta + Xi.k*Xi.s0*Xi.v*Xi.t0) \
                    - parametric_x(Xi.v*Xi.t2, Xi.k*Xi.s2, Xim1.theta + Xi.k*Xi.s0*Xi.v*Xi.t0 + Xi.k*Xi.s1*Xi.v*Xi.t1),
                0
            )

            self.set_equality_constraint(
                "y"+idx,
                Xi.y - Xim1.y \
                    - parametric_y(Xi.v*Xi.t0, Xi.k*Xi.s0, Xim1.theta) \
                    - parametric_y(Xi.v*Xi.t1, Xi.k*Xi.s1, Xim1.theta + Xi.k*Xi.s0*Xi.v*Xi.t0) \
                    - parametric_y(Xi.v*Xi.t2, Xi.k*Xi.s2, Xim1.theta + Xi.k*Xi.s0*Xi.v*Xi.t0 + Xi.k*Xi.s1*Xi.v*Xi.t1),
                0
            )

            self.set_equality_constraint(
                "th"+idx,
                Xi.theta - Xim1.theta - Xi.k*Xi.s0*Xi.v*Xi.t0 - Xi.k*Xi.s1*Xi.v*Xi.t1 - Xi.k*Xi.s2*Xi.v*Xi.t2,
                0
            )

            time_sum += (Xi.t0 + Xi.t1 + Xi.t2)

        self.set_constraint("time_sum", time_sum, 0, self.t_max)

        # Forward calculate trajectory at finer intervals
        # Reason 1: Obstacle Avoidance.
        # Reason 2: To get a finer trajectory output out of casadi during solving time.
        for i in range(1, self.number_of_states):
            # Similar Setup as before
            state_idx = str(i)
            Xi = self.states[i]; Xim1 = self.states[i-1]

            # First Segment Trajectory
            Xim1_fine = self.trajectory(
                state = Xim1,
                v = Xi.v,
                k = Xi.k,
                n = self.granularity,
                s = Xi.s0,
                t = Xi.t0
            )

            # Second Segment Trajectory
            Xim1_fine.extend(self.trajectory(
                # Start from last state of previously generated trajectory
                state = Xim1_fine[-1],
                v = Xi.v,
                k = Xi.k,
                n = self.granularity,
                s = Xi.s1,
                t = Xi.t1
            ))

            # Third Segment Trajectory
            Xim1_fine.extend(self.trajectory(
                state = Xim1_fine[-1],
                v = Xi.v,
                k = Xi.k,
                n = self.granularity,
                s = Xi.s2,
                t = Xi.t2
            ))

            # Ensure to add each state into constraint dict to extract values during inference
            traj_id = 0
            for intermediate_state in Xim1_fine:
                traj_idx = state_idx + str(traj_id)
                self.set_constraint(
                    "intermediate_x"+traj_idx,
                    intermediate_state.x
                )
                self.set_constraint(
                    "intermediate_y"+traj_idx,
                    intermediate_state.y
                )
                
                # Check for any obstacle avoidance here
                obs_id = 0
                for obstacle in self.obstacles:
                    # Distance from center of the circle
                    check_obstacle = lambda x, y : power(obstacle[0] - x, 2) + power(obstacle[1] - y, 2) - power(obstacle[2]+obstacle[3], 2)
                    idx = traj_idx + str(obs_id)
                    self.set_constraint(
                        "check_obs"+idx,
                        check_obstacle(intermediate_state.x, intermediate_state.y),
                        0
                    )
                    obs_id += 1
                traj_id += 1

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
            dth = state.theta + 2*arc_phase*dt

            # All redundant values are set to 0
            trajectory.append(State(
                dx, dy, dth, 0, 0, 0, 0, 0, 0, 0, 0
            ))

        return trajectory

    def objective(self, *args, **kwargs):
        # Distance between two states : Euclidean
        # TODO: Is this really how I should check path length?
        def length(stateA : State, stateB : State):
            return power((stateA.x - stateB.x), 2) + power((stateA.y - stateB.y), 2)

        path_length = 0
        for i in range(1, self.number_of_states):
            path_length += length(self.states[i], self.states[i-1])

        return path_length

    def initial_guess(self, *args, **kwargs):
        # For initial guess, set all variables to random values.
        # TODO: Improve the initial guess system for faster convergence
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
    final = Coords2D(10.0, 10.0, 0.0)

    # X, Y, R -> Circular Obstacles
    obstacles = [
        [5.0, 5.0, 2.0, 1.0]
    ]

    lm = LimoBot()
    dc = DubinCar(lm)

    ex = Executor(dc)
    ex.prep(init=init, final=final, obstacles=obstacles)
    solution, solver = ex.solve()
    
    decision_variables = solution["x"]
    constraints = solution["g"]

    import matplotlib.pyplot as plt
    import matplotlib.patches as patches

    intermediate_values = dc.get_constraint_idx_by_pattern("intermediate")

    note_x = []; note_y = []

    for i in range(0, len(intermediate_values), 2):
        x_idx = intermediate_values[i]; y_idx = intermediate_values[i+1]
        note_x.append(float(constraints[x_idx])); note_y.append(float(constraints[y_idx]))

    fig, ax = plt.subplots()

    for obstacle in obstacles:
        ax.add_patch(patches.Circle(
            (obstacle[0], obstacle[1]), obstacle[2],
            linewidth=1, edgecolor='r', facecolor='none'
        ))

        ax.add_patch(patches.Circle(
            (obstacle[0], obstacle[1]), obstacle[2]+obstacle[3],
            linewidth=1, edgecolor='r', facecolor='none'
        ))

    ax.plot(note_x, note_y)
    plt.show()

    # Generating Control Sequences from State Matrices
    # Control Sequences should be {(v1, w1), ..., (vn, wn)}, S.T. the dt between each control sequence is of a fixed time period.