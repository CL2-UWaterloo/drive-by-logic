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
    s : SX | float
    t : SX | float

    # Velocity and Curvature
    # To imitate Dubin's formulation of a car, the velocity and curvature are constant
    v : SX | float
    k : SX | float

    @staticmethod
    def from_list(input : list[float | DM]):
        return State(
            float(input[0]), float(input[1]), float(input[2]),
            float(input[3]), float(input[4]), float(input[5]),
            float(input[6])
        )

class DubinCar(Problem):

    def __init__(self, robot : CarLikeRobot):
        super(DubinCar, self).__init__()
        self.robot = robot
        self.prep_robot_information()

        # Formulation Parameters
        self.number_of_states = 3

        self.granularity = 20

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
                SX.sym("s"+idx), SX.sym("t"+idx),
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

            # Time Selection Constraint
            self.set_constraint(
                "t"+idx, Xi.t, 0
            )

            # Sigma Selection Constraint
            self.set_constraint(
                "s"+idx, Xi.s, -1, 1
            )

            # Start with establishing the equality constraint between final and initial positions
            self.set_equality_constraint(
                "x"+idx,
                Xi.x - Xim1.x \
                    - parametric_x(Xi.v*Xi.t, Xi.k*Xi.s, Xim1.theta),
                0
            )

            self.set_equality_constraint(
                "y"+idx,
                Xi.y - Xim1.y \
                    - parametric_y(Xi.v*Xi.t, Xi.k*Xi.s, Xim1.theta),
                0
            )

            self.set_equality_constraint(
                "th"+idx,
                Xi.theta - Xim1.theta - Xi.k*Xi.s*Xi.v*Xi.t,
                0
            )

            time_sum += Xi.t

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
                s = Xi.s,
                t = Xi.t
            )

            # Ensure to add each state into constraint dict to extract values during inference
            traj_id = 0
            for intermediate_state in Xim1_fine:
                traj_idx = state_idx + str(traj_id)
                
                # Result Collection from solver
                for key, value in asdict(intermediate_state).items():
                    self.set_constraint(
                        "intermediate_"+key+traj_idx,
                        value
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

            trajectory.append(State(
                dx, dy, dth, s, dt, v, k
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
        [5.0, 5.0, 1.0, 1.0]
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

    # Plot trajectory as imagined by the obstacle avoidance check
    intermediate_values = dc.get_constraint_idx_by_pattern("intermediate")
    note_x = []; note_y = []; note_th = []
    for i in range(0, len(intermediate_values), 7):
        x_idx = intermediate_values[i]; y_idx = intermediate_values[i+1]
        note_x.append(float(constraints[x_idx])); note_y.append(float(constraints[y_idx]))

        th_idx = intermediate_values[i+2]
        note_th.append(float(constraints[th_idx]))

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

    for i in range(len(note_x)):
        ax.arrow(
            note_x[i], note_y[i],
            0.1*cos(note_th[i]),
            0.1*sin(note_th[i]),
            head_width=0.1
        )

    plt.show()

    # Generating Control Sequences from States
    previous = None; commands = []; dt = 0.1
    for i in range(0, decision_variables.shape[0], 7):
        # Get all the information
        current = State.from_list(decision_variables[i:i+7])

        # First Iteration
        if previous == None:
            previous = current
            continue

        time_elapsed = 0
        while time_elapsed < current.t:
            commands.append(
                [
                    current.v,
                    current.s*current.k*current.v
                ]
            )
            time_elapsed += dt

    # Stop after executing commands
    commands.append([0.0, 0.0])

    import rclpy
    rclpy.init()

    ros_interface = TwistPublisher(dt, commands)
    while ros_interface.index < len(commands):
        rclpy.spin_once(ros_interface)

    ros_interface.destroy_node()
    rclpy.shutdown()