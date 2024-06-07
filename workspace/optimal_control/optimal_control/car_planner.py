#!/usr/bin/python3

from dataclasses import asdict
from casadi import *

from optimal_control.utils import *
from optimal_control.problem import Problem

class CarPlanner(Problem):

    def __init__(self, robot : CarLikeRobot):
        super(CarPlanner, self).__init__()
        self.robot = robot
        self.prep_robot_information()

        # Formulation Parameters
        self.number_of_states = 6
        self.granularity = 10
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
        self.states : list[State] = []

        for i in range(self.number_of_states):
            idx = str(i)

            # Make State
            state = State(
                SX.sym("x"+idx), SX.sym("y"+idx), SX.sym("theta"+idx),
                SX.sym("s"+idx), SX.sym("t"+idx), SX.sym("v"+idx),
                SX.sym("k"+idx)
            )

            # Declare Decision Variables
            for _, value in asdict(state).items():
                self.set_variable(value.name(), value)

            self.states.append(state)

    def prep_constraints(self, *args, **kwargs):
        # Get Initial, Final states, and Obstacles from arguments.
        self.initial_state : State = kwargs["init"]
        self.final_state : State = kwargs["final"]
        self.obstacles = kwargs["obstacles"]

        # The first state is X0 and the last is Xn
        X0 = self.states[0]; Xn = self.states[-1]

        # Boundary Conditions
        self.set_equality_constraint("x0", X0.x, self.initial_state.x)
        self.set_equality_constraint("y0", X0.y, self.initial_state.y)
        self.set_equality_constraint("th0", X0.theta, self.initial_state.theta)

        # Final Boundary Conditions are subject to change based on problem description
        self.set_equality_constraint("xn", Xn.x, self.final_state.x)
        self.set_equality_constraint("yn", Xn.y, self.final_state.y)
        self.set_equality_constraint("thn", Xn.theta, self.final_state.theta)

        # Iterate through all states to establish constraints
        time_sum = 0
        for i in range(1, self.number_of_states):
            idx = str(i)

            # Current state and previous state (Xim1 = Xi-1)
            Xi = self.states[i]; Xim1 = self.states[i-1]

            # Velocity Constraint
            self.set_constraint("v"+idx, Xi.v, 0, self.v)

            # Curvature Constraint
            self.set_constraint("k"+idx, Xi.k, -self.k, self.k)

            # Time Constraint
            self.set_constraint("t"+idx, Xi.t, 0)

            # Start with establishing the equality constraint between final and initial positions
            dx = Xim1.x; dy = Xim1.y; dth = Xim1.theta
            dk = Xim1.k; dt = Xi.t*(1/self.granularity)
            for j in range(self.granularity + 1):
                jdx = str(j) + idx

                dk += Xi.s*dt
                dx += Xi.v*cos(dth)*dt
                dy += Xi.v*sin(dth)*dt
                dth += dk*Xi.v*dt

                # Kinematic Limits
                self.set_constraint("k"+jdx, dk, -self.k, self.k)

                # For Plotting
                self.set_constraint("intermediate_x"+jdx, dx)
                self.set_constraint("intermediate_y"+jdx, dy)
                self.set_constraint("intermediate_th"+jdx, dth)

                # Check for obstacles
                obs_id = 0
                for obstacle in self.obstacles:
                    odx = str(obs_id) + jdx
                    # Distance from center of the circle
                    check_obstacle = lambda x, y : \
                        power(obstacle[0] - x, 2) + power(obstacle[1] - y, 2) - power(obstacle[2]+obstacle[3], 2)
                    self.set_constraint("check_obs"+odx, check_obstacle(dx, dy), 0)
                    obs_id += 1

            # G2 Continuity Constraints
            self.set_equality_constraint("x"+idx, Xi.x - dx, 0)
            self.set_equality_constraint("y"+idx, Xi.y - dy, 0)
            self.set_equality_constraint("theta"+idx, Xi.theta - dth, 0)
            self.set_equality_constraint("k"+idx, Xi.k - dk, 0)

            time_sum += Xi.t

        self.set_constraint("time_sum", time_sum, 0, self.t_max)

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
        r = sqrt(
            power((self.final_state.x - self.initial_state.x), 2) \
            + power((self.final_state.y - self.initial_state.y), 2)
        )

        k = 1/r
        x = self.initial_state.x; y = self.initial_state.y; theta = self.initial_state.theta
        dt = self.t_max/self.number_of_states
        guess_variables = []
        for i in range(self.number_of_states):
            guess_variables.append(x)
            guess_variables.append(y)
            guess_variables.append(theta)
            guess_variables.append(0) # Zero change in curvature
            guess_variables.append(dt) # Equal time for all states
            guess_variables.append(self.v) # Unit Speed along the path
            guess_variables.append(k)

            x += self.v*cos(theta)*dt
            y += self.v*sin(theta)*dt
            theta += self.v*k*dt
            theta = normalize_angle(theta)

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