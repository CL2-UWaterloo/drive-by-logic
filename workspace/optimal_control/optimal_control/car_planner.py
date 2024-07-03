#!/usr/bin/python3

from enum import Enum
from dataclasses import asdict

from casadi import *

from optimal_control.utils import *
from optimal_control.problem import Problem
from optimal_control.waypoint import Waypoint

class PlannerType(Enum):
    ClosedForm = 0
    ForwardSim = 1

class CarPlanner(Problem):

    def __init__(self, 
            robot : CarLikeRobot,
            planner_type : PlannerType,
            number_of_waypoints = 10,
            granularity = 10,
            t_max = 120
        ):
        super(CarPlanner, self).__init__()
        self.robot = robot
        self.prep_robot_information()

        # Formulation Parameters
        self.number_of_waypoints = number_of_waypoints
        self.granularity = granularity
        self.t_max = t_max # s

        self.planner_type = planner_type

    def prep_robot_information(self):
        self.minimum_turning_radius = self.robot.get_minimum_turning_radius()
        self.wheel_base = self.robot.get_wheel_base()
        self.max_linear_velocity = self.robot.get_max_linear_velocity()
        self.max_acceleration = self.robot.get_max_acceleration()
        self.max_steering_angle = self.robot.get_max_steering_angle()

        self.k = 1/self.minimum_turning_radius
        self.v = self.max_linear_velocity*0.4

    def prep_problem(self, *args, **kwargs):
        self.waypoints : list[Waypoint] = []

        for i in range(self.number_of_waypoints):
            idx = str(i)

            # Make Waypoint
            waypoint = Waypoint(
                MX.sym("X"+idx, 6, 1),
                MX.sym("U"+idx, 2, 1),
                MX.sym("t"+idx, 1, 1)
            )

            # Declare Decision Variables
            for _, value in asdict(waypoint).items():
                self.set_variable(value.name(), value)

            self.waypoints.append(waypoint)

    def prep_constraints(self, *args, **kwargs):
        # Get Initial, Final waypoints, and Obstacles from arguments.
        self.initial_state : State = kwargs["init"]
        self.final_state : State = kwargs["final"]
        self.obstacles = kwargs["obstacles"]

        # The first state is X0 and the last is Xn
        X0 = self.waypoints[0]; Xn = self.waypoints[-1]

        # Boundary Conditions
        self.set_equality_constraint("x0", X0.x, self.initial_state.x)
        self.set_equality_constraint("y0", X0.y, self.initial_state.y)
        self.set_equality_constraint("th0", X0.theta, self.initial_state.theta)
        self.set_equality_constraint("t0", X0.t, 0.0)
        self.set_equality_constraint("v0", X0.v, 0.0)
        self.set_equality_constraint("a0", X0.a, 0.0)

        # Final Boundary Conditions are subject to change based on problem description
        self.set_equality_constraint("xn", Xn.x, self.final_state.x)
        self.set_equality_constraint("yn", Xn.y, self.final_state.y)
        self.set_equality_constraint("thn", Xn.theta, self.final_state.theta)

        # Iterate through all states to establish constraints
        for i in range(1, self.number_of_waypoints):
            idx = str(i)

            # Current state and previous state (Xim1 = Xi-1)
            Xi = self.waypoints[i]; Xim1 = self.waypoints[i-1]

            # Velocity Constraint
            self.set_constraint("v"+idx, Xi.v, 0, self.v)

            # Curvature Constraint
            self.set_constraint("k"+idx, Xi.k, -self.k, self.k)

            # Acceleration Constraint
            self.set_constraint("a"+idx, Xi.a, -self.max_acceleration, self.max_acceleration)

            # Time constraint
            self.set_constraint("t"+idx, Xi.t, 0.0, self.t_max/self.number_of_waypoints)

            # Start with establishing the equality constraint between final and initial positions
            dx = Xim1.x; dy = Xim1.y
            dth = Xim1.theta; dk = Xim1.k
            dv = Xim1.v; da = Xim1.a
            dt = Xi.t/self.granularity
            for j in range(self.granularity):
                jdx = str(j) + idx

                # Update step
                da += Xi.j*dt
                dv += da*dt
                dk += Xi.s*dt

                if self.planner_type == PlannerType.ForwardSim:
                    dx += dv*cos(dth)*dt
                    dy += dv*sin(dth)*dt
                    dth += dk*dv*dt

                elif self.planner_type == PlannerType.ClosedForm:
                    px, py, pth = parametric_arc(dk, dv, dth, dt)
                    dx += px
                    dy += py
                    dth += pth

                # Kinematic Limits
                self.set_constraint("command_k"+jdx, dk, -self.k, self.k)
                self.set_constraint("command_v"+jdx, dv, 0, self.v)
                self.set_constraint("command_dt"+jdx, dt, 0)
                self.set_constraint("a"+jdx, da, -self.max_acceleration, self.max_acceleration)

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

            # Continuity Constraints
            self.set_equality_constraint("x"+idx, Xi.x - dx, 0)
            self.set_equality_constraint("y"+idx, Xi.y - dy, 0)
            self.set_equality_constraint("theta"+idx, Xi.theta - dth, 0)
            self.set_equality_constraint("k"+idx, Xi.k - dk, 0)
            self.set_equality_constraint("v"+idx, Xi.v - dv, 0)
            self.set_equality_constraint("a"+idx, Xi.a - da, 0)

    def objective(self, *args, **kwargs):
        return 1

    def initial_guess(self, *args, **kwargs):
        dx = self.final_state.x - self.initial_state.x
        dy = self.final_state.y - self.initial_state.y

        if dx != 0:
            slope = dy/dx
        else:
            slope = DM_inf()

        guess_variables = []; first = True
        r = sqrt(power(dx, 2) + power(dy, 2))*0.5

        if abs(slope) < 1e-2 or slope == DM_inf():
            # A straight line guess
            k = 0
            distance = 2*r
        else:
            # An arc like initial guess
            k = self.v/r
            distance = r*pi

        x = self.initial_state.x; y = self.initial_state.y; theta = self.initial_state.theta
        t = distance/self.v; dt = t/self.number_of_waypoints
        for i in range(self.number_of_waypoints):
            guess_variables.append(x)
            guess_variables.append(y)
            if (dx < 0 and dy < 0):
                guess_variables.append(pi + theta)
            elif (dx < 0):
                guess_variables.append(pi + theta)
            else:
                guess_variables.append(theta)
            guess_variables.append(k)
            guess_variables.append(self.v) # Unit Speed along the path
            guess_variables.append(-self.v/dt)
            guess_variables.append(self.v/(dt**2))
            guess_variables.append(0) # Zero change in curvature
            guess_variables.append(dt) # Equal time for all states

            if not first:
                x += sign(dx)*self.v*cos(theta)*dt
                y += sign(dy)*self.v*sin(theta)*dt
                theta += self.v*k*dt
            else:
                first = False

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
                "max_iter": 1000,
                # "max_cpu_time": 1.0,
                "fast_step_computation": "yes",
                "tol": 1e-2
            },
            "jit": True,
            "compiler": "shell",
            "expand": True,
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