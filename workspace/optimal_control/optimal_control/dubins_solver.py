#!/usr/bin/python3

from dataclasses import asdict

from casadi import *

from optimal_control.utils import *
from optimal_control.problem import Problem
from optimal_control.waypoint import Waypoint

class DubinsPlanner(Problem):

    def __init__(self,
            robot : CarLikeRobot,
            planner_mode : PlannerMode
        ):
        super(DubinsPlanner, self).__init__()
        self.robot = robot
        self.prep_robot_information()
        self.planner_mode = planner_mode

        self.name = "Dubins Planner"

    def prep_robot_information(self):
        self.minimum_turning_radius = self.robot.get_minimum_turning_radius()
        self.wheel_base = self.robot.get_wheel_base()
        self.max_linear_velocity = self.robot.get_max_linear_velocity()
        self.max_acceleration = self.robot.get_max_acceleration()
        self.max_steering_angle = self.robot.get_max_steering_angle()

        self.k = 1/self.minimum_turning_radius
        self.v = self.max_linear_velocity*0.4

    def prep_problem(self, *args, **kwargs):
        self.number_of_agents = kwargs["number_of_agents"]
        self.number_of_waypoints = kwargs["number_of_waypoints"]
        self.granularity = kwargs["granularity"]
        self.t_max = kwargs["t_max"]

        self.waypoints : list[Waypoint] = []

        for j in range(self.number_of_agents):
            jdx = str(j)
            for i in range(self.number_of_waypoints):
                idx = jdx + str(i)
                # Make Waypoint
                waypoint = Waypoint(
                    MX.sym("X"+idx, 3, 1),
                    MX.sym("U"+idx, 2, 1),
                    MX.sym("t"+idx, 1, 1)
                )

                for _, value in asdict(waypoint).items():
                    self.set_variable(value.name(), value)

                self.waypoints.append(waypoint)

    def prep_constraints(self, *args, **kwargs):
        # Get Initial, Obstacles from arguments
        self.initial_states : list[State] = kwargs["init"]
        self.obstacles = kwargs["obstacles"]

        self.time_elapsed = []
        for k in range(self.number_of_agents):
            kdx = str(k)
            # First waypoint of kth robot
            X0 = self.waypoints[k*self.number_of_waypoints]

            # Boundary Conditions
            self.set_equality_constraint("x"+kdx+"0", X0.x, self.initial_states[k].x)
            self.set_equality_constraint("y"+kdx+"0", X0.y, self.initial_states[k].y)
            self.set_equality_constraint("th"+kdx+"0", X0.theta, self.initial_states[k].theta)
            self.set_equality_constraint("t"+kdx+"0", X0.t, 0.0)

            self.signals.append(X0)

            self.time_elapsed.append(0)
            for i in range(k*self.number_of_waypoints + 1, (k+1)*self.number_of_waypoints):
                idx = kdx + str(i)

                # Current state and previous state (Xim1 = Xi-1)
                Xi = self.waypoints[i]; Xim1 = self.waypoints[i-1]

                # Velocity Constraint
                self.set_constraint("v"+idx, Xi.v, 0, self.v)

                # Curvature Constraint
                self.set_constraint("k"+idx, Xi.k, -self.k, self.k)

                # Time Constraint
                self.set_constraint("t"+idx, Xi.t, 0)

                dx = Xim1.x; dy = Xim1.y; dth = Xim1.theta
                dt = Xi.t/self.granularity

                for j in range(self.granularity):
                    if self.planner_mode == PlannerMode.ForwardSim:
                        dx += Xi.v*cos(dth)*dt
                        dy += Xi.v*sin(dth)*dt
                        dth += Xi.k*Xi.v*dt
                    
                    elif self.planner_mode == PlannerMode.ClosedForm:
                        px, py, pth = parametric_arc(Xi.k, Xi.v, dth, dt)
                        dx += px
                        dy += py
                        dth += pth

                    self.signals.append(
                        Waypoint(
                            vertcat(dx, dy, dth),
                            vertcat(Xi.v, Xi.k),
                            self.time_elapsed[k]
                        )
                    )
                    self.time_elapsed[k] += dt

                # Continuity Constraints
                self.set_equality_constraint("x"+idx, Xi.x - dx, 0)
                self.set_equality_constraint("y"+idx, Xi.y - dy, 0)
                self.set_equality_constraint("theta"+idx, Xi.theta - dth, 0)

            self.set_equality_constraint("time_elapsed"+kdx, self.time_elapsed[k], self.t_max)

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
                "max_iter": kwargs["iterations"],
                "tol": 1e-2
            },
            "expand": True
        }

        sol = nlpsol("Solver", "ipopt", nlp, opts)

        if "warm_start" in kwargs.keys():
            return sol(
                x0 = kwargs["warm_start"],
                lbg = vertcat(*constraints[1]),
                ubg = vertcat(*constraints[2])
            ), sol
        else:
            return sol(
                lbg = vertcat(*constraints[1]),
                ubg = vertcat(*constraints[2])
            ), sol