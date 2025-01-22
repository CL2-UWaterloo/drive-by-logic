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

        # Set all information of the robot required as class variables.
        self.prep_robot_information(robot)

        # To decide whether the opt uses baseline formulation or this
        self.planner_mode = planner_mode

    def prep_robot_information(self, robot):
        self.minimum_turning_radius = robot.get_minimum_turning_radius()
        self.wheel_base = robot.get_wheel_base()
        self.max_linear_velocity = robot.get_max_linear_velocity()
        self.max_acceleration = robot.get_max_acceleration()
        self.max_steering_angle = robot.get_max_steering_angle()

        self.k = 1/self.minimum_turning_radius
        self.v = self.max_linear_velocity

    def prep_problem(self, *args, **kwargs):
        # Important information on constructing planner variables
        self.number_of_agents = kwargs["number_of_agents"]
        self.number_of_waypoints = kwargs["number_of_waypoints"]

        self.waypoints : list[Waypoint] = []

        # For all the agents, construct waypoints.
        for j in range(self.number_of_agents):
            jdx = str(j)
            for i in range(self.number_of_waypoints):
                idx = jdx + str(i)

                waypoint = Waypoint.construct(idx)

                if i != 0:
                    self.set_variable("X"+idx, waypoint.X)
                    self.set_variable("U"+idx, waypoint.U)
                    self.set_variable("t"+idx, waypoint.t)

                self.waypoints.append(waypoint)

    def prep_constraints(self, *args, **kwargs):
        # Granularity and Horizon Length
        self.granularity = kwargs["granularity"]
        self.hrz = kwargs["hrz"]

        # Get initial states of the agents
        self.initial_states : list[State] = kwargs["init"]

        # Time elapsed for each of the robots along their trajectory
        self.time_elapsed = []
        for k in range(self.number_of_agents):
            kdx = str(k)

            # First waypoint of kth robot
            X0 = self.waypoints[k*self.number_of_waypoints]

            # Boundary Conditions. Should this really be an equality constraint?
            # By definition, this is supposed to be a parameter.
            self.set_parameter("x"+kdx+"0", X0.x, self.initial_states[k].x)
            self.set_parameter("y"+kdx+"0", X0.y, self.initial_states[k].y)
            self.set_parameter("th"+kdx+"0", X0.theta, self.initial_states[k].theta)  
            self.set_parameter("v"+kdx+"0", X0.v, 0.0)
            self.set_parameter("t"+kdx+"0", X0.t, 0.0)
            self.set_parameter("a"+kdx+"0", X0.a, 0.0)
            self.set_parameter("k"+kdx+"0", X0.k, 0.0)

            self.signals.append(X0); self.time_elapsed.append(0)
            for i in range(k*self.number_of_waypoints + 1, (k+1)*self.number_of_waypoints):
                idx = kdx + str(i)

                # Current state and previous state (Xim1 = Xi-1)
                Xi = self.waypoints[i]; Xim1 = self.waypoints[i-1]

                # Acc Constraint
                self.set_constraint("a"+idx, Xi.a, -self.max_acceleration, self.max_acceleration)

                # Curvature Constraint
                self.set_constraint("k"+idx, Xi.k, -self.k, self.k)

                # Time Constraint
                self.set_constraint("t"+idx, Xi.t, 0.0)

                dx = Xim1.x; dy = Xim1.y; dth = Xim1.theta; dv = Xim1.v
                dt = Xi.t/self.granularity
                for j in range(self.granularity):
                    jdx = idx + str(j)

                    dv += Xi.a*dt

                    if jdx != str((k+1)*self.number_of_waypoints-1) + str(self.granularity-1):
                        self.set_constraint("v"+jdx, dv, 0.0, self.v)
                    else:
                        self.set_equality_constraint("v"+jdx, dv, 0.0)

                    if self.planner_mode == PlannerMode.ForwardSim:
                        dx += dv*cos(dth)*dt
                        dy += dv*sin(dth)*dt
                        dth += Xi.k*dv*dt
                    
                    elif self.planner_mode == PlannerMode.ClosedForm:
                        px, py, pth = parametric_arc(Xi.k, dv, dth, dt)
                        dx += px; dy += py; dth += pth

                    self.time_elapsed[k] += dt

                    self.signals.append(
                        Waypoint.from_list(
                            [dx, dy, dth, dv, Xi.a, Xi.k, self.time_elapsed[k]]
                        )
                    )

                # Continuity Constraints
                diff = Xi.X - vertcat(dx, dy, dth, dv)
                self.set_equality_constraint("Xeq"+idx, diff, GenDM_zeros(4, 1))
            
            self.set_equality_constraint("time_elapsed"+kdx, self.time_elapsed[k], self.hrz)

    def solve(self, *args, **kwargs):
        constraints = self.get_constraints()

        nlp = {
            "x" : vertcat(*self.get_variables()),
            "f": self.objective(),
            "g": vertcat(*constraints[0]),
            "p": vertcat(*self.get_parameters_symbols())
        }

        opts = {
            "ipopt": {
                "hessian_approximation": "limited-memory",
                "max_iter": kwargs["iterations"],
                
                # "tol": 1e-2,
                
                # "print_timing_statistics": "yes",
                "linear_solver": "ma27",

                # "tiny_step_tol": float(MARGIN),

                # "warm_start_init_point": "yes",
                # "warm_start_bound_push": 1e-9,
                # "warm_start_bound_frac": 1e-9,
                # "warm_start_slack_bound_frac": 1e-9,
                # "warm_start_slack_bound_push": 1e-9,
                # "warm_start_mult_bound_push": 1e-9,

                # "mu_init": 1e-9,
            },
            "print_time": True,
            "expand": True
        }

        sol = nlpsol("Solver", "ipopt", nlp, opts)

        if "warm_start" in kwargs.keys():
            solution = sol(
                # x0 = kwargs["warm_start"]["x"],
                lbg = vertcat(*constraints[1]),
                ubg = vertcat(*constraints[2]),
                p = vertcat(*self.get_parameters_values())
            )
        else:
            solution = sol(
                lbg = vertcat(*constraints[1]),
                ubg = vertcat(*constraints[2]),
                p = vertcat(*self.get_parameters_values())
            )

        return solution, sol