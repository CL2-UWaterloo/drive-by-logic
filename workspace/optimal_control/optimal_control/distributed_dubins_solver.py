#!/usr/bin/python3

from dataclasses import asdict
from copy import deepcopy
from time import time

from casadi import *

from optimal_control.utils import *
from optimal_control.problem import Problem
from optimal_control.waypoint import Waypoint

class DistributedDubinsPlanner(Problem):

    def __init__(self,
            robot : CarLikeRobot,
            planner_mode : PlannerMode
        ):
        super(DistributedDubinsPlanner, self).__init__()
        self.robot = robot
        self.prep_robot_information()
        self.planner_mode = planner_mode

        self.name = "Distributed Dubins Planner"

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

            self.signals.append(X0)

            self.time_elapsed.append(0)
            dx = X0.x; dy = X0.y; dth = X0.theta
            for i in range(k*self.number_of_waypoints + 1, (k+1)*self.number_of_waypoints):
                idx = kdx + str(i)

                # Current state and previous state (Xim1 = Xi-1)
                Xi = self.waypoints[i]; Xim1 = self.waypoints[i-1]

                # dx = Xim1.x; dy = Xim1.y; dth = Xim1.theta
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

                    self.time_elapsed[k] += dt
                    self.signals.append(
                        Waypoint(
                            vertcat(dx, dy, dth),
                            vertcat(Xi.v, Xi.k),
                            self.time_elapsed[k]
                        )
                    )

    def solve(self, *args, **kwargs):
        # Init Agents and states
        agents = []
        for state in self.initial_states:
            agents.append(
                [Waypoint(
                    X=vertcat(state.x, state.y, state.theta),
                    U=vertcat(0.0, 0.0),
                    t=0.0
                )]*self.number_of_waypoints
            )

        def get_waypoints():
            y = vertcat()
            for agent in agents:
                for waypoint in agent:
                    y = vertcat(y, waypoint.matrix_form())
            return y

        def get_waypoint(i, agents):
            y = vertcat()
            for waypoint in agents[i]:
                y = vertcat(y, waypoint.matrix_form())
            return y

        def set_waypoint(i, vector):
            for idx in range(self.number_of_waypoints):
                agents[i][idx] = Waypoint(
                    vector[idx*6 : idx*6 + 3],
                    vector[idx*6 + 3: idx*6 + 5],
                    vector[idx*6 + 5 : idx*6 + 6]
                )

        def get_sym_waypoint(i):
            agent_vars = self.waypoints[i*self.number_of_waypoints:(i+1)*self.number_of_waypoints]
            return vertcat(*[waypoint.matrix_form() for waypoint in agent_vars])

        # To evaluate robustness
        eval_robustness = Function(
            "robustness_eval",
            [vertcat(*self.get_variables())], [self.cost]
        )

        robustness_input = np.array(get_waypoints())
        robustness_output = np.zeros((1, 1))
        eval_robustness_buffer = eval_robustness.buffer()
        eval_robustness_buffer[0].set_arg(0, memoryview(robustness_input))
        eval_robustness_buffer[0].set_res(0, memoryview(robustness_output))

        gradients = []; sym_waypoints = []; grad_outputs = []
        for agent in range(self.number_of_agents):
            sym_gradient = gradient(self.cost, get_sym_waypoint(agent))
            function = Function(
                "grad_robust_eval"+str(agent),
                [vertcat(*self.get_variables())],
                [sym_gradient]
            )
            gradients.append(function.buffer())

            grad_outputs.append(np.zeros(get_sym_waypoint(agent).shape))

            gradients[-1][0].set_arg(0, memoryview(robustness_input))
            gradients[-1][0].set_res(0, memoryview(grad_outputs[-1]))

            sym_waypoints.append(get_sym_waypoint(agent))

        max_iters = kwargs["iterations"]; iters = 0; alpha = 1e-2
        robustness = [float(eval_robustness(get_waypoints()))]; selected_agent = 0
        best_robustness = robustness[-1]; best_solution = deepcopy(agents)
        buffer = 0; gradient_buffer = 0; robustness_buffer = 0
        while iters < max_iters:
            if selected_agent == 0:
                selected_agent = 1
            else:
                selected_agent = 0

            # Get current states of the agents
            waypoints = get_waypoints(); robustness_input = np.array(waypoints)
            print("Previous Robustness: ", robustness[-1])
            print("Selected Agent: ", selected_agent)

            # If the change in robustness is not large enough, quit.
            robustness_buffer_start = time()
            eval_robustness_buffer[1]()
            robustness_buffer += time() - robustness_buffer_start
            robustness.append(robustness_output[0][0])
            iters += 1
            print("Iteration", iters)
            print("Current Robustness: ", robustness[-1])

            if robustness[-1] > best_robustness:
                best_robustness = robustness[-1]
                best_solution = deepcopy(agents)

            if robustness[iters] > 3.5:
                break

            init = self.initial_states[selected_agent]
            sym_waypoint = sym_waypoints[selected_agent]
            waypoint = get_waypoint(selected_agent, agents)

            gradient_buffer_start = time()
            gradients[selected_agent][1]()
            gradient_buffer += time() - gradient_buffer_start
            solver = qpsol("Solver", "qpoases",
                {
                    "x" : sym_waypoint,
                    "f" : -dot(grad_outputs[selected_agent], (sym_waypoint-waypoint)) + \
                        alpha*0.5*power(norm_2(sym_waypoint - waypoint), 2),
                    "g" : self.time_elapsed[selected_agent],
                }, {
                    "printLevel" : "none"
                }
            )
            buffer_start = time()
            solution = solver(
                lbg = self.t_max,
                ubg = self.t_max,
                lbx = vertcat(
                    [init.x, init.y, init.theta, 0, 0, 0],
                    *([-10, -10, -pi, 0, -self.k, 0]*(self.number_of_waypoints-1))
                ),
                ubx = vertcat(
                    [init.x, init.y, init.theta, 0, 0, 0],
                    *([-10, -10, -pi, self.v, self.k, self.t_max]*(self.number_of_waypoints-1))
                ),
            )
            buffer += (time() - buffer_start)
            set_waypoint(selected_agent, waypoint + (power(0.99, 4.5*iters) + 0.001)*(solution["x"]-waypoint))

        print(robustness_buffer, gradient_buffer)
        return best_solution, buffer, best_robustness, robustness