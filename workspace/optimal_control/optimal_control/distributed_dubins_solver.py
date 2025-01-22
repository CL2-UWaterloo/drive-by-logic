#!/usr/bin/python3

from dataclasses import asdict
from copy import deepcopy
from time import time
from itertools import cycle

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
        self.v = self.max_linear_velocity

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

                waypoint = Waypoint.construct(idx)

                self.set_variable("X"+idx, waypoint.X)
                self.set_variable("U"+idx, waypoint.U)
                self.set_variable("t"+idx, waypoint.t)

                # for _, value in asdict(waypoint).items():
                #     self.set_variable(value.name(), value)

                self.waypoints.append(waypoint)

    def prep_constraints(self, *args, **kwargs):
        # Get Initial, Obstacles from arguments
        self.initial_states : list[State] = kwargs["init"]

        self.time_elapsed = []; self.velocities = []
        for k in range(self.number_of_agents):
            kdx = str(k)
            # First waypoint of kth robot
            X0 = self.waypoints[k*self.number_of_waypoints]

            self.signals.append(X0)

            self.time_elapsed.append(0); self.velocities.append([])
            dx = X0.x; dy = X0.y; dth = X0.theta; dv = X0.v
            for i in range(k*self.number_of_waypoints + 1, (k+1)*self.number_of_waypoints):
                idx = kdx + str(i)

                # Current state and previous state (Xim1 = Xi-1)
                Xi = self.waypoints[i]

                # dx = Xim1.x; dy = Xim1.y; dth = Xim1.theta
                dt = Xi.t/self.granularity

                for j in range(self.granularity):
                    dv += Xi.a*dt
                    
                    if self.planner_mode == PlannerMode.ForwardSim:
                        dx += dv*cos(dth)*dt
                        dy += dv*sin(dth)*dt
                        dth += Xi.k*dv*dt
                    
                    elif self.planner_mode == PlannerMode.ClosedForm:
                        px, py, pth = parametric_arc(Xi.k, dv, dth, dt)
                        dx += px; dy += py; dth += pth

                    self.signals.append(
                        Waypoint.from_list(
                            [dx, dy, dth, dv, Xi.a, Xi.k, self.time_elapsed[k]]
                        )
                    )
                    self.velocities[k].append(dv)
                    self.time_elapsed[k] += dt

    def solve(self, *args, **kwargs):
        # Init Agents and states
        agents = []
        for state in self.initial_states:
            agents.append(
                [Waypoint.from_list(
                    [state.x, state.y, state.theta, 0.0, 0.0, 0.0, 0.0],
                    _castype=DM
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
                    vector[idx*7 : idx*7 + 4],
                    vector[idx*7 + 4: idx*7 + 6],
                    vector[idx*7 + 6 : idx*7 + 7]
                )

        def get_sym_waypoint(i):
            agent_vars = self.waypoints[i*self.number_of_waypoints:(i+1)*self.number_of_waypoints]
            return vertcat(*[waypoint.matrix_form() for waypoint in agent_vars])

        def cycles(iterable, n = 2):
            it = cycle(iterable)
            while True:
                yield [next(it) for _ in range(n)]

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

        robustness = [float(eval_robustness(get_waypoints()))]
        best_robustness = robustness[-1]; best_solution = deepcopy(agents)
        buffer = 0; opts = cycles([k for k in range(self.number_of_agents)], n=1)

        min_robustness = 3.5

        max_iters = kwargs["iterations"]; iters = 0; alpha = 5e-3
        while iters < max_iters:
            selected_agents = next(opts); updates = []
            for selected_agent in selected_agents:
                # Get current states of the agents
                waypoints = get_waypoints(); robustness_input = np.array(waypoints)
                print("Previous Robustness: ", robustness[-1])
                print("Selected Agent: ", selected_agent)

                # If the change in robustness is not large enough, quit.
                eval_robustness_buffer[1]()
                robustness.append(robustness_output[0][0])
                iters += 1
                print("Iteration", iters)
                print("Current Robustness: ", robustness[-1])

                if robustness[-1] > best_robustness:
                    best_robustness = robustness[-1]
                    best_solution = deepcopy(agents)

                if robustness[iters] > min_robustness:
                    break

                init = self.initial_states[selected_agent]
                sym_waypoint = sym_waypoints[selected_agent]
                waypoint = get_waypoint(selected_agent, agents)

                gradients[selected_agent][1]()
                solver = qpsol("Solver", "qpoases",
                    {
                        "x" : sym_waypoint,
                        "f" : -dot(grad_outputs[selected_agent], (sym_waypoint-waypoint)) + \
                            alpha*0.5*power(norm_2(sym_waypoint - waypoint), 2),
                        "g" : vertcat(self.time_elapsed[selected_agent], *self.velocities[selected_agent]),
                    }, {
                        "printLevel" : "none"
                    }
                )
                buffer_start = time()
                solution = solver(
                    lbg = vertcat(self.t_max, *[0]*len(self.velocities[selected_agent])),
                    ubg = vertcat(self.t_max, *[self.v]*len(self.velocities[selected_agent])),
                    lbx = vertcat(
                        [init.x, init.y, init.theta, 0, 0, 0, 0],
                        *([-10, -10, -pi, 0, -self.max_acceleration, -self.k, 0]*(self.number_of_waypoints-1))
                    ),
                    ubx = vertcat(
                        [init.x, init.y, init.theta, 0, 0, 0, 0],
                        *([-10, -10, -pi, self.v, self.max_acceleration, self.k, self.t_max]*(self.number_of_waypoints-1))
                    ),
                )
                buffer += (time() - buffer_start)
                updates.append((selected_agent, solution["x"]))

            for update in updates:
                waypoint = get_waypoint(update[0], agents)
                set_waypoint(update[0], waypoint + (power(0.99, iters) + 0.001)*(solution["x"]-waypoint))
            
            if robustness[iters] > min_robustness:
                break

        return best_solution, buffer, best_robustness, robustness