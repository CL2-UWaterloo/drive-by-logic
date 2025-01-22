#!/usr/bin/python3

# Solver
from optimal_control.dubins_solver import *

# STL
from optimal_control.robustness import *

# Utils
from optimal_control.executor import *
from optimal_control.environment import *
from optimal_control.plotter import *

"""
Margin: 1e-6
"""

def visit_all(signals, finals, finish_by, obstacles):
    goal_signals = []
    for final in finals:
        reach_signals = []
        for signal in signals:
            state = get_state_from_waypoint(signal)
            reach_signals.append((final.inside_region(state), signal))
        goal_signals.append(
            eventually(
                *reach_signals, lb=0, ub=finish_by
            )
        )

    obstacle_signals = []
    for signal in signals:
        state = get_state_from_waypoint(signal)
        for obstacle in obstacles:
            obstacle_signals.append(
                always((obstacle.outside_region(state), signal), lb=0, ub=finish_by)
            )

    return -logsumexp(-vertcat(
        *goal_signals,
        *obstacle_signals
    ), MARGIN)

if __name__ == "__main__":
    import random

    perturb_x = round(random.uniform(-0.1, 0.1), 2)
    perturb_y = round(random.uniform(-0.1, 0.1), 2)

    robot = LimoBot()

    print("perturb x: ", perturb_x)
    print("perturb y: ", perturb_y)

    # Problem Information
    initial_state = State(0.0 + perturb_x, 0.0 + perturb_y, 0.0, 0.0)
    environ = multi_simple_environment_1()
    finish_by = 90 #s
    number_of_waypoints = 4
    granularity = 30
    number_of_agents = 1

    cfhandle = prep(
        initial_state,
        DubinsPlanner(robot, PlannerMode.ClosedForm),
        finish_by,
        number_of_waypoints,
        granularity,
        number_of_agents
    )

    robustness = visit_all(
        cfhandle.problem.signals,
        environ.final, finish_by, environ.obstacles
    )

    cfhandle.problem.set_constraint("robust_bool", robustness, lbg=0.2)
    cfsolution, cfsolver = cfhandle.solve()

    print(cfsolution["x"])

    print("Optimization Conclusion: ", cfsolver.stats()["success"])
    print("Robustness Value: ", '{:f}'.format(float(cfsolution["g"][-1])))

    # # Analyze the solution. Given the decision variables, feed the inputs to an actual model to get the final trajectory.
    # decision_variables = cfsolution["x"]; trajectories = []
    # # How do we split the decision variables? Split it by the number of agents first.
    # for k in range(number_of_agents):
    #     agent_decisions = decision_variables[
    #         k*(number_of_waypoints-1)*Waypoint.total_size :
    #         (k+1)*(number_of_waypoints-1)*Waypoint.total_size
    #     ]

    #     # Next get the waypoints
    #     agent_waypoints = []
    #     for i in range(number_of_waypoints-1):
    #         decision = agent_decisions[i*Waypoint.total_size : (i+1)*Waypoint.total_size]
    #         agent_waypoints.append(Waypoint.from_vector(decision))

    #     # Now forward simulate from the waypoint and note the trajectory
    #     agent_trajectory = []
    #     dx = initial_state.x; dy = initial_state.y; dth = initial_state.theta; dv = initial_state.v
    #     for waypoint in agent_waypoints:
    #         # Helps rationalize some of the decisions
    #         steer_angle = float(atan(waypoint.k*robot.wheel_base))

    #         # Update Dynamic Model
    #         dt = waypoint.t/granularity
    #         for j in range(granularity):
    #             dv += waypoint.a*dt
    #             dx += dv*cos(dth)*dt; dy += dv*sin(dth)*dt
    #             dth += dv*(tan(steer_angle)/robot.wheel_base)*dt

    #             agent_trajectory.append(cast_state(State(dx, dy, dth, dv), float))
    #     trajectories.append(agent_trajectory)
    
    # plotter = Plotter(environ.get_plot_options(), trajectories, environ.get_environment())
    # plotter.show_animation()