#!/usr/bin/python3

# Solver
from optimal_control.dubins_solver import *

# STL
from optimal_control.robustness import *

# Utils
from optimal_control.executor import *
from optimal_control.environment import *
from optimal_control.plotter import *

def reach_avoid(all_signals : list[list[Waypoint]], finals, finish_by, obstacles):
    agent_obstacle_signals = []; agent_goal_signals = []
    for signals in all_signals:
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
                    always(
                        (obstacle.outside_region(state), signal), lb=0, ub=finish_by
                    )
                )

        agent_goal_signals.extend(goal_signals)
        agent_obstacle_signals.extend(obstacle_signals)

    agent_avoid_signals = []
    for k1 in range(len(all_signals)):
        for k2 in range(len(all_signals)):
            if (k1 == k2):
                continue
            for k1dx in range(len(all_signals[k1])):
                k1way = all_signals[k1][k1dx]
                for k2dx in range(len(all_signals[k2])):
                    k2way = all_signals[k2][k2dx]
                    
                    # Calculate how far away in time
                    time_condition = power(k1way.t - k2way.t, 2)

                    # Calculate how far away in distance
                    dxpdy = power(k1way.x - k2way.x, 2) + power(k1way.y - k2way.y, 2)

                    # If time is close enough, then predicate, or else 1e+16
                    stay_away = if_else(time_condition < 0.1, dxpdy - 1, INF)
                    agent_avoid_signals.append(stay_away)

    agent_sequence_signals = []


    return -logsumexp(-vertcat(
        *agent_task_signals,
        *agent_avoid_signals
    ), MARGIN)

if __name__ == "__main__":
    import random

    perturb_x = round(random.uniform(-0.05, 0.05), 2)
    perturb_y = round(random.uniform(-0.05, 0.05), 2)

    robot = LimoBot()

    print("perturb x: ", perturb_x)
    print("perturb y: ", perturb_y)

    initial_states = [
        State(0.0 + perturb_x, 0.0 + perturb_y, 0.0, 0.0),
        State(5.0, 0.0, 0.0, 0.0)
    ]

    environ = multi_simple_environment_1()
    finish_by = 25 #s
    number_of_waypoints = 3
    granularity = 25
    number_of_agents = 2

    cfhandle = prep(
        initial_states,
        DubinsPlanner(robot, PlannerMode.ForwardSim),
        finish_by,
        number_of_waypoints,
        granularity,
        number_of_agents
    )

    agent_wise_signals = []
    for k in range(number_of_agents):
        agent_wise_signals.append(
            cfhandle.problem.signals[
                k*((number_of_waypoints-1)*granularity + 1):
                (k+1)*((number_of_waypoints-1)*granularity + 1)
            ]
        )

    robustness = reach_avoid(
        agent_wise_signals,
        environ.final, finish_by, environ.obstacles
    )

    cfhandle.problem.set_equality_constraint("robust_bool", robustness, 0.1)
    cfsolution, cfsolver = cfhandle.solve()

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
    #     dx = initial_states[k].x; dy = initial_states[k].y; dth = initial_states[k].theta; dv = initial_states[k].v
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