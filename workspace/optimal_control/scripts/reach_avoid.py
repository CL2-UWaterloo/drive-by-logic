#!/usr/bin/python3

# Solver
from optimal_control.dubins_solver import *

# STL
from optimal_control.robustness import *

# Utils
from optimal_control.executor import *
from optimal_control.environment import *
from optimal_control.plotter import *

def reach_avoid(all_signals : list[list[Waypoint]], finals, finish_by, obstacles, dt):
    if type(finals) != list:
        finals = [finals]

    reach_signals = []
    avoid_signals = []
    for signals in all_signals:
        for final in finals:
            agent_reach_signals = []
            for signal in signals:
                state = get_state_from_waypoint(signal)
                agent_reach_signals.append(final.inside_region(state))
            reach_signals.append(
                eventually(
                    agent_reach_signals, dt=dt, ub=finish_by
                )
            )

        agent_avoid_signals = []
        for signal in signals:
            state = get_state_from_waypoint(signal)
            for obstacle in obstacles:
                agent_avoid_signals.append(obstacle.outside_region(state))

        avoid_signals.append(
            always(
                agent_avoid_signals, dt=dt
            )
        )

    for k1 in range(len(all_signals)):
        agent_avoid_signals = []
        for k2 in range(len(all_signals)):
            if k1 != k2:
                for i in range(len(all_signals[k1])):
                    k1way = all_signals[k1][i]
                    k2way = all_signals[k2][i]
                    # dx + dy - e
                    dxpdyme = power(k1way.x - k2way.x, 2) + power(k1way.y - k2way.y, 2) - 2.0
                    agent_avoid_signals.append(dxpdyme)

                avoid_signals.append(
                    always(
                        agent_avoid_signals, dt=dt
                    )
                )

    return smooth_min([*reach_signals, *avoid_signals])

if __name__ == "__main__":
    import random

    perturb_x = round(random.uniform(-0.1, 0.1), 3)
    perturb_y = round(random.uniform(-0.1, 0.1), 3)

    robot = LimoBot()

    print("perturb x: ", perturb_x)
    print("perturb y: ", perturb_y)

    unperturbed_states = [
        State(0.0, 0.0, 0.0, 0.0),
        State(5.0, 0.0, 0.0, 0.0),
        State(0.0, 5.0, 0.0, 0.0),
        State(10.0, 0.0, 0.0, 0.0),
        State(0.0, 10.0, 0.0, 0.0),
        State(2.5, 2.5, 0.0, 0.0)
    ]

    initial_states = unperturbed_states
    # initial_states = [
    #     State(0.0 + perturb_x, 0.0 + perturb_y, 0.0, 0.0),
    #     State(5.0 + perturb_y, 0.0 + perturb_x, 0.0, 0.0),
    #     State(0.0 + perturb_x, 5.0 + perturb_y, 0.0, 0.0),
    #     State(10.0 + perturb_x, 0.0 + perturb_y, 0.0, 0.0),
    #     State(0.0 + perturb_x, 10.0 + perturb_y, 0.0, 0.0),
    #     State(2.5 + perturb_x, 2.5 + perturb_y, 0.0, 0.0)
    # ]

    environ = simple_environment_1()
    finish_by = 30 #s
    number_of_waypoints = 3
    granularity = 25
    number_of_agents = len(initial_states)

    # Compute dt
    dt = finish_by/((number_of_waypoints - 1)*granularity)

    cfhandle = prep(
        initial_states,
        DubinsPlanner(robot, PlannerMode.ClosedForm),
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
        environ.final, finish_by, environ.obstacles, dt
    )

    cfhandle.problem.cost = -1*robustness
    cfhandle.problem.set_constraint("robust_bool", robustness, lbg = 1.0)

    cfsolution, cfsolver = cfhandle.solve()

    print("Optimization Conclusion: ", cfsolver.stats()["success"])
    print("Robustness Value: ", '{:f}'.format(float(cfsolution["g"][-1])))

    # Analyze the solution. Given the decision variables, feed the inputs to an actual model to get the final trajectory.
    decision_variables = cfsolution["x"]; trajectories = []
    # How do we split the decision variables? Split it by the number of agents first.
    for k in range(number_of_agents):
        agent_decisions = decision_variables[
            k*(number_of_waypoints-1)*Waypoint.total_size :
            (k+1)*(number_of_waypoints-1)*Waypoint.total_size
        ]

        # Next get the waypoints
        agent_waypoints = []
        for i in range(number_of_waypoints-1):
            decision = agent_decisions[i*Waypoint.total_size : (i+1)*Waypoint.total_size]
            agent_waypoints.append(Waypoint.from_vector(decision))

        # Now forward simulate from the waypoint and note the trajectory
        agent_trajectory = []
        dx = initial_states[k].x; dy = initial_states[k].y; dth = initial_states[k].theta; dv = initial_states[k].v
        for waypoint in agent_waypoints:
            # Helps rationalize some of the decisions
            steer_angle = float(atan(waypoint.k*robot.wheel_base))

            # Update Dynamic Model
            for j in range(granularity):
                dv += waypoint.a*dt
                dx += dv*cos(dth)*dt; dy += dv*sin(dth)*dt
                dth += dv*(tan(steer_angle)/robot.wheel_base)*dt

                agent_trajectory.append(cast_state(State(dx, dy, dth, dv), float))
        trajectories.append(agent_trajectory)

    plotter = Plotter(environ.get_plot_options(), trajectories, environ.get_environment())

    # # _ = 0
    # # for state in unperturbed_states:
    # #     plotter.axis.plot(state.x, state.y, marker="o", markersize=4.0, color=plotter.colors[_])
    # #     _ += 1

    plotter.axis.legend(handles=[
        lines.Line2D([], [], linestyle="-", color=plotter.colors[i], label="r"+str(i+1))
    for i in range(len(plotter.colors))])


    plotter.show_animation(record=True)