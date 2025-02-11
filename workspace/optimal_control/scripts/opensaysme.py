#!/usr/bin/python3

# Solver
from optimal_control.dubins_solver import *

# STL
from optimal_control.robustness import *

# Utils
from optimal_control.executor import *
from optimal_control.environment import *
from optimal_control.plotter import *

def simple_puzzle(all_signals : list[list[Waypoint]], finals, finish_by, obstacles, dt):
    avoid_signals = []
    for signals in all_signals:
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

    # Agent1 signals
    reach1_C_signals = []
    not_reach1_B_signals = []
    for signal in all_signals[1]:
        state = get_state_from_waypoint(signal)
        reach1_C_signals.append(
            finals[2].inside_region(state)
        )
        not_reach1_B_signals.append(
            finals[1].outside_region(state)
        )
    reach1_C_formula = eventually(
        reach1_C_signals, dt=dt, ub=finish_by
    )

    # Agent2 Signals
    reach2_A_signals = []
    not_reach2_A_signals = []
    for signal in all_signals[0]:
        state = get_state_from_waypoint(signal)
        reach2_A_signals.append(
            finals[0].inside_region(state)
        )
        not_reach2_A_signals.append(
            finals[0].outside_region(state)
        )
    goto_key_formula = until(not_reach1_B_signals, reach2_A_signals, lb=(finish_by)/4, ub=3*(finish_by/4), dt=dt)

    return smooth_min([goto_key_formula, reach1_C_formula, *avoid_signals])

if __name__ == "__main__":
    import random

    perturb_x = round(random.uniform(-0.1, 0.1), 3)
    perturb_y = round(random.uniform(-0.1, 0.1), 3)

    robot = LimoBot()

    # perturb_x = 0.064
    # perturb_y = -0.063

    print("perturb x: ", perturb_x)
    print("perturb y: ", perturb_y)

    initial_states = [
        State(-2.5, 0.0, 0.0, 0.0),
        State(-2.5, 10.0, 0.0, 0.0)
    ]

    environ = bottleneck()
    finish_by = 40 #s
    number_of_waypoints = 6
    granularity = 40
    number_of_agents = 2

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

    robustness = simple_puzzle(
        agent_wise_signals,
        environ.final, finish_by, environ.obstacles, dt
    )

    for i in range(len(cfhandle.problem.signals)):
        idx = str(i)
        signal = cfhandle.problem.signals[i]
        cfhandle.problem.set_constraint("xlimits"+idx, signal.x, lbg=-3, ubg=15)
        cfhandle.problem.set_constraint("ylimits"+idx, signal.y, lbg=-1.5, ubg=11.5)

    cfhandle.problem.cost = -1*robustness

    cfhandle.problem.set_constraint("robust_bool", robustness, lbg=0.11)
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
    # # for state in initial_states:
    # #     plotter.axis.plot(state.x, state.y, marker="o", markersize=4.0, color=plotter.colors[_])
    # #     _ += 1

    plotter.axis.legend(handles=[
        lines.Line2D([], [], linestyle="-", color=plotter.colors[i], label="r"+str(i+1))
    for i in range(len(initial_states))])

    plotter.show_animation(record=True)