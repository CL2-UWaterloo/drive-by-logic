#!/usr/bin/python3

from optimal_control.planner_factory import *
from optimal_control.robustness import *
from optimal_control.plotting_tools import *

def multi_reach_avoid(planner, final, t_max):
    agent_signals = []

    for k in range(planner.number_of_agents):
        agent_signals.append(
            planner.signals[
                k*((planner.number_of_waypoints-1)*planner.granularity + 1):
                (k+1)*((planner.number_of_waypoints-1)*planner.granularity + 1)
            ]
        )

    reach_signals = []
    not_reach_signals = []
    for k in range(planner.number_of_agents):
        reach_k_signals = []
        not_reach_k_signals = []
        for signal in agent_signals[k]:
            dx = power(signal.x - final[0][0], 2)
            dy = power(signal.y - final[0][1], 2)
            reach_k_signals.append(
                (-(dx + dy - 4), signal)
            )
            not_reach_k_signals.append(
                ((dx + dy - 4), signal)
            )
        reach_signals.append(reach_k_signals)
        not_reach_signals.append(not_reach_k_signals)

    part1 = eventually(*reach_signals[0], lb=0, ub=t_max)
    part3 = until(first=not_reach_signals[0], second=reach_signals[1], lb=0, ub=t_max/2)

    pvp_signals = []
    for k1 in range(planner.number_of_agents):
        for idx in range(len(agent_signals[k1])):
            for k2 in range(planner.number_of_agents):
                if k1 == k2:
                    continue
                dx = power(agent_signals[k1][idx].x - agent_signals[k2][idx].x, 2)
                dy = power(agent_signals[k1][idx].y - agent_signals[k2][idx].y, 2)
                pvp_signals.append(
                    ((dx + dy - 2), signal)
                )
    part4 = always(*pvp_signals, lb=0, ub=t_max)

    part5 = []
    for obstacle in planner.obstacles:
        for k in range(planner.number_of_agents):
            for signal in agent_signals[k]:
                dx = power(signal.x - obstacle[0], 2)
                dy = power(signal.y - obstacle[1], 2)
                part5.append(
                    always(
                        (dx + dy - power(obstacle[2], 2), signal), 
                        lb=0, ub=t_max
                    )
                )

    planner.cost = mmin(
        vertcat(
            *[part1, part3, part4, *part5]
        )
    )

if __name__ == "__main__":
    init = [
        State(0.0, 0.0, 0.0),
        State(5.0, 0.0, 0.0)
    ]

    final = [
        [10.0, 10.0, 0.0],
        # [0.0, 10.0, 0.0]
    ]

    obstacles = [
        [5.0, 5.0, 2.0]
    ]

    lm = LimoBot()
    pf = PlannerFactory(lm, DistributedDubinsPlanner, PlannerMode.ClosedForm)
    solution, total_time, score, robustness = plan(multi_reach_avoid, init, final, obstacles, pf, 100, 4, 50, 2)
    print(score, total_time)

    waypoint_logs = []
    for agent in solution:
        waypoint_logs.extend(agent)

    pp = ProcessPlotter(init, pf.planner, waypoint_logs)
    pp.ax.set_title("Closed Form, Robustness: " + str(score) + " Time Taken: " + str(total_time))
    pp()

    plot_vals = []
    for rho in robustness:
        if abs(rho) != 1e+16:
            plot_vals.append(rho)

    plt.plot(plot_vals)
    plt.show()