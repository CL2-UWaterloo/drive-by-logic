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

    # Agent 1
    reach1_signals = []
    not_reach1_signals = []
    for signal in agent_signals[0]:
        dx = power(signal.x - final[0][0], 2)
        dy = power(signal.y - final[0][1], 2)
        reach1_signals.append(
            (-(dx + dy - 4), signal)
        )
        not_reach1_signals.append(
            ((dx + dy - 4), signal)
        )
    part1 = eventually(*reach1_signals, lb=0, ub=t_max)

    reach2_signals = []
    not_reach2_signals = []
    for signal in agent_signals[1]:
        dx = power(signal.x - final[0][0], 2)
        dy = power(signal.y - final[0][1], 2)
        reach2_signals.append(
            (-(dx + dy - 4), signal)
        )
        not_reach2_signals.append(
            ((dx + dy - 4), signal)
        )

    part2 = eventually(*reach2_signals, lb=0, ub=t_max)

    part3 = until(first=not_reach1_signals, second=reach2_signals, lb=t_max/4, ub=t_max/2)

    pvp_signals = []
    for idx in range(len(agent_signals[0])):
        dx = power(agent_signals[0][idx].x - agent_signals[1][idx].x, 2)
        dy = power(agent_signals[0][idx].y - agent_signals[1][idx].y, 2)
        pvp_signals.append(
            ((dx + dy - 2), signal)
        )
    part4 = always(*pvp_signals, lb=0, ub=t_max)

    part5 = []
    for obstacle in planner.obstacles:
        for signal in agent_signals[0]:
            dx = power(signal.x - obstacle[0], 2)
            dy = power(signal.y - obstacle[1], 2)
            part5.append(
                always(
                    (dx + dy - power(obstacle[2], 2), signal), 
                    lb=0, ub=t_max
                )
            )

        for signal in agent_signals[1]:
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

    # planner.set_constraint("robustness", planner.cost, lbg=3.5, ubg=DM_inf())
    planner.cost *= -1

if __name__ == "__main__":
    init = [
        State(0.0, 0.0, 0.0),
        State(10.0, 0.0, 0.0)
    ]

    final = [
        [10.0, 10.0, 0.0],
        # [0.0, 10.0, 0.0]
    ]

    obstacles = [
        [5.0, 5.0, 2.0]
    ]

    lm = LimoBot()
    pf = PlannerFactory(lm, DubinsPlanner, PlannerMode.ClosedForm)
    solution, solver = plan(multi_reach_avoid, init, final, obstacles, pf, 85, 4, 50, 2)
    print(solver.stats()["success"], solver.stats()["t_proc_total"])
    dv = solution["x"]; cs = solution["g"]; tt = solver.stats()["t_proc_total"]

    waypoint_type = type(pf.planner.waypoints[-1])
    state_shape = pf.planner.waypoints[-1].X.shape[0]
    control_shape = pf.planner.waypoints[-1].U.shape[0]
    time_shape = pf.planner.waypoints[-1].t.shape[0]
    waypoint_shape = state_shape + control_shape + time_shape

    waypoint_logs = []
    for i in range(0, dv.shape[0], waypoint_shape):
        waypoint_logs.append(
            waypoint_type(
                dv[i : i + state_shape],
                dv[i + state_shape : i + state_shape + control_shape],
                dv[i + state_shape + control_shape : i + waypoint_shape]
            )
        )

    pp = ProcessPlotter(init, pf.planner, waypoint_logs)
    pp.ax.set_title("Closed Form, Time Taken: " + str(tt))
    pp()