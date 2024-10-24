#!/usr/bin/python3

from optimal_control.planner_factory import *
from optimal_control.robustness import *
from optimal_control.plotting_tools import ProcessPlotter

def reach_avoid(planner, final, t_max):
    goal_signals = []
    for goal in final:
        reach_signals = []
        for signal in planner.signals:
            dx = power(signal.x - goal[0], 2)
            dy = power(signal.y - goal[1], 2)
            reach_signals.append(
                (-(dx + dy - 4), signal)
            )
        goal_signals.append(
            eventuallyAlways(*reach_signals, lbo=0, ubo=t_max, lbi=0, ubi=t_max)
        )

    obstacle_signals = []
    for obstacle in planner.obstacles:
        for signal in planner.signals:
            dx = power(signal.x - obstacle[0], 2)
            dy = power(signal.y - obstacle[1], 2)
            obstacle_signals.append(
                always(
                    (dx + dy - power(obstacle[2], 2), signal), 
                    lb=0, ub=t_max
                )
            )
    
    planner.cost = mmin(
        vertcat(
            *goal_signals,
            *obstacle_signals
        )
    )

    planner.set_constraint("robustness", planner.cost, ubg=4.0)
    planner.cost *= -1

if __name__ == "__main__":
    init = [State(0.0, 0.0, 0.0)]
    final = [
        [10, 10]
    ]
    obstacles = [
        [5.0, 5.0, 1.0]
    ]

    lm = LimoBot()
    pf = PlannerFactory(lm, DubinsPlanner, PlannerMode.ClosedForm)
    solution, solver = plan(reach_avoid, init, final, obstacles, pf, 85, 4, 50, 1)
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
    pp.ax.set_title("Forward Kinematics, Time Taken: " + str(tt))
    pp()