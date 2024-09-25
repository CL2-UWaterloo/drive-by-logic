#!/usr/bin/python3

from optimal_control.stl_planner import *

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.lines as lines
import matplotlib.axes as axes

@dataclass
class Signal:
    x: float
    y: float
    theta: float
    dt: float

def plot_plan(ax : axes.Axes, decision_variables : DM, planner_instance : DubinsPlanner | SmoothPlanner, solver):
    ax.set_title(planner_instance.name + ": " + planner_instance.planner_mode.name + ", time: "  + str(solver.stats()["t_proc_total"]) + ", Status: " + str(solver.stats()["success"]))

    ax.set_xlim(-5, 30)
    ax.set_ylim(-5, 30)

    waypoint_type = type(planner_instance.waypoints[-1])
    state_shape = planner_instance.waypoints[-1].X.shape[0]
    control_shape = planner_instance.waypoints[-1].U.shape[0]
    waypoint_shape = state_shape + control_shape + planner_instance.waypoints[-1].t.shape[0]

    planner_type = type(planner_instance)
    planner_mode = planner_instance.planner_mode

    state_line = lines.Line2D([], [], linestyle=" ", marker="o", color="r", label="Waypoints")
    state_x, state_y = state_line.get_data()
    waypoint_logs = []
    for i in range(0, decision_variables.shape[0], waypoint_shape):
        state_x.append(float(decision_variables[i]))
        state_line.set_xdata(state_x)

        state_y.append(float(decision_variables[i+1]))
        state_line.set_ydata(state_y)

        waypoint_logs.append(
            waypoint_type(
                decision_variables[i : i + state_shape],
                decision_variables[i + state_shape : i + state_shape + control_shape],
                decision_variables[i + state_shape + control_shape : i + waypoint_shape]
            )
        )

    state_line.set_data(state_x, state_y)
    ax.add_line(state_line)

    time_elapsed = 0; signals : list[Signal] = []
    for i in range(1, planner_instance.number_of_waypoints):
        x = waypoint_logs[i-1].x; y = waypoint_logs[i-1].y; rot = waypoint_logs[i-1].theta
        dt = waypoint_logs[i].t/planner_instance.granularity
        
        if planner_type == SmoothPlanner:
            dk = waypoint_logs[i-1].k; dv = waypoint_logs[i-1].v; da = waypoint_logs[i-1].a
        elif planner_type == DubinsPlanner:
            dk = waypoint_logs[i].k; dv = waypoint_logs[i].v
        
        for j in range(planner_instance.granularity):
            if planner_type == SmoothPlanner:
                da += waypoint_logs[i].j*dt
                dv += da*dt
                dk += waypoint_logs[i].s*dt
            
            if planner_mode == PlannerMode.ForwardSim:
                x += dv*cos(rot)*dt
                y += dv*sin(rot)*dt
                rot += dk*dv*dt
            elif planner_mode == PlannerMode.ClosedForm:
                px, py, pth = parametric_arc(dk, dv, rot, dt)
                x += px
                y += py
                rot += pth

            signals.append(
                Signal(
                    float(x), float(y), float(rot), time_elapsed
                )
            )
            ax.arrow(
                float(x), float(y),
                0.1*cos(float(rot)),
                0.1*sin(float(rot)),
                head_width=0.1
            )
            time_elapsed += float(dt)

    for obstacle in planner_instance.obstacles:
        ax.add_patch(patches.Circle(
            (obstacle[0], obstacle[1]), obstacle[2],
            linewidth=1, edgecolor='r', facecolor='none'
        ))
    
    for goal in planner_instance.final:
        ax.add_patch(patches.Circle(
            (goal[0], goal[1]), 2,
            linewidth=1, edgecolor='black', facecolor='none'
        ))

    return signals

def plan(init, final, obstacles, ax, planner_factory):
    stl_planner = STLPlanner(planner_factory)
    (solution, solver), planner = stl_planner.plan(init, final, obstacles, t_max=1000, granularity=50)
    print(solver.stats()["success"], solver.stats()["t_proc_total"])
    decision_variables = solution["x"]; constraints = solution["g"]
    return plot_plan(ax, decision_variables, planner, solver)

if __name__ == "__main__":
    init = State(0.0, 0.0, 0.0)
    final = [
        [10, 10],
        [0, 10],
        [0, 20],
        [10, 20],
        [20, 20],
        [10, 0],
        [20, 0],
        [20, 10]
    ]

    # X, Y, R -> Circular Obstacles
    obstacles = [
        # [1.0, 1.0, 0.2],
        [5.0, 5.0, 1.0],
    ]

    lm = LimoBot()
    fig, ax = plt.subplots(2, 2)
    
    planner_factory = PlannerFactory(lm, DubinsPlanner, PlannerMode.ClosedForm)
    signals = plan(init, final, obstacles, ax[0][0], planner_factory)

    # signal_x_line = lines.Line2D([], [], linestyle="-")
    # signal_t, signal_x = signal_x_line.get_data()
    # for signal in signals:
    #     signal_x.append(signal.x)
    #     signal_t.append(signal.dt)
    # signal_x_line.set_data(signal_t, signal_x)
    # ax[0][1].add_line(signal_x_line)

    # ax[0][1].set_xlim(0, 120)
    # ax[0][1].set_ylim(-15, 15)

    # ax[0][1].vlines(80, -15, 15, color="red")
    # ax[0][1].vlines(100, -15, 15, color="red")

    # signal_y_line = lines.Line2D([], [], linestyle="-")
    # signal_t, signal_y = signal_y_line.get_data()
    # for signal in signals:
    #     signal_y.append(signal.y)
    #     signal_t.append(signal.dt)
    # signal_y_line.set_data(signal_t, signal_y)
    # ax[1][0].add_line(signal_y_line)

    # ax[1][0].set_xlim(0, 120)
    # ax[1][0].set_ylim(-15, 15)

    # ax[1][0].vlines(80, -15, 15, color="red")
    # ax[1][0].vlines(100, -15, 15, color="red")

    # planner_factory = PlannerFactory(lm, SmoothPlanner, PlannerMode.ClosedForm)
    # stl_planner = STLPlanner(planner_factory)
    # (solution, solver), planner = stl_planner.plan(init, final, obstacles, t_max=1000, granularity=10)
    # print(solver.stats()["success"], solver.stats()["t_proc_total"])
    # decision_variables = solution["x"]; constraints = solution["g"]
    # plot_plan(ax[1][0], decision_variables, planner, solver)

    planner_factory = PlannerFactory(lm, DubinsPlanner, PlannerMode.ForwardSim)
    signals = plan(init, final, obstacles, ax[1][0], planner_factory)

    # planner_factory = PlannerFactory(lm, SmoothPlanner, PlannerMode.ForwardSim)
    # stl_planner = STLPlanner(planner_factory)
    # (solution, solver), planner = stl_planner.plan(init, final, obstacles, t_max=1000, granularity=10)
    # print(solver.stats()["success"], solver.stats()["t_proc_total"])
    # decision_variables = solution["x"]; constraints = solution["g"]
    # plot_plan(ax[1][1], decision_variables, planner, solver)

    plt.show()