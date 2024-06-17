#!/usr/bin/python3

from sys import argv

from optimal_control.car_planner import *
from optimal_control.executor import Executor

if __name__ == "__main__":
    init = State(0.0, 0.0, 0.0)
    final = State(-7.0, 7.0, 0.0)

    # X, Y, R -> Circular Obstacles
    obstacles = [
        [5.0, 5.0, 1.0, 0.3],
        [-5.0, 5.0, 1.0, 0.3]
    ]

    lm = LimoBot()
    dc = CarPlanner(lm, granularity=10)

    ex = Executor(dc)
    ex.prep(init=init, final=final, obstacles=obstacles)
    solution, solver = ex.solve()
    print(solver.stats()["success"], solver.stats()["t_proc_total"])

    decision_variables = solution["x"]; constraints = solution["g"]

    if "plot" in argv:
        import matplotlib.pyplot as plt
        import matplotlib.patches as patches

        # Plot trajectory as imagined by the obstacle avoidance check
        intermediate_values = dc.get_constraint_idx_by_pattern("intermediate")
        note_x = []; note_y = []; note_th = []
        for i in range(0, len(intermediate_values), 3):
            x_idx = intermediate_values[i]; y_idx = intermediate_values[i+1]
            note_x.append(float(constraints[x_idx])); note_y.append(float(constraints[y_idx]))

            th_idx = intermediate_values[i+2]
            note_th.append(float(constraints[th_idx]))

        fig, ax = plt.subplots()
        ax.set_xlim(-10, 10)
        ax.set_ylim(-10, 10)

        for obstacle in obstacles:
            ax.add_patch(patches.Circle(
                (obstacle[0], obstacle[1]), obstacle[2],
                linewidth=1, edgecolor='r', facecolor='none'
            ))

            ax.add_patch(patches.Circle(
                (obstacle[0], obstacle[1]), obstacle[2]+obstacle[3],
                linewidth=1, edgecolor='r', facecolor='none'
            ))

        for i in range(len(note_x)):
            ax.arrow(
                note_x[i], note_y[i],
                0.1*cos(note_th[i]),
                0.1*sin(note_th[i]),
                head_width=0.1
            )

        plt.show()

    if "post" in argv:
        # Generating Control Sequences from States
        previous = None; commands = []
        for i in range(0, decision_variables.shape[0], 7):
            # Get all the information
            current = State.from_list(decision_variables[i:i+7])

            # First Iteration
            if previous == None:
                previous = current
                continue

            time_elapsed = 0; dt = current.t*(1/dc.granularity)
            k = previous.k
            while time_elapsed < current.t:
                k += current.s*dt
                commands.append([current.v, k*current.v])
                time_elapsed += dt

        commands.append([0.0, 0.0])

        import rclpy
        rclpy.init()

        ros_interface = TwistPublisher(dt, commands)
        while ros_interface.index < len(commands):
            rclpy.spin_once(ros_interface)

        ros_interface.destroy_node()
        rclpy.shutdown()