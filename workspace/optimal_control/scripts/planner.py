#!/usr/bin/python3

from sys import argv

from optimal_control.car_planner import *
from optimal_control.executor import Executor

if __name__ == "__main__":
    init = State(0.0, 0.0, 0.0)
    final = State(10.0, 10.0, 0.0)

    # X, Y, R -> Circular Obstacles
    obstacles = [
        [5.0, 5.0, 1.0, 1.0],
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
        import matplotlib.lines as lines

        # Plot trajectory as imagined by the obstacle avoidance check
        intermediate_values = dc.get_constraint_idx_by_pattern("intermediate")
        note_x = []; note_y = []; note_th = []
        for i in range(0, len(intermediate_values), 3):
            x_idx = intermediate_values[i]; y_idx = intermediate_values[i+1]
            note_x.append(float(constraints[x_idx])); note_y.append(float(constraints[y_idx]))

            th_idx = intermediate_values[i+2]
            note_th.append(float(constraints[th_idx]))

        state_line = lines.Line2D([], [], linestyle=" ", marker="o", color="r")
        for i in range(0, decision_variables.shape[0], 7):
            line_x = state_line.get_xdata()
            line_x.append(float(decision_variables[i]))
            state_line.set_xdata(line_x)

            line_y = state_line.get_ydata()
            line_y.append(float(decision_variables[i+1]))
            state_line.set_ydata(line_y)

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

        ax.add_line(state_line)

        plt.show()

    if "post" in argv:
        # Generating Control Sequences from States
        previous = None; commands = []
        k = 0
        for i in range(0, decision_variables.shape[0], 7):
            # Get all the information
            current = Waypoint.from_list(decision_variables[i:i+7])

            # First Iteration
            if previous == None:
                previous = current
                k = previous.k
                continue

            dt = current.t*(1/dc.granularity)
            for j in range(dc.granularity + 1):
                k += current.s*dt
                commands.append([float(dc.v), float(k*dc.v), float(dt)])

        commands.append([0.0, 0.0, 1.0])

        import rclpy
        from rclpy.node import Node
        from geometry_msgs.msg import Twist

        index = 0
        msg = Twist()
        rclpy.init()
        ros_interface = Node("twist_publisher")
        publisher = ros_interface.create_publisher(Twist, "/cmd_vel", 10)
        while index < len(commands):
            msg.linear.x = commands[index][0]
            msg.angular.z = commands[index][1]
            publisher.publish(msg)
            dt = commands[index][2]
            index += 1
            sleep(dt)

        ros_interface.destroy_node()
        rclpy.shutdown()