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

    return smooth_min([*reach_signals, *avoid_signals])

def generate_trajectories(robot, plot=False):

    initial_states = [
        State(0.0, 0.0, 0.0, 0.0)
    ]

    environ = real_environment_1()
    finish_by = 10 #s
    number_of_waypoints = 3
    granularity = 50
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
    cfhandle.problem.set_constraint("robust_bool", robustness, lbg = 0.1)

    cfsolution, cfsolver = cfhandle.solve()

    print("Optimization Conclusion: ", cfsolver.stats()["success"])
    print("Robustness Value: ", '{:f}'.format(float(cfsolution["g"][-1])))

    # Analyze the solution. Given the decision variables, feed the inputs to an actual model to get the final trajectory.
    decision_variables = cfsolution["x"]; trajectories = []; agent_waypoints = []
    # Next get the waypoints
    for i in range(number_of_waypoints-1):
        decision = decision_variables[i*Waypoint.total_size : (i+1)*Waypoint.total_size]
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

    DeltaT = dt*granularity

    if plot:
        plotter = Plotter(environ.get_plot_options(), None, environ.get_environment())

        # plotter.show_plot(False)

        return agent_waypoints, DeltaT, plotter

    return agent_waypoints, DeltaT

if __name__ == "__main__":
    vicon_data_path = "/root/workspace/src/optimal_control/data/vicon_data"

    vicon_x = []; vicon_y = []
    with open(vicon_data_path, "r") as f:
        for line in f:
            data_array = line.strip().split(" ")
            vicon_x.append(float(data_array[0]))
            vicon_y.append(float(data_array[1]))

    from time import sleep
    import io
    import imageio
    frames = []


    robot = LimoBot()
    trajectory, DeltaT, plotter = generate_trajectories(robot, True)

    state_line = lines.Line2D([], [], linestyle="-", color="seagreen", label="generated trajectory")
    plotter.axis.add_line(state_line)

    vicon_line = lines.Line2D([], [], linestyle="--", color="royalblue", label="real robot")
    plotter.axis.add_line(vicon_line)


    plotter.axis.legend(handles=[vicon_line, state_line])

    plt.show(block=False)

    z = State(0, 0, 0, 0)
    h = 100; dt = DeltaT/h
    progress = 0
    for waypoint in trajectory:
        steering_angle = float(atan(waypoint.k*robot.wheel_base))
        for j in range(h):
            z.v += float(waypoint.a)*dt
            z.x += z.v*cos(z.theta)*dt
            z.y += z.v*sin(z.theta)*dt
            z.theta += z.v*(tan(steering_angle)/robot.wheel_base)*dt

            x, y = state_line.get_data()
            x.append(z.x), y.append(z.y)
            state_line.set_data(x, y)

            x, y = vicon_line.get_data()
            x.append(vicon_x[progress]); y.append(vicon_y[progress])
            progress += 5
            vicon_line.set_data(x, y)

            frame_buffer = io.BytesIO()
            plotter.figure.savefig(frame_buffer, format="raw"); frame_buffer.seek(0)
            frames.append(
                np.reshape(
                    np.frombuffer(frame_buffer.getvalue(), dtype=np.uint8),
                    newshape=(int(plotter.figure.bbox.bounds[3]), int(plotter.figure.bbox.bounds[2]), -1)
                )
            )
            frame_buffer.close()
    
            plotter.figure.canvas.draw()

            plt.pause(dt)
            sleep(dt)
    
    imageio.mimsave("plotted_real_robot_2.mp4", frames[1:])