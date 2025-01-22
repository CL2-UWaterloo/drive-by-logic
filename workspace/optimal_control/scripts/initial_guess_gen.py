#!/usr/bin/python3

from optimal_control.initial_guess_generator import *

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

    environ = simple_environment_1()
    finish_by = 25 #s
    number_of_waypoints = 3

    decision_variables = []
    for initial_state in initial_states:
        solution, solver = generate_guess(initial_state,
            environ.get_optimum_final_states(), 
            number_of_waypoints, robot, finish_by
        )


        decision_variables.append(solution["x"])

    trajectories = []
    # How do we split the decision variables? Split it by the number of agents first.
    for k in range(len(decision_variables)):
        agent_decisions = decision_variables[k]

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
            dt = waypoint.t/100
            for j in range(100):
                dv += waypoint.a*dt
                dx += dv*cos(dth)*dt; dy += dv*sin(dth)*dt
                dth += dv*(tan(steer_angle)/robot.wheel_base)*dt

                agent_trajectory.append(cast_state(State(dx, dy, dth, dv), float))
        trajectories.append(agent_trajectory)
    
    plotter = Plotter(environ.get_plot_options(), trajectories, environ.get_environment())
    plotter.show_animation()