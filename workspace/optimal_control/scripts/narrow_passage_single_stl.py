#!/usr/bin/python3

# Solver
from optimal_control.dubins_solver import *

# STL
from optimal_control.robustness import *

# Utils
from optimal_control.executor import *
from optimal_control.environment import *
from optimal_control.plotter import *

# Initial Guess
from optimal_control.initial_guess_generator import *

"""
MARGIN 1e-6
"""

# Define the formula for STL based reach and avoid.

# Reach G and Always \neg O
def reach_avoid(signals, final, reach_by, obstacles):
    """
    robot := signals
    Ask the robot to reach "final" by "reach_by" while avoiding "environment"
    """
    reach_signals = []; obstacle_signals = []
    for signal in signals:
        state = get_state_from_waypoint(signal)
        reach_signals.append((final.inside_region(state), signal))

        for obstacle in obstacles:
            obstacle_signals.append(
                obstacle.outside_region(state)
            )

    # Expected Behaviour: Reach the set by "reach by"
    reach_formula = eventually(*reach_signals, lb=0, ub=reach_by)

    final_vector = vertcat(
        reach_formula,
        *obstacle_signals
    )

    return -logsumexp(-final_vector, MARGIN)

if __name__ == "__main__":
    import random

    perturb_x = round(random.uniform(-0.1, 0.1), 3)
    perturb_y = round(random.uniform(-0.1, 0.1), 3)

    # perturb_x = 0.05
    # perturb_y = 0.02

    robot = LimoBot()

    print("perturb x: ", perturb_x)
    print("perturb y: ", perturb_y)

    # Problem Information
    initial_state = State(0.0, 0.0, 0.0, 0.0)
    environ = narrow_environment()
    reach_by = 50 #s
    number_of_waypoints = 6
    granularity = 25
    number_of_agents = 1

    # Construct the closed form planner
    cfhandle = prep(
        initial_state,
        DubinsPlanner(robot, PlannerMode.ClosedForm),
        reach_by,
        number_of_waypoints,
        granularity,
        number_of_agents
    )
    
    # Compute Smooth Robustness Formula
    robustness = reach_avoid(
        cfhandle.problem.signals,
        environ.final, reach_by, environ.obstacles
    )

    # Add other environment constraints to restrain robot to a box
    # for i in range(len(cfhandle.problem.signals)):
    #     idx = str(i)
    #     signal = cfhandle.problem.signals[i]
    #     cfhandle.problem.set_constraint("xlimits"+idx, signal.x, lbg=-2, ubg=15)
    #     cfhandle.problem.set_constraint("ylimits"+idx, signal.y, lbg=-2, ubg=12)

    # # Minimize negative robustness. Not Boolean Mode
    # cfhandle.problem.cost = -1*robustness

    # Set to boolean mode
    cfhandle.problem.set_constraint("robust_bool", robustness, lbg=0.01, ubg=1.0)

    # Get Initial Guess
    gsolution, gsolver = generate_guess(initial_state, environ.get_optimum_final_states(), number_of_waypoints, robot, reach_by)
    cfsolution, cfsolver = cfhandle.solve(initial_guess=gsolution)

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
        dx = initial_state.x; dy = initial_state.y; dth = initial_state.theta; dv = initial_state.v
        for waypoint in agent_waypoints:
            # Helps rationalize some of the decisions
            steer_angle = float(atan(waypoint.k*robot.wheel_base))

            # Update Dynamic Model
            dt = waypoint.t/granularity
            for j in range(granularity):
                dv += waypoint.a*dt
                dx += dv*cos(dth)*dt; dy += dv*sin(dth)*dt
                dth += dv*(tan(steer_angle)/robot.wheel_base)*dt

                agent_trajectory.append(cast_state(State(dx, dy, dth, dv), float))
        trajectories.append(agent_trajectory)
    
    plotter = Plotter(environ.get_plot_options(), trajectories, environ.get_environment())
    plotter.show_plot()