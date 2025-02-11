#!/usr/bin/python3

# Solver
from optimal_control.dubins_solver import *

# STL
from optimal_control.robustness import *

# Utils
from optimal_control.executor import *
from optimal_control.environment import *
from optimal_control.plotter import *

# Define the formula for STL based reach and avoid.

# Reach G and Always \neg O
def reach_avoid(signals, finals, reach_by, obstacles, dt, estates):
    reach_signals = []; avoid_signals = []
    for i in range(len(signals)):
        signal = signals[i]
        state = get_state_from_waypoint(signal); estate = estates[i]
        reach_signals.append(finals[-1].inside_region(state))

        # for obstacle in obstacles:
        #     avoid_signals.append(obstacle.outside_region(state))

        obstacle = Rectangle(0.5, 0.5, (estate.x, estate.y))
        avoid_signals.append(obstacle.outside_region(state))

    # Expected Behaviour: Reach the set by "reach by"
    reach_formula = eventually(reach_signals, dt=dt, ub=reach_by)
    avoid_formula = always(avoid_signals, dt=dt)

    stay_ahead = state.x - estate.x - 0.5

    return smooth_min([reach_formula, avoid_formula, stay_ahead])

if __name__ == "__main__":
    import random

    perturb_x = round(random.uniform(0.0, 0.05), 3)

    robot = LimoBot()

    print("perturb x: ", perturb_x)
    print("perturb y: ", 0.0)

    # Problem Information
    initial_state = State(-5.0 + perturb_x, -0.5, 0.0, 0.0)
    environ = highway()
    reach_by = 22 #s
    number_of_waypoints = 10
    granularity = 50
    number_of_agents = 1

    # Compute dt
    N = (number_of_waypoints - 1)*granularity
    dt = reach_by/N

    e_initial_state = State(-2.5, -0.5, 0.0, 0.0)
    estates = [e_initial_state]
    for i in range(N):
        estate = deepcopy(estates[-1])
        estate.v += 0.05*dt
        estate.x += estate.v*dt
        estates.append(estate)

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
        environ.final, reach_by, environ.obstacles, dt, estates
    )

    # # Minimize negative robustness. Not Boolean Mode
    cfhandle.problem.cost = -1*robustness
    # for signal in cfhandle.problem.signals:
    #     cfhandle.problem.cost += power(signal.k, 2)

    for i in range(len(cfhandle.problem.signals)):
        idx = str(i)
        signal = cfhandle.problem.signals[i]
        cfhandle.problem.set_constraint("ylimits"+idx, signal.y, lbg=-1, ubg=1)

    # Set to boolean mode
    cfhandle.problem.set_constraint("robust_bool", robustness, lbg=0.1, ubg=0.5)
    cfsolution, cfsolver = cfhandle.solve()

    print("Optimization Conclusion: ", cfsolver.stats()["success"])
    print("Robustness Value: ", '{:f}'.format(float(cfsolution["g"][-1])))

    # Analyze the solution. Given the decision variables, feed the inputs to an actual model to get the final trajectory.
    decision_variables = cfsolution["x"]; trajectories = [estates[1:]]
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
            for j in range(granularity):
                dv += waypoint.a*dt
                dx += dv*cos(dth)*dt; dy += dv*sin(dth)*dt
                dth += dv*(tan(steer_angle)/robot.wheel_base)*dt

                agent_trajectory.append(cast_state(State(dx, dy, dth, dv), float))
        trajectories.append(agent_trajectory)

    # Problem Information
    initial_state = State(-5.0, -0.5, 0.0, 0.0)
    environ = highway()
    reach_by = 20 #s
    number_of_waypoints = (number_of_waypoints - 1)*granularity - 1
    granularity = 1
    number_of_agents = 1

    # Compute dt
    N = (number_of_waypoints - 1)*granularity
    dt = reach_by/N

    # Construct the closed form planner
    cfhandle = prep(
        initial_state,
        DubinsPlanner(robot, PlannerMode.ForwardSim),
        reach_by,
        number_of_waypoints,
        granularity,
        number_of_agents
    )

    e_initial_state = State(-2.5, -0.5, 0.0, 0.0)
    estates = [e_initial_state]
    for i in range(N):
        estate = deepcopy(estates[-1])
        estate.v += 0.05*dt
        estate.x += estate.v*dt
        estates.append(estate)

    # Compute Smooth Robustness Formula
    robustness = reach_avoid(
        cfhandle.problem.signals,
        environ.final, reach_by, environ.obstacles, dt, estates
    )

    # # Minimize negative robustness. Not Boolean Mode
    cfhandle.problem.cost = -1*robustness
    # for signal in cfhandle.problem.signals:
    #     cfhandle.problem.cost += power(signal.k, 2)

    for i in range(len(cfhandle.problem.signals)):
        idx = str(i)
        signal = cfhandle.problem.signals[i]
        cfhandle.problem.set_constraint("ylimits"+idx, signal.y, lbg=-1, ubg=1)

    # Set to boolean mode
    cfhandle.problem.set_constraint("robust_bool", robustness, lbg=0.1, ubg=1.0)
    cfsolution, cfsolver = cfhandle.solve()

    print("Optimization Conclusion: ", cfsolver.stats()["success"])
    print("Robustness Value: ", '{:f}'.format(float(cfsolution["g"][-1])))

    # Analyze the solution. Given the decision variables, feed the inputs to an actual model to get the final trajectory.
    decision_variables = cfsolution["x"]
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
            for j in range(granularity):
                dv += waypoint.a*dt
                dx += dv*cos(dth)*dt; dy += dv*sin(dth)*dt
                dth += dv*(tan(steer_angle)/robot.wheel_base)*dt

                agent_trajectory.append(cast_state(State(dx, dy, dth, dv), float))
        trajectories.append(agent_trajectory)

    plotter = Plotter(environ.get_plot_options(), trajectories, environ.get_environment())

    lead_car = Rectangle(0.2, 0.5, (estates[0].x, estates[0].y), color="red")
    trail_car = Rectangle(0.2, 0.5, (initial_state.x, initial_state.y), color="blue")

    lane_divider = lines.Line2D([x-6 for x in range(0, 30)], [0 for y in range(0, 30)], linestyle="--", color="black")
    plotter.axis.add_line(lane_divider)

    plotter.axis.legend(handles=[
        lines.Line2D([], [], linestyle="-", color="red", label="Car B"),
        lines.Line2D([], [], linestyle="-", color="royalblue", label="Car A: Proposed Method"),
        lines.Line2D([], [], linestyle="-", color="seagreen", label="Car A: Baseline Method")
    ])

    plotter.axis.add_patch(lead_car.patch); plotter.axis.add_patch(trail_car.patch)
    plotter.show_animation(record=True)