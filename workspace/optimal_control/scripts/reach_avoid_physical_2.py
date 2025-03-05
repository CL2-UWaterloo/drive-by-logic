#!/usr/bin/python3

# Solver
from optimal_control.dubins_solver import *

# STL
from optimal_control.robustness import *

# Utils
from optimal_control.executor import *
from optimal_control.environment import *
from optimal_control.plotter import *

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import rclpy.qos

from nav2_msgs.action import FollowPath

import math

import os

def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q

def reach_avoid(all_signals : list[list[Waypoint]], finals, finish_by, obstacles, dt):
    if type(finals) != list:
        finals = [finals]

    avoid_signals = []
    for signals in all_signals:
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

    # Robot 1 Goal
    signals = all_signals[0]; final = finals[0]
    reach_signal = []
    for signal in signals:
        state = get_state_from_waypoint(signal)
        reach_signal.append(final.inside_region(state))

    reach_1_last_goal = eventually(reach_signal, dt=dt, ub=finish_by)

    # Robot 1 Goal
    signals = all_signals[1]; final = finals[2]
    reach_signal = []
    for signal in signals:
        state = get_state_from_waypoint(signal)
        reach_signal.append(final.inside_region(state))

    reach_2_last_goal = eventually(reach_signal, dt=dt, ub=finish_by)

    return smooth_min([
        reach_1_last_goal, reach_2_last_goal, 
        *avoid_signals
    ])


def send_goal(trajectory):
    rclpy.init()
    node = Node("PlanPublisher")

    goalClient = ActionClient(node, FollowPath, "/follow_path")
    goalClient.wait_for_server()

    path_msg = Path()
    for state in trajectory:
        pose_msg = PoseStamped()
        pose_msg.pose.position.x = state.x
        pose_msg.pose.position.y = state.y
        
        quat = quaternion_from_euler(0.0, 0.0, state.theta)
        pose_msg.pose.orientation.x = quat[0]
        pose_msg.pose.orientation.y = quat[1]
        pose_msg.pose.orientation.z = quat[2]
        pose_msg.pose.orientation.w = quat[3]

        path_msg.poses.append(pose_msg)
    
    path_msg.header.frame_id="map"

    goal = FollowPath.Goal()
    goal.path = path_msg

    node._goal_future = goalClient.send_goal_async(goal)
    rclpy.shutdown()

if __name__ == "__main__":
    robot = LimoBot()

    initial_states = [
        State(0.0, 1.0, 0.0, 0.0),
        State(0.0, -1.0, 0.0, 0.0)
    ]

    environ = real_environment_2()
    finish_by = 20 #s
    number_of_waypoints = 4
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

    for i in range(len(cfhandle.problem.signals)):
        idx = str(i)
        signal = cfhandle.problem.signals[i]
        cfhandle.problem.set_constraint("xlimits"+idx, signal.x, lbg=0, ubg=5)
        cfhandle.problem.set_constraint("ylimits"+idx, signal.y, lbg=-1.25, ubg=1.25)

    cfhandle.problem.cost = -1*robustness
    cfhandle.problem.set_constraint("robust_bool", robustness, lbg = 0.15)

    cfsolution, cfsolver = cfhandle.solve()

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

    plotter = Plotter(environ.get_plot_options(), trajectories, environ.get_environment())
    plotter.show_plot()

    os.environ["ROS_DOMAIN_ID"] = "148"
    send_goal(trajectories[0])
    os.environ["ROS_DOMAIN_ID"] = "147"
    send_goal(trajectories[1])