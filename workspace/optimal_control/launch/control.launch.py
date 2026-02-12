#!/usr/bin/python3

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os
import yaml
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
import pathlib
import launch.actions
from launch.actions import DeclareLaunchArgument

def generate_launch_description():

    lifecycle_nodes = [
        'waypoint_follower'
    ]

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[os.path.join('/root/workspace/src/optimal_control/config', 'controller.yaml')],
        namespace="limobot1",
        # arguments=['--ros-args', '--log-level', "debug"],
    )

    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[os.path.join('/root/workspace/src/optimal_control/config', 'controller.yaml')],
        namespace="limobot1",
        # arguments=['--ros-args', '--log-level', log_level],
    )

    lf_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        namespace="limobot1",
        parameters=[{'node_names': lifecycle_nodes}, {"autostart": True}],
    )

    ld = LaunchDescription()

    ld.add_action(waypoint_follower)
    ld.add_action(lf_manager)

    return ld
