#!/bin/bash

cd $WORKSPACE_PATH
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build
echo "source $WORKSPACE_PATH/install/setup.bash" >> /root/.bashrc