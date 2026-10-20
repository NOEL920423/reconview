#!/usr/bin/env bash

source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
echo "ros2 setup complete"

ros2_reconview() {
    source /home/noel_614420090/reconview/scripts/setup_ros2.sh
}